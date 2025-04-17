#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "src/ssd1306.h"
#include "src/pin_configuration.h"
#include "src/font.h"

ssd1306_t ssd;                               // Estrutura para manipulação do display
uint32_t current_time;                       // Armazena o tempo atual
static volatile uint32_t last_time_SW = 0;   // Variável para debounce
volatile bool STATE_LEDS = true;             // Varíavel de estado dos LEDs
uint slice_led_r, slice_led_g, slice_buzzer; // Slices PWM dos LEDs e Buzzer
uint LED_ON = 500;                           // Potência entregue para ligar os LEDs
uint LED_OFF = 0;                            // Potência zerada para desligar
uint32_t last_buzzer_time = 0;               // Último momento que o buzzer bipou
uint32_t buzzer_interval = 0;                // Intervalo do buzzer (1s ou 0.5s)
bool buzzer_state = false;

void gpio_irq_handler(uint gpio, uint32_t event); // Prototipação da função de interrupção

// Configuração do PWM para os LEDs RGB ou o Buzzer
void setup_pwm(uint gpio, uint *slice, uint16_t level)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    *slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_clkdiv(*slice, DIVIDER_PWM);
    pwm_set_wrap(*slice, PERIOD);
    pwm_set_gpio_level(gpio, level);
    pwm_set_enabled(*slice, true);
}

// Configura joystick e botão
void setup_joystick_and_button()
{
    adc_init();
    adc_gpio_init(VRX);
    adc_gpio_init(VRY);

    gpio_init(SW);
    gpio_set_dir(SW, GPIO_IN);
    gpio_pull_up(SW);
}

// Configuração do display OLED
void setup_display()
{
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ADDRESS, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
}

// Configuração inicial do sistema
void setup()
{
    stdio_init_all();
    setup_joystick_and_button();
    setup_display();
    setup_pwm(LED_R, &slice_led_r, LED_ON);
    setup_pwm(LED_G, &slice_led_g, LED_ON);
    setup_pwm(BUZZER_A, &slice_buzzer, 0);

    gpio_set_irq_enabled_with_callback(SW, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
}

// Função de alternação dos LEDs
void alter_leds()
{
    STATE_LEDS = !STATE_LEDS;

    if (!STATE_LEDS)
    {
        pwm_set_gpio_level(LED_R, LED_OFF);
        pwm_set_gpio_level(LED_G, LED_OFF);
    }
}

// Função para leitura do joystick
void joystick_read_axis(uint16_t *vrx_value, uint16_t *vry_value)
{
    adc_select_input(ADC_CHANNEL_1);
    sleep_us(2);
    *vrx_value = adc_read();

    adc_select_input(ADC_CHANNEL_0);
    sleep_us(2);
    *vry_value = adc_read();
}

int main()
{
    uint16_t vrx_value, vry_value;                 // Variáveis para armazenar os valores do joysticks
    char str_x[20], str_y[20], status_message[20]; // Buffers para armazenar os valores formatados
    setup();

    while (true)
    {
        joystick_read_axis(&vrx_value, &vry_value);

        // Converte os valores do joystick para porcentagem e ppm
        uint8_t umidade = (vrx_value * 100) / 4070;        // Porcentagem do sensor de umidade
        uint16_t qualidade_ar = (vry_value * 2000) / 4070; // ppm do sensor de qualidade do ar

        // Formata os valores como string
        snprintf(str_x, sizeof(str_x), "Um: %d %%", umidade);
        snprintf(str_y, sizeof(str_y), "Ar: %d ppm", qualidade_ar);

        ssd1306_fill(&ssd, !COLOR);
        ssd1306_rect(&ssd, 3, 3, 120, 56, COLOR, !COLOR, 1); // Desenha a borda

        // Determina as posições centralizadas para os textos
        uint8_t center_x_x = (WIDTH - (strlen(str_x) * 8)) / 2;
        uint8_t center_x_y = (WIDTH - (strlen(str_y) * 8)) / 2;
        uint8_t center_y = HEIGHT / 2 - 8; // Posição vertical ajustada

        uint64_t this_current_time = time_us_64();

        // Define a mensagem de status antes de verificar STATE_LEDS
        if ((umidade >= 40 && umidade <= 60) && (qualidade_ar >= 800 && qualidade_ar <= 1200)) // Dentro da zona segura
        {
            snprintf(status_message, sizeof(status_message), "ESTAVEL");
            buzzer_interval = 0;             // Buzzer desligado
            pwm_set_gpio_level(BUZZER_A, 0); // Desliga o buzzer
        }
        else if (umidade >= 90 || umidade <= 10 || qualidade_ar >= 1800 || qualidade_ar <= 200) // Zona de perigo
        {
            snprintf(status_message, sizeof(status_message), "PERIGO");
            buzzer_interval = 250000; // 0.25 segundos (2 vezes por segundo)
        }
        else // Zona de alerta
        {
            snprintf(status_message, sizeof(status_message), "ALERTA");
            buzzer_interval = 500000; // 0.5 segundos (1 vezes por segundo)
        }

        // Controla ontrola LEDs apenas se STATE_LEDS estiver ativado
        if (STATE_LEDS)
        {
            if (strcmp(status_message, "ESTAVEL") == 0)
            {
                pwm_set_gpio_level(LED_G, LED_ON);
                pwm_set_gpio_level(LED_R, LED_OFF);
            }
            else if (strcmp(status_message, "PERIGO") == 0)
            {
                pwm_set_gpio_level(LED_G, LED_OFF);
                pwm_set_gpio_level(LED_R, LED_ON);
            }
            else if (strcmp(status_message, "ALERTA") == 0)
            {
                pwm_set_gpio_level(LED_G, LED_ON);
                pwm_set_gpio_level(LED_R, LED_ON);
            }
        }

        // Condição para o buzzer bipar
        if (buzzer_interval > 0 && (this_current_time - last_buzzer_time >= buzzer_interval))
        {
            last_buzzer_time = this_current_time;
            buzzer_state = !buzzer_state; // Alterna o estado do buzzer

            if (buzzer_state)
                pwm_set_gpio_level(BUZZER_A, 300); // Ativa o buzzer
            else
                pwm_set_gpio_level(BUZZER_A, 0); // Desliga o buzzer
        }

        // Desenha os valores no centro do display
        ssd1306_draw_string(&ssd, str_x, center_x_x, center_y - 8);
        ssd1306_draw_string(&ssd, str_y, center_x_y, center_y + 2);

        // Desenha o status abaixo das leituras
        uint8_t center_status = (WIDTH - (strlen(status_message) * 8)) / 2;
        ssd1306_draw_string(&ssd, status_message, center_status, center_y + 14);

        ssd1306_send_data(&ssd); // Envia os dados para o leitor

        sleep_ms(10);
    }
}

// Função de interrupção para os botões com debounce
void gpio_irq_handler(uint gpio, uint32_t event)
{
    current_time = to_us_since_boot(get_absolute_time()); // Obtém o tempo atual em microssegundos

    if (gpio == SW) // Se a interrupção veio do botão SW
    {
        if (current_time - last_time_SW > 200000) // 200 ms de debouncing
        {
            last_time_SW = current_time;
            alter_leds();
        }
    }
}