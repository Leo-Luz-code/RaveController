#include <stdio.h>
#include "hardware/i2c.h"
#include "src/ssd1306.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "src/np_led.h"
#include "src/handle_buttonB.h"

// Definição dos pinos
const int VRX = 27;
const int VRY = 26;
const int ADC_CHANNEL_0 = 0;
const int ADC_CHANNEL_1 = 1;
const int SW = 22;
const int BUTTON_A = 5;
const int BUTTON_B = 6;
uint32_t current_time;

// Variáveis para debounce dos botões
static volatile uint32_t last_time_SW = 0;
static volatile uint32_t last_time_A = 0;
static volatile uint32_t last_time_B = 0;

// LEDs RGB, NP e PWM
#define MATRIX_LED_PIN 7
float brightnessScale = 0.01; // 1% de brilho
const int LED_G = 11;
const int LED_B = 12;
const int LED_R = 13;
const float DIVIDER_PWM = 16.0;
const uint16_t PERIOD = 4096;
uint16_t led_b_level, led_r_level = 0;
uint slice_led_b, slice_led_r, slice_buzzer;

// BUZZER
const uint BUZZER_A = 21; // Pino para o buzzer A
volatile bool botao_pressionado = false;
uint32_t last_buzzer_time = 0;
uint32_t buzzer_interval = 250000;
bool buzzer_state = false;

// Configuração do display OLED via I2C
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define ADDRESS 0x3C
ssd1306_t ssd; // Estrutura para manipulação do display

// Configuração do UART
#define UART_ID uart0    // Seleciona a UART0.
#define BAUD_RATE 115200 // Define a taxa de transmissão.
#define UART_TX_PIN 0    // Pino GPIO usado para TX.
#define UART_RX_PIN 1    // Pino GPIO usado para RX.

// Variáveis de controle do sistema
bool COLOR = true;
const int thick = 5;
const int thin = 1;
volatile int width = 1;
volatile bool STATE_BORDER_WIDTH = true;
volatile bool STATE_GREEN_LED = false;
volatile bool STATE_ALL_LEDS = true;
volatile int matrix_led_state = 0;

// Alterna a largura da borda no display
void border_width()
{
    width = (STATE_BORDER_WIDTH) ? thin : thick; // Expressão ternária que define a largura da borda com base no valor de STATE_BORDER_WIDTH
}

void buzzer_on()
{
    pwm_set_gpio_level(BUZZER_A, 300);
    buzzer_state = true;
    last_buzzer_time = time_us_64();
}

void buzzer_off()
{
    pwm_set_gpio_level(BUZZER_A, 0);
    buzzer_state = false;
}

// Função de interrupção para os botões com debounce
void gpio_irq_handler(uint gpio, uint32_t event)
{
    current_time = to_us_since_boot(get_absolute_time()); // Tempo atual

    if (gpio == SW && (current_time - last_time_SW > 200000))
    {
        last_time_SW = current_time;
        STATE_BORDER_WIDTH = !STATE_BORDER_WIDTH;
        border_width();

        if (STATE_ALL_LEDS)
        {
            STATE_GREEN_LED = !STATE_GREEN_LED;
            gpio_put(LED_G, STATE_GREEN_LED);
        }

        buzzer_on();
    }

    if (gpio == BUTTON_A && (current_time - last_time_A > 200000))
    {
        last_time_A = current_time;
        STATE_ALL_LEDS = !STATE_ALL_LEDS;

        if (STATE_GREEN_LED)
        {
            STATE_GREEN_LED = false;
            gpio_put(LED_G, STATE_GREEN_LED);
        }

        buzzer_on();
    }

    if (gpio == BUTTON_B && (current_time - last_time_B > 200000))
    {
        last_time_B = current_time;
        matrix_led_state = (matrix_led_state + 1) % 8;
        buzzer_on();
    }
}

// Configura joystick e botões
void setup_peripherals()
{
    // Inicializa a UART.
    uart_init(UART_ID, BAUD_RATE);

    // Configura os pinos GPIO para a UART.
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART); // Configura o pino 0 para TX
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART); // Configura o pino 1 para RX

    // Mensagem inicial.
    const char *init_message = "CEPEDI - TIC37\r\n"
                               "UART Inicializado";
    uart_puts(UART_ID, init_message);

    // Inicializa matriz de LEDs.
    npInit(MATRIX_LED_PIN);
    npClear();
    npWrite(); // Escreve os dados nos LEDs.

    adc_init();
    adc_gpio_init(VRX);
    adc_gpio_init(VRY);

    gpio_init(SW);
    gpio_set_dir(SW, GPIO_IN);
    gpio_pull_up(SW);

    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    gpio_pull_up(BUTTON_A);

    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);
}

// Configuração do PWM para os LEDs RGB
void setup_pwm(uint led, uint *slice, uint16_t level)
{
    gpio_set_function(led, GPIO_FUNC_PWM);
    *slice = pwm_gpio_to_slice_num(led);
    pwm_set_clkdiv(*slice, DIVIDER_PWM);
    pwm_set_wrap(*slice, PERIOD);
    pwm_set_gpio_level(led, level);
    pwm_set_enabled(*slice, true);
}

// Configuração do LED verde
void setup_green_led()
{
    gpio_init(LED_G);
    gpio_set_dir(LED_G, GPIO_OUT);
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
    setup_peripherals();
    setup_pwm(LED_R, &slice_led_r, led_r_level);
    setup_pwm(LED_B, &slice_led_b, led_b_level);
    setup_pwm(BUZZER_A, &slice_buzzer, 0);
    setup_green_led();
    setup_display();

    gpio_set_irq_enabled_with_callback(SW, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled(BUTTON_A, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BUTTON_B, GPIO_IRQ_EDGE_FALL, true);
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
    uint16_t vrx_value, vry_value;
    setup();
    printf("Trabalho 1 Fase 2 Residência\n");

    applyBrightnessToMatrix(LETRA_L, brightnessScale);
    applyBrightnessToMatrix(LETRA_E, brightnessScale);
    applyBrightnessToMatrix(LETRA_O, brightnessScale);
    applyBrightnessToMatrix(LETRA_N, brightnessScale);
    applyBrightnessToMatrix(LETRA_A, brightnessScale);
    applyBrightnessToMatrix(LETRA_R, brightnessScale);
    applyBrightnessToMatrix(LETRA_D, brightnessScale);

    while (true)
    {
        handle_buttonB(matrix_led_state);

        joystick_read_axis(&vrx_value, &vry_value);

        ssd1306_fill(&ssd, !COLOR);
        ssd1306_rect(&ssd, 3, 3, 120, 56, COLOR, !COLOR, width); // Desenha a borda

        if ((vrx_value > 2500 || vrx_value < 1700) && STATE_ALL_LEDS) // Se sair da zona morta
        {
            pwm_set_gpio_level(LED_R, vrx_value); // Define o level vermelho do LED como o valor do eixo X
        }
        else
        {
            pwm_set_gpio_level(LED_R, 0); // Se não, deixa desligado
        }

        if ((vry_value > 2500 || vry_value < 1700) && STATE_ALL_LEDS) // Se sair da zona morta
        {
            pwm_set_gpio_level(LED_B, vry_value); // Define o level azul do LED como o valor do eixo Y
        }
        else
        {
            pwm_set_gpio_level(LED_B, 0); // Se não, deixa desligado
        }

        // A cada iteração, desenhar o quadrado com base na localização do joystick
        ssd1306_square(&ssd, (((4095 - vry_value) / 79) + 2), ((vrx_value / 35) + 2), 8, 8, COLOR, COLOR);
        ssd1306_send_data(&ssd); // Envia os dados para o leitor

        uint64_t this_current_time = time_us_64();

        if (buzzer_interval > 0 && (this_current_time - last_buzzer_time >= buzzer_interval))
        {
            if (buzzer_state)
            {
                buzzer_off(); // desliga o buzzer após o tempo
            }
        }

        sleep_ms(10);
    }
}