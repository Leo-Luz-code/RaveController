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

// ------------------------
// Definição dos pinos e variáveis globais
// ------------------------

const int VRX = 27;          // Eixo X do joystick (ADC)
const int VRY = 26;          // Eixo Y do joystick (ADC)
const int ADC_CHANNEL_0 = 0; // Canal ADC 0
const int ADC_CHANNEL_1 = 1; // Canal ADC 1
const int SW = 22;           // Botão do joystick
const int BUTTON_A = 5;      // Botão A
const int BUTTON_B = 6;      // Botão B
uint32_t current_time;       // Tempo atual (usado para debounce)

// Variáveis para controle de debounce
static volatile uint32_t last_time_SW = 0;
static volatile uint32_t last_time_A = 0;
static volatile uint32_t last_time_B = 0;

// ------------------------
// LEDs RGB, Neopixel e PWM
// ------------------------

#define MATRIX_LED_PIN 7               // Pino do LED neopixel
float brightnessScale = 0.01;          // Escala de brilho (1%)
const int LED_G = 11;                  // LED verde
const int LED_B = 12;                  // LED azul
const int LED_R = 13;                  // LED vermelho
const float DIVIDER_PWM = 16.0;        // Divisor de clock para PWM
const uint16_t PERIOD = 4096;          // Período do PWM
uint16_t led_b_level, led_r_level = 0; // Intensidade dos LEDs
uint slice_led_b, slice_led_r, slice_buzzer;

// ------------------------
// BUZZER
// ------------------------

const uint BUZZER_A = 21;
volatile bool botao_pressionado = false;
uint32_t last_buzzer_time = 0;
uint32_t buzzer_interval = 250000; // Intervalo para desligar o buzzer
bool buzzer_state = false;         // Estado atual do buzzer

// ------------------------
// Display OLED via I2C
// ------------------------

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define ADDRESS 0x3C
ssd1306_t ssd;

// ------------------------
// UART
// ------------------------

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// ------------------------
// Controle de estado do sistema
// ------------------------

bool COLOR = true;
const int thick = 5;
const int thin = 1;
volatile int width = 1;
volatile bool STATE_BORDER_WIDTH = true;
volatile bool STATE_GREEN_LED = false;
volatile bool STATE_ALL_LEDS = true;
volatile int matrix_led_state = 0;

// ------------------------
// Alterna a largura da borda no display
// ------------------------

void border_width()
{
    width = (STATE_BORDER_WIDTH) ? thin : thick;
}

// ------------------------
// Liga e desliga o buzzer
// ------------------------

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

// ------------------------
// Interrupções dos botões com debounce
// ------------------------

void gpio_irq_handler(uint gpio, uint32_t event)
{
    current_time = to_us_since_boot(get_absolute_time());

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

// ------------------------
// Configuração dos periféricos iniciais
// ------------------------

void setup_peripherals()
{
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_puts(UART_ID, "CEPEDI - TIC37\r\nUART Inicializado");

    npInit(MATRIX_LED_PIN);
    npClear();
    npWrite();

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

// ------------------------
// Inicialização do PWM para LED RGB ou buzzer
// ------------------------

void setup_pwm(uint led, uint *slice, uint16_t level)
{
    gpio_set_function(led, GPIO_FUNC_PWM);
    *slice = pwm_gpio_to_slice_num(led);
    pwm_set_clkdiv(*slice, DIVIDER_PWM);
    pwm_set_wrap(*slice, PERIOD);
    pwm_set_gpio_level(led, level);
    pwm_set_enabled(*slice, true);
}

// ------------------------
// LED verde digital
// ------------------------

void setup_green_led()
{
    gpio_init(LED_G);
    gpio_set_dir(LED_G, GPIO_OUT);
}

// ------------------------
// Inicializa o display OLED via I2C
// ------------------------

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

// ------------------------
// Função de setup principal
// ------------------------

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

// ------------------------
// Leitura dos valores do joystick
// ------------------------

void joystick_read_axis(uint16_t *vrx_value, uint16_t *vry_value)
{
    adc_select_input(ADC_CHANNEL_1);
    sleep_us(2);
    *vrx_value = adc_read();

    adc_select_input(ADC_CHANNEL_0);
    sleep_us(2);
    *vry_value = adc_read();
}

// ------------------------
// Função principal
// ------------------------

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
        ssd1306_rect(&ssd, 3, 3, 120, 56, COLOR, !COLOR, width); // Desenha a borda da tela

        // Verifica se saiu da zona morta do eixo X (VRX)
        if ((vrx_value > 2500 || vrx_value < 1700) && STATE_ALL_LEDS)
        {
            pwm_set_gpio_level(LED_R, vrx_value); // Controla o LED vermelho
        }
        else
        {
            pwm_set_gpio_level(LED_R, 0);
        }

        // Verifica se saiu da zona morta do eixo Y (VRY)
        if ((vry_value > 2500 || vry_value < 1700) && STATE_ALL_LEDS)
        {
            pwm_set_gpio_level(LED_B, vry_value); // Controla o LED azul
        }
        else
        {
            pwm_set_gpio_level(LED_B, 0);
        }

        // Desenha quadrado no display conforme posição do joystick
        ssd1306_square(&ssd, (((4095 - vry_value) / 79) + 2), ((vrx_value / 35) + 2), 8, 8, COLOR, COLOR);
        ssd1306_send_data(&ssd);

        // Controle de tempo para desligar o buzzer automaticamente
        uint64_t this_current_time = time_us_64();
        if (buzzer_interval > 0 && (this_current_time - last_buzzer_time >= buzzer_interval))
        {
            if (buzzer_state)
            {
                buzzer_off();
            }
        }

        sleep_ms(10);
    }
}
