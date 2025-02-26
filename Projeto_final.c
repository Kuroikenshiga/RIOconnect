#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "inc/ssd1306.h"
#include <stdbool.h>
#include <math.h>

const uint PIN_VRX = 27, PIN_BTN = 5, PIN_BUZZER = 21, WRAP = 2000;
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

// Possíveis destinos da embarcação
char DESTINO_UBAITABA[] = "UBAITABA", DESTINO_AURELINO_LEAL[] = "AURELINO LEAL";

ssd1306_t ssd;
// Agrupa os dados que serã oenviados
struct Data
{
    bool status;
    bool status_travessia;
    int32_t tempo;
    char *porto_destino;
};

bool is_connection_successful = false;
volatile bool try_connect = true, status = true;
volatile alarm_id_t CURRENT_ALARM = 0;
uint64_t last_sensor_verification = 0, last_event = 0, last_sensor_value = 0;
// Armazena o slice responsável por gerar o pwm de um pino GPIO específico
uint slice = 0;

int64_t alarm_callback(alarm_id_t id, void *user_data);
void gpio_irq_handler_callback(uint gpio, uint32_t events);
bool network_start();
void start_up();
void send_alert(char *message, bool make_sound);
uint16_t read_sensor();
void send_data(struct Data *data);

int main()
{
    stdio_init_all();

    // Configura os pinos GPIO, I2C e PWM
    start_up();

    // Configuração do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, i2c1); // Inicializa o display
    ssd1306_config(&ssd);                                     // Configura o display
    ssd1306_send_data(&ssd);                                  // Envia os dados para o display

    ssd1306_rect(&ssd, 3, 3, 122, 60, 1, !1); // Desenha um retângulo
    ssd1306_send_data(&ssd);
    // Struct que irá armazenar os dados que serão enviados
    struct Data data_to_send;

    while (true)
    {

        if (try_connect && CURRENT_ALARM == 0)
        {
            // Tenta estabelecer conexão
            is_connection_successful = network_start();
            CURRENT_ALARM = add_alarm_in_ms(5000, alarm_callback, NULL, false);
            if (!is_connection_successful)
            {
                send_alert("Sem sinal", true);
                ssd1306_draw_string(&ssd, "status OFF", 28, 20);
                ssd1306_send_data(&ssd);
            }
            else
            {
                send_alert("Com sinal", false);
                ssd1306_draw_string(&ssd, "status ON ", 28, 20);
                ssd1306_send_data(&ssd);
            }
        }

        uint16_t sensor_value = read_sensor();
        // ssd1306_draw_string(&ssd,"Em funcionamento",0,10);
        // ssd1306_send_data(&ssd);
        if (abs(sensor_value - 2048) != 0)
        {
            // Armazena o status de funcionamento da embarcação
            data_to_send.status = status;
            // Indica que a embarcação está ou não em rota par o destino
            data_to_send.status_travessia = false;
            // Armazena o tempo de travessia em segundos
            uint64_t current_sensor_verification = to_ms_since_boot(get_absolute_time());

            char *destino = sensor_value > 2048 ? DESTINO_AURELINO_LEAL : DESTINO_UBAITABA;
            // Envia os dados somente quando houver mudança nos sensores
            if (destino[0] != data_to_send.porto_destino[0])
            {
                data_to_send.porto_destino = destino;
                data_to_send.tempo = last_sensor_verification == 0 ? -1 : (current_sensor_verification - last_sensor_verification) / 1000;
                last_sensor_verification = current_sensor_verification;
                send_data(&data_to_send);
            }
        }
        else
        {
            // Armazena o status de funcionamento da embarcação
            data_to_send.status = status;
            // Indica que a embarcação está ou não em rota par o destino
            data_to_send.status_travessia = true;
            data_to_send.tempo = -1;
            send_data(&data_to_send);
        }

        sleep_ms(1000);
    }
}
bool network_start()
{
    // Configuração do wifi e protocolo CoAP
    sleep_ms(3000);
    return true;
}
void send_alert(char *message, bool make_sound)
{
    ssd1306_draw_string(&ssd, message, 0, 0);
    ssd1306_send_data(&ssd);
    if (make_sound)
    {
        pwm_set_gpio_level(PIN_BUZZER, WRAP * 0.1);
        sleep_ms(1000);
        pwm_set_gpio_level(PIN_BUZZER, 0);
    }
}
int64_t alarm_callback(alarm_id_t id, void *user_data)
{
    if (!is_connection_successful)
    {
        try_connect = true;
        cancel_alarm(CURRENT_ALARM);
        CURRENT_ALARM = 0;
        return 0;
    }
    try_connect = false;
    cancel_alarm(CURRENT_ALARM);
    CURRENT_ALARM = 0;
    return 0;
}
void gpio_irq_handler_callback(uint gpio, uint32_t events)
{
    uint64_t current_event = to_us_since_boot(get_absolute_time());
    if (current_event - last_event > 200000)
    {
        last_event = current_event;
        status = !status;
        ssd1306_draw_string(&ssd, status ? "status ON " : "status OFF", 28, 20);
        ssd1306_send_data(&ssd);
    }
}
void start_up()
{
    // Configura a comunicação I2c
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Configuração do conversor ADC
    adc_init();
    adc_gpio_init(PIN_VRX);

    // onfiguração do botão
    gpio_init(PIN_BTN);
    gpio_set_dir(PIN_BTN, GPIO_IN);
    gpio_pull_up(PIN_BTN);
    gpio_set_irq_enabled_with_callback(PIN_BTN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler_callback);
    // Configuração do pwm do buzzer
    slice = pwm_gpio_to_slice_num(PIN_BUZZER);
    gpio_set_function(PIN_BUZZER, GPIO_FUNC_PWM);
    pwm_set_wrap(slice, WRAP);
    pwm_set_clkdiv(slice, 8);
    pwm_set_enabled(slice, 1);
    pwm_set_gpio_level(PIN_BUZZER, 0);
}
uint16_t read_sensor()
{
    adc_select_input(1);
    sleep_us(10);
    uint16_t adc_values_x = adc_read();
    adc_values_x = abs(adc_values_x - 2048) > 200 ? adc_values_x : 2048;
    return adc_values_x;
}
void send_data(struct Data *data)
{
    if (is_connection_successful)
    {
        printf("dados :{\nStatus: %d,\nStatus Travessia: %d,\nTempo: %d,\nDestino: %s\n}", data->status, data->status_travessia, data->tempo, data->porto_destino);
        fflush(stdout);
    }
    else
    {
        printf("Erro no envio dos dados");
        fflush(stdout);
    }
    sleep_ms(100);
}