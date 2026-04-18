#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

// ----------- ADC CONFIG -----------
#define ADC_CHANNEL_NIVEL   ADC1_CHANNEL_3   // GPIO4
#define ADC_CHANNEL_HUM     ADC1_CHANNEL_4   // GPIO5
#define ADC_CHANNEL_TEMP    ADC1_CHANNEL_2   // GPIO3

#define DEFAULT_VREF        1100   // mV
#define ADC_SAMPLES         64

// Calibracion para el sensor de nivel
#define NIVEL_EMPTY_MV      200
#define NIVEL_FULL_MV       3200

// Rangos del sensor de humedad de suelo
#define HUMEDAD_SECO_MAX    300
#define HUMEDAD_HUMEDO_MAX  700
#define HUMEDAD_AGUA_MAX    950

static esp_adc_cal_characteristics_t adc_chars;
static esp_adc_cal_characteristics_t adc_chars_temp;

typedef struct {
    uint32_t hum_mv;
    uint32_t nivel_mv;
    uint32_t temp_mv;
    float temperatura;
    uint8_t nivel_pct;
    const char* hum_estado;
    const char* temp_estado;
    float lux;
    const char* lux_estado;
    bool valvulas_on;
} sensor_data_t;

static sensor_data_t g_sensor_data;
static SemaphoreHandle_t g_sensor_mutex;

// ----------- I2C CONFIG -----------
// Sensor de luz BH1750 conectado por I2C
#define I2C_MASTER_SDA_IO       8   // GPIO8
#define I2C_MASTER_SCL_IO       9   // GPIO9
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000

#define BH1750_ADDR             0x23
#define BH1750_CMD_POWER_ON     0x01
#define BH1750_CMD_RESET        0x07
#define BH1750_CMD_CONT_HIGH_RES 0x10

// Pines para activación de electrovalvulas
#define VALVULA1_GPIO           16
#define VALVULA2_GPIO           17
#define VALVULA_ON_LEVEL        1
#define VALVULA_OFF_LEVEL       0

static inline uint8_t nivel_porcentaje(uint32_t mv) {
    if (mv <= NIVEL_EMPTY_MV) {
        return 0;
    }
    if (mv >= NIVEL_FULL_MV) {
        return 100;
    }
    return (uint8_t)((mv - NIVEL_EMPTY_MV) * 100 / (NIVEL_FULL_MV - NIVEL_EMPTY_MV));
}

static inline const char* humedad_estado(uint32_t mv) {
    if (mv <= HUMEDAD_SECO_MAX) {
        return "suelo seco";
    }
    if (mv <= HUMEDAD_HUMEDO_MAX) {
        return "suelo humedo";
    }
    if (mv <= HUMEDAD_AGUA_MAX) {
        return "en el agua";
    }
    return "fuera de rango";
}

// ----------- LM35 CONVERSION -----------
// LM35 = 10 mV por grado C
static inline float lm35_to_celsius(uint32_t mv) {
    return mv / 10.0f;
}

static inline const char* temperatura_estado(float temp_c) {
    if (temp_c < 20.0f) {
        return "temperatura baja";
    }
    if (temp_c <= 30.0f) {
        return "temperatura ambiente";
    }
    return "temperatura alta";
}

static inline const char* luz_estado(float lux) {
    if (lux < 50.0f) {
        return "muy bajo - oscuro";
    }
    if (lux < 200.0f) {
        return "bajo - sombra";
    }
    if (lux < 500.0f) {
        return "moderado - luz indirecta";
    }
    if (lux < 1000.0f) {
        return "bueno - luz brillante";
    }
    if (lux < 10000.0f) {
        return "alto - sol parcial";
    }
    if (lux < 100000.0f) {
        return "muy alto - sol directo";
    }
    return "extremo - exceso de luz";
}

// ----------- ADC READ PROMEDIO -----------
static uint32_t read_adc_mv(adc1_channel_t channel, esp_adc_cal_characteristics_t *chars) {
    uint32_t raw_sum = 0;

    for (int i = 0; i < ADC_SAMPLES; i++) {
        raw_sum += adc1_get_raw(channel);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    uint32_t raw_avg = raw_sum / ADC_SAMPLES;
    return esp_adc_cal_raw_to_voltage(raw_avg, chars);
}

// ----------- I2C INIT -----------
esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// ----------- BH1750 INIT -----------
esp_err_t bh1750_init(void) {
    uint8_t cmd;
    esp_err_t err;

    cmd = BH1750_CMD_POWER_ON;
    err = i2c_master_write_to_device(I2C_MASTER_NUM, BH1750_ADDR, &cmd, 1, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) return err;

    cmd = BH1750_CMD_RESET;
    err = i2c_master_write_to_device(I2C_MASTER_NUM, BH1750_ADDR, &cmd, 1, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) return err;

    cmd = BH1750_CMD_CONT_HIGH_RES;
    err = i2c_master_write_to_device(I2C_MASTER_NUM, BH1750_ADDR, &cmd, 1, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(180));
    return ESP_OK;
}

// ----------- BH1750 READ -----------
float bh1750_read(void) {
    uint8_t data[2];
    esp_err_t err = i2c_master_read_from_device(I2C_MASTER_NUM, BH1750_ADDR, data, 2, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        return -1.0f;
    }

    int raw = (data[0] << 8) | data[1];
    return raw / 1.2f;
}

// ----------- ADC INIT -----------
void adc_init(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);

    adc1_config_channel_atten(ADC_CHANNEL_HUM, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(ADC_CHANNEL_NIVEL, ADC_ATTEN_DB_12);

    // Para LM35 conviene 0 dB porque trabaja en voltajes bajos
    // y asi se obtiene mejor resolucion.
    adc1_config_channel_atten(ADC_CHANNEL_TEMP, ADC_ATTEN_DB_0);

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_chars);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_chars_temp);
}

// ----------- VALVULA INIT -----------
void valvula_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << VALVULA1_GPIO) | (1ULL << VALVULA2_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(VALVULA1_GPIO, VALVULA_OFF_LEVEL);
    gpio_set_level(VALVULA2_GPIO, VALVULA_OFF_LEVEL);
}

static inline void valvula_set(uint32_t gpio_num, bool on) {
    gpio_set_level(gpio_num, on ? VALVULA_ON_LEVEL : VALVULA_OFF_LEVEL);
}

static inline bool valvula_debe_activar(uint32_t hum_mv, uint8_t nivel_pct, float temp_c, float lux) {
    bool sueloSeco = hum_mv <= HUMEDAD_SECO_MAX;
    bool nivelSuficiente = nivel_pct >= 20;
    bool temperaturaSegura = (temp_c >= 10.0f && temp_c <= 40.0f);
    bool luzAdecuada = (lux >= 50.0f);

    return sueloSeco && nivelSuficiente && temperaturaSegura && luzAdecuada;
}

static void sensor_task(void *arg) {
    while (1) {
        uint32_t hum_mv   = read_adc_mv(ADC_CHANNEL_HUM, &adc_chars);
        uint32_t nivel_mv = read_adc_mv(ADC_CHANNEL_NIVEL, &adc_chars);
        uint32_t temp_mv  = read_adc_mv(ADC_CHANNEL_TEMP, &adc_chars_temp);

        float temperatura = lm35_to_celsius(temp_mv);
        uint8_t nivel_pct = nivel_porcentaje(nivel_mv);
        const char* hum_estado = humedad_estado(hum_mv);
        const char* temp_estado = temperatura_estado(temperatura);
        float lux = bh1750_read();
        const char* lux_estado = (lux >= 0) ? luz_estado(lux) : "error de lectura";

        bool activar_valvulas = valvula_debe_activar(hum_mv, nivel_pct, temperatura, lux);
        valvula_set(VALVULA1_GPIO, activar_valvulas);
        valvula_set(VALVULA2_GPIO, activar_valvulas);

        if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
            g_sensor_data.hum_mv = hum_mv;
            g_sensor_data.nivel_mv = nivel_mv;
            g_sensor_data.temp_mv = temp_mv;
            g_sensor_data.temperatura = temperatura;
            g_sensor_data.nivel_pct = nivel_pct;
            g_sensor_data.hum_estado = hum_estado;
            g_sensor_data.temp_estado = temp_estado;
            g_sensor_data.lux = lux;
            g_sensor_data.lux_estado = lux_estado;
            xSemaphoreGive(g_sensor_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void report_task(void *arg) {
    sensor_data_t local_data;

    while (1) {
        if (xSemaphoreTake(g_sensor_mutex, portMAX_DELAY) == pdTRUE) {
            local_data = g_sensor_data;
            xSemaphoreGive(g_sensor_mutex);
        }

        printf("\n------ Lecturas ------\n");
        printf("Humedad: %lu mV (%s)\n", local_data.hum_mv, local_data.hum_estado);
        printf("Nivel: %lu mV (%u%%)\n", local_data.nivel_mv, local_data.nivel_pct);
        printf("Temperatura: %.2f C (%s)\n", local_data.temperatura, local_data.temp_estado);

        if (local_data.lux < 0) {
            printf("Luz: error de lectura\n");
        } else {
            printf("Luz: %.2f lux (%s)\n", local_data.lux, local_data.lux_estado);
        }

        printf("Valvulas: %s\n", local_data.valvulas_on ? "ON" : "OFF");
        printf("----------------------\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ----------- MAIN -----------
void app_main(void) {
    esp_err_t err;

    g_sensor_mutex = xSemaphoreCreateMutex();
    if (g_sensor_mutex == NULL) {
        printf("Error creando mutex\n");
        return;
    }

    g_sensor_data = (sensor_data_t){
        .hum_mv = 0,
        .nivel_mv = 0,
        .temp_mv = 0,
        .temperatura = 0.0f,
        .nivel_pct = 0,
        .hum_estado = "esperando lectura",
        .temp_estado = "esperando lectura",
        .lux = -1.0f,
        .lux_estado = "esperando lectura",
        .valvulas_on = false
    };

    adc_init();
    valvula_init();

    err = i2c_master_init();
    if (err != ESP_OK) {
        printf("I2C init failed: %d\n", err);
        return;
    }

    err = bh1750_init();
    if (err != ESP_OK) {
        printf("BH1750 init failed: %d\n", err);
    }

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(report_task, "report_task", 4096, NULL, 5, NULL);
}
