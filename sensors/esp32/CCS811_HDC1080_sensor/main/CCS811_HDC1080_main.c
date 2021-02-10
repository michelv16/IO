#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

static const char *TAG = "CCS811_HDC1080_sensor";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define CCS811_SENSOR_ADDR CONFIG_I2C_SLAVE_ADDRESS_CCS811   /*!< slave address for CCS811 sensor */
#define CCS811_RES_ADDR 0x02
#define CCS811_APP_START_ADDR 0xF4
#define CCS811_MEAS_MODE_ADDR 0x01
#define CSS811_1S_RES 0x10  // one sec resolution
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

SemaphoreHandle_t print_mux = NULL;

static esp_err_t i2c_configure_cs811(i2c_port_t i2c_num)
{
    int ret;
    i2c_cmd_handle_t cmd;

    /* set cs811 chip to running */
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CCS811_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, CCS811_APP_START_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) 
    {
        return ret;
    }

    vTaskDelay(30 / portTICK_RATE_MS);

    /* set cs811 chip scantime to 1s */
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CCS811_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, CCS811_MEAS_MODE_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, CSS811_1S_RES, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) 
    {
        return ret;
    }

    return ret;
}

static esp_err_t i2c_read_CCS811(i2c_port_t i2c_num, uint8_t *data_co2_h, uint8_t *data_co2_l, uint8_t *data_tvoc_h, uint8_t *data_tvoc_l)
{
    int ret;
    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CCS811_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, CCS811_RES_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) 
    {
        return ret;
    }

    vTaskDelay(30 / portTICK_RATE_MS);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CCS811_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_co2_h, ACK_VAL);
    i2c_master_read_byte(cmd, data_co2_l, ACK_VAL);
    i2c_master_read_byte(cmd, data_tvoc_h, ACK_VAL);
    i2c_master_read_byte(cmd, data_tvoc_l, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void i2c_read_CCS811_task(void *arg)
{
    int ret;
    uint8_t data_co2_h, data_co2_l, data_tvoc_h, data_tvoc_l;

    ret = i2c_configure_cs811(I2C_MASTER_NUM);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Configuring CS811 not OK!, error: %s",  esp_err_to_name(ret)); 
    }

    while (1) 
    {
        data_co2_h = 0, data_co2_l = 0, data_tvoc_h = 0, data_tvoc_l = 0;
        ret = i2c_read_CCS811(I2C_MASTER_NUM, &data_co2_h, &data_co2_l, &data_tvoc_h, &data_tvoc_l);

        xSemaphoreTake(print_mux, portMAX_DELAY);
        if (ret == ESP_ERR_TIMEOUT) 
        {
            ESP_LOGE(TAG, "I2C Timeout");
        } 
        else if (ret == ESP_OK) 
        {
            printf("co2_h: 0x%02x, co2_l: 0x%02x\n", data_co2_h, data_co2_l);
            printf("tvoc_h: 0x%02x, tvoc_l: 0x%02x\n", data_tvoc_h, data_tvoc_l);
            printf("co2: %.02d [ppm], \n", (data_co2_h << 8 | data_co2_l));
            printf("tvoc: %.02d [ppb]\n", (data_tvoc_h << 8 | data_tvoc_l));
        } else 
        {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }

        xSemaphoreGive(print_mux);
        vTaskDelay(5000 / portTICK_RATE_MS);
    }
    vSemaphoreDelete(print_mux);
    vTaskDelete(NULL);
}

void app_main(void)
{
    print_mux = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(i2c_read_CCS811_task, "i2c_read_CCS811_task", 1024 * 2, (void *)0, 10, NULL);
}
