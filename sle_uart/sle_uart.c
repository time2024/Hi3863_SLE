/**
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * Description: SLE UART Sample Source. \n
 *
 * History: \n
 * 2023-07-17, Create file. \n
 */
#include "cmsis_os2.h"
#include "gpio.h"
#include "osal_debug.h"
#include "spi.h"
#include "stdio.h"

#include "./GUI/GUI_Paint.h"
#include "epaper.h"

#define SPI_WAIT_CYCLES 0x10

#define SPI_TASK_STACK_SIZE 0x4000
#define SPI_TASK_DURATION_MS 1000
#define SPI_TASK_PRIO (osPriority_t)(17)

#include "common_def.h"
#include "soc_osal.h"
#include "app_init.h"
#include "pinctrl.h"
#include "uart.h"
// #include "pm_clock.h"
#include "sle_low_latency.h"
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)
#include "securec.h"
#include "sle_uart_server.h"
#include "sle_uart_server_adv.h"
#include "sle_device_discovery.h"
#include "sle_errcode.h"
#elif defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT)
#define SLE_UART_TASK_STACK_SIZE 0x600
#include "sle_connection_manager.h"
#include "sle_ssap_client.h"
#include "sle_uart_client.h"
#endif /* CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT */

#define SLE_UART_TASK_PRIO 28
#define SLE_UART_TASK_DURATION_MS 2000
#define SLE_UART_BAUDRATE 115200
#define SLE_UART_TRANSFER_SIZE 512

static void
app_spi_init_pin(void)
{
    uapi_pin_set_mode(CONFIG_EPD_RST_PIN, PIN_MODE_0);
    uapi_gpio_set_dir(CONFIG_EPD_RST_PIN, GPIO_DIRECTION_OUTPUT);

    uapi_pin_set_mode(CONFIG_EPD_DC_PIN, PIN_MODE_0);
    uapi_gpio_set_dir(CONFIG_EPD_DC_PIN, GPIO_DIRECTION_OUTPUT);

    uapi_pin_set_mode(CONFIG_EPD_CS_PIN, PIN_MODE_0);
    uapi_gpio_set_dir(CONFIG_EPD_CS_PIN, GPIO_DIRECTION_OUTPUT);

    uapi_pin_set_mode(CONFIG_EPD_PWR_PIN, PIN_MODE_3);
    uapi_gpio_set_dir(CONFIG_EPD_PWR_PIN, GPIO_DIRECTION_OUTPUT);

    uapi_pin_set_mode(CONFIG_EPD_BUSY_PIN, PIN_MODE_0);
    uapi_gpio_set_dir(CONFIG_EPD_BUSY_PIN, GPIO_DIRECTION_INPUT);

    uapi_pin_set_mode(CONFIG_EPD_DIN_PIN, PIN_MODE_3);
    uapi_pin_set_mode(CONFIG_EPD_CLK_PIN, PIN_MODE_3);
}
static void app_spi_master_init_config(void)
{
    spi_attr_t config = {0};
    spi_extra_attr_t ext_config = {0};

    config.is_slave = false;
    config.slave_num = 1;
    config.bus_clk = 2000000;
    config.freq_mhz = 2;
    config.clk_polarity = SPI_CFG_CLK_CPOL_0;
    config.clk_phase = SPI_CFG_CLK_CPHA_0;
    config.frame_format = SPI_CFG_FRAME_FORMAT_MOTOROLA_SPI;
    config.spi_frame_format = HAL_SPI_FRAME_FORMAT_STANDARD;
    config.frame_size = HAL_SPI_FRAME_SIZE_8;
    config.tmod = HAL_SPI_TRANS_MODE_TXRX;
    config.sste = SPI_CFG_SSTE_DISABLE;

    ext_config.qspi_param.inst_len = HAL_SPI_INST_LEN_8;
    ext_config.qspi_param.addr_len = HAL_SPI_ADDR_LEN_16;
    ext_config.qspi_param.wait_cycles = SPI_WAIT_CYCLES;
    uapi_spi_init(CONFIG_EPD_MASTER_BUS_ID, &config, &ext_config);
}

static void Sample_Enter(void)
{
    uapi_gpio_set_val(CONFIG_EPD_DC_PIN, GPIO_LEVEL_LOW);
    uapi_gpio_set_val(CONFIG_EPD_CS_PIN, GPIO_LEVEL_LOW);
    uapi_gpio_set_val(CONFIG_EPD_PWR_PIN, GPIO_LEVEL_HIGH);
    uapi_gpio_set_val(CONFIG_EPD_RST_PIN, GPIO_LEVEL_HIGH);
}

static void Sample_Exit(void)
{
    uapi_gpio_set_val(CONFIG_EPD_DC_PIN, GPIO_LEVEL_LOW);
    uapi_gpio_set_val(CONFIG_EPD_CS_PIN, GPIO_LEVEL_LOW);
    uapi_gpio_set_val(CONFIG_EPD_PWR_PIN, GPIO_LEVEL_LOW);
    uapi_gpio_set_val(CONFIG_EPD_RST_PIN, GPIO_LEVEL_LOW);

    uapi_spi_deinit(CONFIG_EPD_MASTER_BUS_ID); // 不使用SPI清除
}

static uint8_t g_app_uart_rx_buff[SLE_UART_TRANSFER_SIZE] = {0};

static uart_buffer_config_t g_app_uart_buffer_config = {
    .rx_buffer = g_app_uart_rx_buff,
    .rx_buffer_size = SLE_UART_TRANSFER_SIZE};

static void uart_init_pin(void)
{
    if (CONFIG_SLE_UART_BUS == 0)
    {
        uapi_pin_set_mode(CONFIG_UART_TXD_PIN, PIN_MODE_1);
        uapi_pin_set_mode(CONFIG_UART_RXD_PIN, PIN_MODE_1);
    }
    else if (CONFIG_SLE_UART_BUS == 1)
    {
        uapi_pin_set_mode(CONFIG_UART_TXD_PIN, PIN_MODE_1);
        uapi_pin_set_mode(CONFIG_UART_RXD_PIN, PIN_MODE_1);
    }
}

static void uart_init_config(void)
{
    uart_attr_t attr = {
        .baud_rate = SLE_UART_BAUDRATE,
        .data_bits = UART_DATA_BIT_8,
        .stop_bits = UART_STOP_BIT_1,
        .parity = UART_PARITY_NONE};

    uart_pin_config_t pin_config = {
        .tx_pin = CONFIG_UART_TXD_PIN,
        .rx_pin = CONFIG_UART_RXD_PIN,
        .cts_pin = PIN_NONE,
        .rts_pin = PIN_NONE};
    uapi_uart_deinit(CONFIG_SLE_UART_BUS);
    uapi_uart_init(CONFIG_SLE_UART_BUS, &pin_config, &attr, NULL, &g_app_uart_buffer_config);
}

// Base64解码表
const unsigned char base64_decode_table[256] = {
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // 0-15
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // 16-31
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 62, 255, 255, 255, 63,   // 32-47
    52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 255, 255, 255, 255, 255, 255,           // 48-63
    255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,                          // 64-79
    15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 255, 255, 255, 255, 255,            // 80-95
    255, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,                // 96-111
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 255, 255, 255, 255, 255,            // 112-127
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // 128-143
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // 144-159
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // 160-175
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // 176-191
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // 192-207
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // 208-223
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // 224-239
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255  // 240-255
};

// Base64解码函数
static uint16_t base64_decode(const char *encoded_data, unsigned char *decoded_data)
{
    uint16_t i = 0, j = 0;
    unsigned char char_array_4[4], char_array_3[3];
    uint16_t decoded_length = 0;

    while (encoded_data[i] != '\0')
    {
        // 跳过非Base64字符
        if (encoded_data[i] == '=' || base64_decode_table[(unsigned char)encoded_data[i]] == 255)
        {
            i++;
            continue;
        }

        char_array_4[j++] = encoded_data[i++];
        if (j == 4)
        {
            for (uint16_t k = 0; k < 4; k++)
            {
                char_array_4[k] = base64_decode_table[(unsigned char)char_array_4[k]];
            }

            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0x0f) << 4) + ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x03) << 6) + char_array_4[3];

            for (uint16_t k = 0; k < 3; k++)
            {
                decoded_data[decoded_length++] = char_array_3[k];
            }
            j = 0;
        }
    }

    if (j)
    {
        for (uint16_t k = 0; k < j; k++)
        {
            char_array_4[k] = base64_decode_table[(unsigned char)char_array_4[k]];
        }

        char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0x0f) << 4) + ((char_array_4[2] & 0x3c) >> 2);

        for (uint16_t k = 0; k < j - 1; k++)
        {
            decoded_data[decoded_length++] = char_array_3[k];
        }
    }

    return decoded_length;
}

#if defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)
#define SLE_UART_SERVER_DELAY_COUNT 5

#define SLE_UART_TASK_STACK_SIZE 0x1200
#define SLE_ADV_HANDLE_DEFAULT 1
#define SLE_UART_SERVER_MSG_QUEUE_LEN 5
#define SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE 32
#define SLE_UART_SERVER_QUEUE_DELAY 0xFFFFFFFF
#define SLE_UART_SERVER_BUFF_MAX_SIZE 800

unsigned long g_sle_uart_server_msgqueue_id;

#define SLE_UART_SERVER_LOG "[sle uart server]"

bool is_first_bag = true; // 是否是第一个包
bool is_base64 = false;   // 是否是图片

char room[28800] = {0};
uint16_t number = 0;

static void ssaps_server_read_request_cbk(uint8_t server_id, uint16_t conn_id, ssaps_req_read_cb_t *read_cb_para,
                                          errcode_t status)
{
    osal_printk("%s ssaps read request cbk callback server_id:%x, conn_id:%x, handle:%x, status:%x\r\n",
                SLE_UART_SERVER_LOG, server_id, conn_id, read_cb_para->handle, status);
}
static void ssaps_server_write_request_cbk(uint8_t server_id, uint16_t conn_id, ssaps_req_write_cb_t *write_cb_para,
                                           errcode_t status)
{
    osal_printk("%s ssaps write request callback cbk server_id:%x, conn_id:%x, handle:%x, status:%x\r\n",
                SLE_UART_SERVER_LOG, server_id, conn_id, write_cb_para->handle, status);
    osal_printk("\n sle uart recived data : %s\r\n", write_cb_para->value);
    uapi_uart_write(CONFIG_SLE_UART_BUS, (uint8_t *)write_cb_para->value, write_cb_para->length, 0);

    if ((write_cb_para->length > 0) && write_cb_para->value) // 如果有接收到数据
    {
        char *received_str = (char *)malloc(write_cb_para->length + 1); // 为终止符预留空间
        if (received_str != NULL)
        {
            memcpy(received_str, write_cb_para->value, write_cb_para->length);
            received_str[write_cb_para->length] = '\0'; // 确保字符串以null字符结尾
        }
        else
        {
            // 处理内存分配失败的情况
            return;
        }

        // 如果收到了终止标志###，则对接收到的数据进行处理
        if (write_cb_para->length == 3 && received_str[0] == '#' && received_str[1] == '#' && received_str[2] == '#')
        {
            if (is_first_bag) // 如果是第一包则不处理
            {
                // 空着，不处理
            }
            else // 对接收到的数据进行处理
            {
                if (is_base64) // 如果是图片
                {
                    app_spi_init_pin();
                    app_spi_master_init_config();

                    Sample_Enter();

                    EPD_Init();
                    EPD_display_NUM(EPD_WHITE);
                    EPD_lut_GC();
                    EPD_refresh();

                    EPD_SendCommand(0x50);
                    EPD_SendData(0x17);

                    osal_mdelay(500);

                    uint8_t *BlackImage;
                    uint16_t Imagesize = ((EPD_WIDTH % 8 == 0) ? (EPD_WIDTH / 8) : (EPD_WIDTH / 8 + 1)) * EPD_HEIGHT;
                    if ((BlackImage = (uint8_t *)malloc(Imagesize)) == NULL)
                    {
                        printf("Failed to apply for black memory...\r\n");
                    }
                    else
                    {
                        printf("Paint_NewImage\r\n");
                        Paint_NewImage(BlackImage, EPD_WIDTH, EPD_HEIGHT, 270, WHITE);
                        Paint_Clear(WHITE);
                        printf("SelectImage:BlackImage\r\n");
                        Paint_SelectImage(BlackImage);
                        Paint_Clear(WHITE);

                        printf("Drawing:BlackImage\r\n");
                        // uint16_t length = write_cb_para->length - 6;
                        //  计算解码后的二进制数据长度
                        room[number] = '\0';
                        printf("%d\r\n",number);
                        // 分配内存存储解码后的二进制数据
                        unsigned char *decoded_data = (unsigned char *)malloc(14400);
                        if (!decoded_data)
                        {
                            //如果没分配到空间，则不处理
                            //printf("1\r\n");
                        }
                        else
                        {
                            // 进行Base64解码
                            uint16_t actual_decoded_length = base64_decode(room, decoded_data);
                            Paint_DrawBitMap(decoded_data);
                            EPD_display(BlackImage);
                            EPD_lut_GC();
                            EPD_refresh();
                            osal_mdelay(2000);
                            printf("Clear...\r\n");
                            EPD_Clear();
                            printf("Goto Sleep...\r\n");
                            EPD_sleep();

                            free(BlackImage);
                            BlackImage = NULL;

                            osal_mdelay(2000); // important, at least 2s
                            printf("close 5V, Module enters 0 power consumption ...\r\n");

                            Sample_Exit();
                        }
                    }
                }
                else // 如果是文本
                {
                    app_spi_init_pin();
                    app_spi_master_init_config();

                    Sample_Enter();

                    EPD_Init();
                    EPD_display_NUM(EPD_WHITE);
                    EPD_lut_GC();
                    EPD_refresh();

                    EPD_SendCommand(0x50);
                    EPD_SendData(0x17);

                    osal_mdelay(500);

                    uint8_t *BlackImage;
                    uint16_t Imagesize = ((EPD_WIDTH % 8 == 0) ? (EPD_WIDTH / 8) : (EPD_WIDTH / 8 + 1)) * EPD_HEIGHT;
                    if ((BlackImage = (uint8_t *)malloc(Imagesize)) == NULL)
                    {
                        printf("Failed to apply for black memory...\r\n");
                    }
                    else
                    {
                        printf("Paint_NewImage\r\n");
                        Paint_NewImage(BlackImage, EPD_WIDTH, EPD_HEIGHT, 270, WHITE);
                        Paint_Clear(WHITE);
                        printf("SelectImage:BlackImage\r\n");
                        Paint_SelectImage(BlackImage);
                        Paint_Clear(WHITE);

                        printf("Drawing:BlackImage\r\n");
                        // const char *received_str = (const char *)write_cb_para->value;
                        room[number] = '\0'; // 加上结束标志
                        //注意，最多只能显示255个字符所以还需要修改
                        Paint_DrawString_EN(10, 0, room, &Font16, BLACK, WHITE);
                        printf("EPD_Display\r\n");
                        EPD_display(BlackImage);
                        EPD_lut_GC();
                        EPD_refresh();
                        osal_mdelay(2000);
                        printf("Clear...\r\n");
                        EPD_Clear();
                        printf("Goto Sleep...\r\n");
                        EPD_sleep();

                        free(BlackImage);
                        BlackImage = NULL;

                        osal_mdelay(2000); // important, at least 2s
                        printf("close 5V, Module enters 0 power consumption ...\r\n");

                        Sample_Exit();
                    }
                }
                number = 0; // 清空存储空间
                is_first_bag = true;
            }
        }
        // 否则，继续接收数据
        else
        {
            if (is_first_bag) // 如果是第一包，判断是文本还是图片
            {
                if (received_str[0] == 'B' && received_str[1] == 'a' && received_str[2] == 's' && received_str[3] == 'e' && received_str[4] == '6' && received_str[5] == '4')
                {
                    // 如果是图片，则：
                    is_base64 = true;
                    is_first_bag = false;
                    received_str += 6; // 摒弃前六个标识符
                    for (int i = 0; i < write_cb_para->length - 6; i++)
                    {
                        room[number] = received_str[i];
                        number++;
                    }
                }
                else
                {
                    // 如果是文本，则：
                    is_base64 = false;
                    is_first_bag = false;
                    received_str += 6; // 摒弃前六个标识符
                    for (int i = 0; i < write_cb_para->length - 6; i++)
                    {
                        room[number] = received_str[i];
                        number++;
                    }
                }
            }
            else
            {
                // 如果不是第一包
                for (int i = 0; i < write_cb_para->length ; i++)
                {
                    room[number] = received_str[i];
                    number++;
                }
            }
        }
    }
}

static void sle_uart_server_read_int_handler(const void *buffer, uint16_t length, bool error)
{
    unused(error);
    if (sle_uart_client_is_connected())
    {
        sle_uart_server_send_report_by_handle(buffer, length);
    }
    else
    {
        osal_printk("%s sle client is not connected! \r\n", SLE_UART_SERVER_LOG);
    }
}

static void sle_uart_server_create_msgqueue(void)
{
    if (osal_msg_queue_create("sle_uart_server_msgqueue", SLE_UART_SERVER_MSG_QUEUE_LEN,
                              (unsigned long *)&g_sle_uart_server_msgqueue_id, 0, SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE) != OSAL_SUCCESS)
    {
        osal_printk("^%s sle_uart_server_create_msgqueue message queue create failed!\n", SLE_UART_SERVER_LOG);
    }
}

static void sle_uart_server_delete_msgqueue(void)
{
    osal_msg_queue_delete(g_sle_uart_server_msgqueue_id);
}

static void sle_uart_server_write_msgqueue(uint8_t *buffer_addr, uint16_t buffer_size)
{
    osal_msg_queue_write_copy(g_sle_uart_server_msgqueue_id, (void *)buffer_addr,
                              (uint32_t)buffer_size, 0);
}

static int32_t sle_uart_server_receive_msgqueue(uint8_t *buffer_addr, uint32_t *buffer_size)
{
    return osal_msg_queue_read_copy(g_sle_uart_server_msgqueue_id, (void *)buffer_addr,
                                    buffer_size, SLE_UART_SERVER_QUEUE_DELAY);
}
static void sle_uart_server_rx_buf_init(uint8_t *buffer_addr, uint32_t *buffer_size)
{
    *buffer_size = SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE;
    (void)memset_s(buffer_addr, *buffer_size, 0, *buffer_size);
}

static void *sle_uart_server_task(const char *arg)
{
    unused(arg);
    uint8_t rx_buf[SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE] = {0};
    uint32_t rx_length = SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE;
    uint8_t sle_connect_state[] = "sle_dis_connect";

    sle_uart_server_create_msgqueue();
    sle_uart_server_register_msg(sle_uart_server_write_msgqueue);
    sle_uart_server_init(ssaps_server_read_request_cbk, ssaps_server_write_request_cbk);

    /* UART pinmux. */
    uart_init_pin();

    /* UART init config. */
    uart_init_config();

    uapi_uart_unregister_rx_callback(CONFIG_SLE_UART_BUS);
    errcode_t ret = uapi_uart_register_rx_callback(CONFIG_SLE_UART_BUS,
                                                   UART_RX_CONDITION_FULL_OR_IDLE,
                                                   1, sle_uart_server_read_int_handler);
    if (ret != ERRCODE_SUCC)
    {
        osal_printk("%s Register uart callback fail.[%x]\r\n", SLE_UART_SERVER_LOG, ret);
        return NULL;
    }
    while (1)
    {
        sle_uart_server_rx_buf_init(rx_buf, &rx_length);
        sle_uart_server_receive_msgqueue(rx_buf, &rx_length);
        if (strncmp((const char *)rx_buf, (const char *)sle_connect_state, sizeof(sle_connect_state)) == 0)
        {
            ret = sle_start_announce(SLE_ADV_HANDLE_DEFAULT);
            if (ret != ERRCODE_SLE_SUCCESS)
            {
                osal_printk("%s sle_connect_state_changed_cbk,sle_start_announce fail :%02x\r\n",
                            SLE_UART_SERVER_LOG, ret);
            }
        }
        osal_msleep(SLE_UART_TASK_DURATION_MS);
    }
    sle_uart_server_delete_msgqueue();
    return NULL;
}
#elif defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT)

void sle_uart_notification_cb(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data,
                              errcode_t status)
{
    unused(client_id);
    unused(conn_id);
    unused(status);
    osal_printk("\n sle uart recived data : %s\r\n", data->data);
    uapi_uart_write(CONFIG_SLE_UART_BUS, (uint8_t *)(data->data), data->data_len, 0);
}

void sle_uart_indication_cb(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data,
                            errcode_t status)
{
    unused(client_id);
    unused(conn_id);
    unused(status);
    osal_printk("\n sle uart recived data : %s\r\n", data->data);
    uapi_uart_write(CONFIG_SLE_UART_BUS, (uint8_t *)(data->data), data->data_len, 0);
}

static void sle_uart_client_read_int_handler(const void *buffer, uint16_t length, bool error)
{
    unused(error);

    // 获取发送参数和连接ID
    ssapc_write_param_t *sle_uart_send_param = get_g_sle_uart_send_param();
    uint16_t g_sle_uart_conn_id = get_g_sle_uart_conn_id();

    // 定义分包大小
    const uint16_t PACKET_SIZE = 100;                  // 每包最大100字节
    uint16_t remaining_length = length;                // 剩余未发送的数据长度
    const uint8_t *data_ptr = (const uint8_t *)buffer; // 指向数据的指针

    // 分包发送
    while (remaining_length > 0)
    {
        // 计算当前包的长度
        uint16_t current_packet_length = (remaining_length > PACKET_SIZE) ? PACKET_SIZE : remaining_length;

        // 设置发送参数
        sle_uart_send_param->data_len = current_packet_length;
        sle_uart_send_param->data = (uint8_t *)data_ptr;

        // 发送当前包
        ssapc_write_req(0, g_sle_uart_conn_id, sle_uart_send_param);

        // 更新剩余长度和数据指针
        remaining_length -= current_packet_length;
        data_ptr += current_packet_length;

        // 可选：添加延时或等待发送完成
        osal_msleep(30); // 例如延时10ms
    }

    // 分包内容发送完毕标志
    // const char *end_marker = "###";                              // 标记字符串
    // sle_uart_send_param->data_len = 3;                           // 标记字符串的长度
    // sle_uart_send_param->data = (uint8_t *)end_marker;           // 将标记字符串赋值给发送参数
    // ssapc_write_req(0, g_sle_uart_conn_id, sle_uart_send_param); // 发送标记字符串
    // osal_msleep(20);                                              // 延时5ms

    //转为手动发送###为结束标记
}

static void *sle_uart_client_task(const char *arg)
{
    unused(arg);
    /* UART pinmux. */
    uart_init_pin();

    /* UART init config. */
    uart_init_config();

    uapi_uart_unregister_rx_callback(CONFIG_SLE_UART_BUS);
    errcode_t ret = uapi_uart_register_rx_callback(CONFIG_SLE_UART_BUS,
                                                   UART_RX_CONDITION_FULL_OR_IDLE,
                                                   1, sle_uart_client_read_int_handler);
    sle_uart_client_init(sle_uart_notification_cb, sle_uart_indication_cb);

    if (ret != ERRCODE_SUCC)
    {
        osal_printk("Register uart callback fail.");
        return NULL;
    }

    return NULL;
}
#endif /* CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT */

static void sle_uart_entry(void)
{
    osal_task *task_handle = NULL;
    osal_kthread_lock();
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)
    task_handle = osal_kthread_create((osal_kthread_handler)sle_uart_server_task, 0, "SLEUartServerTask",
                                      SLE_UART_TASK_STACK_SIZE);
#elif defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT)
    task_handle = osal_kthread_create((osal_kthread_handler)sle_uart_client_task, 0, "SLEUartDongleTask",
                                      SLE_UART_TASK_STACK_SIZE);
#endif /* CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT */
    if (task_handle != NULL)
    {
        osal_kthread_set_priority(task_handle, SLE_UART_TASK_PRIO);
    }
    osal_kthread_unlock();
}

/* Run the sle_uart_entry. */
app_run(sle_uart_entry);