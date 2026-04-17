#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "delay.h"
#include "timer.h"
#include "radio.h"
#include "tremo_system.h"
#include "tremo_gpio.h"
#include "lora_net.h"
#include "lora_driver.h"
#include "address_store.h"
#ifdef CONFIG_GATEWAY
#include "address_manager.h"
#endif
#include "sht_sensor.h"
#include "adc_monitor.h"
#include "lora_protocol.h"

#define FW_DEBUG_VERSION "join-debug-v20260416-1"

/* checksum helper moved to lora_protocol.h (lora_simple_checksum) */

/* Blink state (node only) */
#ifndef CONFIG_GATEWAY
static int blink_ms_remaining = 0;
static int blink_toggle_elapsed = 0;
static int blink_toggle_interval = 250; /* ms */
static int blink_led_state = 0;
static TimerTime_t node_last_report_at = 0;
static bool node_report_due_immediately = false;
#endif

uint8_t SLAVE1_ADDR = 0x00; // 本地地址默认为0
uint8_t Addr_Num = 30;

#if defined(REGION_AS923)

#define RF_FREQUENCY 923000000 // Hz

#elif defined(REGION_AU915)

#define RF_FREQUENCY 915000000 // Hz

#elif defined(REGION_CN470)

#define RF_FREQUENCY 470000000 // Hz

#elif defined(REGION_CN779)

#define RF_FREQUENCY 779000000 // Hz

#elif defined(REGION_EU433)

#define RF_FREQUENCY 433000000 // Hz

#elif defined(REGION_EU868)

#define RF_FREQUENCY 868000000 // Hz

#elif defined(REGION_KR920)

#define RF_FREQUENCY 920000000 // Hz

#elif defined(REGION_IN865)

#define RF_FREQUENCY 865000000 // Hz

#elif defined(REGION_US915)

#define RF_FREQUENCY 915000000 // Hz

#elif defined(REGION_US915_HYBRID)

#define RF_FREQUENCY 915000000 // Hz

#else
#error "Please define a frequency band in the compiler options."
#endif

#define TX_OUTPUT_POWER 14 // dBm

#if defined(USE_MODEM_LORA)

#define LORA_BANDWIDTH 0        // [0: 125 kHz,
                                //  1: 250 kHz,
                                //  2: 500 kHz,
                                //  3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1       // [1: 4/5,
                                //  2: 4/6,
                                //  3: 4/7,
                                //  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#elif defined(USE_MODEM_FSK)

#define FSK_FDEV 25000            // Hz
#define FSK_DATARATE 50000        // bps
#define FSK_BANDWIDTH 1000000     // Hz >> DSB in sx126x
#define FSK_AFC_BANDWIDTH 1000000 // Hz
#define FSK_PREAMBLE_LENGTH 5     // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON false

#else
#error "Please define a modem in the compiler options."
#endif

typedef enum
{
    LORA_IDLE,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT
} States_t;

#define RX_TIMEOUT_VALUE 1800
#define BUFFER_SIZE 256 // Define the payload size here

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];
char ACKBuf[] = "11111111111122222222222222222333333333333333333333334444444444444444444445555555555555555555555556666666666666666666666677777777777777777777777777777888888888888888888889999999999999990000000000000000"; // 发送的数据

volatile States_t State = LORA_IDLE;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

uint32_t ChipId[2] = {0};
uint8_t sendMsgFlag = 2; // 0：空闲状态可以发送数�?�?1：�?�在发送，等待发送完成；2：发送完�?

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void);

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError(void);

static void get_unique_id_bytes(uint8_t *uid);

#ifndef CONFIG_GATEWAY
static bool node_handle_rx_packet(void)
{
    /* Strict legacy set-address packet: A0 <addr> 00 00 00 00 A1 */
    bool is_legacy_set_addr =
        (BufferSize == 7) &&
        (Buffer[0] == 0xA0) &&
        (Buffer[2] == 0x00) &&
        (Buffer[3] == 0x00) &&
        (Buffer[4] == 0x00) &&
        (Buffer[5] == 0x00) &&
        (Buffer[6] == 0xA1);

    if (BufferSize >= 8 && Buffer[0] == 0xA0 && Buffer[1] == 0x11 && Buffer[BufferSize - 1] == 0xA1)
    {
        uint8_t uid[4];
        get_unique_id_bytes(uid);
        printf("[JOIN] got ASSIGN packet, checking UID...\r\n");
        if (memcmp(uid, &Buffer[2], 4) == 0)
        {
            uint8_t new_addr = Buffer[6];
            SLAVE1_ADDR = new_addr;
            printf("[JOIN] assigned address in RAM: %d\r\n", SLAVE1_ADDR);
            printf("[JOIN] skip flash store during join (avoid reset risk)\r\n");
            {
                uint8_t ack[4] = {0xA0, 0x12, SLAVE1_ADDR, 0xA1};
                printf("[JOIN] send ASSIGN ACK for addr %d\r\n", SLAVE1_ADDR);
                Radio.Send(ack, sizeof(ack));
            }
            printf("[JOIN] assigned address %d\r\n", SLAVE1_ADDR);
            return true;
        }
        printf("[JOIN] ASSIGN packet UID mismatch\r\n");
        return false;
    }

    if (is_legacy_set_addr)
    {
        uint8_t new_addr = Buffer[1];
        SLAVE1_ADDR = new_addr;
        printf("[JOIN] legacy assigned address in RAM: %d\r\n", SLAVE1_ADDR);
        printf("[JOIN] skip flash store for legacy assign (avoid reset risk)\r\n");
        {
            uint8_t ack[4] = {0xA0, 0x1E, SLAVE1_ADDR, 0xA1};
            printf("[JOIN] send legacy ACK for addr %d\r\n", SLAVE1_ADDR);
            Radio.Send(ack, sizeof(ack));
        }
        return true;
    }

    return false;
}
#endif

/**
 * 功能：发送数�?�?
 * 参数�?
 *       buffer:数据包地址
 *       len:数据包长�?
 * 返回值：None
 */
void transmitPackets(unsigned char *sendBuf, unsigned char len)
{

    for (int i = 0; i < len; i++)
    {
        printf("[] transmitPackets Buffer %d-%d\r\n", i, sendBuf[i]);
    }

    printf("State: %d ,lenth %d ,Node send message \r\n", State, len);

#ifdef CONFIG_GATEWAY

#else
    delay_ms(500);
#endif
    Radio.Send((uint8_t *)sendBuf, len);
}

uint8_t rx_data[8] = {0};
uint8_t rx_index = 0;
/* AT command buffer for ASCII commands on UART */
static char at_cmd_buf[64];
static uint8_t at_cmd_idx = 0;

int serial_output(uint8_t *buffer, int len)
{

    for (int i = 0; i < len; i++)
    {
        uart_send_data(UART0, buffer[i]);
    }
    return 0;
}

void handler_uart_data(uint8_t data)
{
    /* ASCII AT command accumulation (for gateway) */
    if ((data >= 0x20 && data <= 0x7E) || data == '\r' || data == '\n') {
        if (at_cmd_idx < sizeof(at_cmd_buf) - 1) {
            at_cmd_buf[at_cmd_idx++] = (char)data;
        }
        /* On newline, try to parse AT command */
        if (data == '\n' || data == '\r') {
            if (at_cmd_idx > 0) {
                at_cmd_buf[at_cmd_idx] = '\0';
                /* parse AT+FIND on gateway only */
#ifdef CONFIG_GATEWAY
                if ((strncmp(at_cmd_buf, "AT+FIND=", 8) == 0) || (strncmp(at_cmd_buf, "at+find=", 8) == 0)) {
                    /* parse address and optional duration */
                    int addr = 0;
                    int duration = 5; /* default */
                    char *p = at_cmd_buf + 8;
                    if (p) {
                        addr = atoi(p);
                        char *comma = strchr(p, ',');
                        if (comma) {
                            duration = atoi(comma + 1);
                        }
                    }
                    if (addr < 0 || addr > 255) addr = 0xFF;
                    if (duration <= 0) duration = 5;
                    if (duration > 60) duration = 60; /* clamp */

                    /* build FIND packet: A0 OPCODE_FIND addr duration checksum A1 */
                    uint8_t pkt[6];
                    pkt[0] = 0xA0;
                    pkt[1] = OPCODE_FIND;
                    pkt[2] = (uint8_t)addr;
                    pkt[3] = (uint8_t)duration;
                    pkt[4] = (uint8_t)((pkt[1] + pkt[2] + pkt[3]) & 0xFF);
                    pkt[5] = 0xA1;
                    printf("AT+FIND -> send FIND pkt: addr=%d dur=%d\r\n", addr, duration);
                    Radio.Send(pkt, sizeof(pkt));
                }
#endif
                /* reset buffer */
                at_cmd_idx = 0;
                at_cmd_buf[0] = '\0';
            }
        }
        /* If this byte was printable ASCII, don't feed into binary parser below */
        if (data != 0xA0) {
            return;
        }
    }

    int isOK = 0;
    rx_data[rx_index++] = data;
    // A0 F1 C0 03 11 22 28 A1
    if (rx_index > 5 && rx_data[0] == 0xA0 && rx_data[rx_index - 1] == 0xA1)
    {
#ifdef CONFIG_GATEWAY
        // rx_data[rx_index - 1] = '\0';
        // printf("%s", rx_data);

        uint8_t checkData = 0;

        for (int i = 0; i < rx_index; i++)
        {
            checkData = rx_data[i] + checkData;
        }

        checkData = checkData - rx_data[rx_index - 2];
        if (checkData == rx_data[rx_index - 2])
        {
            // printf("OK Slave:%d , data:%d \n", rx_data[2], rx_data[3]);
            if (rx_data[1] == 0xF2)
            {
                Addr_Num = rx_data[2];
                printf("set polling device num: %d\r\n", Addr_Num);
            }
            else if (rx_data[1] == 0xF1)
            {
                Radio.Send(rx_data, rx_index);
            }
            else
            {
                printf("command ERROR");
            }
            isOK = 1;
            rx_data[0] = '\0';
            rx_index = 0;
        }
        else
        {

            for (int i = 0; i < rx_index; i++)
            {
                printf("%02X ", rx_data[i]);
            }

            printf("-%02X ", checkData);
            rx_data[0] = '\0';
            rx_index = 0;
        }
#else
        SLAVE1_ADDR = rx_data[1];
        printf("set Node addr: %d\r\n", SLAVE1_ADDR);
        if (store_node_address(SLAVE1_ADDR)) {
            printf("Stored node address to flash: %d\r\n", SLAVE1_ADDR);
        } else {
            printf("Failed to store node address to flash\r\n");
        }
#endif
    }
    else if (rx_data[0] != 0xA0)
    {
        rx_data[0] = '\0';
        rx_index = 0;
    }

    if (rx_index > 15)
    {
        rx_data[0] = '\0';
        rx_index = 0;
    }

    if (isOK)
    {
        gpio_write(GPIOA, GPIO_PIN_4, GPIO_LEVEL_LOW);
        gpio_write(GPIOA, GPIO_PIN_5, GPIO_LEVEL_LOW);
        gpio_write(GPIOA, GPIO_PIN_7, GPIO_LEVEL_HIGH);
    }
    else
    {
        gpio_write(GPIOA, GPIO_PIN_4, GPIO_LEVEL_LOW);
        gpio_write(GPIOA, GPIO_PIN_5, GPIO_LEVEL_HIGH);
        gpio_write(GPIOA, GPIO_PIN_7, GPIO_LEVEL_LOW);
    }
}

// Helper: fill 4-byte UID from ChipId (little-endian)
static void get_unique_id_bytes(uint8_t *uid)
{
    if (uid == NULL) return;
    // Use lower 4 bytes of ChipId[0]
    uid[0] = (uint8_t)(ChipId[0] & 0xFF);
    uid[1] = (uint8_t)((ChipId[0] >> 8) & 0xFF);
    uid[2] = (uint8_t)((ChipId[0] >> 16) & 0xFF);
    uid[3] = (uint8_t)((ChipId[0] >> 24) & 0xFF);
}

#ifndef CONFIG_GATEWAY
static void node_send_sensor_report(const char *reason)
{
    float t = 0.0f;
    float rh = 0.0f;
    float vcap = 0.0f;
    printf("[NODE] %s begin sensor read\r\n", reason);
    int ok = sht_sensor_read(&t, &rh);
    vcap = adc_monitor_read_voltage();
    if (vcap < 0.0f) {
        printf("[NODE] %s ADC read timeout, sending Vcap=0\r\n", reason);
        vcap = 0.0f;
    }

    if (!ok) {
        printf("[NODE] %s sensor read failed, sending zeros\r\n", reason);
        t = 0.0f;
        rh = 0.0f;
    } else {
        printf("[NODE] %s sensor read: T=%.2fC RH=%.2f%%\r\n", reason, t, rh);
    }

    int16_t t100 = (int16_t)(t * 100.0f);
    uint16_t rh100 = (uint16_t)(rh * 100.0f);
    uint16_t vcap_mv = (uint16_t)(vcap * 1000.0f);
    uint8_t uid[4];
    uint8_t pkt[13];

    get_unique_id_bytes(uid);
    pkt[0] = 0xA0;
    pkt[1] = 0x21;
    pkt[2] = uid[0];
    pkt[3] = uid[1];
    pkt[4] = uid[2];
    pkt[5] = uid[3];
    pkt[6] = (uint8_t)((t100 >> 8) & 0xFF);
    pkt[7] = (uint8_t)(t100 & 0xFF);
    pkt[8] = (uint8_t)((rh100 >> 8) & 0xFF);
    pkt[9] = (uint8_t)(rh100 & 0xFF);
    pkt[10] = (uint8_t)((vcap_mv >> 8) & 0xFF);
    pkt[11] = (uint8_t)(vcap_mv & 0xFF);
    pkt[12] = 0xA1;

    if (sendMsgFlag == 2) {
        printf("[NODE] %s direct report send uid=%02X%02X%02X%02X T=%.2fC RH=%.2f%% Vcap=%.3fV\r\n",
               reason, uid[0], uid[1], uid[2], uid[3], t100 / 100.0f, rh100 / 100.0f, vcap_mv / 1000.0f);
        sendMsgFlag = 1;
        Radio.Send(pkt, sizeof(pkt));
        node_last_report_at = TimerGetCurrentTime();
    } else {
        printf("[NODE] %s direct report deferred sendMsgFlag=%d state=%d\r\n", reason, sendMsgFlag, State);
    }
}
#endif

int Ra08KitLoraTestStart(void)
{
    static uint8_t ledStatus = 0;
    uint8_t send_buff[8] = {0xA0, 0xF1, 0x01, 0x01, 0x11, 0x22, 0x33, 0xA1};
    DeviceSta_Strcture device = {0};
    DeviceBlock DeviceBlock_Structure;
    DeviceBlock DeviceBlock_StructureArray[2];
    int i = 0;

    printf("Ra-08-kit test Start! \r\n");
    printf("[FW] %s\r\n", FW_DEBUG_VERSION);

    (void)system_get_chip_id(ChipId);

#ifdef CONFIG_GATEWAY
    // initialize in-memory address manager
    init_address_manager();
#endif

    // mode：工作模式，LoRa�?片可工作在FSK模式/LoRa模式�?
    // power：LoRa工作功率
    // fdev：FSK模式下�?�置，不考虑使用
    // bandwidth：带�?-125K�?250K�?500K
    // datarate：扩频因�?
    // coderate�? 编码�?
    // preambleLen：前导码长度
    // fixLen：数�?包长度是否固�?
    // crcOn：是否启动CRC校验
    // freqHopOn：调频使能开�?
    // hopPeriod：若�?动调频，调�?�周期确�?
    // iqInverted：反转IQ信号(笔者也没搞过呀)
    // timeout：�?�置超时时间（一�?�?接收超时时间�?

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);

    // 设置LoRa�?片工作�?�率 #define RF_FREQUENCY 470000000 // Hz 953525
    Radio.SetChannel(433953525);

#if defined(USE_MODEM_LORA)

    printf("Ra-08-kit USE_MODEM_LORA \r\n");

    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    printf("freq: %lu\r\n,Lora Mode\r\nversion:0.0.0\r\ntx power:%d\r\nSF:%d\r\nCR:4/%d\r\n", RF_FREQUENCY, TX_OUTPUT_POWER, LORA_SPREADING_FACTOR, LORA_CODINGRATE + 4);

    switch (LORA_BANDWIDTH)
    {
    case 0:
        printf("BW:125kHz\r\n");
        break;
    case 1:
        printf("BW:250kHz\r\n");
        break;
    case 2:
        printf("BW:500kHz\r\n");
        break;
    default:
        printf("BW:Unknown:%d\r\n", LORA_BANDWIDTH);
        break;
    }
#elif defined(USE_MODEM_FSK)
    printf("Ra-08-kit USE_MODEM_FSK \r\n");

    Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                      FSK_DATARATE, 0,
                      FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, 0, 3000);

    Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                      0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                      0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                      0, 0, false, true);

#else
#error "Please define a frequency band in the compiler options."
#endif

    Radio.Rx(RX_TIMEOUT_VALUE);

#ifndef CONFIG_GATEWAY
    {
        uint8_t stored = 0;
        if (load_node_address(&stored)) {
            SLAVE1_ADDR = stored;
            printf("Loaded node address from flash: %d\r\n", SLAVE1_ADDR);
        } else {
            printf("No stored node address in flash, using default: %d\r\n", SLAVE1_ADDR);
        }

        if (sht_sensor_init()) {
            printf("SHT sensor init OK\r\n");
        } else {
            printf("SHT sensor init FAILED\r\n");
        }

        adc_monitor_init();
        printf("ADC monitor init OK (IO8/PA8)\r\n");

        node_report_due_immediately = true;
        node_last_report_at = TimerGetCurrentTime();
        State = LORA_IDLE;
        Radio.Rx(RX_TIMEOUT_VALUE);
        printf("[NODE] radio initialized, direct sensor reporting enabled\r\n");
    }
#else
    printf("[GW] radio initialized, waiting for sensor reports\r\n");
#endif

    while (1)
    {

#ifdef CONFIG_GATEWAY
        static long wait_log_count = 0;
        wait_log_count++;
        if (wait_log_count > 500000)
        {
            wait_log_count = 0;
            printf("[GW] waiting sensor reports\r\n");
        }
#endif

        switch (State)
        {
        case RX:
            Radio.Rx(RX_TIMEOUT_VALUE);
            State = LORA_IDLE;

#ifdef CONFIG_GATEWAY
            if (Buffer[0] == 0xFF)
            {
                printf("[GW] legacy data from addr=%d\r\n", Buffer[1]);
                printf("data: %s\r\n", (char *)(Buffer + 2));
            }
            else if (BufferSize == 13 && Buffer[0] == 0xA0 && Buffer[1] == 0x21 && Buffer[12] == 0xA1)
            {
                uint8_t uid0 = Buffer[2];
                uint8_t uid1 = Buffer[3];
                uint8_t uid2 = Buffer[4];
                uint8_t uid3 = Buffer[5];
                int16_t t100 = (int16_t)(((uint16_t)Buffer[6] << 8) | Buffer[7]);
                uint16_t rh100 = (uint16_t)(((uint16_t)Buffer[8] << 8) | Buffer[9]);
                uint16_t vcap_mv = (uint16_t)(((uint16_t)Buffer[10] << 8) | Buffer[11]);
                printf("[GW] sensor report uid=%02X%02X%02X%02X T=%.2fC RH=%.2f%% Vcap=%.3fV\r\n",
                       uid0, uid1, uid2, uid3, t100 / 100.0f, rh100 / 100.0f, vcap_mv / 1000.0f);
            }
            else if (BufferSize == 11 && Buffer[0] == 0xA0 && Buffer[1] == 0x21 && Buffer[10] == 0xA1)
            {
                uint8_t uid0 = Buffer[2];
                uint8_t uid1 = Buffer[3];
                uint8_t uid2 = Buffer[4];
                uint8_t uid3 = Buffer[5];
                int16_t t100 = (int16_t)(((uint16_t)Buffer[6] << 8) | Buffer[7]);
                uint16_t rh100 = (uint16_t)(((uint16_t)Buffer[8] << 8) | Buffer[9]);
                printf("[GW] sensor report(legacy) uid=%02X%02X%02X%02X T=%.2fC RH=%.2f%%\r\n",
                       uid0, uid1, uid2, uid3, t100 / 100.0f, rh100 / 100.0f);
            }
            /* Handle FIND ACK from nodes: A0 OPCODE_FIND_ACK <addr> <status> <cs> A1 */
            else if (BufferSize >= 6 && Buffer[0] == 0xA0 && Buffer[1] == OPCODE_FIND_ACK && Buffer[BufferSize - 1] == 0xA1)
            {
                uint8_t addr = Buffer[2];
                uint8_t status = Buffer[3];
                uint8_t cs = Buffer[4];
                if (cs == lora_simple_checksum(Buffer[1], Buffer[2], Buffer[3])) {
                    if (status == 0x01) {
                        printf("Node %d started blinking\r\n", addr);
                    } else if (status == 0x02) {
                        printf("Node %d finished blinking\r\n", addr);
                    } else {
                        printf("Node %d unknown FIND ACK status: %02X\r\n", addr, status);
                    }
                }
            }
            // sendMasterAsk(SLAVE1_ADDR, OP_R_SENSOR, PRAM_R_ALL); //主机发送指�?
            // printf("[%s()-%d] CONFIG_GATEWAY send message:%s\r\n", __func__, __LINE__, sendBuf);
            // printf("GATEWAY ...\r\n");
#else
            {

                // Handle ASSIGN_ADDR from gateway: A0 0x11 UID[4] ADDR A1
                if (node_handle_rx_packet())
                {
                    break;
                }
                /* Handle FIND broadcast: A0 OPCODE_FIND <addr> <duration> <cs> A1 */
                else if (BufferSize >= 6 && Buffer[0] == 0xA0 && Buffer[1] == OPCODE_FIND && Buffer[BufferSize - 1] == 0xA1)
                {
                    uint8_t target = Buffer[2];
                    uint8_t duration = Buffer[3];
                    uint8_t cs = Buffer[4];
                    if (cs == lora_simple_checksum(Buffer[1], Buffer[2], Buffer[3])) {
                        if (target == SLAVE1_ADDR || target == 0xFF) {
                            /* start blink */
                            if (duration == 0) duration = 5;
                            if (duration > 60) duration = 60;
                            blink_ms_remaining = duration * 1000;
                            blink_toggle_elapsed = 0;
                            blink_led_state = 0;
                            printf("FIND packet matched: start blinking for %d s\r\n", duration);
                            /* send start ACK: A0 OPCODE_FIND_ACK <addr> 01 cs A1 */
                            uint8_t ack[6];
                            ack[0] = 0xA0;
                            ack[1] = OPCODE_FIND_ACK;
                            ack[2] = SLAVE1_ADDR;
                            ack[3] = 0x01; /* started */
                            ack[4] = lora_simple_checksum(ack[1], ack[2], ack[3]);
                            ack[5] = 0xA1;
                            Radio.Send(ack, sizeof(ack));
                        }
                    }
                    break;
                }
                // Fallback: strict legacy set-address packet A0 <addr> 00 00 00 00 A1
                else if ((BufferSize == 7) &&
                         (Buffer[0] == 0xA0) &&
                         (Buffer[2] == 0x00) &&
                         (Buffer[3] == 0x00) &&
                         (Buffer[4] == 0x00) &&
                         (Buffer[5] == 0x00) &&
                         (Buffer[6] == 0xA1))
                {
                    uint8_t new_addr = Buffer[1];
                    SLAVE1_ADDR = new_addr;
                    if (store_node_address(SLAVE1_ADDR)) {
                        printf("Stored node address from LoRa packet: %d\r\n", SLAVE1_ADDR);
                    } else {
                        printf("Failed to store node address from LoRa packet\r\n");
                    }
                    uint8_t ack[4] = {0xA0, 0x1E, SLAVE1_ADDR, 0xA1};
                    Radio.Send(ack, sizeof(ack));
                    break;
                }

                // printf("Recieve New Msg , length:[%d] \r\n", BufferSize);
                // printf("Recieve SLAVE1_ADDR: %d, receive addr: %d\r\n", SLAVE1_ADDR, Buffer[2]);

                if (Buffer[1] == 0xFF)
                {
                    // 设置�?的�?�色
                    ledStatus = Buffer[3];
                    switch (ledStatus)
                    {
                    case 1:
                        gpio_write(GPIOA, GPIO_PIN_4, GPIO_LEVEL_HIGH);
                        gpio_write(GPIOA, GPIO_PIN_5, GPIO_LEVEL_LOW);
                        gpio_write(GPIOA, GPIO_PIN_7, GPIO_LEVEL_LOW);
                        break;
                    case 2:
                        gpio_write(GPIOA, GPIO_PIN_4, GPIO_LEVEL_LOW);
                        gpio_write(GPIOA, GPIO_PIN_5, GPIO_LEVEL_HIGH);
                        gpio_write(GPIOA, GPIO_PIN_7, GPIO_LEVEL_LOW);
                        break;
                    case 3:
                        gpio_write(GPIOA, GPIO_PIN_4, GPIO_LEVEL_LOW);
                        gpio_write(GPIOA, GPIO_PIN_5, GPIO_LEVEL_LOW);
                        gpio_write(GPIOA, GPIO_PIN_7, GPIO_LEVEL_HIGH);
                        break;
                    case 4:
                        gpio_write(GPIOA, GPIO_PIN_4, GPIO_LEVEL_LOW);
                        gpio_write(GPIOA, GPIO_PIN_5, GPIO_LEVEL_LOW);
                        gpio_write(GPIOA, GPIO_PIN_7, GPIO_LEVEL_LOW);
                        break;

                    case 5:
                        gpio_write(GPIOA, GPIO_PIN_4, GPIO_LEVEL_HIGH);
                        gpio_write(GPIOA, GPIO_PIN_5, GPIO_LEVEL_HIGH);
                        gpio_write(GPIOA, GPIO_PIN_7, GPIO_LEVEL_HIGH);
                        break;

                    default:
                        break;
                    }
                }
                else // 判断�?否为控制�?己？
                {
                    if (Buffer[2] == SLAVE1_ADDR)
                    {
                        printf("Recieve New Msg , length:[%d] \r\n", BufferSize);
                        printf("Recieve SLAVE1_ADDR: %d, receive addr: %d data: \r\n", SLAVE1_ADDR, Buffer[2]);
                        for (int i = 0; i < BufferSize; i++)
                        {
                            printf("%02X ", Buffer[i]);
                        }
                        printf("\r\n");
                        uint8_t data[205] = {0};
                        data[0] = 0xFF;
                        data[1] = SLAVE1_ADDR;
                        memmove(data + 2, ACKBuf, sizeof(ACKBuf));
                        Radio.Send(data, sizeof(ACKBuf));
                        // 设置�?的�?�色
                        if (Buffer[1] == 0xF1)
                        {
                            ledStatus = Buffer[3];
                            switch (ledStatus)
                            {
                            case 1:
                                gpio_write(GPIOA, GPIO_PIN_4, GPIO_LEVEL_HIGH);
                                gpio_write(GPIOA, GPIO_PIN_5, GPIO_LEVEL_LOW);
                                gpio_write(GPIOA, GPIO_PIN_7, GPIO_LEVEL_LOW);
                                break;
                            case 2:
                                gpio_write(GPIOA, GPIO_PIN_4, GPIO_LEVEL_LOW);
                                gpio_write(GPIOA, GPIO_PIN_5, GPIO_LEVEL_HIGH);
                                gpio_write(GPIOA, GPIO_PIN_7, GPIO_LEVEL_LOW);
                                break;
                            case 3:
                                gpio_write(GPIOA, GPIO_PIN_4, GPIO_LEVEL_LOW);
                                gpio_write(GPIOA, GPIO_PIN_5, GPIO_LEVEL_LOW);
                                gpio_write(GPIOA, GPIO_PIN_7, GPIO_LEVEL_HIGH);
                                break;

                            case 4:
                                gpio_write(GPIOA, GPIO_PIN_4, GPIO_LEVEL_LOW);
                                gpio_write(GPIOA, GPIO_PIN_5, GPIO_LEVEL_LOW);
                                gpio_write(GPIOA, GPIO_PIN_7, GPIO_LEVEL_LOW);
                                break;

                            case 5:
                                gpio_write(GPIOA, GPIO_PIN_4, GPIO_LEVEL_HIGH);
                                gpio_write(GPIOA, GPIO_PIN_5, GPIO_LEVEL_HIGH);
                                gpio_write(GPIOA, GPIO_PIN_7, GPIO_LEVEL_HIGH);
                                break;
                            default:
                                break;
                            }
                        }
                        // 读取�?的状�?
                        else if (Buffer[1] == 0xF2)
                        {
                            uint8_t data[] = {
                                0xF3,
                                SLAVE1_ADDR,
                                ledStatus,
                                0xF4};
                            Radio.Send(data, 4);
                        }
                    }
                    // else
                    // printf("[] Node is not me , error\r\n");
                }
            }
#endif

            break;
        case TX:
            // printf("[%s()-%d]Tx done\r\n", __func__, __LINE__);
            Radio.Rx(RX_TIMEOUT_VALUE);
            sendMsgFlag = 2;
            State = LORA_IDLE;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            // printf("[%s()-%d]Rx timeout/error\r\n",__func__,__LINE__);
            Radio.Rx(RX_TIMEOUT_VALUE);
            State = LORA_IDLE;
            break;
        case TX_TIMEOUT:
            printf("[%s()-%d]Tx timeout\r\n", __func__, __LINE__);
            Radio.Rx(RX_TIMEOUT_VALUE);
            sendMsgFlag = 2;
            State = LORA_IDLE;
            break;
        case LORA_IDLE:
            if (0 == sendMsgFlag)
            {
                // Radio.Send((uint8_t *)sendBuf, strlen(sendBuf) + 1);
                sendMsgFlag = 1;
                // #ifdef CONFIG_GATEWAY
                //                 {
                //                     sendMasterAsk(SLAVE1_ADDR, OP_R_SENSOR, PRAM_R_ALL); //主机发送指�?
                //                     printf("[%s()-%d] CONFIG_GATEWAY send message:%s\r\n", __func__, __LINE__, sendBuf);
                //                 }
                // #else
                //                 {
                //                     // sendMasterAsk(SLAVE1_ADDR, OP_R_SENSOR, PRAM_R_ALL); //主机发送指�?
                //                     // printf("[%s()-%d] CONFIG_GATEWAY send message:%s\r\n", __func__, __LINE__, sendBuf);
                //                     printf("[%s()-%d] Node do not send message:%s \r\n", __func__, __LINE__);
                //                 }
                // #endif
            }
            break;
        default:
            printf("[%s()-%d]unknown case:%d\r\n", __func__, __LINE__, State);
            break;
        }

        // Process Radio IRQ
        Radio.IrqProcess();

        /* Periodic SHT3x read and send (node only). Uses a simple software timer
         * paced by a short delay to avoid busy-looping. Interval ~10s. */
#ifndef CONFIG_GATEWAY
        const TimerTime_t sht_interval_ms = 10000;

        delay_ms(50);
        if (node_report_due_immediately) {
            node_report_due_immediately = false;
            node_send_sensor_report("startup");
        } else if (TimerGetElapsedTime(node_last_report_at) >= sht_interval_ms) {
            node_send_sensor_report("periodic");
        }

        /* Blink state update (50ms tick) - only toggle GPIO_PIN_4 now */
        if (blink_ms_remaining > 0) {
            blink_ms_remaining -= 50;
            blink_toggle_elapsed += 50;
            if (blink_toggle_elapsed >= blink_toggle_interval) {
                blink_toggle_elapsed = 0;
                blink_led_state = !blink_led_state;
                if (blink_led_state) {
                    gpio_write(GPIOA, GPIO_PIN_4, GPIO_LEVEL_HIGH);
                } else {
                    gpio_write(GPIOA, GPIO_PIN_4, GPIO_LEVEL_LOW);
                }
            }
            if (blink_ms_remaining <= 0) {
                /* ensure led off (only PIN_4) */
                gpio_write(GPIOA, GPIO_PIN_4, GPIO_LEVEL_LOW);
                /* send finished ACK: A0 OPCODE_FIND_ACK <addr> 02 cs A1 */
                uint8_t ackf[6];
                ackf[0] = 0xA0;
                ackf[1] = OPCODE_FIND_ACK;
                ackf[2] = SLAVE1_ADDR;
                ackf[3] = 0x02; /* finished */
                ackf[4] = lora_simple_checksum(ackf[1], ackf[2], ackf[3]);
                ackf[5] = 0xA1;
                Radio.Send(ackf, sizeof(ackf));
                printf("Blink finished, sent finish ACK\r\n");
            }
        }
#endif
    }
}

void OnTxDone(void)
{
    Radio.Sleep();
    printf("[RADIO] OnTxDone\r\n");
    State = TX;
}

/**
 * 功能：接收数�?�?
 * 参数�?
 *       buffer:数据包存放地址
 * 返回值：
 * 		 如果成功接收返回1，否则返�?0
 */
unsigned char receivePackets(unsigned char *buffer)
{
    if (BufferSize)
    {
        memset(buffer, 0, BUFFER_SIZE);
        memcpy(buffer, Buffer, BufferSize);
        return 1;
    }
    else
        return 0;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    if (size == 0) {
        printf("[RADIO] OnRxDone size=0 ignored\r\n");
        return;
    }
    Radio.Sleep();
    BufferSize = size;
    memset(Buffer, 0, BUFFER_SIZE);
    memcpy(Buffer, payload, BufferSize);
    RssiValue = rssi;
    SnrValue = snr;
    printf("[RADIO] OnRxDone size=%d rssi=%d snr=%d\r\n", size, rssi, snr);
    State = RX;
}

void OnTxTimeout(void)
{
    Radio.Sleep();
    printf("[RADIO] OnTxTimeout\r\n");
    State = TX_TIMEOUT;
}

void OnRxTimeout(void)
{
    Radio.Sleep();
    printf("[RADIO] OnRxTimeout\r\n");
    State = RX_TIMEOUT;
}

void OnRxError(void)
{
    Radio.Sleep();
    printf("[RADIO] OnRxError\r\n");
    State = RX_ERROR;
}
