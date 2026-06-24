/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_host.h"
#include "usbh_cdc.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>   // ← AGREGAR
#include <ctype.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*(Private Defines): Aquí pones tus #define.*/
/* Para STM32F401 (256KB), el último sector es el 5 */
#define FLASH_LOG_SECTOR      FLASH_SECTOR_5
#define CONFIG_FLASH_SECTOR    FLASH_SECTOR_5
#define FLASH_LOG_START_ADDR  0x08020000  // Dirección de inicio del Sector 5
#define FLASH_CONFIG_ADDRESS   0x0803C000
//#define FLASH_CONFIG_ADDRESS 0x08020000UL

#define MODEM_DEBUG_LINES
//#define MODEM_DEBUG_URC
#define QUEUE_SIZE 16
#define LINE_BUFFER_SIZE 128
#define MAX_RETRIES 3
#define UART_BUF_SIZE  2048
#define CFG_RX_BUF_SIZE  2048
#define CDC_BUF_SIZE  64
#define CDC_ACCUM_SIZE       1024
#define CDC_FLUSH_TIMEOUT_MS   50U
#define CDC_CHUNK_SIZE         64U   /* bytes máx por flush para no bloquear USB */
#define UART_IDLE_TIMEOUT_MS  20U
#define RING_BUFFER_SIZE 512
#define BRIDGE_TIMEOUT_MS   60000UL
#define CONFIG_MAGIC         0xDEADBEEFUL
//#define CONFIG_MAGIC 0xA7670A55UL

#define CONFIG_STRUCT_VERSION 0x0001
#define CONFIG_VERSION 0x0001
#define PROTOCOL_UDP    0
#define PROTOCOL_TCP    1

#define EEPROM_SIZE_BYTES     65536UL
#define EEPROM_24LC512_ADDR     (0x50 << 1)   // A2=A1=A0=0
#define LOG_HEADER_ADDR   0x0000
#define LOG_START_ADDR    0x0100
#define EEPROM_LOG_BASE   0x0100
#define LOG_RECORD_SIZE   sizeof(ModemLogRecord_t)
//#define MAX_LOG_RECORDS  ((EEPROM_SIZE_BYTES - EEPROM_LOG_BASE) / LOG_RECORD_SIZE) // 4080 registros.
#define MAX_LOG_RECORDS 	4080
#define EEPROM_HEAD_ADDR    0x0000
#define EEPROM_COUNT_ADDR   0x0002

// volatile uint8_t flags_FSM_Modem ==
#define SIM_READY 0x01
#define NETWORK_REGISTERED 0x02
#define PDN_ACTIVE 0x04
#define PDN_STABLE 0x08
#define NET_OPEN 0x10
#define SOCKET_OPEN 0x20
#define UNDEFINED_1 0x40
#define UNDEFINED_2 0x80

// volatile uint8_t flags_Comunicaciones ==
#define READY_TO_SEND_UDP 0x01
#define SENDING_UDP 0x02
#define UDP_ACK_OK 0x04
#define REC_ABONADO_CID 0x08
#define REC_ABONADO_HB 0x10
#define SEND_ABONADO_ACK 0x20
#define UNDEFINED_11 0x40
#define UNDEFINED_22 0x80

/* USER CODE END PD */


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*Private Typedefs): Reservado para tus estructuras (struct) y enumeraciones (enum). Por ejemplo, la definición de tu ModemState_t o Event_t.*/

typedef enum {
	ST_IDLE, 			// Esperar PDN estable.
	ST_SEND_CPIN,
	ST_CHECK_SIM, 		// Enviar AT+CPIN?
	ST_CHECK_SIGNAL,	// Enviar AT+CSQ
	ST_CHECK_CFUN,		// Enviar AT+CFUN?
	ST_CHECK_CSREG, 	// Enviar AT+CREG?
	ST_CHECK_PSREG, 	// Enviar AT+CEREG?
	ST_CHECK_GPRSREG,	// Enviar AT+CGREG?
	ST_ENABLE_TIME_SYNC,
	ST_GET_TIME,
	ST_CHECK_CPSI,		// Enviar AT+CPSI?
	ST_ACTIVATE_PDP, 	// AT+CGDCONT (si no se hizo) y AT+NETOPEN
	ST_SET_MODE,		// (opcional) AT+CIPMODE=0
	ST_NETOPEN_WAIT,	// AT+CIPOPEN o AT+SERVERSTART
	ST_GET_DNS,
	ST_GET_IP_WAIT,
	ST_UDP_OPEN_WAIT,
	ST_WAIT_FOR_PROMPT,
	ST_WAIT_SEND_OK,
	ST_READY,			// Espera petición (botón/SMS)
	ST_SEND,			// Preparar AT+CIPSEND
	ST_WAIT_TX,			// Enviar datos y esperar OK
	ST_CLOSE,			// (Opcional) AT+CIPCLOSE
	ST_NETCLOSE,		// AT+NETCLOSE
	ST_NETCLOSE_WAIT,
	ST_ERROR			// Manejo de error / reinicio
} modem_state_t;

typedef enum {
	EVT_NONE = 0,
	EVT_ATREADY,
	EVT_CPINREADY,
    EVT_SIM_NOT_INSERTED,
    EVT_SIM_PIN_REQUIRED,
    EVT_SIM_PUK_REQUIRED,
	EVT_SMS_DONE,
	EVT_CSQ,
	EVT_CFUN,
    EVT_OK,
    EVT_ERROR,
	EVT_CMEE_ERROR,
    EVT_TIMEOUT,
    EVT_CREG_REGISTERED,
    EVT_CREG_SEARCHING,
	EVT_CREG_DENIED,
	EVT_CEREG_REGISTERED,
	EVT_CEREG_SEARCHING,
	EVT_CEREG_DENIED,
	EVT_CGREG_REGISTERED,
	EVT_CGREG_SEARCHING,
	EVT_CGREG_DENIED,
	EVT_TIME_SYNC_OK,
	EVT_CPSI_READY,
	EVT_PDN_ACT,
	EVT_PDN_DEACT,
	EVT_NETWORK_DETACH,
	EVT_NETOPEN_SUCCESS,
	EVT_DNS_OK,
	EVT_DNS_ERROR,
	EVT_NETOPEN_FAILED,
    EVT_NETCLOSE_SUCCESS,
    EVT_NETCLOSE_FAILED,
	EVT_UDP_OPEN_SUCCESS,
	EVT_UDP_OPEN_FAILED,
	EVT_IP_READY,
    EVT_SOCKET_OPEN,
    EVT_SOCKET_CLOSED,
	EVT_SOCKET_LOST,
	EVT_SEND_PROMPT_READY,
    EVT_PROMPT,
    EVT_SEND_OK,
	EVT_DATA_RECEIVED,
	EVT_SMS_RECEIVED,
	EVT_SMS_READ
} modem_event_t;

typedef enum {
    MODEM_OFF,
    MODEM_INIT,
    MODEM_ATTACH,
    MODEM_DATA,
    MODEM_ERROR
} modem_main_state_t;

typedef enum {
    SMS_IDLE,
    SMS_READ,
    SMS_PARSE,
    SMS_EXECUTE,
    SMS_DELETE
} sms_state_t;

typedef enum
{
    CMD_INVALID = 0,
    CMD_READ,
    CMD_WRITE,
    CMD_SAVE,
    CMD_DEFAULT,
    CMD_REBOOT,
    CMD_MODEMREINIT,

    CMD_LOG_INFO,
    CMD_LOG_READ,
    CMD_LOG_CLEAR

} config_command_t;


typedef struct
{
    uint8_t buffer[UART_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;

} RingBuffer_uart;

typedef enum
{
    PARSER_IDLE,
    PARSER_COLLECT

} ParserState_t;

// Estructura para diagnóstico (Global o de instancia)
struct {
    char tech[8];
    char band[20];
    uint32_t cell_id;
} lte_diag;

typedef struct {
    modem_event_t queue[QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
} event_queue_t;

typedef struct {
    uint32_t timestamp;      // HAL_GetTick() o RTC si tenés
    uint8_t  last_state;     // El estado donde ocurrió el fallo
    uint8_t  msg_index;      // Qué mensaje de la ráfaga era
    int8_t   rssi;           // Nivel de señal al momento del error
    uint8_t  error_code;     // Código CME/CMS o interno (0: Timeout, 1: Denied, etc)
} Modem_Error_Log_t;

typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
} RingBuffer_t;

typedef enum
{
    PKT_NONE = 0,
    PKT_EVENT,
    PKT_HEARTBEAT

} PacketType_t;

typedef struct
{
    PacketType_t type;
    char account[5];
    char qualifier;
    char event_code[4];
    char partition[3];
    char zone[4];

    uint8_t hb_numabo;
    uint8_t hb_alarm;
    uint8_t hb_dev_status;
    uint8_t hb_dev_memory;
} CID_Message_t;

typedef enum
{
    SYS_MODE_NORMAL = 0,
    SYS_MODE_BRIDGE,
    SYS_MODE_CONFIG

} SystemMode_t;

typedef enum
{
    CFG_WAIT_START = 0,
    CFG_RECEIVE_FRAME,
    CFG_VERIFY_FORMAT,
    CFG_VERIFY_CRC,
	CFG_VERIFY_PASSWORD,
    CFG_PARSE_COMMAND,
    CFG_EXECUTE_COMMAND
} ConfigFSM_t;

typedef enum
{
	WAIT_STX = 0,
	RECEIVE_HEADER,
	RECEIVE_JSON,
	WAIT_ETX,
	VERIFY_CRC,
	PARSE_JSON,
	SAVE_FLASH,
	SEND_ACK
}Conf_Mode_States;

typedef struct
{
	uint16_t config_version;
    char config_timestamp[20];

    char serial_number[9];
    char phone_number[11];

    char modem_account[5];
    char panel_account[5];

    char sms_password[5];
    char config_password[5];

    char apn_claro[32];
    char apn_movistar[32];

    uint16_t heartbeat_seconds;

    struct
    {
    	char server_domain[64];
    	char ip[16];
        uint16_t port;
        uint8_t protocol; // 0=UDP 1=TCP

    } servers[3];

} modem_config_t;

typedef struct
{
    const char *text;
    config_command_t cmd;
} CommandTable_t;

typedef struct __attribute__((aligned(4)))
{
    uint16_t version;
    uint32_t magic;
    modem_config_t cfg;
    uint32_t crc;
} flash_config_t;

typedef struct __attribute__((packed))
{
    uint32_t timestamp;      // HAL_GetTick()
    uint16_t event_id;       // código del evento
    uint32_t param1;
    uint32_t param2;
    uint16_t crc;
} ModemLogRecord_t;

typedef enum
{
	// Diagnóstico
    LOG_CMEE_CONFIG_OK = 1,
	LOG_CMEE_ERROR,
    LOG_TIMEOUT_CMEE,

	// SIM.
	LOG_SIM_READY,
	LOG_SIM_NOT_INSERTED,
	LOG_SIM_PIN_REQUIRED,
	LOG_SIM_PUK_REQUIRED,
	LOG_TIMEOUT_CPIN,

	// Registro LTE
    LOG_SIGNAL_OK,
    LOG_SIGNAL_LOW,
	LOG_NO_SIGNAL,
    LOG_WAIT_REG,
    LOG_REG_DENIED,
    LOG_TIMEOUT_REG,

    LOG_CFUN_OK,
    LOG_CFUN_ERR,

	LOG_NETWORK_REGISTERED,
	LOG_NETWORK_LOST,

    LOG_LTE_READY,
    LOG_LTE_DENIED,
    LOG_TIMEOUT_LTE,

    LOG_GPRS_READY,
    LOG_NET_DIAG,
    LOG_NETOPEN_START,
    LOG_NETOPEN_OK,
    LOG_NETOPEN_ERR,
    LOG_NETOPEN_CMD_ERR,
    LOG_TIMEOUT_NETOPEN,

	// Datos (PDN)
	LOG_PDN_ACT,
	LOG_PDN_DEACT,
	LOG_NETWORK_DETACH,
	LOG_APN_OK,
	LOG_APN_ERR,
    LOG_TIMEOUT_APN,
	LOG_NET_OPEN_OK,
	LOG_NET_OPEN_ERR,
	LOG_IP_ASSIGNED,
    LOG_IP_NOT_FOUND,
    LOG_IP_CMD_ERR,
    LOG_TIMEOUT_IP,

	// UDP
	LOG_SOCKET_CLOSED,
	LOG_SOCKET_LOST,
	LOG_UDP_OPEN_START,
    LOG_UDP_OPEN_OK,
    LOG_UDP_OPEN_ERR,
    LOG_UDP_CMD_ERR,
    LOG_TIMEOUT_UDP,

	// DNS
	LOG_DNS_START,
	LOG_DNS_OK,
	LOG_DNS_ERROR,
	LOG_TIMEOUT_DNS,

    LOG_SEND_OK,
    LOG_SEND_ERR,
    LOG_SEND_REJECTED,
    LOG_TIMEOUT_PROMPT,
    LOG_TIMEOUT_SEND,

    LOG_NETCLOSE_START,
    LOG_NETCLOSE_OK,
    LOG_NETCLOSE_ERR,
    LOG_TIMEOUT_NETCLOSE,

	LOG_MODEM_TIMEOUT,
	LOG_USB_CDC_LOST,

	LOG_RS232_TIMEOUT,
	LOG_RS232_RESTORED,

	LOG_CID_RECEIVED,
	LOG_HEARTBEAT_RECEIVED,

	LOG_SERVER_ACK,
	LOG_SERVER_TIMEOUT,

	LOG_RTC_SYNC_OK,
	LOG_RTC_SYNC_TIMEOUT,

	//Arranque
	LOG_MODEM_BOOT,
	LOG_MODEM_RESET,
	LOG_RESET_POR,
	LOG_RESET_SOFTWARE,
	LOG_RESET_WATCHDOG,
	LOG_FATAL_ERROR

} ModemEvent_t;

typedef struct
{
    uint32_t write_index;
} LogHeader_t;

typedef enum
{
	LED_IDLE,
    LED_OFF,
    LED_ON,
	LED_ON_TIMER,
    LED_BLINK_SLOW,
    LED_BLINK_FAST
} LedMode_t;

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    LedMode_t mode;
    uint32_t lastToggle;
    uint8_t state;
    uint32_t dutycicle;

} LedControl_t;

/*
MAGIC
Ejemplo:
0xDEADBEEF
para validar memoria válida.
 */

/*
 1. FORMATO JSON (PC)
  {

  "version": {
    "config_version": "0001",
  },

  "device": {
    "serial_number": "12345678",
    "phone_number": "1122334455"
  },

  "accounts": {
    "modem_account": "1234",
    "panel_account": "5678"
  },

  "security": {
    "sms_password": "1234",
    "config_password": "0000"
  },

  "network": {
    "apn_claro": "igprs.claro.com.ar",
    "apn_movistar": "internet.movistar.com.ar",
    "heartbeat_seconds": 60
  },

  "servers": [
    {
      "ip": "190.111.217.188",
      "port": 57777,
      "protocol": "UDP"
    },
    {
      "ip": "190.111.217.189",
      "port": 57778,
      "protocol": "UDP"
    },
    {
      "ip": "192.168.0.100",
      "port": 5000,
      "protocol": "TCP"
    }
  ]
}

 */

/* USER CODE END PTD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;
UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/*(Private Variables): Acá declaras tus variables globales. Tus colas, el buffer de transmisión de 20 bytes, la bandera cereg_ok,
 * o el puntero de estado de la FSM: volatile uint32_t state_timer;.*/

// ============================================================================
// 1. CONTROL PRINCIPAL Y MÁQUINA DE ESTADOS (FSM)
// ============================================================================
volatile SystemMode_t systemMode = SYS_MODE_NORMAL;
uint32_t bridge_last_activity = 0;

modem_state_t ModemState  = ST_IDLE;
static modem_state_t prevState = -1;
uint32_t state_timer      = 0;
static uint32_t boot_time = 0;
static uint8_t at_busy     = 0;
ConfigFSM_t cfgModeState = CFG_WAIT_START;
uint16_t eeprom_log_head = 0;
uint16_t eeprom_log_count = 0;
uint32_t rtc_last_sync;

// ============================================================================
// 2. COLAS DE EVENTOS Y BUFFERS CIRCULARES
// ============================================================================
event_queue_t lte_queue;
event_queue_t sms_queue;
RingBuffer_t modemBuffer = {{0}, 0, 0};
volatile uint8_t send_request = 0;
static char cfg_rx_buffer[CFG_RX_BUF_SIZE];
static uint16_t cfg_rx_index = 0;
char command[16];
RingBuffer_uart uart1Buffer = {{0}, 0, 0};

// ============================================================================
// 3. TELEMETRÍA Y LOG DE COMUNICACIÓN (CONTACT-ID / SMS)
// ============================================================================
uint8_t current_msg_index   = 0;
uint8_t last_confirmed_index = 0;
bool burst_mode_active       = false;
static uint8_t msg_index     = 0;
static uint8_t retry_count   = 0;

// Matriz de mensajes de eventos (20 bytes + \0)
const char alarm_messages[10][21] = {
    "ALERTA TEMP ALTA 001", "BATERIA BAJA NODO 04",
    "SENSOR HUMEDAD FALLO", "PUERTA ACCESO ABIERT",
    "SOBREVOLTAJE LINEA 2", "CORTE ENERGIA GRUPO3",
    "NIVEL CRITICO TANQ01", "FUGA DE GAS SECTOR 5",
    "SISTEMA REINICIADO01", "PRESION ALTA BOMBA 2"
};

static const CommandTable_t cmdTable[] =
{
    {"$CFG,READ,",        CMD_READ},
    {"$CFG,WRITE,",       CMD_WRITE},
    {"$CFG,SAVE,",        CMD_SAVE},
    {"$CFG,DEFAULT,",     CMD_DEFAULT},
    {"$CFG,REBOOT,",      CMD_REBOOT},
    {"$CFG,MODEMREINIT,", CMD_MODEMREINIT},
    {"$CFG,LOGINFO,",     CMD_LOG_INFO},
    {"$CFG,LOGREAD,",     CMD_LOG_READ},
    {"$CFG,LOGCLEAR,",    CMD_LOG_CLEAR}
};

// ============================================================================
// 4. CONTROL DE SMS (Mantenimiento / Comandos)
// ============================================================================
sms_state_t smsState = SMS_IDLE;
static int sms_index = -1;
static char sms_text[160];
static char sms_sender[32];
static uint8_t sms_ready = 0;

// ============================================================================
// 5. DIAGNÓSTICO DE RED CELULAR (Métricas de Señal y Capa IP)
// ============================================================================
int16_t dbm;
int8_t last_rssi_raw       = 99;
int16_t last_rssi_dbm      = -113;
int rssi_tmp, ber_tmp;
int8_t last_cfun_value     = -1;
char local_ip[16]          = "0.0.0.0";
int8_t ip_received         = 0;
int last_udp_err           = 0;
uint32_t pdn_last_event_time = 0;

volatile uint8_t flags_FSM_Modem = 0;
volatile uint8_t flags_Comunicaciones = 0;

char dns_ip[16];
uint8_t dns_pending = 0;

// ============================================================================
// 6. BUFFERS DE INTERFACES (UART Abonado / USB Módem)
// ============================================================================
char tx_buffer[64];
char lastTarget[32] = {0};
uint16_t searchIdx  = 0;

/* Puerto Serie Abonado (Comandos / Configuración) */
static char lineBuffer[LINE_BUFFER_SIZE];
static uint8_t lineIndex = 0;
static uint8_t uart_rx_char;
uint32_t uart1_last_activity;
uint8_t uart1_idle_alarm = 0;

modem_config_t runtime_cfg;
modem_config_t temp_runtime_cfg;
CID_Message_t last_cid_msg;
CID_Message_t last_hb_msg;

static ParserState_t parserState = PARSER_IDLE;
static char lineBufferUart1[LINE_BUFFER_SIZE];
static uint16_t lineIndexUart1 = 0;

/* Canal USB CDC (Comunicación con BKP-A7670SA) */
uint8_t cdc_rx_buf[CDC_BUF_SIZE];
static uint8_t cdc_accum_buf[CDC_ACCUM_SIZE];
static uint16_t cdc_accum_len    = 0;
static uint32_t cdc_last_rx_tick = 0;


// ============================================================================
// 7. DRIVERS DE HARDWARE LOCAL (LEDs y Ticks)
// ============================================================================
volatile uint8_t modem_listo = 0;
uint16_t rx_index            = 0;
uint8_t rx_data;

LedControl_t ledRojoSIM;
LedControl_t ledVerdeSIM;
LedControl_t ledRojoConexion;
LedControl_t ledVerdeConexion;
LedControl_t ledVerdeBlackPill;

LedControl_t *leds[] =
{
    &ledRojoSIM,
    &ledVerdeSIM,
    &ledRojoConexion,
    &ledVerdeConexion,
	&ledVerdeBlackPill
};



/* --- Handles externos de librería (Definidos en sus respectivos .c) --- */
extern USBH_HandleTypeDef hUsbHostFS;
extern ApplicationTypeDef Appli_state;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
void MX_USB_HOST_Process(void);
void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
void EventQueue_Init(void);
void Modem_Reset(void);
int Queue_Pop(event_queue_t *q, modem_event_t *evt);
void Modem_Log(ModemEvent_t id, uint32_t param1, uint32_t param2);
void EEPROM_Log_Write(ModemLogRecord_t *rec);
HAL_StatusTypeDef EEPROM_Write_Buffer(uint16_t memAddr,uint8_t *data,uint16_t len);
HAL_StatusTypeDef EEPROM_Read_Buffer(uint16_t memAddr,uint8_t *data,uint16_t len);
const char *ModemEvent_ToString(ModemEvent_t event);
int16_t RingBuffer_Read(void);
void RingBuffer_Clear(void);
void System_Mode_Task(void);

uint8_t Modem_Send_AT(const char *cmd);
void Modem_Process_Buffer(void);
void Modem_FSM_Run(void);
void SMS_FSM_Run(void);
void Bridge_Process(void);
void Config_Mode_UART_TX_RX(void);
void Button_Process(void);
void RTC_Fill_ASCII_DateTime(uint8_t *out);
void RTC_Update_From_Modem(uint8_t year,uint8_t month,uint8_t day,uint8_t hour,uint8_t minute,uint8_t second);
void RTC_Set_From_CCLK(const char *line);
void Debug_Print(const char *str);
void CID_Parser_Byte(uint8_t c);
void Config_Mode_Parser(uint8_t c);
void Load_Default_Config(void);
void Flash_Load_Config(void);
void Flash_Save_Config(void);
uint32_t CRC32(const uint8_t *data, uint32_t len);
static uint8_t buffer_contiene(const char *str);
void Parse_JSON_To_Runtime_Config(const char *json);
config_command_t Parse_Command(void);
uint8_t JSON_Get_String(const char *json, const char *key, char *out, uint16_t out_size);
uint8_t JSON_Get_Int(const char *json, const char *key, uint16_t *value);
void RS232_Rx_Byte(uint8_t c);
void Build_Config_JSON(char *json);

void CMD_Read_Config(void);
void CMD_Write_Config(void);
void CMD_Save_Config(void);
void CMD_Default_Config(void);
void CMD_Reboot(void);

void Send_ACK(const char *cmd);
void Send_Error(const char *error);
uint16_t CRC16_CCITT(const uint8_t *data, uint16_t len);
uint8_t Verify_Frame_Format(void);
uint8_t Verify_Frame_CRC(void);
uint8_t Verify_Password(void);

void CID_Send_ACK_RS232(void);
uint8_t UDP_Is_ACK(uint8_t *rx, uint16_t len);
uint16_t UDP_Build_Heartbeat(CID_Message_t *msg, uint8_t *out, uint32_t serial, uint8_t seq);
uint16_t UDP_Build_Event(CID_Message_t *msg,uint8_t *out,uint32_t serial,uint8_t seq);
uint8_t CID_Checksum1(uint8_t *buf);
uint8_t XOR_Checksum(uint8_t *data, uint16_t start, uint16_t end);
static uint8_t pack_bcd(char high, char low);
uint8_t CID_Parse_Heartbeat(uint8_t *rx, uint16_t len, CID_Message_t *msg);
uint8_t CID_Parse_Event(uint8_t *rx, uint16_t len, CID_Message_t *msg);
int Parse_SMS_Index(const char* line);
void Dispatch_Event(modem_event_t evt);

void Process_Heartbeat(char *line);
uint8_t Is_Heartbeat(char *line);
void Process_CID_Event(char *line);
uint8_t Is_CID_Event(char *line);
void Process_AT_Command(char *line);
void Process_Config_Command(char *line);
uint8_t Is_AT_Command(char *line);
uint8_t Is_Config_Command(char *line);
void Process_Line(char *line);
void Parser_Feed(uint8_t c);
void UART1_Process_RingBuffer(void);
uint8_t RingBufferUart_Get(RingBuffer_uart *rb, uint8_t *c);
uint8_t RingBufferUart_Put(RingBuffer_uart *rb, uint8_t c);
void RingBufferUart_Clear(RingBuffer_uart *rb);
uint32_t RingBufferUart_Count(RingBuffer_uart *rb);
void Queue_CID_Message(CID_Message_t *msg);
void Queue_Heartbeat(CID_Message_t *msg);

uint8_t EEPROM_IsPresent(void);
HAL_StatusTypeDef EEPROM_Write(uint16_t addr, uint8_t *data, uint16_t len);
HAL_StatusTypeDef EEPROM_Read(uint16_t addr,uint8_t *data, uint16_t len);
void PruebaSimpleEEPROM(void);
uint8_t EEPROM_Log_Read(uint16_t index,ModemLogRecord_t *rec);
void EEPROM_Log_Init(void);
void CMD_Log_Info(void);
void CMD_Log_Read(uint16_t index);
uint16_t Get_LogRead_Index(void);
void CMD_Log_Clear(void);
void LED_Init(void);
void LED_Update_From_Event(ModemEvent_t event);
void LED_Task(void);
void LED_Set(LedControl_t *led, LedMode_t mode, uint32_t tiempo);
void RS232_Monitor_Task(void);
void Parse_DNS_Response(char *line);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  Flash_Load_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_USB_HOST_Init();

  /* USER CODE BEGIN 2 */
  // Arrancamos la recepción por interrupción de 1 byte.
  HAL_UART_Receive_IT(&huart1, &uart_rx_char, 1);
  //HAL_UARTEx_ReceiveToIdle_DMA(...)

  /* --- Diagnóstico de causa de reset --- */
  uint8_t boot_msg[80];
  uint8_t boot_len;
  uint32_t csr = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF;  /* limpiar flags para el próximo reset */

  /* Si venimos de un brownout, esperar a que el modem termine
   * de inicializarse antes de arrancar el USB Host */

  if (csr & RCC_CSR_PORRSTF)
	  boot_len = snprintf((char*)boot_msg, sizeof(boot_msg), "\r\nReset: POR/BOR (brownout o power-on)\r\n");
  else if (csr & RCC_CSR_PINRSTF && !(csr & RCC_CSR_SFTRSTF))
      boot_len = snprintf((char*)boot_msg, sizeof(boot_msg), "\r\nReset: PIN (NRST externo)\r\n");
  else if (csr & RCC_CSR_SFTRSTF)
      boot_len = snprintf((char*)boot_msg, sizeof(boot_msg), "\r\nReset: SOFTWARE (NVIC_SystemReset)\r\n");
  else if (csr & RCC_CSR_IWDGRSTF)
      boot_len = snprintf((char*)boot_msg, sizeof(boot_msg), "\r\nReset: IWDG watchdog\r\n");
  else if (csr & RCC_CSR_WWDGRSTF)
      boot_len = snprintf((char*)boot_msg, sizeof(boot_msg), "\r\nReset: WWDG watchdog\r\n");
  else
      boot_len = snprintf((char*)boot_msg, sizeof(boot_msg), "\r\nReset: desconocido (CSR=0x%08lX)\r\n", csr);

  // HAL_UART_Transmit(&huart1, boot_msg, boot_len, 200);

  uint8_t ready_msg[] = "Black Pill Iniciada\r\n";
  HAL_UART_Transmit(&huart1, ready_msg, sizeof(ready_msg)-1, 100);

  boot_time = HAL_GetTick();

  EventQueue_Init();
  RingBufferUart_Clear(&uart1Buffer);

  if(EEPROM_IsPresent())
  {
      Debug_Print("\r\nEEPROM OK\r\n");
      EEPROM_Log_Init();
  }
  else
  {
      Debug_Print("\r\nEEPROM ERROR\r\n");
  }

  Modem_Reset();

  LED_Init();
  uart1_last_activity = HAL_GetTick();
  rtc_last_sync = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* USER CODE END WHILE */
	/* USER CODE BEGIN 3 */
    MX_USB_HOST_Process();
    LED_Task();
    System_Mode_Task();

    if(systemMode == SYS_MODE_NORMAL)
    {
    	UART1_Process_RingBuffer();

    	// 1. Procesar buffer read CDC → generar eventos.
        Modem_Process_Buffer();

        // 2. Ejecutar FSM LTE.
        Modem_FSM_Run();

        // 3. Ejecutar FSM SMS.
        SMS_FSM_Run();

        // 5. Botón.
        Button_Process();
    }
    else if(systemMode == SYS_MODE_BRIDGE)
    {
        // 4. Bridge (debug / comandos manuales).
        Bridge_Process();
    }
    else if(systemMode == SYS_MODE_CONFIG)
    {
        // 6. Recibiendo los comandos y paquetes que vengan por UART.
    	Config_Mode_UART_TX_RX();
    }

    RS232_Monitor_Task();
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  * ??? tengo que ver cómo funciona el RTC y como se actualiza cada vez que se corta la energía y de que servidor de fecha y hora tomará ese valor.
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0xA5A5)
  {
      /* Primera vez */

      sTime.Hours   = 0;
      sTime.Minutes = 0;
      sTime.Seconds = 0;

      if(HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
      {
          Error_Handler();
      }

      sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
      sDate.Month   = RTC_MONTH_JANUARY;
      sDate.Date    = 1;
      sDate.Year    = 26;      // 2026

      if(HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
      {
          Error_Handler();
      }

      HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0xA5A5);
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C1_Init(void)
{
	/* USER CODE BEGIN MX_I2C1_Init_1 */

	/* USER CODE END MX_I2C1_Init_1 */

	hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);

	/* USER CODE BEGIN MX_I2C1_Init_2 */

	/* USER CODE END MX_I2C1_Init_2 */
}



/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WDT_PULSE_GPIO_Port, WDT_PULSE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LedRojoSIM_Pin|LedVerdeSIM_Pin|LedRojoConexion_Pin|LedVerdeConexion_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MODEM_ON_OFF_Pin|MEM_ON_OFF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SLEEP_MODEM_GPIO_Port, SLEEP_MODEM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_VERDE_Pin */
  GPIO_InitStruct.Pin = LED_VERDE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_VERDE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WDT_PULSE_Pin */
  GPIO_InitStruct.Pin = WDT_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WDT_PULSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART2_TX_Pin USART2_RX_Pin */
  GPIO_InitStruct.Pin = USART2_TX_Pin|USART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LedRojoSIM_Pin LedVerdeSIM_Pin LedRojoConexion_Pin LedVerdeConexion_Pin */
  GPIO_InitStruct.Pin = LedRojoSIM_Pin|LedVerdeSIM_Pin|LedRojoConexion_Pin|LedVerdeConexion_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MODEM_ON_OFF_Pin MEM_ON_OFF_Pin */
  GPIO_InitStruct.Pin = MODEM_ON_OFF_Pin|MEM_ON_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SLEEP_MODEM_Pin */
  GPIO_InitStruct.Pin = SLEEP_MODEM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SLEEP_MODEM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ConfigPIN_Pin */
  GPIO_InitStruct.Pin = ConfigPIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ConfigPIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C1_SCL_Pin I2C1_SDA_Pin */
  GPIO_InitStruct.Pin = I2C1_SCL_Pin|I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 *
 *void Flash_Write_Log(Modem_Error_Log_t *log_entry)
{
    HAL_FLASH_Unlock();

    // Limpiamos los flags de estado por si hubo errores previos
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = FLASH_LOG_SECTOR;
    EraseInitStruct.NbSectors = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7V a 3.6V

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK) {
        uint32_t *data_ptr = (uint32_t *)log_entry;
        uint32_t addr = FLASH_LOG_START_ADDR;

        // Calculamos cuántas palabras de 32 bits hay en la estructura
        uint32_t words = (sizeof(Modem_Error_Log_t) + 3) / 4;

        for (uint32_t i = 0; i < words; i++) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data_ptr[i]) != HAL_OK) {
                break; // Error al programar
            }
            addr += 4;
        }
    }

    HAL_FLASH_Lock();
}
 *
 */

/*
 *
 */
uint8_t Modem_Send_AT(const char *cmd)
{
    if (at_busy) return 0;

    at_busy = 1;
    USBH_CDC_Transmit(&hUsbHostFS, (uint8_t*)cmd, strlen(cmd));
    return 1;
}


/*
 * Configurar modem como: ATE1, AT+CMEE=2, AT+IPR=9600, AT&W. ???
 *
 * Nuestra implementación:
  	AT+CMEE=2
	AT+CPIN?
	AT+CSQ
	AT+CFUN?
	AT+CREG?
	AT+CEREG?
	AT+CGREG?
	AT+CPSI?
	AT+CGDCONT=1,...
	AT+NETOPEN
	AT+IPADDR
	AT+CIPOPEN=0,"UDP",...
	AT+CIPSEND...

 */

void Modem_FSM_Run(void)
{
	modem_event_t evt = EVT_NONE;
	char cmd[128];

    // Evaluación de estabilidad PDN.
	if((flags_FSM_Modem & PDN_ACTIVE) && !(flags_FSM_Modem & PDN_STABLE))
    {
        if ((HAL_GetTick() - pdn_last_event_time) > 60000)
        {
        	flags_FSM_Modem |= PDN_STABLE;
        }
    }

    if (ModemState != prevState)
    {
        prevState = ModemState;
    }

	Queue_Pop(&lte_queue, &evt);

	switch (ModemState)
	{
		// =========================================================
		case ST_IDLE:
			//Orden, Comando, Propósito Industrial:

			// 1,	ATE0,				Silencia el eco. Evita contaminar el buffer con tus propios comandos.
			// 2,	AT+CMEE=2,			"Errores en texto. Fundamental para que tu ""Caja Negra"" sea legible."
			// 3,	AT+CFUN=1,			"Fuerza funcionalidad completa. En lugar de solo consultar (?), aseguras que la radio esté activa."
			// 4,	AT+CPIN?,			"Validación de SIM. Si esto falla, no tiene sentido seguir con el resto."
			// 5,	AT+CSQ,				Nivel de señal. Verificas si hay antena antes de intentar registrarte.
			// 6,	AT+CREG?,			Registro de Red (Circuit Switched). Voz y SMS.
			// 7,	AT+CEREG?,			Registro de Datos (LTE). Es el más importante para tus paquetes UDP/TCP en el AMBA.
			// 8,	AT+CPSI?,			Diagnóstico final. Para saber exactamente en qué banda y tecnología quedó el equipo.
			// 9,	AT+CGDCONT=1...,	Perfecto. Fija el APN de Movistar.
			// 10,	AT+CIPMODE=0
			// 11,	AT+NETOPEN,			Ojo: Espera el URC +NETOPEN: 0 antes de pasar al 12.
			// 12,	AT+IPADDR,			"Validación: Si no te da una IP real, no sigas al 14."
			// 13,	AT+CIPRXGET=0
			// 14,	AT+CIPOPEN=0...,	"Conexión: Espera el +CIPOPEN: 0,0 (URC de éxito)."
			// 15,	"AT+CIPSEND=0,20...",Crítico: Espera el > antes de escupir los bytes por el UART.
			// 16,	AT+CIPCLOSE=0,		Prescindible: En UDP suele fallar o ser innecesario.
			// 17,	AT+NETCLOSE,		"Solo para dormir: Si envías datos seguido, déjalo abierto."

			if (flags_FSM_Modem & PDN_STABLE)
			{
				// Configuración inicial
				// Modem_Send_AT("ATE0\r\n");
				Modem_Send_AT("AT+CMEE=2\r\n");
				state_timer = HAL_GetTick();
				ModemState = ST_SEND_CPIN;
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}

			break;

			// =========================================================
		case ST_SEND_CPIN:
			if (flags_FSM_Modem & PDN_STABLE)
			{
				if (evt == EVT_OK)
				{
					Modem_Log(LOG_CMEE_CONFIG_OK, 2, 0);
					// Verificar SIM
					Modem_Send_AT("AT+CPIN?\r\n");
					state_timer = HAL_GetTick();
					ModemState = ST_CHECK_SIM;
				}
				else if (evt == EVT_CMEE_ERROR)
				{
					// Si el módem no soporta CMEE=2 (raro), igual intentamos seguir
					state_timer = HAL_GetTick();
					ModemState = ST_ERROR;
				}

				else if (HAL_GetTick() - state_timer > 3000)
				{
				    Modem_Log(LOG_TIMEOUT_CMEE, 0, 0);
				    ModemState = ST_ERROR;
				}
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}
			break;

		// =========================================================
		case ST_CHECK_SIM:
			if (flags_FSM_Modem & PDN_STABLE)
			{
				if (evt == EVT_CPINREADY) // +CPIN: READY
				{
					Modem_Log(LOG_SIM_READY, 0, 0);
					// Verificar calidad y nivel de señal de portadora.
					Modem_Send_AT("AT+CSQ\r\n");
					flags_FSM_Modem |= SIM_READY;
					state_timer = HAL_GetTick();
					ModemState = ST_CHECK_SIGNAL;
				}
			    else if(evt == EVT_SIM_NOT_INSERTED)
			    {
			    	state_timer = HAL_GetTick();
			        Modem_Log(LOG_SIM_NOT_INSERTED,0,0);
			        ModemState = ST_ERROR;
			        flags_FSM_Modem &= ~SIM_READY;
			        flags_FSM_Modem &= ~NETWORK_REGISTERED;
			        flags_FSM_Modem &= ~PDN_ACTIVE;
			        flags_FSM_Modem &= ~SOCKET_OPEN;
			        // ??? falta ver que hacemos con este error (Mientras tanto se queda en este estado inerte).
			        // ??? En este punto el modem debería ser capaz de ponerse en modo bridge mediante el comando +++AT.
			        // ??? También podría configurarse, leer Logs, etc..
			    }

			    else if(evt == EVT_SIM_PIN_REQUIRED)
			    {
			    	state_timer = HAL_GetTick();
			        Modem_Log(LOG_SIM_PIN_REQUIRED,0,0);
			        ModemState = ST_ERROR;
			        flags_FSM_Modem &= ~SIM_READY;
			        flags_FSM_Modem &= ~NETWORK_REGISTERED;
			        flags_FSM_Modem &= ~PDN_ACTIVE;
			        flags_FSM_Modem &= ~SOCKET_OPEN;
			        // ??? falta ver que hacemos con este error (Mientras tanto se queda en este estado inerte).
			        // ??? En este punto el modem debería ser capaz de ponerse en modo bridge mediante el comando +++AT.
			        // ??? También podría configurarse, leer Logs, etc..
			    }

			    else if(evt == EVT_SIM_PUK_REQUIRED)
			    {
			    	state_timer = HAL_GetTick();
			        Modem_Log(LOG_SIM_PUK_REQUIRED,0,0);
			        ModemState = ST_ERROR;
			        flags_FSM_Modem &= ~SIM_READY;
			        flags_FSM_Modem &= ~NETWORK_REGISTERED;
			        flags_FSM_Modem &= ~PDN_ACTIVE;
			        flags_FSM_Modem &= ~SOCKET_OPEN;
			        // ??? falta ver que hacemos con este error (Mientras tanto se queda en este estado inerte).
			        // ??? En este punto el modem debería ser capaz de ponerse en modo bridge mediante el comando +++AT.
			        // ??? También podría configurarse, leer Logs, etc..
			    }

				else if ((HAL_GetTick()-state_timer) > 5000)
				{
					// No CPIN en 5s: reiniciar módulo
					Modem_Log(LOG_TIMEOUT_CPIN,0,0);
					ModemState = ST_ERROR;
			        // ??? falta ver que hacemos con este error (Mientras tanto se queda en este estado inerte).
			        // ??? En este punto el modem debería ser capaz de ponerse en modo bridge mediante el comando +++AT.
			        // ??? También podría configurarse, leer Logs, etc..
				}
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}
			break;

		// =========================================================
		case ST_CHECK_SIGNAL:
			if (flags_FSM_Modem & PDN_STABLE)
			{
				if (evt == EVT_CSQ) // +CSQ: 28,99
				{
					// El Parser ya llenó 'last_rssi_dbm' por nosotros.
					// Solo tomamos la decisión lógica.
					if (last_rssi_dbm > -95) // Señal aceptable (aprox RSSI > 9)
					{
						Modem_Send_AT("AT+CFUN?\r\n");
						state_timer = HAL_GetTick();
						ModemState = ST_CHECK_CFUN;
						Modem_Log(LOG_SIGNAL_OK, (uint32_t)last_rssi_dbm, 0);
						// No saltamos de estado aún, esperamos el OK del comando
					}
					else
					{
						Modem_Log(LOG_SIGNAL_LOW, (uint32_t)last_rssi_dbm, 0);
						ModemState = ST_IDLE;
					}
				}
				else if ((HAL_GetTick()-state_timer) > 2500)
				{
					Modem_Reset();
					ModemState = ST_IDLE;
				}
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}
			break;

		// =========================================================
		case ST_CHECK_CFUN:
			if (flags_FSM_Modem & PDN_STABLE)
			{
				if (evt == EVT_CFUN)
				{
					if (last_cfun_value == 1)
					{
						Modem_Log(LOG_CFUN_OK, 0, 0);
						// TODO PERFECTO: Procedemos a ver si hay red
						Modem_Send_AT("AT+CREG?\r\n");
						state_timer = HAL_GetTick();
						ModemState = ST_CHECK_CSREG;
					} else
					{
						Modem_Log(LOG_CFUN_ERR, (uint32_t)last_cfun_value, 0);
						// El módem respondió OK pero está en modo avión (CFUN: 4) o similar
						// Podríamos intentar forzarlo con AT+CFUN=1 o ir a ERROR
						//Modem_Send_AT("AT+CFUN=1\r\n");
						Modem_Reset();
						ModemState = ST_IDLE;
					}
				}
				else if ((HAL_GetTick()-state_timer) > 2500)
				{
					Modem_Reset();
					ModemState = ST_IDLE;
				}
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}
			break;

		// =========================================================
		case ST_CHECK_CSREG:
			if (flags_FSM_Modem & PDN_STABLE)
			{
				if (evt == EVT_CREG_REGISTERED) // +CREG: 0,1
				{
					// Solo avanzamos si recibimos el OK Y el registro fue exitoso
                    Modem_Send_AT("AT+CEREG?\r\n"); // Chequeo de red de datos (LTE)
					state_timer = HAL_GetTick();
					ModemState = ST_CHECK_PSREG;
				}
				else if (evt == EVT_CREG_SEARCHING) // +CREG: 0,2
				{
					// Si llegó el OK pero no estamos registrados, reintentamos en 2 segundos
					// Esto evita saturar el UART pero mantiene la búsqueda activa
					Modem_Log(LOG_WAIT_REG, 0, 0);
					HAL_Delay(2000); // Pequeña espera no bloqueante o vía timer
					Modem_Send_AT("AT+CREG?\r\n");
				}
				else if (evt == EVT_CREG_DENIED) // +CREG: 0,2
				{
					// Si la torre nos rechaza (ej. SIM sin crédito o bloqueada)
					Modem_Log(LOG_REG_DENIED, 0, 0);
					ModemState = ST_ERROR;
				}
				else if ((HAL_GetTick()-state_timer) > 5000)
				{
					Modem_Log(LOG_TIMEOUT_REG, 0, 0);
					Modem_Reset();
					ModemState = ST_IDLE;
				}
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}
			break;

		// =========================================================
		case ST_CHECK_PSREG:
			if (flags_FSM_Modem & PDN_STABLE)
			{
				if (evt == EVT_CEREG_REGISTERED) // +CEREG: 0,1
				{
					Modem_Log(LOG_NETWORK_REGISTERED, 0, 0);
					// ¡Éxito! Avanzamos al siguiente comando de tu lista: CGREG
					// (O a CPSI si decides que ya es suficiente con CEREG)
					Modem_Send_AT("AT+CGREG?\r\n");
					flags_FSM_Modem |= NETWORK_REGISTERED;
					state_timer = HAL_GetTick();
					ModemState = ST_CHECK_GPRSREG; // Tu paso 6
				}
				else if (evt == EVT_CEREG_SEARCHING) // +CEREG: 0,2
				{
					// Si llegó el OK pero no estamos registrados, reintentamos en 2 segundos
					// Esto evita saturar el UART pero mantiene la búsqueda activa
					Modem_Log(LOG_WAIT_REG, 0, 0);
					HAL_Delay(2000); // Pequeña espera no bloqueante o vía timer
					Modem_Send_AT("AT+CEREG?\r\n");
					flags_FSM_Modem &= ~NETWORK_REGISTERED;
					flags_FSM_Modem &= ~PDN_ACTIVE;
					flags_FSM_Modem &= ~SOCKET_OPEN;
				}
				else if (evt == EVT_CEREG_DENIED) // +CEREG: 0,2
				{
					// Error grave: la red rechaza el equipo (posible problema de plan de datos o IMEI)
					Modem_Log(LOG_NETWORK_LOST,0,0);
	                ModemState = ST_ERROR;					// Si la torre nos rechaza (ej. SIM sin crédito o bloqueada)
	                flags_FSM_Modem &= ~NETWORK_REGISTERED;
	                flags_FSM_Modem &= ~PDN_ACTIVE;
	                flags_FSM_Modem &= ~SOCKET_OPEN;
				}
				else if ((HAL_GetTick()-state_timer) > 5000)
				{
					// Timeout de seguridad (LTE puede ser lento en zonas rurales)
					Modem_Log(LOG_NETWORK_LOST,0,0);
					Modem_Send_AT("AT+CREG?\r\n");
					state_timer = HAL_GetTick();
					ModemState = ST_CHECK_CSREG;
					flags_FSM_Modem &= ~NETWORK_REGISTERED;
					flags_FSM_Modem &= ~PDN_ACTIVE;
					flags_FSM_Modem &= ~SOCKET_OPEN;
				}
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}
			break;

		// =========================================================
		case ST_CHECK_GPRSREG:
			if (flags_FSM_Modem & PDN_STABLE)
			{
				if (evt == EVT_CGREG_REGISTERED) // +CGREG: 0,1
				{
					Modem_Log(LOG_GPRS_READY, 0, 0);
					// Con que uno de los dos (LTE o GPRS) esté registrado,
					// el equipo ya tiene salida a internet.
					Modem_Send_AT("AT+CTZU=1\r");
					state_timer = HAL_GetTick();
					ModemState = ST_ENABLE_TIME_SYNC;
				}
				else if (evt == EVT_CGREG_SEARCHING) // +CGREG: 0,2
				{
					// Si llegó el OK pero no estamos registrados, reintentamos en 2 segundos
					// Esto evita saturar el UART pero mantiene la búsqueda activa
					Modem_Log(LOG_WAIT_REG, 0, 0);
					HAL_Delay(2000); // Pequeña espera no bloqueante o vía timer
					Modem_Send_AT("AT+CEREG?\r\n");
				}
				else if (evt == EVT_CGREG_DENIED) // +CGREG: 0,2
				{
					// Error grave: la red rechaza el equipo (posible problema de plan de datos o IMEI)
	                Modem_Log(LOG_REG_DENIED, 0, 0);
	                ModemState = ST_ERROR;					// Si la torre nos rechaza (ej. SIM sin crédito o bloqueada)
				}
				else if ((HAL_GetTick()-state_timer) > 5000)
				{
					// Timeout de seguridad (LTE puede ser lento en zonas rurales)
					Modem_Log(LOG_TIMEOUT_LTE, 0, 0);
					Modem_Reset();
					ModemState = ST_ERROR;
				}
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}
			break;

		// =========================================================
		case ST_ENABLE_TIME_SYNC:
			if (flags_FSM_Modem & PDN_STABLE)
			{
			    if(evt == EVT_OK)
			    {
			    	Modem_Send_AT("AT+CCLK?\r");
			    	ModemState = ST_GET_TIME;
			    }

			    else if ((HAL_GetTick()-state_timer) > 5000)
			    {
			    	Modem_Send_AT("AT+CCLK?\r");
			    	ModemState = ST_GET_TIME;
			    }
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}
			break;

		case ST_GET_TIME:
			if (flags_FSM_Modem & PDN_STABLE)
			{
			    if(evt == EVT_TIME_SYNC_OK)
			    {
			        Modem_Log(LOG_RTC_SYNC_OK,0,0);
					Modem_Send_AT("AT+CPSI?\r\n"); // ??? ¿Implementamos CPSITD?
					state_timer = HAL_GetTick();
					ModemState = ST_CHECK_CPSI;
			    }
			    else if ((HAL_GetTick()-state_timer) > 5000)
			    {
			        Modem_Log(LOG_RTC_SYNC_TIMEOUT,0,0);
					Modem_Send_AT("AT+CPSI?\r\n"); // ??? ¿Implementamos CPSITD?
					state_timer = HAL_GetTick();
					ModemState = ST_CHECK_CPSI;
			    }
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}
			break;

		// =========================================================
		case ST_CHECK_CPSI:
			if (flags_FSM_Modem & PDN_STABLE)
			{
				if (evt == EVT_CPSI_READY)
				{
					// Logueamos la red para el reporte técnico
				    Modem_Log(LOG_NET_DIAG, lte_diag.cell_id, 0);
				    // Aquí podrías imprimir lte_diag.band por el puerto serial de debug.
				    // Siguiente paso: Configuración del APN de Movistar
					Modem_Send_AT("AT+CGDCONT=1,\"IP\",\"internet.movistar.org.ar\"\r\n");
					state_timer = HAL_GetTick();
					ModemState = ST_ACTIVATE_PDP;
				}
				else if (evt == EVT_OK)
				{
					// Finalizó la respuesta. Pasamos al último paso de init.
					Modem_Send_AT("AT+CGDCONT=1,\"IP\",\"internet.movistar.org.ar\"\r\n");
					state_timer = HAL_GetTick();
					ModemState = ST_ACTIVATE_PDP;
				}
				else if (evt == EVT_ERROR)
				{
				}
				else if ((HAL_GetTick()-state_timer) > 5000)
				{
					// Si el módem se cuelga procesando el CPSI
					Modem_Send_AT("AT+CMEE=2\r\n");
					//ModemState = ST_SET_ERROR_FORMAT;
				}
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}
			break;

		// =========================================================
		case ST_ACTIVATE_PDP:
			if (flags_FSM_Modem & PDN_STABLE)
			{
				if (evt == EVT_OK)
				{
					Modem_Log(LOG_APN_OK, 0, 0);
					// PASO 11: Abrir la pila de red (Network Open)
					Modem_Send_AT("AT+NETOPEN\r\n");
					state_timer = HAL_GetTick();
					ModemState = ST_NETOPEN_WAIT;
				}
				else if (evt == EVT_ERROR)
				{
					// Si falla aquí, suele ser porque el módem está ocupado
					// o el formato del APN es inválido.
					Modem_Log(LOG_APN_ERR, 0, 0);
					state_timer = HAL_GetTick();
					Modem_Reset();
					ModemState = ST_ERROR;
				}
				else if ((HAL_GetTick()-state_timer) > 5000)
				{
					// 150s sin respuesta, reiniciar
					Modem_Log(LOG_TIMEOUT_APN, 0, 0);
					Modem_Reset();
					ModemState = ST_IDLE;
				}
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}
			break;

		// =========================================================
		case ST_NETOPEN_WAIT:
			if (flags_FSM_Modem & PDN_STABLE)
			{
				if (evt == EVT_OK)
				{
					// El módem aceptó el comando, pero seguimos esperando el URC.
					Modem_Log(LOG_NETOPEN_START, 0, 0);
				}
				else if (evt == EVT_NETOPEN_SUCCESS)
				{
					// ¡Pila IP abierta con éxito!
					Modem_Log(LOG_NETOPEN_OK, 0, 0);
					Modem_Log(LOG_DNS_START,0,0);
					snprintf(cmd, sizeof(cmd), "AT+CDNSGIP=\"%s\"\r\n", runtime_cfg.servers[0].server_domain);
					Modem_Send_AT(cmd);
					dns_pending = 1;
					state_timer = HAL_GetTick();
					ModemState = ST_GET_DNS;
				}
				else if (evt == EVT_NETOPEN_FAILED)
				{
					// La red rechazó la apertura (posible problema de APN o señal)
					Modem_Log(LOG_NETOPEN_ERR, 1, 0);
					ModemState = ST_ERROR;
				}
				else if (evt == EVT_ERROR)
				{
					// Si el comando AT+NETOPEN ya devuelve ERROR de entrada
					Modem_Log(LOG_NETOPEN_CMD_ERR, 0, 0);
					ModemState = ST_ERROR;
				}
				else if ((HAL_GetTick()-state_timer) > 15000)
				{
					// Timeout socket open
					Modem_Log(LOG_TIMEOUT_NETOPEN, 0, 0);
					ModemState = ST_IDLE;
					flags_FSM_Modem &= ~NET_OPEN;
					flags_FSM_Modem &= ~SOCKET_OPEN;
					Modem_Reset();
				}
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}
			break;

		// =========================================================
		case ST_GET_DNS:
			if (flags_FSM_Modem & PDN_STABLE)
			{
				if (evt == EVT_DNS_OK)
				{
				    Modem_Log(LOG_DNS_OK,0,0);

				    // PASO 12: Pedir la dirección IP asignada
				    flags_FSM_Modem |= SOCKET_OPEN;
					msg_index = 0;
					retry_count = 0;
					ip_received = 0;
					dns_pending = 0;
					Modem_Send_AT("AT+IPADDR\r\n");
					state_timer = HAL_GetTick();
					ModemState = ST_GET_IP_WAIT;
				}
				else if ((evt == EVT_DNS_ERROR) || (evt == EVT_CMEE_ERROR))
				{
				    Modem_Log(LOG_DNS_ERROR,4,0);
				    ModemState = ST_ERROR;
				}
				else if ((HAL_GetTick()-state_timer) > 10000)
				{
					// Timeout socket open
					Modem_Log(LOG_TIMEOUT_DNS, 0, 0);
					ModemState = ST_IDLE;
					flags_FSM_Modem &= ~NET_OPEN;
					flags_FSM_Modem &= ~SOCKET_OPEN;
					Modem_Reset();
				}
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}
			break;

		// =========================================================
		case ST_GET_IP_WAIT:
			if (flags_FSM_Modem & PDN_STABLE)
			{
				if (evt == EVT_OK)
				{
					if (ip_received)
					{
						Modem_Log(LOG_IP_ASSIGNED, 0, 0); // Opcional: loguear la IP
						Modem_Log(LOG_UDP_OPEN_START, 0, 0);
						// PASO 14: Abrir Socket UDP
						// Usamos los parámetros que definiste: ID 0, UDP, IP destino y Puerto
						// AT+CIPOPEN=0,"UDP","190.111.217.188",57777,0
						// Modem_Send_AT("AT+CIPOPEN=0,\"UDP\",\"190.111.217.188\",57777,0\r\n");
					    snprintf(cmd, sizeof(cmd), "AT+CIPOPEN=0,\"UDP\",\"%s\",%u,0\r\n", runtime_cfg.servers[0].ip, runtime_cfg.servers[0].port);
					    Modem_Send_AT(cmd);
						state_timer = HAL_GetTick();
						ModemState = ST_UDP_OPEN_WAIT;
					}
					else
					{
						// Si llegó el OK pero no la IP, el túnel no está listo
						Modem_Log(LOG_IP_NOT_FOUND, 0, 0);
						ModemState = ST_ERROR;
					}
				}
				else if (evt == EVT_IP_READY)
				{
					// Ya tenemos la IP en la variable global
					if (strcmp(local_ip, "0.0.0.0") == 0) ip_received = 0;
					else ip_received = 1;
				}
				else if (evt == EVT_ERROR)
				{
					Modem_Log(LOG_IP_CMD_ERR, 0, 0);
					ModemState = ST_ERROR;
				}

				else if (HAL_GetTick() - state_timer > 5000)
				{
					Modem_Log(LOG_TIMEOUT_IP, 0, 0);
					ModemState = ST_ERROR;
				}
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}
			break;

		// =========================================================
		case ST_UDP_OPEN_WAIT:
			if (flags_FSM_Modem & PDN_STABLE)
			{
				if (evt == EVT_OK)
				{
					// El módem aceptó la orden de abrir el socket UDP.
					state_timer = HAL_GetTick();
					ModemState = ST_WAIT_FOR_PROMPT;
				}
				else if (evt == EVT_UDP_OPEN_SUCCESS)
				{
					// ¡Socket abierto! El túnel hacia 190.111.217.188 está listo.
					Modem_Log(LOG_UDP_OPEN_OK, 0, 0);

					// PASO 15: Enviar el comando de transmisión
					// Solicitamos enviar 20 bytes al destino
					// AT+CIPSEND=0,20,"190.111.217.188",57777
					// Modem_Send_AT("AT+CIPSEND=0,20,\"190.111.217.188\",57777\r\n");
					// ??? La primera vez debe enviarse un Heartbeat de UDP.

					char payload_len = 20;
				    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=0,%u,\"%s\",%u\r\n", payload_len, runtime_cfg.servers[0].ip, runtime_cfg.servers[0].port);

				    Modem_Send_AT(cmd);

					state_timer = HAL_GetTick();
					ModemState = ST_WAIT_FOR_PROMPT;
				}
				else if (evt == EVT_UDP_OPEN_FAILED)
				{
					// Error en la apertura (ej. puerto cerrado o red bloqueada)
					Modem_Log(LOG_UDP_OPEN_ERR, (uint32_t)last_udp_err, 0);
					ModemState = ST_ERROR;
				}
				else if (evt == EVT_ERROR)
				{
					// Error inmediato en el comando AT+CIPOPEN
					Modem_Log(LOG_UDP_CMD_ERR, 0, 0);
					ModemState = ST_ERROR;
				}
				else if (HAL_GetTick() - state_timer > 10000)
				{
					Modem_Log(LOG_TIMEOUT_UDP, 0, 0);
					ModemState = ST_ERROR;
				}
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}
			break;

		// =========================================================
		case ST_WAIT_FOR_PROMPT:
			if (flags_FSM_Modem & PDN_STABLE)
			{
				if (evt == EVT_OK)
				{
					// EL MOMENTO CRÍTICO: Enviamos los 20 bytes exactos.
					// Usamos Transmit bloqueante por ser pocos bytes y requerir precisión.
				    USBH_CDC_Transmit(&hUsbHostFS, (uint8_t*)alarm_messages[msg_index], strlen(alarm_messages[msg_index]));
					state_timer = HAL_GetTick();
					ModemState = ST_WAIT_SEND_OK;
				}
				else if (evt == EVT_ERROR)
				{
					Modem_Log(LOG_SEND_REJECTED, 0, 0);
					ModemState = ST_ERROR;
				}
				else if (HAL_GetTick() - state_timer > 5000)
				{
					Modem_Log(LOG_TIMEOUT_PROMPT, 0, 0);
				    ModemState = ST_ERROR;
				}
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}
			break;


		// =========================================================
		case ST_WAIT_SEND_OK:
			if (flags_FSM_Modem & PDN_STABLE)
			{
				if (evt == EVT_OK)
				{
					// El módem dice: "Recibí los 20 bytes y los puse en la cola de salida"
					Modem_Log(LOG_SEND_OK, 0, 0);

					state_timer = HAL_GetTick();
					ModemState = ST_WAIT_SEND_OK;
				}
				else if (evt == EVT_ERROR)
				{
					Modem_Log(LOG_SEND_ERR, 0, 0);
					ModemState = ST_ERROR;
				}
				else if (HAL_GetTick() - state_timer > 10000)
				{
					Modem_Log(LOG_TIMEOUT_SEND, 0, 0);
				    ModemState = ST_ERROR;
				}
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}
			break;


		// =========================================================
			/*
		case ST_NETCLOSE_WAIT:
			if (flags_FSM_Modem & PDN_STABLE)
			{
				if (evt == EVT_OK)
				{
				}
				else if (evt == EVT_ERROR)
				{
				}
				else if (HAL_GetTick() - state_timer > 5000)
				{
				}
			}
			else
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_IDLE;
			}
			break;
			*/

		// =========================================================
		case ST_READY:
		    if(evt == EVT_SOCKET_LOST)
		    {
				Modem_Send_AT("AT+NETOPEN\r\n");
				state_timer = HAL_GetTick();
				ModemState = ST_NETOPEN_WAIT;
				flags_FSM_Modem &= ~SOCKET_OPEN;
		    }
		    else if(!(flags_FSM_Modem & NETWORK_REGISTERED))
		    {
				Modem_Send_AT("AT+CREG?\r\n");
				state_timer = HAL_GetTick();
				ModemState = ST_CHECK_CSREG;
		    }
		    else if (flags_FSM_Modem & PDN_STABLE)
			{
				// Espera petición de envío (p.ej. botón o evento SMS)
				if  (send_request && (flags_FSM_Modem & NET_OPEN) && (flags_FSM_Modem & SOCKET_OPEN))
				{
					send_request = 0;
					ModemState = ST_SEND;
				}

				if((HAL_GetTick() - rtc_last_sync) > 3600000UL)
			    {
					rtc_last_sync = HAL_GetTick();
			    	Modem_Send_AT("AT+CCLK?\r");
			    }
			}
			else // ??? ¿Hay que agregar algo más acá?
			{
				// El socket ya no sirve. La IP probablemente tampoco. El contexto LTE quedó invalidado.
		        ModemState = ST_CLOSE;
			}
			break;

		// =========================================================
		case ST_SEND:
			if (msg_index < 10)
			{
				// Preparar y enviar AT+CIPSEND.
				snprintf(tx_buffer,sizeof(tx_buffer) ,"%d|%s", msg_index, alarm_messages[msg_index]);
				//snprintf(cmd,sizeof(cmd) ,"AT+CIPSEND=0,%d,\"190.111.217.188\",57777\r\n",strlen(tx_buffer));

			    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=0,%u,\"%s\",%u\r\n", strlen(tx_buffer), runtime_cfg.servers[0].ip, runtime_cfg.servers[0].port);

				ModemState = ST_WAIT_TX;
				state_timer = HAL_GetTick();
			    Modem_Send_AT(cmd);
			}
			else
			{
				ModemState = ST_CLOSE;
			}
			break;

		// =========================================================
		case ST_WAIT_TX:

			if (evt == EVT_PROMPT)
			{
				USBH_CDC_Transmit(&hUsbHostFS, (uint8_t*)tx_buffer, strlen(tx_buffer));
			}
			else if (evt == EVT_SEND_OK)
			{
				msg_index++;
				retry_count = 0;
				ModemState = ST_SEND;
			}
			else if ((evt == EVT_ERROR) || (HAL_GetTick() - state_timer) > 5000)
			{
				retry_count++;

				if (retry_count < 3)
				{
					ModemState = ST_SEND;
				}
				else
				{
					ModemState = ST_ERROR;
				}
			}
			break;

		// =========================================================
		case ST_CLOSE:
			// Cerrar socket y PDP
			Modem_Send_AT("AT+CIPCLOSE=0\r\n");
			state_timer = HAL_GetTick();
			flags_FSM_Modem &= ~SOCKET_OPEN;
			flags_FSM_Modem &= ~NET_OPEN;
			ModemState = ST_NETCLOSE;
			break;

		// =========================================================
		case ST_NETCLOSE:
			// Cerrar socket y PDP
			if ((HAL_GetTick()-state_timer) > 1500)
			{
				Modem_Send_AT("AT+NETCLOSE\r\n");
				flags_FSM_Modem &= ~SOCKET_OPEN;
				flags_FSM_Modem &= ~NET_OPEN;
				ModemState = ST_NETCLOSE_WAIT;
			}
			break;

		// =========================================================
		case ST_NETCLOSE_WAIT:
			if (evt == EVT_OK)
			{
				// El módem aceptó la orden de cierre
				Modem_Log(LOG_NETCLOSE_START, 0, 0);
				ModemState = ST_IDLE;
			}
			else if (evt == EVT_NETCLOSE_SUCCESS)
			{
				Modem_Log(LOG_NETCLOSE_OK, 0, 0);
				// ¡Misión cumplida! Volvemos al estado de espera
				// Aquí podrías apagar el módem o entrar en modo Low Power
				ModemState = ST_IDLE;
				state_timer = HAL_GetTick();
			}
			else if (evt == EVT_NETCLOSE_FAILED)
			{
				Modem_Log(LOG_NETCLOSE_ERR, 1, 0);
				// Si falla el cierre, a veces es mejor forzar un Reset
				// para asegurar que la próxima vez arranque de cero.
				ModemState = ST_ERROR;
			}
			else if (HAL_GetTick() - state_timer > 5000)
			{
				Modem_Log(LOG_TIMEOUT_NETCLOSE, 0, 0);
				ModemState = ST_IDLE; // Forzamos IDLE de todas formas
			}
			break;

		// =========================================================
		case ST_ERROR:
			dns_pending = 0;
			flags_FSM_Modem &= ~SOCKET_OPEN;
			flags_FSM_Modem &= ~NET_OPEN;
			msg_index = 0;
			retry_count = 0;
			flags_FSM_Modem &= ~NETWORK_REGISTERED;
			Modem_Send_AT("AT+NETCLOSE\r\n");
			ModemState = ST_IDLE;
			break;

		// =========================================================
		case ST_SET_MODE:
		    break;
	}
}


/*
 *
 */
void SMS_FSM_Run(void)
{
	modem_event_t evt = EVT_NONE;
	char cmd[32];

    Queue_Pop(&sms_queue, &evt); //while (PopEvent(&evt))

	switch (smsState)
	{
		case SMS_IDLE:

			if (evt == EVT_SMS_RECEIVED)
			{
				if (sms_index >= 0)
				{
					snprintf(cmd,sizeof(cmd) ,"AT+CMGR=%d\r\n", sms_index);
					Modem_Send_AT(cmd);

					smsState = SMS_READ;
				}
			}
			break;

		case SMS_READ:

			if (evt == EVT_SMS_READ)
			{
				smsState = SMS_PARSE;
			}
			break;

		case SMS_PARSE:

			if (strlen(sms_text) == 0)
			{
				smsState = SMS_DELETE;
				break;
			}

			// -------------------------
			// COMANDOS
			// -------------------------

			if (strcmp(sms_text, "RESET") == 0)
			{
				//Modem_Send_AT("AT+CFUN=1,1\r\n");
				Modem_Reset();
				smsState = SMS_IDLE;
			}
			else if (strcmp(sms_text, "ACT1") == 0)
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				smsState = SMS_EXECUTE;
			}
			else if (strcmp(sms_text, "ACT2") == 0)
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
				smsState = SMS_EXECUTE;
			}
			else if (strncmp(sms_text, "ECHO", 5) == 0)
			{
				// responder con el mismo mensaje.
				snprintf(cmd,sizeof(cmd) ,"AT+CMGS=\"%s\"\r\n", sms_sender);
				Modem_Send_AT(cmd);
				smsState = SMS_EXECUTE;

				// ojo: esperar EVT_PROMPT
				// lo manejamos abajo
			}
			if (strcmp(sms_text, "SEND") == 0)
			{
				send_request = 1;
				smsState = SMS_IDLE;
			}
			break;

		case SMS_EXECUTE:

			if (evt == EVT_PROMPT)
			{
				// enviar ACK o respuesta
				char msg[160];

				snprintf(msg,sizeof(msg),"ACK: %s", sms_text);

				USBH_CDC_Transmit(&hUsbHostFS,
					(uint8_t*)msg,
					strlen(msg));

				// CTRL+Z
				uint8_t ctrlz = 0x1A;
				USBH_CDC_Transmit(&hUsbHostFS, &ctrlz, 1);

				smsState = SMS_DELETE;
			}
			else if (evt == EVT_OK)
			{
				smsState = SMS_DELETE;
			}
			break;

		case SMS_DELETE:

			if (sms_index >= 0)
			{
				snprintf(cmd,sizeof(cmd), "AT+CMGD=%d\r\n", sms_index);
				Modem_Send_AT(cmd);
			}

			sms_index = -1;
			sms_ready = 0;
			memset(sms_text, 0, sizeof(sms_text));

			smsState = SMS_IDLE;
			break;
    }
}

/*
 *
 */
void Bridge_Process(void)
{
    uint8_t c;

    /*----------------------------------------------------
      CDC -> UART
    ----------------------------------------------------*/
    if (cdc_accum_len > 0 && (HAL_GetTick() - cdc_last_rx_tick) >= CDC_FLUSH_TIMEOUT_MS)
    {
        uint16_t sent = 0;

        while(sent < cdc_accum_len)
        {
            uint16_t chunk = cdc_accum_len - sent;
            if(chunk > CDC_CHUNK_SIZE) chunk = CDC_CHUNK_SIZE;
            HAL_UART_Transmit(&huart1, cdc_accum_buf + sent, chunk, 100);
            sent += chunk;
            MX_USB_HOST_Process();
        }

        cdc_accum_len = 0;
    }

    /*-----------------------------------------
      UART -> LINE BUFFER
    -----------------------------------------*/
    while(RingBufferUart_Get(&uart1Buffer, &c))
    {
        if(lineIndexUart1 < (sizeof(lineBufferUart1) - 2))
        {
            lineBufferUart1[lineIndexUart1++] = c;
        }

        /* Línea completa */
        if(c == '\r')
        {
            lineBufferUart1[lineIndexUart1++] = '\n';
            lineBufferUart1[lineIndexUart1] = '\0';

            bridge_last_activity = HAL_GetTick();

            /* Detectar salida de bridge */
            if(strstr(lineBufferUart1, "+++AT") || strstr(lineBufferUart1, "+++at"))
            {
                systemMode = SYS_MODE_NORMAL;
                Debug_Print("\r\n[MODO] NORMAL\r\n");
                RingBufferUart_Clear(&uart1Buffer);
                lineIndexUart1 = 0;
                lineBufferUart1[0] = 0;
                break;
            }
            if(strstr(lineBufferUart1, "$CFG") || strstr(lineBufferUart1, "$cfg"))
            {
                systemMode = SYS_MODE_CONFIG;
                Debug_Print("\r\n[MODO] CONFIG\r\n");
                RingBufferUart_Clear(&uart1Buffer);
                lineIndexUart1 = 0;
                lineBufferUart1[0] = 0;
                break;
            }
            else if(Appli_state == APPLICATION_READY)
            {
                /*-----------------------------------------
                  UART -> CDC  Enviar línea completa por CDC.
                -----------------------------------------*/
                USBH_CDC_Transmit(&hUsbHostFS, (uint8_t *)lineBufferUart1, lineIndexUart1);
                lineIndexUart1 = 0;
                lineBufferUart1[0] = 0;
            }
        }
    }
}


/*
 *ConfigFSM_t cfgModeState = CFG_WAIT_START;
	Opción 1: Leer configuración
	PC:
	$CFG,READ,9999*CRC
	Respuesta:
	$CFG,DATA,<length>,{JSON}*CRC

	Opción 2: Escribir configuración
	PC:
	$CFG,WRITE,9999,<length>,{JSON}*CRC
	Respuesta:
	$CFG,WRITE,OK*CRC

	Opción 3: Guardar en FLASH
	PC:
	$CFG,SAVE,9999*CRC
	Respuesta:
	$CFG,SAVE,OK*CRC

	Opción 4: Restaurar valores de fábrica
	PC:
	$CFG,DEFAULT,9999*CRC
	Respuesta:
	$CFG,DEFAULT,OK*CRC

	Opción 5: Reiniciar módem
	PC:
	$CFG,REBOOT,9999*CRC
	Respuesta:
	$CFG,REBOOT,OK*CRC

	Opción 6: Reiniciar modulito módem
	PC:
	$CFG,MODEMREINIT,9999*CRC
	Respuesta:
	$CFG,MODEMREINIT,OK*CRC

	Opción 7: Salir

 *
 */
void Config_Mode_UART_TX_RX(void)
{
    uint8_t c;
    config_command_t cmd;

    while(RingBufferUart_Get(&uart1Buffer, &c))
    {
        switch(cfgModeState)
        {
            case CFG_WAIT_START:

                if(c == '$')
                {
                    cfg_rx_index = 0;

                    cfg_rx_buffer[cfg_rx_index++] = c;

                    cfgModeState = CFG_RECEIVE_FRAME;
                }

                break;

            case CFG_RECEIVE_FRAME:

                if(cfg_rx_index < (CFG_RX_BUF_SIZE - 1))
                {
                    cfg_rx_buffer[cfg_rx_index++] = c;

                    if(c == '\n')
                    {
                        cfg_rx_buffer[cfg_rx_index] = 0;

                        cfgModeState = CFG_VERIFY_FORMAT;
                    }
                }
                else
                {
                    cfg_rx_index = 0;
                    cfgModeState = CFG_WAIT_START;
                }

                break;

            default:
                break;
        }
    }

    switch(cfgModeState)
    {
        case CFG_VERIFY_FORMAT:

            if(Verify_Frame_Format())
            {
                cfgModeState = CFG_VERIFY_CRC;
            }
            else
            {
                Send_Error("FORMAT");

                cfg_rx_index = 0;
                cfgModeState = CFG_WAIT_START;
            }

            break;

        case CFG_VERIFY_CRC:

            if(Verify_Frame_CRC())
            {
                cfgModeState = CFG_VERIFY_PASSWORD;
            }
            else
            {
                Send_Error("CRC");

                cfg_rx_index = 0;
                cfgModeState = CFG_WAIT_START;
            }

            break;

        case CFG_VERIFY_PASSWORD:

            if(Verify_Password())
            {
                cfgModeState = CFG_PARSE_COMMAND;
            }
            else
            {
                Send_Error("PASSWORD");

                cfg_rx_index = 0;
                cfgModeState = CFG_WAIT_START;
            }

            break;

        case CFG_PARSE_COMMAND:

            cmd = Parse_Command();

            switch(cmd)
            {
                case CMD_READ:

                    CMD_Read_Config();
                    break;

                case CMD_WRITE:

                    CMD_Write_Config();
                    break;

                case CMD_SAVE:

                    CMD_Save_Config();
                    break;

                case CMD_REBOOT:

                    CMD_Reboot();
                    break;

                case CMD_MODEMREINIT:

                    Modem_Reset();
                    Send_ACK("REBOOT");
                    break;

                case CMD_DEFAULT:

                    CMD_Default_Config();
                    break;

                case CMD_LOG_INFO:

                	CMD_Log_Info();
                    break;

                case CMD_LOG_READ:

                    uint16_t index;

                    index = Get_LogRead_Index();

                    if(index == 0xFFFF)
                    {
                        Send_Error("LOGINDEX");
                    }
                    else
                    {
                        CMD_Log_Read(index);
                    }
                    break;

                case CMD_LOG_CLEAR:

                	CMD_Log_Clear();
                    break;

                default:

                    Send_Error("INVALIDO");
                    break;
            }

            cfg_rx_index = 0;
            cfgModeState = CFG_WAIT_START;

            break;

        default:
            break;
    }
}

/*
 *
 */
void Button_Process(void)
{
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
    {
    	if (ModemState == ST_READY)
        {
    		send_request = 1;
        }

    	LED_Set(&ledVerdeBlackPill, LED_ON_TIMER, 3000); //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }
}

/**
 * @brief Realiza un pulso de reset físico al módem A7670
 * Utiliza el pin PB12 (conectado al RESET o PWRKEY según tu hardware)
 */
void Modem_Reset(void)
{
    // 1. Log del evento (opcional)
    Modem_Log(LOG_MODEM_RESET, 0, 0);

    // 2. Pulso de Reset físico
	HAL_GPIO_WritePin(GPIOB, MODEM_ON_OFF_Pin, GPIO_PIN_SET);
    HAL_Delay(500); // El A7670 requiere pulsos de al menos 500ms para asegurar el reset/pwrkey
    HAL_GPIO_WritePin(GPIOB, MODEM_ON_OFF_Pin, GPIO_PIN_RESET);

    // 3. Tiempo de gracia para que el módem reinicie el stack UART
    // Como estamos fuera de la FSM (en un evento de pánico), un Delay aquí es aceptable
    HAL_Delay(1000);

    // 4. Limpiamos buffer para ignorar la "basura" del arranque
    RingBuffer_Clear();

    // 5. Llama al reset del sistema provisto por las librerías HAL
}

/*
 *
 */
bool Modem_Get_Line(char* line)
{
    int16_t byte;

    while ((byte = RingBuffer_Read()) != -1)
    {
        char c = (char)byte;

        if (c == '\n')
        {
            lineBuffer[lineIndex] = '\0';
            strcpy(line, lineBuffer);
            lineIndex = 0;
            return true;
        }

        if (lineIndex < LINE_BUFFER_SIZE - 1)
        {
            lineBuffer[lineIndex++] = c;
        }
    }

    return false;
}

/*
 *
 */
void RingBuffer_Write(uint8_t data)
{
    uint32_t next = (modemBuffer.head + 1) % RING_BUFFER_SIZE;
    if (next != modemBuffer.tail) {
        modemBuffer.buffer[modemBuffer.head] = data;
        modemBuffer.head = next;
    }
}

/*
 *
 */
int16_t RingBuffer_Read(void)
{
    if (modemBuffer.head == modemBuffer.tail) return -1;
    uint8_t data = modemBuffer.buffer[modemBuffer.tail];
    modemBuffer.tail = (modemBuffer.tail + 1) % RING_BUFFER_SIZE;
    return data;
}

/*
 *
 */
void RingBuffer_Clear(void)
{
    modemBuffer.head = modemBuffer.tail = 0;
    searchIdx = 0;
}

/*
 *
 */
void EventQueue_Init(void)
{
	lte_queue.head = 0;
	sms_queue.tail = 0;
}

/*
 *
 */
void Queue_Push(event_queue_t *q, modem_event_t evt)
{
    uint8_t next = (q->head + 1) % QUEUE_SIZE;
    if (next != q->tail)
    {
        q->queue[q->head] = evt;
        q->head = next;
    }
}

/*
 *
 */
int Queue_Pop(event_queue_t *q, modem_event_t *evt)
{
    if (q->head == q->tail)
        return 0;

    *evt = q->queue[q->tail];
    q->tail = (q->tail + 1) % QUEUE_SIZE;
    return 1;
}


/*
 * 	EVT_NONE = 0,
    EVT_OK,
    EVT_ERROR,
    EVT_TIMEOUT,
    EVT_CREG_REGISTERED,
    EVT_CREG_SEARCHING,
	EVT_PDN_ACT,
	EVT_PDN_DEACT,
	EVT_NETWORK_DETACH,
    EVT_NETOPEN,
    EVT_NETCLOSED,
    EVT_SOCKET_OPEN,
    EVT_SOCKET_CLOSED,
    EVT_PROMPT,
    EVT_SEND_OK,
	EVT_SMS_RECEIVED,
	EVT_SMS_READ
 *
 *
 */
void Dispatch_Event(modem_event_t evt)
{
    switch (evt)
    {
        // 🔷 LTE
		case EVT_ATREADY:
		case EVT_CPINREADY:
		case EVT_SIM_NOT_INSERTED:
		case EVT_SIM_PIN_REQUIRED:
		case EVT_SIM_PUK_REQUIRED:
		case EVT_SMS_DONE:
		case EVT_CSQ:
		case EVT_CFUN:
        case EVT_OK:
        case EVT_ERROR:
        case EVT_CMEE_ERROR:
        case EVT_TIMEOUT:
        case EVT_CREG_REGISTERED:
        case EVT_CREG_SEARCHING:
        case EVT_CREG_DENIED:
        case EVT_CEREG_REGISTERED:
        case EVT_CEREG_SEARCHING:
        case EVT_CEREG_DENIED:
        case EVT_CGREG_REGISTERED:
        case EVT_CGREG_SEARCHING:
        case EVT_CGREG_DENIED:
        case EVT_TIME_SYNC_OK:
        case EVT_CPSI_READY:
        case EVT_PDN_ACT:
        case EVT_PDN_DEACT:
        case EVT_NETWORK_DETACH:
        case EVT_NETOPEN_SUCCESS:
        case EVT_DNS_OK:
        case EVT_DNS_ERROR:
        case EVT_NETOPEN_FAILED:
        case EVT_UDP_OPEN_SUCCESS:
        case EVT_UDP_OPEN_FAILED:
        case EVT_IP_READY:
        case EVT_NETCLOSE_SUCCESS:
        case EVT_NETCLOSE_FAILED:
        case EVT_SOCKET_OPEN:
        case EVT_SOCKET_CLOSED:
        case EVT_SOCKET_LOST:
        case EVT_SEND_PROMPT_READY:
        case EVT_PROMPT:
        case EVT_SEND_OK:

            Queue_Push(&lte_queue, evt);
            break;

        // 🔷 SMS
        case EVT_SMS_RECEIVED:
        case EVT_SMS_READ:

            Queue_Push(&sms_queue, evt);
            break;

        default:
            break;
    }
}

/*
 *
 */
int Parse_SMS_Index(const char* line)
{
    int idx = -1;
    sscanf(line, "+CMTI: \"SM\",%d", &idx);
    return idx;
}

/*
 *
 */
void Modem_Process_Buffer(void)
{
    char line[128];
    char dbg[130];

    if (Modem_Get_Line(line))
    {
    	at_busy = 0;
        // limpiar CR/LF
        line[strcspn(line, "\r\n")] = 0;

        if (strlen(line) < 2) return;

        // DEBUG CONTROLADO.
        #ifdef MODEM_DEBUG_LINES
        snprintf(dbg, sizeof(dbg), "\r\n[LINE]: %.100s\r\n", line);
        HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
        #endif

        // PARSER PRINCIPAL (EVENTOS)
        if (strcmp(line, "OK") == 0)
        {
    	    Dispatch_Event(EVT_OK);
        }
        // 2. NUEVO: Capturar errores detallados (Verbose)
        else if(strncmp(line, "+CME ERROR:", 11) == 0)
        {
            uint32_t last_cme_error;
            last_cme_error = strtoul(line + 11, NULL, 10);
            Modem_Log(LOG_CMEE_ERROR, last_cme_error, 0);
            Dispatch_Event(EVT_CMEE_ERROR);

            switch(last_cme_error)
            {
                case 10:
                    // SIM no insertada
                	// 10 -> "SIM_NOT_INSERTED"
                	// Agregar la tabla de traducción: const char *CME_Error_ToString(uint32_t code);
                    break;

                case 13:
                    // SIM defectuosa
                	// 13 -> "SIM_FAILURE"
                    break;

                case 30:
                    // Sin servicio
                	// 30 -> "NO_NETWORK_SERVICE"
                	// $ACK,LOGREAD,125,345600,CME_ERROR,30,NO_NETWORK_SERVICE
                    break;

                case 515:
                    // No registrado
                	// 515 -> "NOT_REGISTERED"
                    break;

                default:
                    break;
            }
        }

        else if (strstr(line, "+IPD"))
        {
            // Ejemplo: +IPD5 o +IPD,5
            // Acá podés preparar un buffer para recibir payload
            // (si querés parsear contenido entrante)
        	Dispatch_Event(EVT_DATA_RECEIVED);
            // Ejemplo debug:
            char dbg[] = "\r\n[URC]\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)dbg, sizeof(dbg)-1, 100);
        }

        else if (strncmp(line, "+CREG:", 6) == 0)
        {
            int n, stat;
            // sscanf extrae los valores sin importar espacios o variaciones
            if (sscanf(line, "+CREG: %d,%d", &n, &stat) == 2)
            {
                if (stat == 1 || stat == 5) // 1: Home, 5: Roaming (Indispensable en el Interior)
                {
                    Dispatch_Event(EVT_CREG_REGISTERED);
                }
                else if (stat == 2 || stat == 0) // 2: Searching, 0: Not searching
                {
                    Dispatch_Event(EVT_CREG_SEARCHING);
                }
                else if (stat == 3) // Registration Denied
                {
                    Dispatch_Event(EVT_CREG_DENIED);
                }
            }
        }

        else if (strncmp(line, "+CEREG:", 7) == 0)
        {
            int n, stat;
            // Formato: +CEREG: <n>,<stat>[,<tac>,<ci>,<AcT>]
            if (sscanf(line, "+CEREG: %d,%d", &n, &stat) >= 2)
            {
                if (stat == 1 || stat == 5) // 1: Home Network, 5: Roaming
                {
                    Dispatch_Event(EVT_CEREG_REGISTERED);
                }
                else if (stat == 2 || stat == 0) // Searching o Idle
                {
                    Dispatch_Event(EVT_CEREG_SEARCHING);
                }
                else if (stat == 3) // Registration Denied
                {
                    Dispatch_Event(EVT_CEREG_DENIED);
                }
            }
        }

        else if (strncmp(line, "+CGREG:", 7) == 0)
        {
            int n, stat;
            // Formato: +CGREG: <n>,<stat>[,<lac>,<ci>,<AcT>]
            if (sscanf(line, "+CGREG: %d,%d", &n, &stat) >= 2)
            {
                if (stat == 1 || stat == 5) // 1: Home Network, 5: Roaming
                {
                    Dispatch_Event(EVT_CGREG_REGISTERED);
                }
                else if (stat == 2 || stat == 0) // Searching o Idle
                {
                    Dispatch_Event(EVT_CGREG_SEARCHING);
                }
                else if (stat == 3) // Registration Denied
                {
                    Dispatch_Event(EVT_CGREG_DENIED);
                }
            }
        }

        else if(strncmp(line,"+CCLK:",6) == 0)
        {
        	Dispatch_Event(EVT_TIME_SYNC_OK);
            RTC_Set_From_CCLK(line);
        }

        else if (strncmp(line, "+CPSI:", 6) == 0)
        {
            char mode[12], mcc_mnc[12], tac[12];
            int pcid;
            // Usamos sscanf con especificadores de campo para saltar comas
            // %[^,] lee todo hasta encontrar una coma
            if (sscanf(line, "+CPSI: %[^,],%[^,],%[^,],%[^,],%lu,%d,%[^,]",
                       lte_diag.tech, mode, mcc_mnc, tac,
                       &lte_diag.cell_id, &pcid, lte_diag.band) >= 5)
            {
                // Ya tenemos los datos esenciales.
                // mcc_mnc dirá "722-07" (Movistar AR)
                // lte_diag.band dirá "EUTRAN-BAND4"
                Dispatch_Event(EVT_CPSI_READY);
            }
        }

        else if (strncmp(line, "+CSQ:", 5) == 0)
        {
			// sscanf busca el patrón numérico dentro de la línea ya capturada
			// Formato esperado: "+CSQ: 28,99"
			if (sscanf(line, "+CSQ: %d,%d", &rssi_tmp, &ber_tmp) == 2)
			{
				last_rssi_raw = (int8_t)rssi_tmp;

				// Conversión industrial a dBm (útil para tu log en Flash)
				if (last_rssi_raw != 99)
				{
					last_rssi_dbm = (last_rssi_raw * 2) - 113;
				} else
				{
					last_rssi_dbm = -113; // Valor de "sin señal"
				}
			}
			Dispatch_Event(EVT_CSQ);
        }

        else if(strncmp(line,"+CDNSGIP:",9) == 0)
        {
            Parse_DNS_Response(line);
        }

        else if (strstr(line, "+IPADDR:"))
        {
            // Buscamos la IP después del espacio
            // Formato esperado: "+IPADDR: 10.45.67.89"
            if (sscanf(line, "+IPADDR: %15s", local_ip) == 1)
            {
                Dispatch_Event(EVT_IP_READY);
            }
        }

        else if (strncmp(line, "+CFUN:", 6) == 0)
        {
            // Extraemos el número (usualmente 0, 1 o 4)
            sscanf(line, "+CFUN: %hhd", &last_cfun_value);
            Dispatch_Event(EVT_CFUN);
        }

        else if (strcmp(line, ">") == 0)
        {
        	Dispatch_Event(EVT_SEND_PROMPT_READY);
        	//Modem_Clear_Buffer();
        }

        else if (strstr(line, "RECV FROM"))
        {
            #ifdef MODEM_DEBUG_URC
            snprintf(dbg,sizeof(dbg) ,"\r\n[URC][UDP RX]: %s\r\n", line);
            HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
            #endif
        }

        else if (strncmp(line, "+NETOPEN:", 9) == 0)
        {
            int res = -1;
            sscanf(line, "+NETOPEN: %d", &res);

            if (res == 0) {
                Dispatch_Event(EVT_NETOPEN_SUCCESS);
            } else {
                Dispatch_Event(EVT_NETOPEN_FAILED);
            }
        }

        else if (strncmp(line, "+NETCLOSE:", 10) == 0)
        {
            int close_res = -1;
            // Buscamos: +NETCLOSE: 0 (Éxito) o +NETCLOSE: 1 (Error)
            if (sscanf(line, "+NETCLOSE: %d", &close_res) == 1)
            {
                if (close_res == 0)
                {
                    Dispatch_Event(EVT_NETCLOSE_SUCCESS);
                } else
                {
                    Dispatch_Event(EVT_NETCLOSE_FAILED);
                }
            }
        }

        else if (strncmp(line, "+CIPSEND:", 9) == 0)
        {
        	Dispatch_Event(EVT_SEND_OK);
        }

        else if (strncmp(line, "+CIPOPEN:", 9) == 0)
        {
            int link_id, err_code;
            // Formato: +CIPOPEN: 0,0
            if (sscanf(line, "+CIPOPEN: %d,%d", &link_id, &err_code) == 2)
            {
                if (err_code == 0)
                {
                    Dispatch_Event(EVT_UDP_OPEN_SUCCESS);
                } else
                {
                    last_udp_err = err_code;
                    Dispatch_Event(EVT_UDP_OPEN_FAILED);
                }
            }
        }

        else if (strstr(line, "+CGEV: EPS PDN ACT") || strstr(line, "+CGEV: ME PDN ACT"))
        {
    	    Dispatch_Event(EVT_PDN_ACT);
    	    Modem_Log(LOG_PDN_ACT,0,0);

			#ifdef MODEM_DEBUG_URC
            snprintf(dbg,sizeof(dbg),"[TIME %lu ms] [URC]: %s\r\n",HAL_GetTick() - boot_time,line);
            HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
			#endif

            flags_FSM_Modem &= ~PDN_STABLE;
    	    flags_FSM_Modem |= PDN_ACTIVE;
    	    pdn_last_event_time = HAL_GetTick();
        }

        else if (strstr(line, "+CGEV: NW PDN DEACT") || strstr(line, "+CGEV: ME PDN DEACT") || strstr(line, "+CGEV: PDN DEACT"))
        {
    	    Dispatch_Event(EVT_PDN_DEACT);
    	    Modem_Log(LOG_PDN_DEACT,0,0);

			#ifdef MODEM_DEBUG_URC
    	    snprintf(dbg,sizeof(dbg),"[TIME %lu ms] [URC]: %s\r\n",HAL_GetTick() - pdn_last_event_time,line);
            HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
			#endif

            flags_FSM_Modem &= ~PDN_ACTIVE;
            flags_FSM_Modem &= ~SOCKET_OPEN;
        }

        else if (strstr(line, "+CGEV: NW DETACH") || strstr(line, "+CGEV: ME DETACH"))
        {
            Dispatch_Event(EVT_NETWORK_DETACH);
            Modem_Log(LOG_NETWORK_DETACH,0,0);


			#ifdef MODEM_DEBUG_URC
    	    snprintf(dbg,sizeof(dbg),"[TIME %lu ms] [URC]: %s\r\n",HAL_GetTick() - pdn_last_event_time,line);
            HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
			#endif

            flags_FSM_Modem &= ~PDN_ACTIVE;
            flags_FSM_Modem &= ~SOCKET_OPEN;
            flags_FSM_Modem &= ~NET_OPEN;
            flags_FSM_Modem &= ~PDN_STABLE;
    	    pdn_last_event_time = HAL_GetTick();
        }

        else if(strncmp(line, "+CIPCLOSE:", 10) == 0)
        {
            Modem_Log(LOG_SOCKET_CLOSED,0,0);
            Dispatch_Event(EVT_SOCKET_LOST);
            flags_FSM_Modem &= ~SOCKET_OPEN;
        }

        else if(strcmp(line, "CLOSED") == 0)
        {
            Modem_Log(LOG_SOCKET_CLOSED,0,0);
            Dispatch_Event(EVT_SOCKET_LOST);
            flags_FSM_Modem &= ~SOCKET_OPEN;
        }

        else if (strstr(line, "+CIPERROR"))
        {
            snprintf(dbg,sizeof(dbg),"\r\n[URC]: %s\r\n", line);
            HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);

            // OPCIONAL: podrías generar evento adicional si querés
            Dispatch_Event(EVT_ERROR);
            flags_FSM_Modem &= ~SOCKET_OPEN;
        }

        else if (strncmp(line, "+CMTI:", 6) == 0)
        {
            sms_index = Parse_SMS_Index(line);
            Dispatch_Event(EVT_SMS_RECEIVED);

			#ifdef MODEM_DEBUG_URC
			snprintf(dbg,sizeof(dbg),"\r\n[SMS RECEIVED]: %s\r\n", line);
			HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);

			sscanf(line, "+CMTI: \"%*[^\"]\",%d", &sms_index);
			snprintf(dbg,sizeof(dbg),"\r\n[SMS IDX]: %d\r\n", sms_index);
			HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
			#endif
        }

        else if (strstr(line, "*ATREADY: 1"))
        {
            Dispatch_Event(EVT_ATREADY);
        }

        else if(strcmp(line, "+CPIN: READY") == 0)
        {
            Dispatch_Event(EVT_CPINREADY);
        }

        else if(strcmp(line, "+CPIN: NOT INSERTED") == 0)
        {
        	Dispatch_Event(EVT_SIM_NOT_INSERTED);
        }

        else if(strcmp(line, "+CPIN: SIM PIN") == 0)
        {
        	Dispatch_Event(EVT_SIM_PIN_REQUIRED);
        }

        else if(strcmp(line, "+CPIN: SIM PUK") == 0)
        {
        	Dispatch_Event(EVT_SIM_PUK_REQUIRED);
        }

        else if (strstr(line, "SMS DONE"))
        {
            Dispatch_Event(EVT_SMS_DONE);
        }
    }
}


/* ??? Que hacemos con esta función? La utilizamos o la borramos?
 */
bool Modem_Find_Response(const char* target) {
    // --- PROTECCIÓN CONTRA CONTAMINACIÓN (Punto 4 de tu análisis) ---
    // Si el string a buscar es distinto al de la llamada anterior,
    // reiniciamos el índice para no arrastrar matches parciales basura.
    if (strcmp(target, lastTarget) != 0) {
        searchIdx = 0;
        // Guardamos el nuevo target para la próxima comparación
        strncpy(lastTarget, target, sizeof(lastTarget) - 1);
    }

    int16_t byteRead;

    // --- PROCESAMIENTO DEL BUFFER ---
    // Leemos todo lo que haya en el RingBuffer en esta iteración
    while ((byteRead = RingBuffer_Read()) != -1) {
        uint8_t c = (uint8_t)byteRead;

        if (c == target[searchIdx]) {
            searchIdx++;
            // ¿Encontramos el string completo?
            if (target[searchIdx] == '\0') {
                searchIdx = 0;             // Reset para la próxima búsqueda
                lastTarget[0] = '\0';      // Limpiamos el rastro del target
                return true;               // ¡MATCH!
            }
        } else {
            // --- LÓGICA DE RE-INTENTO (Backtracking simple) ---
            // Si el carácter no coincide, verificamos si este nuevo carácter
            // podría ser el inicio del target (ej: buscando "OK" y recibimos "OOK")
            searchIdx = (c == target[0]) ? 1 : 0;
        }
    }

    return false; // No se encontró el patrón en esta pasada
}


/**
 * Callback UART: byte recibido de la PC.
 * - Eco inmediato al terminal.
 * - Acumula en buffer hasta \r → marca línea lista.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        RingBufferUart_Put(&uart1Buffer, uart_rx_char);

        HAL_UART_Receive_IT(&huart1, &uart_rx_char, 1);
    }
}

/*/
 * Esta función se ejecuta cuando se está en "modo normal" en el main loop.
 */
void UART1_Process_RingBuffer(void)
{
    uint8_t c;

    while(RingBufferUart_Get(&uart1Buffer, &c))
    {
        Parser_Feed(c);
    }
}

/*
 * Alimentación carácter por carácter.
 */
void Parser_Feed(uint8_t c)
{
    switch(parserState)
    {
        case PARSER_IDLE:
            lineIndexUart1 = 0;
            lineBufferUart1[lineIndexUart1++] = c;
            parserState = PARSER_COLLECT;
            break;

        case PARSER_COLLECT:
            if(lineIndexUart1 < sizeof(lineBufferUart1)-1)
            {
            	lineBufferUart1[lineIndexUart1++] = c;
                if(c == '\r')
                {
                	lineBufferUart1[lineIndexUart1] = '\n';
                    Process_Line(lineBufferUart1);
                    parserState = PARSER_IDLE;
                }
            }
            else
            {
				lineBufferUart1[lineIndexUart1] = '\n';
				Process_Line(lineBufferUart1);
				parserState = PARSER_IDLE;
            }
            break;

        default:
            lineIndexUart1 = 0;
            lineBufferUart1[lineIndexUart1++] = c;
            parserState = PARSER_COLLECT;
        	break;
    }
}

/*
 * Procesamiento de línea completa.
 */
void Process_Line(char *line)
{
    if(Is_AT_Command(line))
    {
        Process_AT_Command(line);
        return;
    }

    if(Is_Config_Command(line))
    {
        Process_Config_Command(line);
        return;
    }

    if(Is_CID_Event(line))
    {
        Process_CID_Event(line);
        return;
    }

    if(Is_Heartbeat(line))
    {
        Process_Heartbeat(line);
        return;
    }
}

/*
 * Detección de +++AT.
 */
uint8_t Is_AT_Command(char *line)
{
    if(strstr(line,"+++AT") != NULL)
        return 1;

    if(strstr(line,"+++at") != NULL)
        return 1;

    return 0;
}

/*
 * Detección de $CFG.
 */
uint8_t Is_Config_Command(char *line)
{
    if(strstr(line,"$CFG") != NULL)
        return 1;

    if(strstr(line,"$cfg") != NULL)
        return 1;

    return 0;
}

/*
 * Parser CID.
 */
uint8_t Is_CID_Event(char *line)
{
    if(strlen(line) < 20)
        return 0;

    if(isdigit((unsigned char)line[0]) &&
       isdigit((unsigned char)line[1]) &&
       isdigit((unsigned char)line[2]) &&
       isdigit((unsigned char)line[3]))
    {
        return 1;
    }

    return 0;
}

/*
 *
 */
void Process_CID_Event(char *line)
{
    CID_Message_t msg;

    if(CID_Parse_Event((uint8_t*)line,strlen(line),&msg))
    {
        Queue_CID_Message(&msg);
        uart1_last_activity = HAL_GetTick();
    }
}

/*
 *
 */
void Process_Heartbeat(char *line)
{
    CID_Message_t msg;

    if(CID_Parse_Heartbeat((uint8_t*)line, strlen(line), &msg))
    {
        Queue_Heartbeat(&msg);
        uart1_last_activity = HAL_GetTick();
    }
}

/*
 * Parser Heartbeat
 */
uint8_t Is_Heartbeat(char *line)
{
    return (memcmp(line,"@@HB",4) == 0);
}

/*
 * Conmutación a modo bridge.
 */
void Process_AT_Command(char *line)
{
    systemMode = SYS_MODE_BRIDGE;

    bridge_last_activity = HAL_GetTick();

    Debug_Print("\r\n[MODO] BRIDGE\r\n");
}

/*
 * Conmutación a modo config.
 */
void Process_Config_Command(char *line)
{
    systemMode = SYS_MODE_CONFIG;

    Debug_Print("\r\n[MODO] CONFIG\r\n");
}


/*
 * void RingBuffer_Put(RingBuffer_uart *rb, uint8_t c)
 */
uint8_t RingBufferUart_Put(RingBuffer_uart *rb, uint8_t data)
{
    uint16_t next = (rb->head + 1) % UART_BUF_SIZE;

    if(next == rb->tail)
        return 0;   // Buffer lleno

    rb->buffer[rb->head] = data;
    rb->head = next;

    return 1;
}

/*
 * uint8_t RingBuffer_Get(RingBuffer_uart *rb, uint8_t *c)
 */
uint8_t RingBufferUart_Get(RingBuffer_uart *rb, uint8_t *data)
{
    if(rb->head == rb->tail)
        return 0;

    *data = rb->buffer[rb->tail];

    rb->tail = (rb->tail + 1) % UART_BUF_SIZE;

    return 1;
}

// ??? ¿Donde podemos usarla?
uint32_t RingBufferUart_Count(RingBuffer_uart *rb)
{
    if(rb->head >= rb->tail)
        return rb->head - rb->tail;

    return UART_BUF_SIZE - rb->tail + rb->head;
}

void RingBufferUart_Clear(RingBuffer_uart *rb)
{
    rb->head = 0;
    rb->tail = 0;
}

void Queue_CID_Message(CID_Message_t *msg)
{
    memcpy(&last_cid_msg, msg, sizeof(CID_Message_t));
}

void Queue_Heartbeat(CID_Message_t *msg)
{
    memcpy(&last_hb_msg, msg, sizeof(CID_Message_t));
}

/*
 * Callback CDC: paquete recibido del modem (max 64 bytes por llamada).
 * Acumula en cdc_accum_buf. El flush a UART se hace en el main loop
 * cuando pasan CDC_FLUSH_TIMEOUT_MS ms sin datos nuevos.

void USBH_CDC_ReceiveCallback(USBH_HandleTypeDef *phost)
{
    CDC_HandleTypeDef *cdc = (CDC_HandleTypeDef *)phost->pActiveClass->pData;
    if (cdc == NULL) return;

    uint32_t len = USBH_LL_GetLastXferSize(phost, cdc->DataItf.InPipe);

    if (len > 0 && (cdc_accum_len + len) <= CDC_ACCUM_SIZE)
    {
        memcpy(cdc_accum_buf + cdc_accum_len, cdc_rx_buf, len);
        cdc_accum_len    += (uint16_t)len;
        cdc_last_rx_tick  = HAL_GetTick();
    }

    // Re-armar recepción inmediatamente para no perder el próximo paquete...
    USBH_CDC_Receive(phost, cdc_rx_buf, CDC_BUF_SIZE);
}
 *
 */

// DESPUÉS:
void USBH_CDC_ReceiveCallback(USBH_HandleTypeDef *phost)
{
    CDC_HandleTypeDef *cdc = (CDC_HandleTypeDef *)phost->pActiveClass->pData;
    if (cdc == NULL) return;

    uint32_t len = USBH_LL_GetLastXferSize(phost, cdc->DataItf.InPipe);

    if (len > 0)
    {
        /* --- Bridge transparente: acumula para flush a UART --- */
        if ((cdc_accum_len + len) <= CDC_ACCUM_SIZE)
        {
            memcpy(cdc_accum_buf + cdc_accum_len, cdc_rx_buf, len);
            cdc_accum_len    += (uint16_t)len;
            cdc_last_rx_tick  = HAL_GetTick();
        }

        /* --- Gateway autónomo: escribe byte a byte en el RingBuffer ---
         * Permite que Modem_Find_Response() detecte el prompt '>'
         * y otras respuestas AT en tiempo real */
        for (uint32_t i = 0; i < len; i++)
        {
            RingBuffer_Write(cdc_rx_buf[i]);
        }
    }

    USBH_CDC_Receive(phost, cdc_rx_buf, CDC_BUF_SIZE);
}

/*
 * Busca un substring en el buffer CDC acumulado.
 */
static uint8_t buffer_contiene(const char *str) // ??? Lo utilizamos o lo borramos?
{
    uint16_t slen = (uint16_t)strlen(str);
    if (slen == 0 || slen > cdc_accum_len) return 0;
    for (uint16_t i = 0; i <= cdc_accum_len - slen; i++)
    {
        if (memcmp(cdc_accum_buf + i, str, slen) == 0)
            return 1;
    }
    return 0;
}

/*
 * 1. ESTRUCTURAS NECESARIAS: PacketType_t; y CID_Message_t;
 * 2. PARSER DEL PROTOCOLO RS232
 * EVENTO CID ASCII → estructura
 * Entrada:5051 185427E35011000
 * Formato:5051 18 AAAA Q EEE PP ZZZ
 */
uint8_t CID_Parse_Event(uint8_t *rx, uint16_t len, CID_Message_t *msg)
{
    if (len < 20)
        return 0;

    uart1_last_activity = HAL_GetTick();

    msg->type = PKT_EVENT;

    memcpy(msg->account, &rx[7], 4);
    msg->account[4] = 0;

    msg->qualifier = rx[11];

    memcpy(msg->event_code, &rx[12], 3);
    msg->event_code[3] = 0;

    memcpy(msg->partition, &rx[15], 2);
    msg->partition[2] = 0;

    memcpy(msg->zone, &rx[17], 3);
    msg->zone[3] = 0;

    return 1;
}

/*
 * HEARTBEAT RS232 → estructura
 * Entrada: @@HB[5427][1][2][3][4][XX]
 */
uint8_t CID_Parse_Heartbeat(uint8_t *rx, uint16_t len, CID_Message_t *msg)
{
    if (memcmp(rx, "@@HB", 4) != 0)
        return 0;

    msg->type = PKT_HEARTBEAT;

    memcpy(msg->account, &rx[5], 4);
    msg->account[4] = 0;

    msg->hb_numabo      = rx[11];
    msg->hb_alarm       = rx[14];
    msg->hb_dev_status  = rx[17];
    msg->hb_dev_memory  = rx[20];

    uart1_last_activity = HAL_GetTick();

    return 1;
}

/*
 * 3. UTILIDADES
 */

/*
 * Conversión ASCII → nibble BCD
 */
static uint8_t pack_bcd(char high, char low)
{
    return ((high - '0') << 4) | (low - '0');
}

/*
 * XOR checksum
 */
uint8_t XOR_Checksum(uint8_t *data, uint16_t start, uint16_t end)
{
    uint8_t xorv = 0;

    for (uint16_t i = start; i <= end; i++)
    {
        xorv ^= data[i];
    }

    return xorv;
}

/*
 * checksum1 módulo 15.
 */
uint8_t CID_Checksum1(uint8_t *buf)
{
    uint16_t checksum = 0;

    for (uint16_t i = 6; i < 21; i++)
    {
        if (buf[i] == 0 && i < 12)
            buf[i] = 0x0A;

        checksum += buf[i];
    }

    while (checksum > 15)
        checksum -= 15;

    checksum += 15;

    return (uint8_t)checksum;
}

/*
 *
 */
void Parse_DNS_Response(char *line)
{
	char domain[64];
	char ip[16];

	if(sscanf(line, "+CDNSGIP: 1,\"%63[^\"]\",\"%15[^\"]\"", domain, ip) == 2)
	{
	    strcpy(runtime_cfg.servers[0].ip,ip);

	    if(runtime_cfg.servers[0].ip[0] == '\0')
	    {
	    	Dispatch_Event(EVT_DNS_ERROR);
	    }
	    else Dispatch_Event(EVT_DNS_OK);
	}
	else
	{
	    Dispatch_Event(EVT_DNS_ERROR);
	}
}


/*
 * GENERADOR DE PAQUETE UDP EVENTO
 */
uint16_t UDP_Build_Event(CID_Message_t *msg,uint8_t *out,uint32_t serial,uint8_t seq)
{
    memset(out, 0, 43);

    out[0] = 0x40;
    out[1] = 0xE8;

    char serial_str[9];
    snprintf(serial_str,sizeof(serial_str), "%08lu", serial);

    out[2] = pack_bcd(serial_str[6], serial_str[7]);
    out[3] = pack_bcd(serial_str[4], serial_str[5]);
    out[4] = pack_bcd(serial_str[2], serial_str[3]);
    out[5] = pack_bcd(serial_str[0], serial_str[1]);

    out[6] = msg->account[0];
    out[7] = msg->account[1];
    out[8] = msg->account[2];
    out[9] = msg->account[3];

    out[10] = 0x01;
    out[11] = 0x08;

    if (msg->qualifier == 'E')
        out[12] = 0x01;
    else
        out[12] = 0x03;

    out[13] = msg->event_code[0];
    out[14] = msg->event_code[1];
    out[15] = msg->event_code[2];

    out[16] = msg->partition[0];
    out[17] = msg->partition[1];

    out[18] = msg->zone[0];
    out[19] = msg->zone[1];
    out[20] = msg->zone[2];

    out[21] = CID_Checksum1(out);

    out[22] = seq;
    out[23] = 0x00;
    out[24] = 0x51;

    RTC_Fill_ASCII_DateTime(&out[28]);

    out[42] = XOR_Checksum(out, 1, 41);

    return 43;
}

/*
 * 5. GENERADOR HEARTBEAT UDP
 */

uint16_t UDP_Build_Heartbeat(CID_Message_t *msg, uint8_t *out, uint32_t serial, uint8_t seq)
{
    memset(out, 0, 43);

    out[0] = 0x40;
    out[1] = 0xE8;

    char serial_str[9];
    snprintf(serial_str,sizeof(serial_str),"%08lu", serial);

    out[2] = pack_bcd(serial_str[6], serial_str[7]);
    out[3] = pack_bcd(serial_str[4], serial_str[5]);
    out[4] = pack_bcd(serial_str[2], serial_str[3]);
    out[5] = pack_bcd(serial_str[0], serial_str[1]);

    out[6] = msg->account[0];
    out[7] = msg->account[1];
    out[8] = msg->account[2];
    out[9] = msg->account[3];

    out[10] = 0x0A;
    out[11] = 0x0A;

    out[12] = 0x00;
    out[13] = 0x06;

    out[15] = 0x00;

    out[17] = msg->hb_numabo;
    out[18] = msg->hb_alarm;
    out[19] = msg->hb_dev_status;
    out[20] = msg->hb_dev_memory;

    out[21] = CID_Checksum1(out);

    out[22] = seq;
    out[23] = 0x00;
    out[24] = 0x51;

    RTC_Fill_ASCII_DateTime(&out[28]);

    out[42] = XOR_Checksum(out, 1, 41);

    return 43;
}

/*
 * 6. VALIDADOR DE ACK UDP
 */
uint8_t UDP_Is_ACK(uint8_t *rx, uint16_t len)
{
    if (len < 2)
        return 0;

    if (rx[0] != 0x40)
        return 0;

    if ((rx[1] & 0xF0) != 0x30)
        return 0;

    return 1;
}

/*
 * 7. RESPUESTA RS232 HACIA CENTRAL
 * Cuando llega ACK UDP válido:
 */
void CID_Send_ACK_RS232(void)
{
    uint8_t ack = 0x06;
    HAL_UART_Transmit(&huart1, &ack, 1, 100);
}


/*
 * 4. TIMEOUT DEL MODO BRIDGE
 */
void System_Mode_Task(void)
{
    //-------------------------------------------------
    // CONFIG MODE por pin
    //-------------------------------------------------

    if (HAL_GPIO_ReadPin(GPIOB, ConfigPIN_Pin) == GPIO_PIN_RESET)
    {
        if (systemMode != SYS_MODE_CONFIG)
        {
        	RingBufferUart_Clear(&uart1Buffer);
            cfgModeState = CFG_WAIT_START;
            systemMode = SYS_MODE_CONFIG;
            Debug_Print("\r\n[MODO] CONFIG\r\n");
        }
    }

    else if (systemMode == SYS_MODE_BRIDGE)
    {
        //-------------------------------------------------
        // timeout bridge
        //-------------------------------------------------
        if ((HAL_GetTick() - bridge_last_activity) > BRIDGE_TIMEOUT_MS)
		{
			bridge_last_activity = HAL_GetTick();
			systemMode = SYS_MODE_NORMAL;
			Debug_Print("\r\n[MODO] NORMAL\r\n");
			RingBufferUart_Clear(&uart1Buffer);
		}
    }

    else if (systemMode == SYS_MODE_CONFIG)
    {
        //-------------------------------------------------
        // salir de config mode
        //-------------------------------------------------
        systemMode = SYS_MODE_NORMAL;
		Debug_Print("\r\n[MODO] NORMAL\r\n");
		RingBufferUart_Clear(&uart1Buffer);
    }
    else
    {
        systemMode = SYS_MODE_NORMAL;
        RingBufferUart_Clear(&uart1Buffer);

    }
}

/*
1. FORMATO JSON (PC)
 {

 "version": {
   "config_version": "0001",
 },

 "device": {
   "serial_number": "12345678",
   "phone_number": "1122334455"
 },

 "accounts": {
   "modem_account": "1234",
   "panel_account": "5678"
 },

 "security": {
   "sms_password": "1234",
   "config_password": "0000"
 },

 "network": {
   "apn_claro": "igprs.claro.com.ar",
   "apn_movistar": "internet.movistar.com.ar",
   "heartbeat_seconds": 60
 },

 "servers": [
   {
     "ip": "190.111.217.188",
     "port": 57777,
     "protocol": "UDP"
   },
   {
     "ip": "190.111.217.189",
     "port": 57778,
     "protocol": "UDP"
   },
   {
     "ip": "192.168.0.100",
     "port": 5000,
     "protocol": "TCP"
   }
 ]
}

EJEMPLO JSON SOPORTADO
{
	"config_version": 12,
   "server1_ip":"190.111.217.188",
   "server1_port":57777,
   "server1_proto":"UDP",

   "server2_ip":"192.168.0.100",
   "server2_port":6000,
   "server2_proto":"TCP",

   "server3_ip":"10.0.0.5",
   "server3_port":7000,
   "server3_proto":"UDP",

   "apn_movistar":"internet.movistar.org.ar",
   "apn_claro":"igprs.claro.com.ar",

   "modem_account":"1234",
   "panel_account":"5678",

   "sms_password":"0000",
   "cfg_password":"9999",

   "heartbeat_seconds":60,

   "serial_number":"12345678",

   "phone_number":"1156320411"
}
*/


/*
 *
 */
config_command_t Parse_Command(void)
{
    uint32_t i;

    for(i = 0; i < (sizeof(cmdTable)/sizeof(cmdTable[0])); i++)
    {
        if(strncmp((char *)cfg_rx_buffer,cmdTable[i].text,strlen(cmdTable[i].text)) == 0)
        {
            strcpy(command, cmdTable[i].text + 5); // salta "$CFG,"
            return cmdTable[i].cmd;
        }
    }

    command[0] = '\0';

    return CMD_INVALID;
}

/*
 *CMD_Read_Config()
 *Genera el JSON desde runtime_cfg o desde FLASH y lo envía a la PC.
 */
void CMD_Read_Config(void)
{
    char json[1024];
    char tx[1200];
    uint16_t len;
    uint16_t crc;

    Build_Config_JSON(json);

    len = strlen(json);

    sprintf(tx,"$CFG,DATA,%u,%s",len,json);

    crc = CRC16_CCITT((uint8_t*)tx, strlen(tx));

    sprintf(&tx[strlen(tx)],"*%04X\r\n",crc);

    HAL_UART_Transmit( &huart1, (uint8_t*)tx, strlen(tx), 1000);
}

/*
 *Build_Config_JSON()
 *Esta función arma el JSON desde runtime_cfg.
 */
void Build_Config_JSON(char *json)
{
    sprintf(json,
    "{"
    "\"config_version\":%u,"
    "\"config_timestamp\":\"%s\","

    "\"server1_ip\":\"%s\","
    "\"server1_port\":%u,"
    "\"server1_proto\":\"%s\","

    "\"server2_ip\":\"%s\","
    "\"server2_port\":%u,"
    "\"server2_proto\":\"%s\","

    "\"server3_ip\":\"%s\","
    "\"server3_port\":%u,"
    "\"server3_proto\":\"%s\","

    "\"apn_movistar\":\"%s\","
    "\"apn_claro\":\"%s\","

    "\"modem_account\":\"%s\","
    "\"panel_account\":\"%s\","

    "\"sms_password\":\"%s\","
    "\"cfg_password\":\"%s\","

    "\"heartbeat_seconds\":%u,"

    "\"serial_number\":\"%s\","
    "\"phone_number\":\"%s\""
    "}",

    runtime_cfg.config_version,
    runtime_cfg.config_timestamp,

    runtime_cfg.servers[0].ip,
    runtime_cfg.servers[0].port,
    runtime_cfg.servers[0].protocol ? "TCP" : "UDP",

    runtime_cfg.servers[1].ip,
    runtime_cfg.servers[1].port,
    runtime_cfg.servers[1].protocol ? "TCP" : "UDP",

    runtime_cfg.servers[2].ip,
    runtime_cfg.servers[2].port,
    runtime_cfg.servers[2].protocol ? "TCP" : "UDP",

    runtime_cfg.apn_movistar,
    runtime_cfg.apn_claro,

    runtime_cfg.modem_account,
    runtime_cfg.panel_account,

    runtime_cfg.sms_password,
    runtime_cfg.config_password,

    runtime_cfg.heartbeat_seconds,

    runtime_cfg.serial_number,
    runtime_cfg.phone_number);
}

/*
 *CMD_Write_Config()  Recibe el JSON. Lo parsea. Lo carga en RAM. NO escribe FLASH.
 */
void CMD_Write_Config(void)
{
    Parse_JSON_To_Runtime_Config(cfg_rx_buffer);
    Send_ACK("WRITE");
}

/*
 * CMD_Save_Config(); Graba en FLASH.
 */
void CMD_Save_Config(void)
{
	temp_runtime_cfg = runtime_cfg;
    Flash_Save_Config();
    Send_ACK("SAVE");
}

/*
 *CMD_Default_Config(); Carga valores de fábrica. Los graba.
 */
void CMD_Default_Config(void)
{
    Load_Default_Config();
    temp_runtime_cfg = runtime_cfg;
    Flash_Save_Config();
    Send_ACK("DEFAULT");
}

/*
 * CMD_Reboot(); Responde ACK y reinicia.
 */
void CMD_Reboot(void)
{
    Send_ACK("REBOOT");
    HAL_Delay(100);
    NVIC_SystemReset();
}

/*
 * Función común Send_ACK();
 * Ejemplos:
	$CFG,ACK,WRITE,OK*7A23
	$CFG,ACK,SAVE,OK*5B11
	$CFG,ACK,DEFAULT,OK*E913
	$CFG,ACK,REBOOT,OK*CC81
	$CFG,ACK,MODEMREINIT,OK*CC81
 */

void Send_ACK(const char *cmd)
{
    char tx[64];
    uint16_t crc;

    sprintf(tx,"$CFG,ACK,%s,OK",cmd);

    crc = CRC16_CCITT((uint8_t*)tx, strlen(tx));

    sprintf(&tx[strlen(tx)], "*%04X\r\n", crc);

    HAL_UART_Transmit( &huart1, (uint8_t*)tx, strlen(tx), 1000);
}

/*
 * Función común Send_Error()
 * Ejemplos:
	$CFG,ERROR,PASSWORD*22F1
	$CFG,ERROR,CRC*8A11
	$CFG,ERROR,FORMAT*917A
 */
void Send_Error(const char *error)
{
    char tx[64];
    uint16_t crc;

    sprintf(tx, "$CFG,ERROR,%s", error);

    crc = CRC16_CCITT((uint8_t*)tx, strlen(tx));

    sprintf(&tx[strlen(tx)], "*%04X\r\n", crc);

    HAL_UART_Transmit( &huart1, (uint8_t*)tx, strlen(tx), 1000);
}

/*
 *	Por lo tanto necesitas una función que valide que la trama tenga la estructura:
	$CFG,WRITE,9999,512,{....}*ABCD
	o
	$CFG,READ,9999*ABCD
 */
uint8_t Verify_Frame_Format(void)
{
    if(strncmp(cfg_rx_buffer, "$CFG,", 5) != 0)
        return 0;

    if(strchr(cfg_rx_buffer, '*') == NULL)
        return 0;

    return 1;
}

/*
 *2. Comandos propuestos:
	Información del log.
	Solicita cantidad de registros y posición actual.

	$CFG,LOGINFO,1234*ABCD

	Respuesta:

	$ACK,LOGINFO,COUNT=152,NEXT=153*XXXX

	Leer un registro específico
	$CFG,LOGREAD,25,1234*ABCD

	donde:

	25 = índice del registro
	1234 = password

	Respuesta:

	$ACK,LOGREAD,25,125600,LOG_NET_OPEN_OK,0,0*XXXX

	Formato:

	indice,timestamp,evento,param1,param2

	Leer un rango
	$CFG,LOGREAD,0,50,1234*ABCD

	Respuesta:

	$LOG,0,1200,LOG_CMEE_CONFIG_OK,2,0
	$LOG,1,1500,LOG_SIGNAL_OK,-73,0
	$LOG,2,1700,LOG_APN_OK,0,0
	...
	$ACK,END

	Muy útil para descargar todo el histórico.

	Borrar log
	$CFG,LOGCLEAR,1234*ABCD

	Respuesta:

	$ACK,LOGCLEAR

	Leer Log Completo:
	$CFG,LOGDUMP,<password>*CRC
	Respuesta:
	$ACK,LOGDUMP,........

 */
/*
 * Implementaría desde el inicio:

$CFG,LOGINFO,<pwd>*crc
$CFG,LOGREAD,<inicio>,<cantidad>,<pwd>*crc
$CFG,LOGCLEAR,<pwd>*crc

Porque cuando tengas 1000 o 2000 registros almacenados será mucho más práctico descargar bloques completos que pedirlos uno por uno.
Además luego podrás hacer una aplicación en PC que reconstruya automáticamente todo el historial del módem leyendo la EEPROM de forma remota por UART.

 */
uint8_t EEPROM_Log_Read(uint16_t index,ModemLogRecord_t *rec)
{
    uint32_t addr;

    if(index >= eeprom_log_count)
        return 0;

    addr = EEPROM_LOG_BASE + (index * sizeof(ModemLogRecord_t));

    EEPROM_Read_Buffer(addr,(uint8_t *)rec,sizeof(ModemLogRecord_t));

    return 1;
}

/*
 *
 */
void EEPROM_Log_Init(void)
{
    EEPROM_Read_Buffer(EEPROM_HEAD_ADDR,(uint8_t *)&eeprom_log_head, sizeof(eeprom_log_head));

    EEPROM_Read_Buffer(EEPROM_COUNT_ADDR,(uint8_t *)&eeprom_log_count,sizeof(eeprom_log_count));

    if(eeprom_log_head > MAX_LOG_RECORDS)
        eeprom_log_head = 0;

    if(eeprom_log_count > MAX_LOG_RECORDS)
        eeprom_log_count = 0;

}

/*
 *
 */
void CMD_Log_Info(void)
{
    char buf[80];

    snprintf(buf,sizeof(buf),"$ACK,LOGINFO,%u,%u\r\n", eeprom_log_count, eeprom_log_head);

    HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), 1000);
}

/*
 *
 */
void CMD_Log_Read(uint16_t index)
{
    ModemLogRecord_t rec;
    char buf[128];

    if(!EEPROM_Log_Read(index,&rec))
    {
        Send_Error("LOGINDEX");
        return;
    }

    snprintf(buf,sizeof(buf),"$ACK,LOGREAD,%u,%lu,%s,%lu,%lu\r\n",index,rec.timestamp,ModemEvent_ToString(rec.event_id),rec.param1,rec.param2);

    HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), 1000);
}

/*
 *
 */
uint16_t Get_LogRead_Index(void)
{
    char *p;
    char *pEnd;
    char index_str[8];
    uint16_t len;

    /*
        $CFG,LOGREAD,index,password*CRC
    */

    p = strchr(cfg_rx_buffer, ',');
    if(p == NULL) return 0xFFFF;

    /* LOGREAD */
    p = strchr(p + 1, ',');
    if(p == NULL) return 0xFFFF;

    p++;

    /* buscar fin del índice */
    pEnd = strchr(p, ',');
    if(pEnd == NULL) return 0xFFFF;

    len = pEnd - p;

    if(len == 0 || len >= sizeof(index_str))
        return 0xFFFF;

    memcpy(index_str, p, len);
    index_str[len] = '\0';

    return (uint16_t)strtoul(index_str, NULL, 10);
}

/*
 *
 */
void CMD_Log_Clear(void)
{
    eeprom_log_head  = 0;
    eeprom_log_count = 0;

    EEPROM_Write_Buffer(EEPROM_HEAD_ADDR, (uint8_t *)&eeprom_log_head, sizeof(eeprom_log_head));

    EEPROM_Write_Buffer(EEPROM_COUNT_ADDR,(uint8_t *)&eeprom_log_count, sizeof(eeprom_log_count));

    Send_ACK("LOGCLEAR");
}

/*
 *
 */
void Modem_Log(ModemEvent_t event_id, uint32_t param1, uint32_t param2)
{
    ModemLogRecord_t rec;

    memset(&rec,0,sizeof(rec));

    rec.timestamp = HAL_GetTick();
    rec.event_id = (uint16_t)event_id;
    rec.param1 = param1;
    rec.param2 = param2;

    EEPROM_Log_Write(&rec);

    LED_Update_From_Event(event_id);
}


/*
 * 	Prueba rápida: Antes de continuar con los logs, te recomiendo probar:
	uint8_t tx[8] = {1,2,3,4,5,6,7,8};
	uint8_t rx[8];
	EEPROM_Write_Buffer(0x0000, tx, sizeof(tx));
	HAL_Delay(10);
	EEPROM_Read_Buffer(0x0000, rx, sizeof(rx));
	y verificar:
	memcmp(tx, rx, sizeof(tx)) == 0
 */
void EEPROM_Log_Write(ModemLogRecord_t *rec)
{
    uint16_t addr;

    addr = EEPROM_LOG_BASE + (eeprom_log_count * LOG_RECORD_SIZE);

    EEPROM_Write_Buffer(addr, (uint8_t *)rec, LOG_RECORD_SIZE);

    eeprom_log_count++;

    if(eeprom_log_count >= MAX_LOG_RECORDS)
    {
    	eeprom_log_count = 0;
    }

    EEPROM_Write_Buffer(EEPROM_COUNT_ADDR, (uint8_t *)&eeprom_log_count, sizeof(eeprom_log_count));
}

/*
 * 	typedef enum
   	{
  	  HAL_OK       = 0x00U,
  	  HAL_ERROR    = 0x01U,
  	  HAL_BUSY     = 0x02U,
  	  HAL_TIMEOUT  = 0x03U
	} HAL_StatusTypeDef;
 */
HAL_StatusTypeDef EEPROM_Write_Buffer(uint16_t memAddr,uint8_t *data,uint16_t len)
{
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Write(&hi2c1,EEPROM_24LC512_ADDR,memAddr,I2C_MEMADD_SIZE_16BIT,data,len,1000);

    if(ret != HAL_OK)
        return ret;

    /* tiempo interno de programación */
    HAL_Delay(5);

    return HAL_OK;
}

HAL_StatusTypeDef EEPROM_Read_Buffer(uint16_t memAddr,uint8_t *data,uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1,EEPROM_24LC512_ADDR,memAddr,I2C_MEMADD_SIZE_16BIT,data,len,1000);
}

/*
 *
 */
uint8_t EEPROM_IsPresent(void)
{
    if(HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_24LC512_ADDR, 3, 100) == HAL_OK)
    {
        return 1;
    }

    return 0;
}

/*
 *
 */
HAL_StatusTypeDef EEPROM_Write(uint16_t addr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Write(&hi2c1,EEPROM_24LC512_ADDR,addr,I2C_MEMADD_SIZE_16BIT,data,len,1000);

    if(ret != HAL_OK) return ret;

    HAL_Delay(5);

    return HAL_OK;
}

/*
 *
 */
HAL_StatusTypeDef EEPROM_Read(uint16_t addr,uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1,EEPROM_24LC512_ADDR,addr,I2C_MEMADD_SIZE_16BIT,data,len,1000);
}

void PruebaSimpleEEPROM(void)
{
	uint8_t tx[16] = "HOLA";
	uint8_t rx[16];

	EEPROM_Write(0x0000, tx, 5);

	memset(rx,0,sizeof(rx));

	EEPROM_Read(0x0000, rx, 5);

	Debug_Print((char*)rx);
}

/*
 *	Necesita:
	localizar el '*'
	calcular CRC sobre todo lo anterior
	comparar contra los 4 hexadecimales recibidos
 */
uint8_t Verify_Frame_CRC(void)
{
    char *pStar;
    uint16_t crc_rx;
    uint16_t crc_calc;
    uint16_t len;

    pStar = strchr(cfg_rx_buffer, '*');

    if(pStar == NULL)
        return 0;

    crc_rx = (uint16_t)strtoul(pStar + 1, NULL, 16);

    len = pStar - cfg_rx_buffer;

    crc_calc = CRC16_CCITT((uint8_t*)cfg_rx_buffer,len);

    return (crc_calc == crc_rx);
}

const char *ModemEvent_ToString(ModemEvent_t event)
{
    switch(event)
    {
        case LOG_CMEE_CONFIG_OK:
            return "CMEE_CONFIG_OK";

        case LOG_CMEE_ERROR:
            return "CMEE_ERROR";

        case LOG_TIMEOUT_CMEE:
            return "TIMEOUT_CMEE";

        case LOG_SIM_READY:
        	return "SIM_READY";

        case LOG_SIM_NOT_INSERTED:
            return "SIM_NOT_INSERTED";

        case LOG_SIM_PIN_REQUIRED:
            return "SIM_PIN_REQUIRED";

        case LOG_SIM_PUK_REQUIRED:
            return "SIM_PUK_REQUIRED";

        case LOG_TIMEOUT_CPIN:
        	return "TIMEOUT_CPIN";

        case LOG_SIGNAL_OK:
            return "SIGNAL_OK";

        case LOG_SIGNAL_LOW:
            return "SIGNAL_LOW";

        case LOG_CFUN_OK:
            return "CFUN_OK";

        case LOG_CFUN_ERR:
            return "CFUN_ERR";

        case LOG_NETWORK_REGISTERED:
            return "NETWORK_REGISTERED";

        case LOG_NETWORK_LOST:
            return "NETWORK_LOST";

        case LOG_WAIT_REG:
            return "WAIT_REGISTRATION";

        case LOG_REG_DENIED:
            return "REG_DENIED";

        case LOG_TIMEOUT_REG:
            return "TIMEOUT_REG";

		case LOG_LTE_READY:
            return "LTE_READY";

        case LOG_LTE_DENIED:
            return "LTE_DENIED";

        case LOG_TIMEOUT_LTE:
            return "TIMEOUT_LTE";

        case LOG_GPRS_READY:
            return "GPRS_READY";

        case LOG_NET_DIAG:
            return "NET_DIAG";

        case LOG_APN_OK:
            return "APN_OK";

        case LOG_APN_ERR:
            return "APN_ERR";

        case LOG_TIMEOUT_APN:
            return "TIMEOUT_APN";

        case LOG_NETOPEN_START:
            return "NETOPEN_START";

        case LOG_NETOPEN_OK:
            return "NETOPEN_OK";

        case LOG_NETOPEN_ERR:
            return "NETOPEN_ERR";

        case LOG_NETOPEN_CMD_ERR:
            return "NETOPEN_CMD_ERR";

        case LOG_PDN_ACT:
            return "PDN_ACT";

        case LOG_PDN_DEACT:
            return "PDN_DEACT";

        case LOG_NETWORK_DETACH:
            return "NETWORK_DETACH";

        case LOG_TIMEOUT_NETOPEN:
            return "TIMEOUT_NETOPEN";

        case LOG_SOCKET_CLOSED:
        	return "SOCKET_CLOSED";

        case LOG_IP_ASSIGNED:
            return "IP_ASSIGNED";

        case LOG_IP_NOT_FOUND:
            return "IP_NOT_FOUND";

        case LOG_IP_CMD_ERR:
            return "IP_CMD_ERR";

        case LOG_TIMEOUT_IP:
            return "TIMEOUT_IP";

        case LOG_DNS_START:
        	return "DNS_START";

        case LOG_DNS_OK:
        	return "DNS_OK";

        case LOG_DNS_ERROR:
        	return "DNS_ERROR";

        case LOG_TIMEOUT_DNS:
        	return "DNS_TIMEOUT";

        case LOG_UDP_OPEN_START:
            return "UDP_OPEN_START";

        case LOG_UDP_OPEN_OK:
            return "UDP_OPEN_OK";

        case LOG_UDP_OPEN_ERR:
            return "UDP_OPEN_ERR";

        case LOG_UDP_CMD_ERR:
            return "UDP_CMD_ERR";

        case LOG_TIMEOUT_UDP:
            return "TIMEOUT_UDP";

        case LOG_SEND_REJECTED:
            return "SEND_REJECTED";

        case LOG_TIMEOUT_PROMPT:
            return "TIMEOUT_PROMPT";

        case LOG_SEND_OK:
            return "SEND_OK";

        case LOG_SEND_ERR:
            return "SEND_ERR";

        case LOG_TIMEOUT_SEND:
            return "TIMEOUT_SEND";

        case LOG_NETCLOSE_START:
            return "NETCLOSE_START";

        case LOG_NETCLOSE_OK:
            return "NETCLOSE_OK";

        case LOG_NETCLOSE_ERR:
            return "NETCLOSE_ERR";

        case LOG_TIMEOUT_NETCLOSE:
            return "TIMEOUT_NETCLOSE";

        case LOG_MODEM_RESET:
            return "MODEM_RESET";

        case LOG_RTC_SYNC_OK:
        	return "RTC_SYNC_OK";

        case LOG_RTC_SYNC_TIMEOUT:
        	return "RTC_SYNC_TIMEOUT";

        case LOG_FATAL_ERROR:
        	return "ERROR_FATAL";

        default:
            return "UNKNOWN";
    }
}

/*
 * 	La regla pasa a ser:
	El password siempre es el último campo antes del *CRC.
	Ejemplos válidos:

	$CFG,READ,1234*CRC
	$CFG,SAVE,1234*CRC
	$CFG,DEFAULT,1234*CRC
	$CFG,REBOOT,1234*CRC

	$CFG,LOGINFO,1234*CRC
	$CFG,LOGCLEAR,1234*CRC

	$CFG,LOGREAD,25,1234*CRC

	$CFG,WRITE,{"server":"1.2.3.4"},1234*CRC

	De esta forma no importa cuántos parámetros tenga cada comando.
 */

/*
 * 	Ventaja adicional. Ahora también conviene redefinir los comandos para que el password quede siempre al final.
	Por ejemplo:
	$CFG,READ,1234*CRC
	$CFG,LOGINFO,1234*CRC
	$CFG,LOGREAD,25,1234*CRC
	$CFG,LOGCLEAR,1234*CRC
	$CFG,WRITE,{JSON},1234*CRC

	Observa que incluso en WRITE el password queda al final.
	Así el protocolo completo sigue una única regla:

	$CFG,<CMD>,<PARAM1>,<PARAM2>,...,<PASSWORD>*<CRC16>

	y tanto Verify_Password() como Verify_Frame_CRC() permanecen invariables aunque agregues nuevos comandos en el futuro.
	Esto simplifica mucho el mantenimiento del firmware.
 */
uint8_t Verify_Password(void)
{
    char password_rx[16];

    char *pStar;
    char *pStart;

    uint16_t len;

    /* Buscar '*' */

    pStar = strchr((char *)cfg_rx_buffer, '*');

    if(pStar == NULL)
        return 0;

    /*
     * Buscar la última coma antes del CRC.
     * El password comienza después de ella.
     */

    pStart = pStar;

    while(pStart > (char *)cfg_rx_buffer)
    {
        if(*pStart == ',')
            break;

        pStart--;
    }

    if(*pStart != ',')
        return 0;

    pStart++;

    len = pStar - pStart;

    if(len == 0)
        return 0;

    if(len >= sizeof(password_rx))
        return 0;

    memcpy(password_rx, pStart, len);

    password_rx[len] = '\0';

    if(strcmp(password_rx, runtime_cfg.config_password) == 0)
        return 1;

    return 0;
}

/*
 * Parse_JSON_To_Runtime_Config(cfg_rx_buffer)
 */
void Parse_JSON_To_Runtime_Config(const char *json)
{
    char temp[64];
    uint16_t value;

    memset(&temp_runtime_cfg, 0, sizeof(temp_runtime_cfg));

    /*
    ============================================================
    VERSIÓN
    ============================================================
    */
    if(JSON_Get_Int(json, "config_version", &value))
    {
    	temp_runtime_cfg.config_version = value;
    }

    /*
    ============================================================
    SERVER 1
    ============================================================
    */

    if (JSON_Get_String(json, "server1_ip", temp, sizeof(temp)))
    {
    	strncpy(temp_runtime_cfg.servers[0].ip, temp, sizeof(temp_runtime_cfg.servers[0].ip)-1);
    }

    if (JSON_Get_Int(json, "server1_port", &value))
    {
    	temp_runtime_cfg.servers[0].port = (uint16_t)value;
    }

    if (JSON_Get_String(json, "server1_proto", temp, sizeof(temp)))
    {
    	if(strcmp(temp, "TCP") == 0)
    		temp_runtime_cfg.servers[0].protocol = 1;
    	else
    		temp_runtime_cfg.servers[0].protocol = 0;
    }

    /*
    ============================================================
    SERVER 2
    ============================================================
    */

    if (JSON_Get_String(json, "server2_ip", temp, sizeof(temp)))
    {
    	strncpy(temp_runtime_cfg.servers[1].ip, temp, sizeof(temp_runtime_cfg.servers[1].ip)-1);
    }

    if (JSON_Get_Int(json, "server2_port", &value))
    {
    	temp_runtime_cfg.servers[1].port = (uint16_t)value;
    }

    if (JSON_Get_String(json, "server2_proto", temp, sizeof(temp)))
    {
    	if(strcmp(temp, "TCP") == 0)
    		temp_runtime_cfg.servers[1].protocol = 1;
    	else
    		temp_runtime_cfg.servers[1].protocol = 0;
    }

    /*
    ============================================================
    SERVER 3
    ============================================================
    */

    if (JSON_Get_String(json, "server3_ip", temp, sizeof(temp)))
    {
    	strncpy(temp_runtime_cfg.servers[2].ip, temp, sizeof(temp_runtime_cfg.servers[2].ip)-1);
    }

    if (JSON_Get_Int(json, "server3_port", &value))
    {
    	temp_runtime_cfg.servers[2].port = (uint16_t)value;
    }

    if (JSON_Get_String(json, "server3_proto", temp, sizeof(temp)))
    {
    	if(strcmp(temp, "TCP") == 0)
    		temp_runtime_cfg.servers[2].protocol = 1;
    	else
    		temp_runtime_cfg.servers[2].protocol = 0;
    }

    /*
    ============================================================
    APN
    ============================================================
    */

    if (JSON_Get_String(json, "apn_movistar", temp, sizeof(temp)))
    {
        strncpy(temp_runtime_cfg.apn_movistar, temp, sizeof(temp_runtime_cfg.apn_movistar) - 1);
        temp_runtime_cfg.apn_movistar[sizeof(temp_runtime_cfg.apn_movistar)-1] = 0;
    }

    if (JSON_Get_String(json, "apn_claro", temp, sizeof(temp)))
    {
        strncpy(temp_runtime_cfg.apn_claro, temp, sizeof(temp_runtime_cfg.apn_claro) - 1);
        temp_runtime_cfg.apn_claro[sizeof(temp_runtime_cfg.apn_claro)-1] = 0;
    }

    /*
    ============================================================
    CUENTAS
    ============================================================
    */

    if (JSON_Get_String(json, "modem_account", temp, sizeof(temp)))
    {
        strncpy(temp_runtime_cfg.modem_account, temp, sizeof(temp_runtime_cfg.modem_account) - 1);
    }

    if (JSON_Get_String(json, "panel_account", temp, sizeof(temp)))
    {
        strncpy(temp_runtime_cfg.panel_account, temp, sizeof(temp_runtime_cfg.panel_account) - 1);
    }

    /*
    ============================================================
    PASSWORDS
    ============================================================
    */

    if (JSON_Get_String(json, "sms_password", temp, sizeof(temp)))
    {
        strncpy(temp_runtime_cfg.sms_password, temp, sizeof(temp_runtime_cfg.sms_password) - 1);
    }

    if (JSON_Get_String(json, "config_password", temp, sizeof(temp)))
    {
        strncpy(temp_runtime_cfg.config_password, temp, sizeof(temp_runtime_cfg.config_password) - 1);
    }

    /*
    ============================================================
    HEARTBEAT
    ============================================================
    */

    if(JSON_Get_Int(json, "heartbeat_seconds", &value))
    {
    	temp_runtime_cfg.heartbeat_seconds = value;
    }

    /*
    ============================================================
    SERIAL NUMBER
    ============================================================
    */

    if (JSON_Get_String(json, "serial_number", temp, sizeof(temp)))
    {
        strncpy(temp_runtime_cfg.serial_number, temp, sizeof(temp_runtime_cfg.serial_number) - 1);
    }

    /*
    ============================================================
    PHONE NUMBER
    ============================================================
    */

    if (JSON_Get_String(json, "phone_number", temp, sizeof(temp)))
    {
        strncpy(temp_runtime_cfg.phone_number, temp, sizeof(temp_runtime_cfg.phone_number) - 1);
    }
}

/*
 *
 */
uint8_t JSON_Get_String(const char *json, const char *key, char *out, uint16_t out_size)
{
    char pattern[64];

    snprintf(pattern, sizeof(pattern),"\"%s\":\"", key);

    char *p = strstr(json, pattern);

    if (!p) return 0;

    p += strlen(pattern);

    char *end = strchr(p, '"');

    if (!end) return 0;

    uint16_t len = end - p;

    if (len >= out_size) len = out_size - 1;

    memcpy(out, p, len);

    out[len] = 0;

    return 1;
}

/*
 *
 */
uint8_t JSON_Get_Int(const char *json, const char *key, uint16_t *value)
{
    char pattern[64];
    char *ptr;

    snprintf(pattern, sizeof(pattern), "\"%s\":", key);

    ptr = strstr(json, pattern);

    if(ptr == NULL) return 0;

    ptr += strlen(pattern);

    while(*ptr == ' ') ptr++;

    *value = (uint16_t)atoi(ptr);

    return 1;
}


/*
 * Checksum trama de datos de configuración.
 */
uint16_t CRC16_CCITT(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for(uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;

        for(uint8_t j = 0; j < 8; j++)
        {
            if(crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}

/*
 * Imprimir en la UART1.
 */
void Debug_Print(const char *str)
{
    HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),100);
}

/*
 * Para guardar configuración inicial:
 * Esta función:✅ carga valores por defecto en RAM
 */

void Load_Default_Config(void)
{
    memset(&runtime_cfg, 0, sizeof(runtime_cfg));

    runtime_cfg.config_version= 0x0001;

    strcpy(runtime_cfg.config_timestamp, "2026-07-20 00:00:00");

    strcpy(runtime_cfg.serial_number, "12345678");
    strcpy(runtime_cfg.phone_number, "1151605741");

    strcpy(runtime_cfg.modem_account, "4321");
    strcpy(runtime_cfg.panel_account, "2800");

    strcpy(runtime_cfg.apn_movistar, "internet.movistar.org.ar");
    strcpy(runtime_cfg.apn_claro,    "igprs.claro.com.ar");

    strcpy(runtime_cfg.servers[0].server_domain, "190.111.217.188");
    strcpy(runtime_cfg.servers[1].server_domain, "gprs0.eyse.com");
    strcpy(runtime_cfg.servers[2].server_domain, "gprs2.eyse.com");

    strcpy(runtime_cfg.servers[0].ip, "190.111.217.188");
    strcpy(runtime_cfg.servers[1].ip, "190.12.115.42");
    strcpy(runtime_cfg.servers[2].ip, "190.12.115.42");

    runtime_cfg.servers[0].port= 57777;
    runtime_cfg.servers[1].port= 18023;
    runtime_cfg.servers[2].port= 28036;

    runtime_cfg.servers[0].protocol= PROTOCOL_UDP;
    runtime_cfg.servers[1].protocol= PROTOCOL_UDP;
    runtime_cfg.servers[2].protocol= PROTOCOL_UDP;

    runtime_cfg.heartbeat_seconds = 60;

    strcpy(runtime_cfg.sms_password, "1234");
    strcpy(runtime_cfg.config_password, "5678");
}

/*
 * Esta función:

✅ lee FLASH
✅ verifica MAGIC
✅ verifica CRC
✅ si es válido: copia FLASH → RAM
✅ si es inválido: usa defaults
 */
void Flash_Load_Config(void)
{
	flash_config_t *flash_cfg = (flash_config_t*)FLASH_CONFIG_ADDRESS;

	if (flash_cfg->magic != CONFIG_MAGIC)
	{
	    Load_Default_Config();
	}
	else
	{
	    uint32_t crc = CRC32((uint8_t*)&flash_cfg->cfg, sizeof(modem_config_t));

	    if (crc != flash_cfg->crc)
	    {
	        Load_Default_Config();
	    }
	    else
	    {
	        memcpy(&runtime_cfg, &flash_cfg->cfg, sizeof(modem_config_t));
	    }
	}
}

/*
 *Esta función:
✅ toma runtime_cfg
✅ calcula CRC
✅ agrega MAGIC
✅ borra FLASH
✅ escribe FLASH
 */
void Flash_Save_Config(void)
{
    flash_config_t flash_cfg;
    uint32_t flash_addr;
    uint32_t *src;
    uint32_t i;

    HAL_FLASH_Unlock();

    //-----------------------------------------
    // PREPARAR ESTRUCTURA
    //-----------------------------------------

    flash_cfg.version = CONFIG_STRUCT_VERSION;

    flash_cfg.magic = CONFIG_MAGIC;

    memcpy(&flash_cfg.cfg, &temp_runtime_cfg, sizeof(modem_config_t));

    flash_cfg.crc = CRC32((uint8_t*)&flash_cfg.cfg, sizeof(modem_config_t));

    //-----------------------------------------
    // BORRAR SECTOR
    //-----------------------------------------

    FLASH_EraseInitTypeDef erase;

    uint32_t sector_error = 0;

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;

    erase.Sector = CONFIG_FLASH_SECTOR;

    erase.NbSectors = 1;

    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if(HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
    {
        HAL_FLASH_Lock();

        Debug_Print("FLASH ERASE ERROR\r\n");

        return;
    }

    //-----------------------------------------
    // ESCRIBIR ESTRUCTURA COMPLETA
    //-----------------------------------------

    flash_addr = FLASH_CONFIG_ADDRESS;

    src = (uint32_t*)&flash_cfg;

    for(i = 0; i < (sizeof(flash_config_t) / 4); i++)
    {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr, src[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();

            Debug_Print("FLASH WRITE ERROR\r\n");

            return;
        }

        flash_addr += 4;
    }

    //-----------------------------------------
    // LOCK FLASH
    //-----------------------------------------

    HAL_FLASH_Lock();

    Debug_Print("FLASH CONFIG SAVED\r\n");

    //-----------------------------------------
    // VERIFICACIÓN FINAL
    //-----------------------------------------
    flash_config_t *verify;

    verify = (flash_config_t*)FLASH_CONFIG_ADDRESS;

    if(verify->magic == CONFIG_MAGIC)
    {
        Debug_Print("VERIFY OK\r\n");
    }
}


/*
 * Para la verificación de la memoria flush:
 */
uint32_t CRC32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;

    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }

    return ~crc;
}

/*
 *
 */
void LED_Init(void)
{
    ledRojoSIM.port = GPIOA;
    ledRojoSIM.pin  = LedRojoSIM_Pin;

    ledVerdeSIM.port = GPIOA;
    ledVerdeSIM.pin  = LedVerdeSIM_Pin;

    ledRojoConexion.port = GPIOA;
    ledRojoConexion.pin  = LedRojoConexion_Pin;

    ledVerdeConexion.port = GPIOA;
    ledVerdeConexion.pin  = LedVerdeConexion_Pin;

    ledVerdeBlackPill.port = GPIOC;
    ledVerdeBlackPill.pin  = GPIO_PIN_13;

    ledRojoSIM.mode = LED_OFF;
    ledVerdeSIM.mode = LED_OFF;
    ledRojoConexion.mode = LED_OFF;
    ledVerdeConexion.mode = LED_OFF;
    ledVerdeBlackPill.mode = LED_OFF;
}

void LED_Set(LedControl_t *led, LedMode_t mode, uint32_t timing)
{
    led->mode = mode;
    led->lastToggle = HAL_GetTick();
    led->dutycicle = timing;
}

void LED_Task(void)
{
    uint32_t now = HAL_GetTick();

    for(uint8_t i=0;i<5;i++)
    {
        LedControl_t *led = leds[i];

        switch(led->mode)
        {
			case LED_IDLE:
					break;

            case LED_OFF:

                HAL_GPIO_WritePin(led->port,led->pin,GPIO_PIN_SET);
                led->mode = LED_IDLE;

                break;

            case LED_ON:

                HAL_GPIO_WritePin(led->port,led->pin,GPIO_PIN_RESET);
                led->mode = LED_IDLE;

                break;

            case LED_ON_TIMER:

                if((now - led->lastToggle) > led->dutycicle)
                {
                    HAL_GPIO_WritePin(led->port,led->pin,GPIO_PIN_RESET);
                }
                else
                {
                	HAL_GPIO_WritePin(led->port,led->pin,GPIO_PIN_SET);
                	led->mode = LED_IDLE;
                }

                break;

            case LED_BLINK_SLOW:

                if((now - led->lastToggle) >= 1000)
                {
                    led->lastToggle = now;
                    led->state ^= 1;

                    HAL_GPIO_WritePin(led->port,led->pin,led->state);
                }

                break;

            case LED_BLINK_FAST:

                if((now - led->lastToggle) >= 200)
                {
                    led->lastToggle = now;
                    led->state ^= 1;

                    HAL_GPIO_WritePin(led->port,led->pin,led->state);
                }

                break;
        }
    }
}

/*/
 *
 */
void LED_Update_From_Event(ModemEvent_t event)
{
    switch(event)
    {
        /*
         * SIM / MÓDEM
         */

        case LOG_CFUN_OK:

            LED_Set(&ledVerdeSIM, LED_ON, 0);
            LED_Set(&ledRojoSIM, LED_OFF, 0);

            break;

        case LOG_CFUN_ERR:

            LED_Set(&ledVerdeSIM, LED_OFF, 0);
            LED_Set(&ledRojoSIM, LED_ON, 0);

            break;

        case LOG_CMEE_ERROR:

            LED_Set(&ledVerdeSIM, LED_OFF, 0);
            LED_Set(&ledRojoSIM, LED_BLINK_FAST, 0);

            break;


        case LOG_SIM_READY:

            LED_Set(&ledVerdeSIM, LED_ON, 0);
            LED_Set(&ledRojoSIM, LED_OFF, 0);

            break;

        case LOG_SIM_PIN_REQUIRED:

            LED_Set(&ledVerdeSIM, LED_OFF, 0);
            LED_Set(&ledRojoSIM, LED_BLINK_SLOW, 0);

            break;

        case LOG_SIM_PUK_REQUIRED:

            LED_Set(&ledVerdeSIM, LED_OFF, 0);
            LED_Set(&ledRojoSIM, LED_BLINK_FAST, 0);

            break;

        case LOG_SIM_NOT_INSERTED:

            LED_Set(&ledVerdeSIM, LED_OFF, 0);
            LED_Set(&ledRojoSIM, LED_ON, 0);

            break;

        case LOG_TIMEOUT_CPIN:

            LED_Set(&ledVerdeSIM, LED_BLINK_SLOW, 0);
            LED_Set(&ledRojoSIM, LED_ON, 0);

            break;

        /*
         * REGISTRO CELULAR
         */

        case LOG_NETWORK_REGISTERED:

            LED_Set(&ledVerdeSIM, LED_ON, 0);
            LED_Set(&ledRojoSIM, LED_OFF, 0);

            break;

        case LOG_NETWORK_LOST:

            LED_Set(&ledVerdeSIM, LED_OFF, 0);
            LED_Set(&ledRojoSIM, LED_BLINK_SLOW, 0);

            break;

        case LOG_WAIT_REG:

            LED_Set(&ledVerdeSIM, LED_BLINK_SLOW, 0);
            LED_Set(&ledRojoSIM, LED_OFF, 0);

            break;

        case LOG_REG_DENIED:

            LED_Set(&ledVerdeSIM, LED_OFF, 0);
            LED_Set(&ledRojoSIM, LED_ON, 0);

            break;

        case LOG_TIMEOUT_REG:

            LED_Set(&ledVerdeSIM, LED_OFF, 0);
            LED_Set(&ledRojoSIM, LED_BLINK_FAST, 0);

            break;

        /*
         * RED LTE
         */

        case LOG_LTE_DENIED:

            LED_Set(&ledVerdeSIM, LED_OFF, 0);
            LED_Set(&ledRojoSIM, LED_ON, 0);

            break;

        case LOG_PDN_DEACT:
        case LOG_NETWORK_DETACH:

            LED_Set(&ledVerdeSIM, LED_OFF, 0);
            LED_Set(&ledRojoSIM, LED_BLINK_SLOW, 0);

        	break;

        /*
         * UDP
         */

        case LOG_UDP_OPEN_OK:

            LED_Set(&ledVerdeConexion, LED_ON, 0);
            LED_Set(&ledRojoConexion, LED_OFF, 0);

            break;

        case LOG_UDP_OPEN_ERR:

            LED_Set(&ledVerdeConexion, LED_OFF, 0);
            LED_Set(&ledRojoConexion, LED_ON, 0);

            break;

        /*
         * TRANSMISIÓN
         */

        case LOG_SEND_OK:

            LED_Set(&ledVerdeConexion, LED_BLINK_FAST, 0);

            break;

        case LOG_SEND_ERR:

            LED_Set(&ledRojoConexion, LED_BLINK_FAST, 0);

            break;

        case LOG_TIMEOUT_SEND:

            LED_Set(&ledRojoConexion, LED_BLINK_FAST, 0);

            break;

        case LOG_RS232_TIMEOUT:

            LED_Set(&ledVerdeBlackPill, LED_BLINK_FAST, 0);

            break;

        case LOG_RS232_RESTORED:

            LED_Set(&ledVerdeBlackPill, LED_BLINK_SLOW, 0);

            break;

        /*
         * HARD RESET
         */

        case LOG_MODEM_RESET:

            LED_Set(&ledRojoSIM, LED_BLINK_FAST, 0);
            LED_Set(&ledRojoConexion, LED_BLINK_FAST, 0);

            break;

        default:

            break;
    }
}

/*
 * Función de supervisión: Llamada desde el loop principal.
 */
void RS232_Monitor_Task(void)
{
    if((HAL_GetTick() - uart1_last_activity) > 65000)
    {
        if(uart1_idle_alarm == 0)
        {
        	uart1_idle_alarm = 1;
            Modem_Log(LOG_RS232_TIMEOUT,0,0);
            LED_Update_From_Event(LOG_RS232_TIMEOUT);
        }
    }
    else
    {
    	if(uart1_idle_alarm == 1)
    	{
    		uart1_idle_alarm = 0;
    		Modem_Log(LOG_RS232_RESTORED,0,0);
    		LED_Update_From_Event(LOG_RS232_RESTORED);
    	}
    }
}

/*
 *
 */
void RTC_Fill_ASCII_DateTime(uint8_t *dst)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BIN);

    sprintf((char*)dst,"%02u/%02u/%02u %02u:%02u:%02u",sDate.Date,sDate.Month,sDate.Year,sTime.Hours,sTime.Minutes,sTime.Seconds);
}

/*
 *
 */
void RTC_Update_From_Modem(uint8_t year,uint8_t month,uint8_t day,uint8_t hour,uint8_t minute,uint8_t second)
{
    RTC_TimeTypeDef t;
    RTC_DateTypeDef d;

    t.Hours   = hour;
    t.Minutes = minute;
    t.Seconds = second;

    d.Year    = year;
    d.Month   = month;
    d.Date    = day;

    HAL_RTC_SetTime(&hrtc,&t,RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc,&d,RTC_FORMAT_BIN);
}

void RTC_Set_From_CCLK(const char *line)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    int yy,MM,dd,hh,mm,ss;

    if(sscanf(line,"+CCLK: \"%2d/%2d/%2d,%2d:%2d:%2d",&yy,&MM,&dd,&hh,&mm,&ss) != 6) return;

    sDate.Year  = yy;
    sDate.Month = MM;
    sDate.Date  = dd;

    sTime.Hours   = hh;
    sTime.Minutes = mm;
    sTime.Seconds = ss;

    HAL_RTC_SetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
    HAL_RTC_SetTime(&hrtc,&sTime,RTC_FORMAT_BIN);

    // Debug_Print("\r\n[RTC] Actualizado desde red\r\n");
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   *
	  Las funciones HAL suelen devolver:
	  HAL_OK
	  HAL_ERROR
	  HAL_BUSY
	  HAL_TIMEOUT
   *
   *
   */
  Modem_Log(LOG_FATAL_ERROR, 1, 0);
  HAL_Delay(100);
  NVIC_SystemReset();

  //__disable_irq();

  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
