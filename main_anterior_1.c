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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_host.h"
#include "usbh_cdc.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>   // ← AGREGAR


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* Para STM32F401 (256KB), el último sector es el 5 */
/* Si estás seguro que tu chip es F411 con 512KB, verifica el micro en el .ioc */
#define FLASH_LOG_SECTOR      FLASH_SECTOR_5
#define FLASH_LOG_START_ADDR  0x08020000  // Dirección de inicio del Sector 5


typedef enum {
    MODEM_STATE_IDLE,
	MODEM_STATE_CHECK_SIGNAL,
	MODEM_STATE_CHECK_SIGNAL_SEND, MODEM_STATE_CHECK_SIGNAL_WAIT,
	MODEM_STATE_CHECK_REG,
	MODEM_STATE_CHECK_REG_SEND,    MODEM_STATE_CHECK_REG_WAIT,
	MODEM_STATE_WAIT_PDN,
	MODEM_STATE_CHECK_NET_SEND,    MODEM_STATE_CHECK_NET_WAIT,
	MODEM_STATE_CHECK_SOCKET_SEND, MODEM_STATE_CHECK_SOCKET_WAIT,
    MODEM_STATE_NETCLOSE_SEND,     MODEM_STATE_NETCLOSE_WAIT,
    MODEM_STATE_CIPCLOSE_SEND,     MODEM_STATE_CIPCLOSE_WAIT,
    MODEM_STATE_CGDCONT_SEND,      MODEM_STATE_CGDCONT_WAIT,
    MODEM_STATE_CSOCKSETPN_SEND,   MODEM_STATE_CSOCKSETPN_WAIT,
    MODEM_STATE_NETOPEN_SEND,      MODEM_STATE_NETOPEN_WAIT,
    MODEM_STATE_IPADDR_SEND,       MODEM_STATE_IPADDR_WAIT,
    MODEM_STATE_CIPOPEN_SEND,      MODEM_STATE_CIPOPEN_WAIT,
    MODEM_STATE_SEND_CMD,          MODEM_STATE_WAIT_PROMPT,
    MODEM_STATE_SEND_DATA,         MODEM_STATE_WAIT_SEND_OK,
	MODEM_STATE_CLEAN_UP_1,        MODEM_STATE_CLEAN_UP_2,
	MODEM_STATE_READY,
    MODEM_STATE_ERROR
} ModemState_t;

ModemState_t modemState = MODEM_STATE_IDLE;

uint32_t timeoutTimer = 0;
char lastTarget[32] = {0};
uint16_t searchIdx = 0;

// Array con los 10 mensajes alfanuméricos de 20 caracteres
const char alarm_messages[10][21] = {
    "ALERTA TEMP ALTA 001",
    "BATERIA BAJA NODO 04",
    "SENSOR HUMEDAD FALLO",
    "PUERTA ACCESO ABIERT",
    "SOBREVOLTAJE LINEA 2",
    "CORTE ENERGIA GRUPO3",
    "NIVEL CRITICO TANQ01",
    "FUGA DE GAS SECTOR 5",
    "SISTEMA REINICIADO01",
    "PRESION ALTA BOMBA 2"
};

typedef enum {
	EVT_NONE = 0,
    EVT_OK,
    EVT_ERROR,
    EVT_TIMEOUT,
    EVT_CREG_REGISTERED,
    EVT_CREG_SEARCHING,
	EVT_PDN_ACT,
	EVT_PDN_DEACT,
    EVT_NET_OPEN,
    EVT_NET_CLOSED,
    EVT_SOCKET_OPEN,
    EVT_SOCKET_CLOSED,
    EVT_PROMPT,
    EVT_SEND_OK,
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

sms_state_t smsState = SMS_IDLE;

#define EVENT_QUEUE_SIZE 16

typedef struct {
    modem_event_t queue[EVENT_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
} EventQueue_t;

static EventQueue_t evtQueue;

typedef struct {
    uint32_t timestamp;      // HAL_GetTick() o RTC si tenés
    uint8_t  last_state;     // El estado donde ocurrió el fallo
    uint8_t  msg_index;      // Qué mensaje de la ráfaga era
    int8_t   rssi;           // Nivel de señal al momento del error
    uint8_t  error_code;     // Código CME/CMS o interno (0: Timeout, 1: Denied, etc)
} Modem_Error_Log_t;

#define LINE_BUFFER_SIZE 128

static char lineBuffer[LINE_BUFFER_SIZE];
static uint8_t lineIndex = 0;

static int sms_index = -1;
uint8_t current_msg_index = 0; // Índice para el envío secuencial

uint8_t last_confirmed_index = 0; // Último mensaje que recibió el +CIPSEND: 0,20,20
bool burst_mode_active = false;   // Indica si estamos en medio de los 10 envíos

#define MAX_RETRIES 3

uint8_t retry_count = 0; // Contador de intentos para el mensaje actual

RTC_HandleTypeDef hrtc;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;

/* --- Buffer UART RX: acumula chars hasta \r --- */
#define UART_BUF_SIZE  256
static uint8_t  uart_rx_char;
static uint8_t  uart_cmd_buf[UART_BUF_SIZE];
uint16_t uart_cmd_len   = 0;
volatile uint8_t uart_line_ready = 0;

/* --- Buffer CDC: recibe respuestas del modem --- */
/* --- Buffer CDC RX: un paquete a la vez (64 bytes FS) --- */
#define CDC_BUF_SIZE  64
uint8_t cdc_rx_buf[CDC_BUF_SIZE];

/* --- Buffer de acumulación CDC: junta todos los paquetes de una respuesta --- */
#define CDC_ACCUM_SIZE       1024
#define CDC_FLUSH_TIMEOUT_MS   50U
#define CDC_CHUNK_SIZE         64U   /* bytes máx por flush para no bloquear USB */
static uint8_t  cdc_accum_buf[CDC_ACCUM_SIZE];
static uint16_t cdc_accum_len     = 0;
static uint32_t cdc_last_rx_tick  = 0;

/* --- Handles externos --- */
extern USBH_HandleTypeDef hUsbHostFS; // Definida en usb_host.c
extern ApplicationTypeDef Appli_state; // Estado de la conexión

/* --- Estado del modem --- */
volatile uint8_t modem_listo = 0;

volatile uint32_t timer_led = 0;
volatile uint32_t timer_uart = 0;
// Nuevas variables para el buffer de recepción.
uint16_t rx_index = 0;
uint8_t rx_data; // Aquí recibimos el caracter actual.

static uint32_t uart_last_rx_tick = 0;
#define UART_IDLE_TIMEOUT_MS  20U

#define RING_BUFFER_SIZE 512
typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
} RingBuffer_t;

RingBuffer_t modemBuffer = {{0}, 0, 0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
void MX_USB_HOST_Process(void);
void USBH_CDC_ReceiveCallback(USBH_HandleTypeDef *phost);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
static uint8_t buffer_contiene(const char *str);
void Modem_Process_Buffer(void);
bool Modem_Find_Response(const char* expected);
void Modem_Gateway_Service(void);
void RingBuffer_Write(uint8_t data);
int16_t RingBuffer_Read(void);
void RingBuffer_Clear(void);
void Modem_Log(const char* etiqueta, uint8_t msg_idx, uint8_t intento);
void Modem_Send_AT(const char *cmd);
void Flash_Write_Log(Modem_Error_Log_t *log_entry);
void PushEvent(modem_event_t evt);
bool PopEvent(modem_event_t *evt);
void EventQueue_Init(void);
int Parse_SMS_Index(void);
void Modem_Request_Signal(void);
void Modem_Dispatch(void);
void Modem_FSM_Run(void);
void SMS_FSM_Run(void);
void Bridge_Process(void);
void Button_Process(void);
bool Modem_Get_Line(char* line);
void Modem_Handle_URC(char* line);

/* USER CODE BEGIN PFP */

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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_USB_HOST_Init();

  /* USER CODE BEGIN 2 */
  // Arrancamos la recepción por interrupción de 1 byte.
  HAL_UART_Receive_IT(&huart1, &uart_rx_char, 1);

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

  HAL_UART_Transmit(&huart1, boot_msg, boot_len, 200);

  uint8_t ready_msg[] = "Black Pill lista. Esperando modem USB...\r\n";
  HAL_UART_Transmit(&huart1, ready_msg, sizeof(ready_msg)-1, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* Tarea obligatoria del USB Host */
	MX_USB_HOST_Process();

	//Modem_Gateway_Service();

    // 1. Procesar buffer → generar eventos
    Modem_Process_Buffer();

    // 2. Ejecutar FSM LTE
    Modem_FSM_Run();

    // 3. Ejecutar FSM SMS
    SMS_FSM_Run();

    // 4. Bridge (debug / comandos manuales)
    Bridge_Process();

    // 5. Botón
    Button_Process();



	/* --- Flush CDC → UART ---
     * Cuando pasaron CDC_FLUSH_TIMEOUT_MS ms sin datos nuevos,
     * enviar todo lo acumulado de una vez a la PC
    if (cdc_accum_len > 0 && (HAL_GetTick() - cdc_last_rx_tick) >= CDC_FLUSH_TIMEOUT_MS)
    {
        HAL_UART_Transmit(&huart1, cdc_accum_buf, cdc_accum_len, cdc_accum_len * 2U + 100U);
         cdc_accum_len = 0;
    }
    */

	/* Flush por idle: si hay datos y pasaron 20ms sin bytes nuevos → enviar
	if (uart_cmd_len > 0 && !uart_line_ready && (HAL_GetTick() - uart_last_rx_tick) >= UART_IDLE_TIMEOUT_MS)
	{
	    uart_line_ready = 1;
	}
	 */

    /* --- Flush CDC → UART en chunks ---
     * Transmite de a CDC_CHUNK_SIZE bytes para no bloquear el USB */
    if (cdc_accum_len > 0 && (HAL_GetTick() - cdc_last_rx_tick) >= CDC_FLUSH_TIMEOUT_MS)
    {
        /* Detectar URC de red lista ANTES de flushear */
        if (!modem_listo)
        {
            if (buffer_contiene("+CGEV: EPS PDN ACT") || buffer_contiene("+CGEV: ME PDN ACT"))
            {
                modem_listo = 1;
                uint8_t aviso[] = "\r\n[BRIDGE] MODEM_LISTO\r\n";
                HAL_UART_Transmit(&huart1, aviso, sizeof(aviso)-1, 100);
            }
        }
        else
        {
            /* Si la red desactiva el contexto, resetear y esperar nuevo ACT */
            if (buffer_contiene("+CGEV: NW PDN DEACT") || buffer_contiene("+CGEV: ME PDN DEACT"))
            {
                modem_listo = 0;
                uint8_t aviso[] =
                    "\r\n[BRIDGE] MODEM_NO_LISTO\r\n";
                HAL_UART_Transmit(&huart1, aviso, sizeof(aviso)-1, 100);
            }
        }

    	uint16_t sent = 0;
        while (sent < cdc_accum_len)
        {
            uint16_t chunk = cdc_accum_len - sent;
            if (chunk > CDC_CHUNK_SIZE) chunk = CDC_CHUNK_SIZE;

            HAL_UART_Transmit(&huart1, cdc_accum_buf + sent, chunk, 100);
            sent += chunk;

            /* Mantener vivo el USB entre chunks */
            MX_USB_HOST_Process();
        }
        cdc_accum_len = 0;
    }

    /* --- Bridge UART → CDC ---
     * Bloqueado hasta que modem_listo = 1.
     * Eco del comando completo antes de enviarlo.
     * Si el modem está listo y hay una línea completa → transmitir */
    if (uart_line_ready && (Appli_state == APPLICATION_READY) && modem_listo)
    {
        // Eco del comando completo antes de enviarlo al modem
        // HAL_UART_Transmit(&huart1, uart_cmd_buf, uart_cmd_len, 100);

    	USBH_CDC_Transmit(&hUsbHostFS, uart_cmd_buf, uart_cmd_len);
        uart_cmd_len    = 0;
        uart_line_ready = 0;
    }

    /* --- Botón PA0: debug de estado USB--- */
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
    {
    	if(modemState == MODEM_STATE_IDLE) modemState = MODEM_STATE_CHECK_SIGNAL_SEND;

		// 1. Efecto Visual: Encender LED_VERDE.
		timer_led = 3000;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

        uint8_t dbg[64];
        //uint8_t len = snprintf((char*)dbg, sizeof(dbg),"[BTN] Estado=%d BufLen=%d\r\n", Appli_state, uart_cmd_len);
        //uint8_t dlen = snprintf((char*)dbg, sizeof(dbg), "[BTN] Estado=%d\r\n", Appli_state);
        uint8_t dlen = snprintf((char*)dbg, sizeof(dbg), "[BTN] USB=%d ModemListo=%d BufLen=%d\r\n", Appli_state, modem_listo, uart_cmd_len);

        HAL_UART_Transmit(&huart1, dbg, dlen, 100);
    }

    // 2. Efecto Visual: Apagar LED_VERDE.
	if (timer_led == 0)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	}

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
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

  // CONFIGURACIÓN PARA CRISTAL DE 25MHz
  //RCC_OscInitStruct.PLL.PLLM = 25;  // 25MHz / 25 = 1MHz
  //RCC_OscInitStruct.PLL.PLLN = 192; // 1MHz * 192 = 192MHz
  //RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; // 192 / 4 = 48MHz (CPU)
  //RCC_OscInitStruct.PLL.PLLQ = 4;   // 192 / 4 = 48MHz (USB EXACTO)

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(SLEEP_MODEM_GPIO_Port, SLEEP_MODEM_Pin, GPIO_PIN_SET);

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

  /*Configure GPIO pin : SLEEP_MODEM_Pin */
  GPIO_InitStruct.Pin = SLEEP_MODEM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SLEEP_MODEM_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Flash_Write_Log(Modem_Error_Log_t *log_entry) {
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

void Modem_Send_AT(const char *cmd)
{
    USBH_CDC_Transmit(&hUsbHostFS, (uint8_t*)cmd, strlen(cmd));
}


void Modem_FSM_Run(void)
{
    modem_event_t evt;

    while (PopEvent(&evt))
    {
        switch (modemState)
        {
            case MODEM_STATE_IDLE:

                // esperar botón → lo maneja Button_Process
                break;

            case MODEM_STATE_CHECK_SIGNAL:

                if (evt == EVT_OK)
                {
                    modemState = MODEM_STATE_CHECK_REG;
                    Modem_Send_AT("AT+CEREG?\r\n");
                }
                else if (evt == EVT_ERROR)
                {
                    modemState = MODEM_STATE_ERROR;
                }
                break;

            case MODEM_STATE_CHECK_REG:

                if (evt == EVT_CREG_REGISTERED)
                {
                    modemState = MODEM_STATE_WAIT_PDN;
                }
                else if (evt == EVT_ERROR)
                {
                    modemState = MODEM_STATE_ERROR;
                }
                break;

            case MODEM_STATE_WAIT_PDN:

                if (evt == EVT_PDN_ACT)
                {
                    modem_listo = 1;
                    modemState = MODEM_STATE_READY;
                }
                break;

            case MODEM_STATE_READY:

                if (evt == EVT_PDN_DEACT)
                {
                    modem_listo = 0;
                    modemState = MODEM_STATE_WAIT_PDN;
                }
                break;

            case MODEM_STATE_ERROR:

                // simple recovery
                modemState = MODEM_STATE_IDLE;
                break;
        }
    }
}

void SMS_FSM_Run(void)
{
    modem_event_t evt;

    while (PopEvent(&evt))
    {
        switch (smsState)
        {
            case SMS_IDLE:

                if (evt == EVT_SMS_RECEIVED)
                {
                    smsState = SMS_READ;
                }
                break;

            case SMS_READ:

                Modem_Send_AT("AT+CMGR=1\r\n"); // índice fijo temporal
                smsState = SMS_PARSE;
                break;

            case SMS_PARSE:
                // TODO: parsear contenido
                smsState = SMS_DELETE;
                break;

            case SMS_DELETE:

                Modem_Send_AT("AT+CMGD=1\r\n");
                smsState = SMS_IDLE;
                break;
        }
    }
}

void Bridge_Process(void)
{
    // Flush CDC → UART
    if (cdc_accum_len > 0 &&
        (HAL_GetTick() - cdc_last_rx_tick) >= CDC_FLUSH_TIMEOUT_MS)
    {
        uint16_t sent = 0;

        while (sent < cdc_accum_len)
        {
            uint16_t chunk = cdc_accum_len - sent;
            if (chunk > CDC_CHUNK_SIZE) chunk = CDC_CHUNK_SIZE;

            HAL_UART_Transmit(&huart1,
                              cdc_accum_buf + sent,
                              chunk,
                              100);

            sent += chunk;
            MX_USB_HOST_Process();
        }

        cdc_accum_len = 0;
    }

    // UART → CDC
    if (uart_line_ready &&
        (Appli_state == APPLICATION_READY) &&
        modem_listo)
    {
        USBH_CDC_Transmit(&hUsbHostFS,
                          uart_cmd_buf,
                          uart_cmd_len);

        uart_cmd_len = 0;
        uart_line_ready = 0;
    }
}

void Button_Process(void)
{
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
    {
        if (modemState == MODEM_STATE_IDLE)
        {
            modemState = MODEM_STATE_CHECK_SIGNAL;
            Modem_Send_AT("AT+CSQ\r\n");
        }

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        timer_led = 3000;
    }

    if (timer_led == 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }
}

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

void Modem_Handle_URC(char* line)
{
    if (strstr(line, "+CGEV: EPS PDN ACT"))
        PushEvent(EVT_PDN_ACT);

    else if (strstr(line, "+CGEV: NW PDN DEACT"))
        PushEvent(EVT_PDN_DEACT);

    else if (strstr(line, "+CMTI:"))
        PushEvent(EVT_SMS_RECEIVED);

    else if (strcmp(line, "OK") == 0)
        PushEvent(EVT_OK);

    else if (strcmp(line, "ERROR") == 0)
        PushEvent(EVT_ERROR);
}

void Modem_Gateway_Service(void) {
    switch (modemState)
    {

        case MODEM_STATE_IDLE:
        	// En el loop principal de este programa se detecta la acción de la presión de un botón y se realiza la siguiente acción:
        	// if(modemState == MODEM_STATE_IDLE) modemState = MODEM_STATE_CHECK_SIGNAL_SEND;
        	// En el loop principal de este programa se llama en forma recurrente a la siguiente función: Modem_Gateway_Service();
            break;

        // --- NIVEL 1: CAPA FÍSICA (RADIO) ---
		case MODEM_STATE_CHECK_SIGNAL_SEND:
			RingBuffer_Clear();
			Modem_Send_AT("AT+CSQ\r\n");
			timeoutTimer = HAL_GetTick();
			modemState = MODEM_STATE_CHECK_SIGNAL_WAIT;
			break;

		case MODEM_STATE_CHECK_SIGNAL_WAIT:
			if (Modem_Find_Response("+CSQ: "))
				{
				// Filtramos señales nulas (99) o críticas (0-7)
				if (Modem_Find_Response("99,") || Modem_Find_Response("0,"))
				{
					Modem_Log("ERROR: SIN COBERTURA", 0, 0);
					modemState = MODEM_STATE_IDLE;
				} else
				{
					modemState = MODEM_STATE_CHECK_REG_SEND; // Señal OK, vamos a registro
				}
			} else if (HAL_GetTick() - timeoutTimer > 2000) {
				modemState = MODEM_STATE_ERROR;
			}
			break;

			// --- NUEVO: NIVEL 2: REGISTRO EN RED (CREG) ---
			case MODEM_STATE_CHECK_REG_SEND:
			    RingBuffer_Clear();
			    Modem_Send_AT("AT+CEREG?\r\n");
			    timeoutTimer = HAL_GetTick();
			    modemState = MODEM_STATE_CHECK_REG_WAIT;
			    break;

			case MODEM_STATE_CHECK_REG_WAIT:
			    // Buscamos las dos opciones de registro exitoso:
			    // +CREG: 0,1 -> Registrado en red local (Home)
			    // +CREG: 0,5 -> Registrado en Roaming (Común en el interior)

				if (Modem_Find_Response("+CEREG: 0,1") || Modem_Find_Response("+CEREG: 0,5"))
				{
			        modemState = MODEM_STATE_CHECK_NET_SEND;
			    }
			    else if (Modem_Find_Response("+CEREG: 0,2"))
			    {
			        // Estado 2: "Searching...". No es un error, pero aún no podemos avanzar.
			        // Si pasan más de 15 seg en este estado, ahí sí vamos a ERROR.
			        if (HAL_GetTick() - timeoutTimer > 15000)
			        {
			            Modem_Log("FALLO REGISTRO (SEARCHING)", 0, 0);
			            modemState = MODEM_STATE_ERROR;
			        }
			    }
			    else if (Modem_Find_Response("+CEREG: 0,0") || Modem_Find_Response("+CEREG: 0,3"))
			    {
			        // 0: No registrado, 3: Registro denegado (Problema de SIM o Antena)
			        Modem_Log("REGISTRO DENEGADO", 0, 0);
			        modemState = MODEM_STATE_ERROR;
			    }
			    else if (Modem_Find_Response("OK"))
			    {
					// Si solo responde OK sin el URC, reintentamos el envío
					//if (HAL_GetTick() - timeoutTimer > 2000) modemState = MODEM_STATE_CHECK_REG_SEND;
			    	modemState = MODEM_STATE_CHECK_REG_SEND;
				}
			    else if (HAL_GetTick() - timeoutTimer > 15000)
			    {
			        modemState = MODEM_STATE_ERROR;
			    }
			    break;

			// --- NIVEL 3: CAPA DE RED (IP) ---
	        case MODEM_STATE_CHECK_NET_SEND:
            RingBuffer_Clear();
            Modem_Send_AT("AT+NETOPEN?\r\n");
            timeoutTimer = HAL_GetTick();
            modemState = MODEM_STATE_CHECK_NET_WAIT;
            break;

        case MODEM_STATE_CHECK_NET_WAIT:
            if (Modem_Find_Response("+NETOPEN: 1"))
            {
                // Red lista, ahora preguntamos por el socket
                modemState = MODEM_STATE_CHECK_SOCKET_SEND;
            }
            else if (Modem_Find_Response("+NETOPEN: 0"))
            {
                // Red caída, hay que configurar APN y abrir
                modemState = MODEM_STATE_CGDCONT_SEND;
            }
            else if (HAL_GetTick() - timeoutTimer > 3000)
            {
                    modemState = MODEM_STATE_ERROR;
            }
            break;

        case MODEM_STATE_CHECK_SOCKET_SEND:
            RingBuffer_Clear();
            Modem_Send_AT("AT+CIPOPEN?\r\n");
            timeoutTimer = HAL_GetTick();
            modemState = MODEM_STATE_CHECK_SOCKET_WAIT;
            break;

        case MODEM_STATE_CHECK_SOCKET_WAIT:
            // 1. Buscamos primero la versión con COMA (Estado abierto)
            if (Modem_Find_Response("+CIPOPEN: 0,")) {
                modemState = MODEM_STATE_SEND_CMD; // Directo al envío
            }
            // 2. Si no estaba la coma, buscamos el patrón de "Cerrado"
            // Pero ojo: busca el fin de línea para estar seguro
            else if (Modem_Find_Response("+CIPOPEN: 0\r\n")) {
                modemState = MODEM_STATE_CIPOPEN_SEND; // Hay que abrirlo
            }
            // ... timeout ...
            break;

        // --- PASO 1: LIMPIEZA ---
        case MODEM_STATE_NETCLOSE_SEND:
            RingBuffer_Clear();
            Modem_Send_AT("AT+NETCLOSE\r\n");
            timeoutTimer = HAL_GetTick();
            modemState = MODEM_STATE_NETCLOSE_WAIT;
            break;

        case MODEM_STATE_NETCLOSE_WAIT:
        	if (Modem_Find_Response("OK") || Modem_Find_Response("+NETCLOSE: 0") || Modem_Find_Response("ERROR")) {
				modemState = MODEM_STATE_CIPCLOSE_SEND;
				timeoutTimer = HAL_GetTick();
        	} else if (HAL_GetTick() - timeoutTimer > 3000) {
       	        modemState = MODEM_STATE_CIPCLOSE_SEND;
        	}
            break;

        case MODEM_STATE_CIPCLOSE_SEND:
            RingBuffer_Clear();
            Modem_Send_AT("AT+CIPCLOSE=0\r\n");
            timeoutTimer = HAL_GetTick();
            modemState = MODEM_STATE_CIPCLOSE_WAIT;
            break;

        case MODEM_STATE_CIPCLOSE_WAIT:
        	// Aceptamos OK o ERROR (si ya estaba cerrado) para seguir adelante
			if (Modem_Find_Response("OK") || Modem_Find_Response("ERROR")) {
				// Si venimos de un ERROR, volvemos a IDLE para que el botón reinicie todo
				// Si venimos de CLEAN_UP (éxito), también.
				modemState = MODEM_STATE_IDLE;
			} else if (HAL_GetTick() - timeoutTimer > 2000) {
				modemState = MODEM_STATE_IDLE;
			}

        	/*if (Modem_Find_Response("OK") || Modem_Find_Response("ERROR")) {
                modemState = MODEM_STATE_CGDCONT_SEND;
            } else if (HAL_GetTick() - timeoutTimer > 2000) {
                modemState = MODEM_STATE_CGDCONT_SEND; // Avanzamos igual, quizás ya estaba cerrado.
            }*/
            break;

        // --- PASO 2: CONFIGURACIÓN ---
        case MODEM_STATE_CGDCONT_SEND:
            RingBuffer_Clear();
            Modem_Send_AT("AT+CGDCONT=1,\"IP\",\"internet.movistar.arg\"\r\n");
            timeoutTimer = HAL_GetTick();
            modemState = MODEM_STATE_CGDCONT_WAIT;
            break;

        case MODEM_STATE_CGDCONT_WAIT:
            if (Modem_Find_Response("OK")) {
                modemState = MODEM_STATE_CSOCKSETPN_SEND;
            } else if (HAL_GetTick() - timeoutTimer > 2000) {
                modemState = MODEM_STATE_ERROR;
            }
            break;

        case MODEM_STATE_CSOCKSETPN_SEND:
            RingBuffer_Clear();
            Modem_Send_AT("AT+CSOCKSETPN=1\r\n");
            timeoutTimer = HAL_GetTick();
            modemState = MODEM_STATE_CSOCKSETPN_WAIT;
            break;

        case MODEM_STATE_CSOCKSETPN_WAIT:
            if (Modem_Find_Response("OK")) {
                modemState = MODEM_STATE_NETOPEN_SEND;
            } else if (HAL_GetTick() - timeoutTimer > 2000) {
                modemState = MODEM_STATE_ERROR;
            }
            break;

         // --- PASO 3: APERTURA DE RED ---
        case MODEM_STATE_NETOPEN_SEND:
            RingBuffer_Clear();
            Modem_Send_AT("AT+NETOPEN\r\n");
            timeoutTimer = HAL_GetTick();
            modemState = MODEM_STATE_NETOPEN_WAIT;
            break;

        case MODEM_STATE_NETOPEN_WAIT:
            // Buscamos el URC confirmando la red abierta.
            if (Modem_Find_Response("+NETOPEN: 0")) {
                modemState = MODEM_STATE_IPADDR_SEND;
            } else if (HAL_GetTick() - timeoutTimer > 10000) { // El NETOPEN puede tardar.
                modemState = MODEM_STATE_ERROR;
            }
            break;

        case MODEM_STATE_IPADDR_SEND:
            RingBuffer_Clear();
            Modem_Send_AT("AT+IPADDR\r\n");
            timeoutTimer = HAL_GetTick();
            modemState = MODEM_STATE_IPADDR_WAIT;
            break;

        case MODEM_STATE_IPADDR_WAIT:
            if (Modem_Find_Response("+IPADDR:")) {
                modemState = MODEM_STATE_CIPOPEN_SEND;
            } else if (HAL_GetTick() - timeoutTimer > 3000) {
                modemState = MODEM_STATE_NETOPEN_SEND; // Reintentar apertura de red.
            }
            break;

        // --- PASO 4: SOCKET Y ENVÍO ---
        case MODEM_STATE_CIPOPEN_SEND:
            RingBuffer_Clear();
            // AT+CIPOPEN=0,"UDP","190.111.217.188",57777,0
            Modem_Send_AT("AT+CIPOPEN=0,\"UDP\",\"190.111.217.188\",57777,0\r\n");
            timeoutTimer = HAL_GetTick();
            modemState = MODEM_STATE_CIPOPEN_WAIT;
            break;

        case MODEM_STATE_CIPOPEN_WAIT:
            if (Modem_Find_Response("+CIPOPEN: 0,0")) {
                modemState = MODEM_STATE_SEND_CMD;
            } else if (HAL_GetTick() - timeoutTimer > 5000) {
                modemState = MODEM_STATE_ERROR;
            }
            break;

        case MODEM_STATE_SEND_CMD:
        	// AT+CIPSEND=0,20,"190.111.217.188",57777
            RingBuffer_Clear();
            Modem_Send_AT("AT+CIPSEND=0,20,\"190.111.217.188\",57777\r\n");
            timeoutTimer = HAL_GetTick();
            modemState = MODEM_STATE_WAIT_PROMPT;
            break;

        case MODEM_STATE_WAIT_PROMPT:
            if (Modem_Find_Response(">")) {
                modemState = MODEM_STATE_SEND_DATA;
            } else if (HAL_GetTick() - timeoutTimer > 3000) {
                modemState = MODEM_STATE_ERROR;
            }
            break;

        case MODEM_STATE_SEND_DATA:
            // Enviamos la alarma actual sin \r\n.
            // Modem_Send_AT(alarm_messages[current_msg_index]);
            USBH_CDC_Transmit(&hUsbHostFS, (uint8_t*)alarm_messages[current_msg_index], 20);

            RingBuffer_Clear();
            timeoutTimer = HAL_GetTick();
            modemState = MODEM_STATE_WAIT_SEND_OK;
            break;

        case MODEM_STATE_WAIT_SEND_OK:
        	// 1. Buscamos la confirmación del envío exitoso.
			if (Modem_Find_Response("+CIPSEND: 0,20,20"))
			{
				// ÉXITO: Marcamos este mensaje como entregado.
				last_confirmed_index = current_msg_index;
				retry_count = 0; // Reset de reintentos locales.

				// 2. Verificamos si terminamos la ráfaga de 10 mensajes.
				// Suponiendo que quieres enviar los 10 de un solo tirón:
				if (current_msg_index < 9)
				{
					// Aún faltan mensajes: incrementamos y volvemos a enviar el comando.
					current_msg_index++;
					modemState = MODEM_STATE_SEND_CMD; // <-- EL SALTO CLAVE
					timeoutTimer = HAL_GetTick();
				}
				else
				{
					// Ya enviamos los 10: reiniciamos índice y cerramos para ahorrar energía.
					current_msg_index = 0;
					burst_mode_active = false;
					modemState = MODEM_STATE_CLEAN_UP_1;
					timeoutTimer = HAL_GetTick();
				}
			}
			else if (HAL_GetTick() - timeoutTimer > 5000)
			{
				// Si hay error en un mensaje de la ráfaga, mejor cerrar y reintentar todo.
                retry_count++;
				if (retry_count < MAX_RETRIES) {
					// Log de diagnóstico para el Minicom.
					Modem_Log("REINTENTO_MSG", current_msg_index, retry_count);
					// REINTENTO "SUAVE": Intentar enviar el mismo mensaje de nuevo
					// Volvemos a pedir el prompt '>'
					modemState = MODEM_STATE_SEND_CMD;
					timeoutTimer = HAL_GetTick();
				} else {
					// REINTENTO "DURO": Agotamos los intentos, vamos a limpieza profunda.
					// Log de fallo total
					Modem_Log("ERROR_FATAL_MSG", current_msg_index, 0);
					retry_count = 0;
					modemState = MODEM_STATE_NETCLOSE_SEND;
				}
			}
			break;

        case MODEM_STATE_CLEAN_UP_1:
        	RingBuffer_Clear();
			Modem_Send_AT("AT+CIPCLOSE=0\r\n");
			timeoutTimer = HAL_GetTick();
			modemState = MODEM_STATE_CLEAN_UP_2;
            break;

        case MODEM_STATE_CLEAN_UP_2:
            if (HAL_GetTick() - timeoutTimer > 1000)
            {
            	RingBuffer_Clear();
                Modem_Send_AT("AT+NETCLOSE\r\n");
                modemState = MODEM_STATE_IDLE;
            }
            break;

        case MODEM_STATE_ERROR:
			// Registro de error para diagnóstico remoto.
			Modem_Log("CRITICAL_FAIL", current_msg_index, retry_count);

            // 2. Si el contador de errores es muy alto, reset físico.
            static uint8_t total_failures = 0;
            total_failures++;

            if (total_failures > 3) {
                // Supongamos que el pin PB12 está conectado al RESET o PWR_KEY del A7670.
            	// RESET FÍSICO (Pin PB12 a transistor/RESET del módem).
            	modemState = MODEM_STATE_IDLE;

                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
                HAL_Delay(500); // Pulso de reset
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
                total_failures = 0;
            }
            else
            {
				// SANEAMIENTO: Vamos a los estados que antes estaban "muertos"
				// para cerrar todo antes de reintentar desde cero.
            	// --- CAJA NEGRA ---
				Modem_Error_Log_t current_error;
				//current_error.timestamp = HAL_GetTick();
				//current_error.last_state = previousState; // Debes guardar el estado anterior antes de cambiar a ERROR
				//current_error.msg_index = current_msg_index;
				//current_error.error_code = last_cme_error;

				// Función que escribe en el último sector de la Flash del STM32F411
				Flash_Write_Log(&current_error);

				timeoutTimer = HAL_GetTick();
				modemState = MODEM_STATE_NETCLOSE_SEND;
            }
            break;
    }
}

void RingBuffer_Write(uint8_t data) {
    uint32_t next = (modemBuffer.head + 1) % RING_BUFFER_SIZE;
    if (next != modemBuffer.tail) {
        modemBuffer.buffer[modemBuffer.head] = data;
        modemBuffer.head = next;
    }
}

int16_t RingBuffer_Read(void) {
    if (modemBuffer.head == modemBuffer.tail) return -1;
    uint8_t data = modemBuffer.buffer[modemBuffer.tail];
    modemBuffer.tail = (modemBuffer.tail + 1) % RING_BUFFER_SIZE;
    return data;
}

void RingBuffer_Clear(void)
{
    modemBuffer.head = modemBuffer.tail = 0;
    searchIdx = 0;
}

void Modem_Log(const char* etiqueta, uint8_t msg_idx, uint8_t intento) {
    char buf[64];
    // Formato: [REINTENTO 1/3] Msg: 4
    if (intento > 0) {
        sprintf(buf, "\r\n[REINTENTO %d/%d] Msg: %d (%s)\r\n",
                intento, MAX_RETRIES, msg_idx, alarm_messages[msg_idx]);
    } else {
        sprintf(buf, "\r\n[ERROR CRITICO] Msg %d fallo tras %d intentos.\r\n",
                msg_idx, MAX_RETRIES);
    }

    // Enviamos al Minicom (ajusta según tu configuración de UART/USB)
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
    // O vía USB CDC:
    // USBH_CDC_Transmit(&hUsbHostFS, (uint8_t*)buf, strlen(buf));
}

void PushEvent(modem_event_t evt)
{
    uint8_t next = (evtQueue.head + 1) % EVENT_QUEUE_SIZE;

    if (next != evtQueue.tail)  // evitar overflow
    {
        evtQueue.queue[evtQueue.head] = evt;
        evtQueue.head = next;
    }
}

bool PopEvent(modem_event_t *evt)
{
    if (evtQueue.head == evtQueue.tail)
        return false;

    *evt = evtQueue.queue[evtQueue.tail];
    evtQueue.tail = (evtQueue.tail + 1) % EVENT_QUEUE_SIZE;

    return true;
}

void EventQueue_Init(void)
{
    evtQueue.head = 0;
    evtQueue.tail = 0;
}

int Parse_SMS_Index(void)
{
    // TODO: implementar parsing real de +CMTI
    return 0;
}

void Modem_Request_Signal(void)
{
    Modem_Send_AT("AT+CSQ\r\n");
    StartTimeout(2000);
}

void Modem_Dispatch(void)
{
    Modem_Process_Buffer();   // genera eventos
    Modem_FSM_Run();          // LTE
    SMS_FSM_Run();            // SMS
}

/*
void Modem_Process_Buffer(void)
{
    if (Modem_Find_Response("OK")) PushEvent(EVT_OK);

    if (Modem_Find_Response("ERROR")) PushEvent(EVT_ERROR);

    if (Modem_Find_Response("+CREG: 0,1")) PushEvent(EVT_CREG_REGISTERED);

    if (Modem_Find_Response("+CREG: 0,2")) PushEvent(EVT_CREG_SEARCHING);

    if (Modem_Find_Response("+NETOPEN: 0")) PushEvent(EVT_NET_OPEN);

    if (Modem_Find_Response(">")) PushEvent(EVT_PROMPT);

    if (Modem_Find_Response("+CIPSEND: 0,20,20")) PushEvent(EVT_SEND_OK);

    if (Modem_Find_Response("+CIPSEND:")) PushEvent(EVT_SEND_OK);

    if (Modem_Find_Response("+CGEV: EPS PDN ACT") || Modem_Find_Response("+CGEV: ME PDN ACT")) PushEvent(EVT_PDN_ACT);

    if (Modem_Find_Response("+CGEV: NW PDN DEACT") || Modem_Find_Response("+CGEV: ME PDN DEACT")) PushEvent(EVT_PDN_DEACT);

    if (Modem_Find_Response("+CMTI:"))
    {
        sms_index = Parse_SMS_Index();
        PushEvent(EVT_SMS_RECEIVED);
    }

}
*/

void Modem_Process_Buffer(void)
{
	char line[128];
	char dbg[160];

    while (Modem_Get_Line(line))
    {
        Modem_Handle_URC(line);
    }

    sprintf(dbg, "\r\n[LINE]: %s\r\n", line);
	HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
}

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

/*
bool Modem_Find_Response(const char* expected)
{
    int16_t byte;
    size_t targetLen = strlen(expected);

    while ((byte = RingBuffer_Read()) != -1)
    {
        if ((uint8_t)byte == expected[searchIdx])
        {
            searchIdx++;
            if (searchIdx == targetLen)
            {
                searchIdx = 0; // Encontrado
                return true;
            }
        }
        else
        {
            // El truco para no perder caracteres en patrones repetitivos
        	searchIdx = ( (uint8_t)byte == expected[0] ) ? 1 : 0;
        }
    }
    return false;
}


bool Modem_Find_Response(const char* expected)
{
    int16_t byte;
    size_t targetLen = strlen(expected);

    while ((byte = RingBuffer_Read()) != -1)
    {
        if ((uint8_t)byte == expected[searchIdx])
        {
            searchIdx++;
            if (searchIdx == targetLen)
            {
                searchIdx = 0; // Encontrado
                return true;
            }
        } else
        {
            searchIdx = 0;
        }
    }
    return false;
}*/



/**
 * Callback UART: byte recibido de la PC.
 * - Eco inmediato al terminal.
 * - Acumula en buffer hasta \r → marca línea lista.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* SIN eco aquí — se hace en el main loop para no bloquear la recepción */
    	// Eco inmediato: el usuario ve lo que escribe
        // HAL_UART_Transmit(&huart1, &uart_rx_char, 1, 5);

        if (uart_cmd_len < (UART_BUF_SIZE - 2))
        {
            uart_cmd_buf[uart_cmd_len++] = uart_rx_char;
            uart_last_rx_tick = HAL_GetTick();

            if (uart_rx_char == '\r')
            {
                uart_cmd_buf[uart_cmd_len++] = '\n';
                uart_line_ready = 1;
            }
        }
        else
        {
            uart_cmd_len    = 0;
            uart_line_ready = 0;
        }
        HAL_UART_Receive_IT(&huart1, &uart_rx_char, 1);
    }
}


/**
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



/**
 * Busca un substring en el buffer CDC acumulado.
 */
static uint8_t buffer_contiene(const char *str)
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
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
