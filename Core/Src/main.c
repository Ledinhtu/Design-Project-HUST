/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiquidCrystal_I2C.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	INIT=0,
	IDLE,
	HEATING,
	MEASURING,
	COMMUNICATING
} StateDevice_Typdef;

typedef struct
{
	uint8_t btn_current;
	uint8_t btn_last;
	uint8_t btn_filter;
	bool is_debouncing;
	uint32_t time_deboune;
	uint32_t time_start_press;
	bool is_press_timeout;
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
} Button_Typdef;

typedef struct
{
	GPIO_TypeDef *HT_GPIOx;
	uint16_t HT_GPIO_Pin;
	uint32_t start_heating_time;
	uint32_t previous_heating_time;
	ADC_HandleTypeDef* hadc;
	uint32_t Channel;
	uint32_t adc_value;
	bool is_heating;
	uint32_t ppm;
} MQSensor_Typdef;

#pragma pack(1)
typedef struct {
	uint8_t flag;
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	uint32_t ppm;
} Sensor_Data_Typdef;
#pragma pack(1)

typedef struct {
	HAL_StatusTypeDef status;
	uint32_t pageError;

} MLB_EraseError;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RTC_BKP_SET_TIME 0x2606
#define LEL 1000
#define SHORT_PRESS_TIME 1000
#define HOLD_DOWN_TIME 3000

#define sizeofBuff 20

#define ADDRESS_DATA_STORAGE (0x08000000 + 63*1024)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
LiquidCrystal_I2C hlcd;
uint8_t press_count = 0;
uint8_t double_count = 0;
uint8_t hold_count = 0;
bool flag = false;
Button_Typdef button1;
MQSensor_Typdef MQSensor;
StateDevice_Typdef stateDevice = INIT;
Sensor_Data_Typdef data;

uint8_t rx_buffer[sizeofBuff];
uint8_t tx_buffer[sizeofBuff];
bool uart_flag = false;
bool ext_param = false;
bool disp_flag = true;
uint8_t ret;

uint8_t arr_w[10] = {1,2,3,4,5,6,7,8,9,10};
uint8_t arr_r[10];
Sensor_Data_Typdef data_r;
Sensor_Data_Typdef data_w;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

void MQSensor_powerOn(MQSensor_Typdef *sensor, bool power);
void MQSensor_Init(MQSensor_Typdef *sensor, GPIO_TypeDef *HT_GPIOx, uint16_t HT_GPIO_Pin, ADC_HandleTypeDef* hadc,  uint32_t Channel);
HAL_StatusTypeDef MQSensor_get_adc(MQSensor_Typdef *sensor);
void MQSensor_powerOn(MQSensor_Typdef *sensor, bool power);
void MQSensor_calc(MQSensor_Typdef *sensor);

void btn_pressing_callback(Button_Typdef *ButtonX);
void btn_press_short_callback(Button_Typdef *ButtonX);
void btn_release_callback(Button_Typdef *ButtonX);
void btn_press_timeout_callback(Button_Typdef *ButtonX);
void button_init(Button_Typdef *ButtonX,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void button_handle(Button_Typdef *ButtonX);

void heating_display();
void measuring_display(bool isCpltMeas);
void idle_display();
void store_data();
void warning();
void send_OK();
void send_allData();
void communicating_handle();
void uart_handle();

HAL_StatusTypeDef send_test();

MLB_EraseError Flash_Earse(uint32_t address);

HAL_StatusTypeDef Flash_Write_Int(uint32_t address, int value);
void Flash_Write_Float(uint32_t address, float value);
HAL_StatusTypeDef Flash_Write_Array(uint32_t address, uint8_t *arr,  uint16_t len);
HAL_StatusTypeDef Flash_Write_Struct(uint32_t address, Sensor_Data_Typdef data);

int	Flash_Read_Int(uint32_t address);
float Flash_Read_Fload(uint32_t address);
HAL_StatusTypeDef Flash_Read_Array(uint32_t address, uint8_t *arr,  uint16_t len);
HAL_StatusTypeDef Flash_Read_Struct(uint32_t address, Sensor_Data_Typdef *data);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void btn_pressing_callback(Button_Typdef *ButtonX)
{
	if (stateDevice == IDLE) {
		MQSensor_powerOn(&MQSensor, true);
		MQSensor.start_heating_time = HAL_GetTick();
		stateDevice = HEATING;
		lcd_clear_display(&hlcd);
		return;
	}

	if (stateDevice == HEATING) {
		lcd_clear_display(&hlcd);
		MQSensor_powerOn(&MQSensor, false);
		stateDevice = IDLE;
		return;
	}

	if (stateDevice == COMMUNICATING) {
		lcd_clear_display(&hlcd);
		stateDevice = IDLE;
		char str[] = "AHT03\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), 500);
		return;
	}

}

void btn_press_short_callback(Button_Typdef *ButtonX)
{

}

void btn_release_callback(Button_Typdef *ButtonX)
{

}

void btn_press_timeout_callback(Button_Typdef *ButtonX)
{
	hold_count++;
	flag = true;
}

void button_init(Button_Typdef *ButtonX,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	ButtonX->GPIOx = GPIOx;
	ButtonX->GPIO_Pin = GPIO_Pin;
}

void button_handle(Button_Typdef *ButtonX)
{
	/* */
	uint8_t sta = HAL_GPIO_ReadPin(ButtonX->GPIOx, ButtonX->GPIO_Pin);
	if (sta != ButtonX->btn_filter) {
		ButtonX->btn_filter = sta;
		ButtonX->is_debouncing = true;
		ButtonX->time_deboune = HAL_GetTick();
	}

	/* */
	if ((sta == ButtonX->btn_filter) && (HAL_GetTick() - ButtonX->time_deboune) > 15) {
		ButtonX->is_debouncing = false;
		ButtonX->btn_current = ButtonX->btn_filter;
	}

	/* */
	if (ButtonX->btn_current != ButtonX->btn_last)
	{
		if(ButtonX->btn_current == GPIO_PIN_RESET) // Press the button
		{
			ButtonX->is_press_timeout = true;
			btn_pressing_callback(ButtonX);
			ButtonX->time_start_press = HAL_GetTick();
		}
		else if(ButtonX->btn_current == GPIO_PIN_SET) // Release the button
		{
			if(HAL_GetTick() - ButtonX->time_start_press <= SHORT_PRESS_TIME)
			{
				btn_press_short_callback(ButtonX);
			}
			btn_release_callback(ButtonX);
			ButtonX->is_press_timeout = false;
		}
		ButtonX->btn_last = ButtonX->btn_current;
	}

	/* */
	if(ButtonX->is_press_timeout && (HAL_GetTick() - ButtonX->time_start_press >= HOLD_DOWN_TIME))
	{
		ButtonX->is_press_timeout = false;
		btn_press_timeout_callback(ButtonX);
	}

} /* END button_handle */

void MQSensor_Init(MQSensor_Typdef *sensor, GPIO_TypeDef *HT_GPIOx, uint16_t HT_GPIO_Pin, ADC_HandleTypeDef* hadc,  uint32_t Channel)
{
	sensor->HT_GPIOx = HT_GPIOx;
	sensor->HT_GPIO_Pin = HT_GPIO_Pin;
	sensor->hadc = hadc;
	sensor->Channel = Channel;

	HAL_ADCEx_Calibration_Start(sensor->hadc);

}

HAL_StatusTypeDef MQSensor_get_adc(MQSensor_Typdef *sensor)
{
	MQSensor_powerOn(&MQSensor, true);
	HAL_Delay(10);
	HAL_StatusTypeDef ret = HAL_OK;
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = sensor->Channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(sensor->hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	ret = HAL_ADC_Start(sensor->hadc);
	if(ret) {
		MQSensor_powerOn(&MQSensor, false);
		return ret;
	}
	ret = HAL_ADC_PollForConversion(sensor->hadc, 1000);
	if(ret) {
		MQSensor_powerOn(&MQSensor, false);
		return ret;
	}
	sensor->adc_value = HAL_ADC_GetValue(sensor->hadc);
	ret = HAL_ADC_Stop(sensor->hadc);
	if(ret) {
		MQSensor_powerOn(&MQSensor, false);
		return ret;
	}
	MQSensor_powerOn(&MQSensor, false);
	return ret;
}

void MQSensor_powerOn(MQSensor_Typdef *sensor, bool power)
{
	HAL_GPIO_WritePin(sensor->HT_GPIOx, sensor->HT_GPIO_Pin, power ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void MQSensor_calc(MQSensor_Typdef *sensor)
{
	sensor->ppm = sensor->adc_value;

}

MLB_EraseError Flash_Earse(uint32_t address)
{
	MLB_EraseError eraseError;
	eraseError.status = HAL_ERROR;
	eraseError.pageError = 0;

	FLASH_EraseInitTypeDef EraseInit;
	EraseInit.Banks = FLASH_BANK_1;
	EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInit.PageAddress = address;
	EraseInit.NbPages = 1;

	eraseError.status = HAL_FLASH_Unlock();
	if(eraseError.status)
		return eraseError;

	eraseError.status = HAL_FLASHEx_Erase(&EraseInit, &eraseError.pageError);
	if(eraseError.status)
		return eraseError;

	eraseError.status = HAL_FLASH_Lock();
	if(eraseError.status)
		return eraseError;

	return eraseError;
}

HAL_StatusTypeDef Flash_Write_Array(uint32_t address, uint8_t *arr,  uint16_t len)
{
	HAL_StatusTypeDef ret = HAL_OK;
	uint16_t *pt = (uint16_t *)arr;
	ret = HAL_FLASH_Unlock();
	if (ret)
		return ret;
	for (uint16_t i = 0; i < (len+1)/2; ++i) {
		ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + 2*i, *pt);
		if (ret)
			return ret;
		pt++;
	}
	ret = HAL_FLASH_Lock();
	if (ret)
		return ret;

	return ret;
}

HAL_StatusTypeDef Flash_Read_Array(uint32_t address, uint8_t *arr,  uint16_t len)
{
	HAL_StatusTypeDef ret = HAL_OK;
	uint16_t *pt = (uint16_t *)arr;
	ret = HAL_FLASH_Unlock();
	if (ret)
		return ret;
	for (uint16_t i = 0; i < (len+1)/2; ++i) {
		*pt = *(uint16_t *)(address + 2*i);
		if (ret)
			return ret;
		pt++;
	}
	ret = HAL_FLASH_Lock();
	if (ret)
		return ret;
	return ret;
}

HAL_StatusTypeDef Flash_Write_Struct(uint32_t address, Sensor_Data_Typdef data)
{
	HAL_StatusTypeDef ret = Flash_Write_Array(address, (uint8_t*)&data, sizeof(data));
	return ret;
}

HAL_StatusTypeDef Flash_Read_Struct(uint32_t address, Sensor_Data_Typdef *data)
{
	HAL_StatusTypeDef ret = Flash_Read_Array(address, (uint8_t*)data, sizeof(Sensor_Data_Typdef));
	return ret;
}

void heating_display()
{
	if ((HAL_GetTick()-MQSensor.previous_heating_time) < 60000 && MQSensor.is_heating) {
		stateDevice = MEASURING;
		return;
	}
	uint8_t tick = 20 - (HAL_GetTick()-MQSensor.start_heating_time)/1000;
	lcd_set_cursor(&hlcd, 0, 0);
	lcd_printf(&hlcd, "Heating %2ds", tick);
	if (tick == 255) {
		MQSensor.previous_heating_time = HAL_GetTick();
		MQSensor.is_heating = true;
		stateDevice = MEASURING;
	}
}

void measuring_display(bool isCpltMeas)
{
	if (isCpltMeas)
	{
		lcd_clear_display(&hlcd);
		lcd_set_cursor(&hlcd, 0, 0);
		lcd_printf(&hlcd, "ADC value %4ld", MQSensor.adc_value);
		stateDevice = IDLE;

	}
	else {
		lcd_clear_display(&hlcd);
		lcd_set_cursor(&hlcd, 0, 0);
		lcd_printf(&hlcd, "Measuring ...");
	}
}

void idle_display()
{
	lcd_set_cursor(&hlcd, 1, 0);
	lcd_printf(&hlcd, "Press to measure");
}

void store_data()
{
	HAL_RTC_GetTime(&hrtc, &data.sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &data.sDate, RTC_FORMAT_BIN);
	data.ppm = MQSensor.ppm;

	// store
}

void warning()
{
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, (MQSensor.ppm > LEL) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void send_OK()
{
	char str[] = "AHT02\n";
	HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), 500);
}

void send_allData()
{

}

HAL_StatusTypeDef send_test()
{
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDay = {0};
	uint8_t tx_buff[50];
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	sprintf((char *)tx_buff, "%d:%d:%d\n", sTime.Hours , sTime.Minutes, sTime.Seconds);
	ret = HAL_UART_Transmit(&huart1, tx_buff, strlen((char*)tx_buff), 1000);

	HAL_RTC_GetDate(&hrtc, &sDay, RTC_FORMAT_BIN);
	sprintf((char *)tx_buff, "%d-%d/%d/%d\n", sDay.WeekDay , sDay.Date, sDay.Month, sDay.Year);
	ret = HAL_UART_Transmit(&huart1, tx_buff, strlen((char*)tx_buff), 1000);

	uint32_t check = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
	sprintf((char *)tx_buff, "RTC_BKP_DR2=%2ld\n", check);
	ret = HAL_UART_Transmit(&huart1, tx_buff, strlen((char*)tx_buff), 1000);

	/*
	Flash_Earse(ADDRESS_DATA_STORAGE);
	Flash_Write_Array(ADDRESS_DATA_STORAGE, arr_w, 10);
	Flash_Read_Array(ADDRESS_DATA_STORAGE, arr_r, 10);
	for (uint8_t i = 0; i < 10; ++i) {
		sprintf((char *)tx_buff, "%d\n", arr_r[i]);
		ret = HAL_UART_Transmit(&huart1, tx_buff, strlen((char*)tx_buff), 100);
	}
	*/

	data_w.flag = 0;
	data_w.sDate = sDay;
	data_w.sTime = sTime;
	data_w.ppm = MQSensor.ppm;
	sprintf((char *)tx_buff, "Address data_w=%ld\nSzie data_w=%d\n", (uint32_t)&data_w, sizeof(data_w));
	ret = HAL_UART_Transmit(&huart1, tx_buff, strlen((char*)tx_buff), 1000);

	ret = Flash_Read_Struct(check+((sizeof(Sensor_Data_Typdef)+1)/2)*2, &data_r);
	if ((!check) || (data_r.flag==0)) {
		Flash_Earse(ADDRESS_DATA_STORAGE);
		check = ADDRESS_DATA_STORAGE - ((sizeof(Sensor_Data_Typdef)+1)/2)*2;
	}

//	sprintf((char *)tx_buff, "data_r.flag=%d\n!data_r.flag=%d\n", data_r.flag, !data_r.flag);
//	ret = HAL_UART_Transmit(&huart1, tx_buff, strlen((char*)tx_buff), 1000);

	ret = Flash_Write_Struct(check+((sizeof(Sensor_Data_Typdef)+1)/2)*2, data_w);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, check+((sizeof(Sensor_Data_Typdef)+1)/2)*2);

	check = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
	sprintf((char *)tx_buff, "RTC_BKP_DR2=%2ld\n", check);
	ret = HAL_UART_Transmit(&huart1, tx_buff, strlen((char*)tx_buff), 1000);

	ret = Flash_Read_Struct(check, &data_r);
	sprintf((char *)tx_buff, "Time=%d:%d:%d\n", data_r.sTime.Hours , data_r.sTime.Minutes, data_r.sTime.Seconds);
	ret = HAL_UART_Transmit(&huart1, tx_buff, strlen((char*)tx_buff), 1000);
	sprintf((char *)tx_buff, "Date=%d-%d/%d/%d\n", data_r.sDate.WeekDay , data_r.sDate.Date, data_r.sDate.Month, data_r.sDate.Year);
	ret = HAL_UART_Transmit(&huart1, tx_buff, strlen((char*)tx_buff), 1000);
	sprintf((char *)tx_buff, "PPM=%ld\n", data_r.ppm);
	ret = HAL_UART_Transmit(&huart1, tx_buff, strlen((char*)tx_buff), 1000);

	return ret;
}

void communicating_handle()
{
	if (disp_flag) {
		lcd_set_cursor(&hlcd, 0, 0);
		lcd_printf(&hlcd, "Communicating ...");
		disp_flag = false;
	}
	if (uart_flag)
	{
		uart_flag = false;

		// Request Connect
		if (!strcmp((char *)rx_buffer, "AT01")) {
			send_OK();
			memset((char *)rx_buffer, '\0', sizeofBuff);
			ret = HAL_UART_Receive_IT(&huart1, rx_buffer, 5);
			return;
		}

		// Close Connect
		if (!strcmp((char *)rx_buffer, "AT04")) {
			send_OK();
			memset((char *)rx_buffer, '\0', sizeofBuff);
			ret = HAL_UART_Receive_IT(&huart1, rx_buffer, 5);
			stateDevice = IDLE;
			lcd_clear_display(&hlcd);
			return;
		}

		// TEST
		if (!strcmp((char *)rx_buffer, "AT05")) {
//			send_OK();
			memset((char *)rx_buffer, '\0', sizeofBuff);
			ret = HAL_UART_Receive_IT(&huart1, rx_buffer, 5);
			send_test();
			return;
		}

		// all Data
		if (!strcmp((char *)rx_buffer, "AT06")) {
			send_allData();
			return;
		}

		// set Time
		if (!strcmp((char *)rx_buffer, "AT07")) {
			send_OK();
			memset((char *)rx_buffer, '\0', sizeofBuff);
			ret = HAL_UART_Receive_IT(&huart1, rx_buffer, 5);
			return;
		}

		// set Day
		if (!strcmp((char *)rx_buffer, "AT07")) {
			send_OK();
			memset((char *)rx_buffer, '\0', sizeofBuff);
			ret = HAL_UART_Receive_IT(&huart1, rx_buffer, 5);
			return;
		}
	}
}

void uart_handle()
{
	if (uart_flag) {
		if (stateDevice == IDLE) {
			if (!strcmp((char *)rx_buffer, "AT01")) {
				lcd_clear_display(&hlcd);
				stateDevice = COMMUNICATING;
				disp_flag = true;
				return;
			}
			else
			{
				memset((char *)rx_buffer, '\0', sizeofBuff);
				HAL_UART_Receive_IT(&huart1, rx_buffer, 5);
				return;
			}
		}

		if (stateDevice == COMMUNICATING) {

		}
	}
}


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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  lcd_init(&hlcd, &hi2c2, LCD_ADDR_DEFAULT);
  button_init(&button1, GPIOA, GPIO_PIN_8);
  MQSensor_Init(&MQSensor, HT_CTRL_GPIO_Port, HT_CTRL_Pin, &hadc1, ADC_CHANNEL_0);

  stateDevice = IDLE;

  disp_flag = true;
  HAL_UART_Receive_IT(&huart1, rx_buffer, 5);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  button_handle(&button1);
	  switch (stateDevice) {
	  	case IDLE:
	  		idle_display();
	  		uart_handle();
	  		break;
		case HEATING:
			heating_display();
			break;
		case MEASURING:
			measuring_display(false);
			MQSensor_get_adc(&MQSensor);
			MQSensor_calc(&MQSensor);
			warning();
			measuring_display(true);
			store_data();
			break;
		case COMMUNICATING:
			communicating_handle();
			break;
		default:
			stateDevice = IDLE;
			break;
	}

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */
  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */
  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */
  /* USER CODE END I2C2_Init 2 */

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
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != RTC_BKP_SET_TIME)
  {
	  	sTime.Hours = 18;
	    sTime.Minutes = 5;
	    sTime.Seconds = 0;

	    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	    {
	      Error_Handler();
	    }
	    DateToUpdate.WeekDay = RTC_WEEKDAY_SATURDAY;
	    DateToUpdate.Month = RTC_MONTH_SEPTEMBER;
	    DateToUpdate.Date = 23;
	    DateToUpdate.Year = 23;

	    if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
	    {
	      Error_Handler();
	    }
  }

  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, RTC_BKP_SET_TIME);
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 17;
  sTime.Minutes = 56;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_SATURDAY;
  DateToUpdate.Month = RTC_MONTH_SEPTEMBER;
  DateToUpdate.Date = 23;
  DateToUpdate.Year = 23;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|HT_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin HT_CTRL_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|HT_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
  if (GPIO_Pin == GPIO_PIN_8) {

  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
  uart_flag = true;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
