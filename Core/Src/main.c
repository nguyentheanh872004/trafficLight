/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include"string.h"
#include"stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Định nghĩa thời gian cho các đèn (tính bằng giây)
volatile uint8_t TIME_GREEN = 10;
volatile uint8_t TIME_YELLOW = 3;
volatile uint8_t TIME_RED = 13;
volatile uint8_t time_left_lane1 = 0;
volatile uint8_t time_left_lane2 = 0;
volatile uint8_t flag_scan_keypad = 0;
volatile uint8_t adjust_mode = 0; // Biến để kích hoạt chế độ điều chỉnh
volatile uint8_t new_green_time = 0; // Lưu thời gian mới cho đèn xanh
volatile uint8_t new_yellow_time = 0; // Lưu thời gian mới cho đèn vàng
volatile uint8_t input_step = 0; // 0: nhập xanh, 1: nhập vàng
volatile uint8_t c_pressed = 0; // Biến theo dõi trạng thái phím 'C'
// Định nghĩa các trạng thái
enum TrafficState {
    STATE1_L1_GREEN_L2_RED,
    STATE2_L1_YELLOW_L2_RED,
    STATE3_L1_RED_L2_GREEN,
    STATE4_L1_RED_L2_YELLOW
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define row1 GPIO_PIN_8
#define row2 GPIO_PIN_9
#define row3 GPIO_PIN_10
#define row4 GPIO_PIN_11

#define col1 GPIO_PIN_12
#define col2 GPIO_PIN_13
#define col3 GPIO_PIN_14
#define col4 GPIO_PIN_15

//static uint8_t time_receive=15;
uint8_t system_started = 0;  // 0 = chưa bắt đầu, 1 = đã nhấn A và bắt đầu
uint8_t mode=0;
uint8_t cnt=0;
char key_adjust=0;
char key_press=0;
static char key_pressed=0;
char led7[10]={0xFC,0x60,0xDA,0xF2,0x66,0xB6,0xBE,0xE0,0xFE,0xF6};
uint16_t row[4]={row1,row2,row3,row4};
uint16_t col[4]={col1,col2,col3,col4};
char buffer[2];
uint8_t buffer_index=0;
char key_pad[4][4]=
{
		{'1','2','3','A'},
		{'4','5','6','B'},
		{'7','8','9','C'},
		{'*','0','#','D'}
};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
// Biến toàn cục để lưu trạng thái hiện tại và bộ đếm thời gian
volatile enum TrafficState currentState = STATE1_L1_GREEN_L2_RED;
volatile uint8_t timer_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void setLightState(enum TrafficState state) {
    // Tắt hết đèn trước khi bật trạng thái mới để tránh xung đột
    // Lưu ý: Tất cả các chân đều thuộc GPIOA

		HAL_GPIO_WritePin(GPIOA, Red_Lane1_Pin|Yellow_Lane1_Pin|Green_Lane1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, Red_Lane2_Pin|Yellow_Lane2_Pin|Green_Lane2_Pin, GPIO_PIN_RESET);

		switch (state) {
			case STATE1_L1_GREEN_L2_RED:
				// Làn 1 Xanh, Làn 2 Đỏ
				HAL_GPIO_WritePin(Green_Lane1_GPIO_Port, Green_Lane1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Red_Lane2_GPIO_Port, Red_Lane2_Pin, GPIO_PIN_SET);
				break;

			case STATE2_L1_YELLOW_L2_RED:
				// Làn 1 Vàng, Làn 2 Đỏ
				HAL_GPIO_WritePin(Yellow_Lane1_GPIO_Port, Yellow_Lane1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Red_Lane2_GPIO_Port, Red_Lane2_Pin, GPIO_PIN_SET);
				break;

			case STATE3_L1_RED_L2_GREEN:
				// Làn 1 Đỏ, Làn 2 Xanh
				HAL_GPIO_WritePin(Red_Lane1_GPIO_Port, Red_Lane1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Green_Lane2_GPIO_Port, Green_Lane2_Pin, GPIO_PIN_SET);
				break;

			case STATE4_L1_RED_L2_YELLOW:
				// Làn 1 Đỏ, Làn 2 Vàng
				HAL_GPIO_WritePin(Red_Lane1_GPIO_Port, Red_Lane1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Yellow_Lane2_GPIO_Port, Yellow_Lane2_Pin, GPIO_PIN_SET);
				break;

	}

}
void scan_keypad(void)
{
    for(uint8_t i = 0; i < 4; i++)
    {
        HAL_GPIO_WritePin(GPIOD, row[i], GPIO_PIN_SET);
        for(uint8_t j = 0; j < 4; j++)
        {
            if(HAL_GPIO_ReadPin(GPIOD, col[j]) == GPIO_PIN_SET)
            {
                key_press = key_pad[i][j];
                key_pressed = key_press;

                // Nếu đang ở chế độ chỉnh giờ thì gán riêng ra key_adjust
                if (adjust_mode)
                    key_adjust = key_press;

                while(HAL_GPIO_ReadPin(GPIOD, col[j]) == GPIO_PIN_SET);
            }
            else key_press = 0;
        }
        HAL_GPIO_WritePin(GPIOD, row[i], GPIO_PIN_RESET);
    }
}
void processTwoDigitInput() {
    if (buffer_index == 2) {
        uint8_t value = (buffer[0] - '0') * 10 + (buffer[1] - '0');
        buffer_index = 0; // Reset buffer index
        memset(buffer, 0, sizeof(buffer)); // Xóa buffer

        if (input_step == 0) {
            new_green_time = value; // Gán giá trị cho đèn xanh
            input_step = 1; // Chuyển sang nhập thời gian vàng
        } else if (input_step == 1) {
            new_yellow_time = value; // Gán giá trị cho đèn vàng
            input_step = 0; // Reset bước nhập
            adjust_mode = 0; // Thoát chế độ điều chỉnh

            // Cập nhật thời gian vào biến toàn cục
            TIME_GREEN = new_green_time;
            TIME_YELLOW = new_yellow_time;
            TIME_RED = TIME_GREEN + TIME_YELLOW; // Tự động tính thời gian đỏ
        }
    }
}
void string_keypad() {
    if (adjust_mode && key_adjust != 0 && buffer_index < 2) {
        if (key_adjust >= '0' && key_adjust <= '9') {
            buffer[buffer_index] = key_adjust;
            buffer_index++;
            if (buffer_index == 2) {
                processTwoDigitInput();
            }
        }
        key_adjust = 0; // ✅ Reset sau khi xử lý xong để tránh lặp
    } else if (key_pressed != 0) {
        if (key_pressed == '#') {
            mode = 0;
            adjust_mode = 0;
            input_step = 0;
            buffer_index = 0;
            memset(buffer, 0, sizeof(buffer));
        } else if (key_pressed == 'C') {
            c_pressed = 1;
        }
    }
}

void led7dispaly(int number)
{
	uint8_t digit=led7[number];
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, digit>>7 & 0x01  ); //A
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, digit>>6 & 0x01  ); //B
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, digit>>5 & 0x01  ); //C
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, digit>>4 & 0x01  ); //D
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, digit>>3 & 0x01  ); //E
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, digit>>2 & 0x01  ); //F
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, digit>>1 & 0x01  ); //G
}
void led7dispaly_1(int number)
{
	uint8_t digit=led7[number];
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, digit>>7 & 0x01  ); //A
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, digit>>6 & 0x01  ); //B
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, digit>>5 & 0x01  ); //C
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, digit>>4 & 0x01  ); //D
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, digit>>3 & 0x01  ); //E
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, digit>>2 & 0x01  ); //F
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, digit>>1 & 0x01  ); //G
}
void quetLed_combined(uint8_t a,uint8_t b)	//tạo hàm 2 số tương ứng 2 làn( cái này chắc phải update thêm khi có đèn xanh và vàng )
{
    uint8_t chuc = a / 10;
    uint8_t donvi = a % 10;

    uint8_t chuc_1 = b / 10;
	uint8_t donvi_1 = b % 10;
    // Hiển thị chữ số đơn vị trên màn hình đầu tiên (GPIOB)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    led7dispaly(donvi); // Màn hình đầu tiên
    HAL_Delay(1);

    // Hiển thị chữ số chục trên màn hình đầu tiên (GPIOB)	//Đây là làn 1 thì v ới PB8 và PB9 là đơn vị và chục
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    led7dispaly(chuc); // Màn hình đầu tiên
    HAL_Delay(1);

    // Hiển thị chữ số đơn vị trên màn hình thứ hai (GPIOA/GPIOC)	////Đây là làn 2 thì v ới PC8 và PC9 là đơn vị và chục
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    led7dispaly_1(donvi_1); // Màn hình thứ hai
    HAL_Delay(1);

    // Hiển thị chữ số chục trên màn hình thứ hai (GPIOA/GPIOC)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    led7dispaly_1(chuc_1); // Màn hình thứ hai
    HAL_Delay(1);
}
void turnoff_led()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, Red_Lane1_Pin|Yellow_Lane1_Pin|Green_Lane1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Red_Lane2_Pin|Yellow_Lane2_Pin|Green_Lane2_Pin, GPIO_PIN_RESET);
}
//void lan1_2(uint8_t counter,uint8_t timeLine)
//{
//	static time = timeLine;
//	if(counter == 10)
//	{
//		time--;
//	}
//	if(time == 0)
//	{
//		time = timeLine;
//	}
//}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(key_pressed=='B'||key_pressed=='C')
		    {
		system_started=0;
		    }
	if (htim->Instance == TIM2) {
	    if (key_pressed == 'A' && system_started == 0) {
	        system_started = 1;
	        currentState = STATE1_L1_GREEN_L2_RED;
	        setLightState(currentState);
	        timer_counter = 0;
	    }

	    if (system_started == 1 &&key_pressed !='B'&&key_pressed !='C') {
	        timer_counter++;

	        switch (currentState) {
	            case STATE1_L1_GREEN_L2_RED:
	                time_left_lane1 = TIME_GREEN - timer_counter;
	                time_left_lane2 = TIME_RED - timer_counter;
	                if (timer_counter >= TIME_GREEN) {
	                    currentState = STATE2_L1_YELLOW_L2_RED;
	                    setLightState(currentState);
	                    timer_counter = 0;
	                }
	                break;

	            case STATE2_L1_YELLOW_L2_RED:
	                time_left_lane1 = TIME_YELLOW - timer_counter;
	                time_left_lane2 = TIME_YELLOW - timer_counter;
	                if (timer_counter >= TIME_YELLOW) {
	                    currentState = STATE3_L1_RED_L2_GREEN;
	                    setLightState(currentState);
	                    timer_counter = 0;
	                }
	                break;

	            case STATE3_L1_RED_L2_GREEN:
	                time_left_lane1 = TIME_RED - timer_counter;
	                time_left_lane2 = TIME_GREEN - timer_counter;
	                if (timer_counter >= TIME_GREEN) {
	                    currentState = STATE4_L1_RED_L2_YELLOW;
	                    setLightState(currentState);
	                    timer_counter = 0;
	                }
	                break;

	            case STATE4_L1_RED_L2_YELLOW:
	                time_left_lane1 = TIME_YELLOW - timer_counter;
	                time_left_lane2 = TIME_YELLOW - timer_counter;
	                if (timer_counter >= TIME_YELLOW) {
	                    currentState = STATE1_L1_GREEN_L2_RED;
	                    setLightState(currentState);
	                    timer_counter = 0;
	                }
	                break;
	        }
	    }
	}

	if (htim->Instance == TIM3) {
	        flag_scan_keypad = 1;
	        if (key_pressed == '*') {
	            if (c_pressed && system_started == 0) { // Kích hoạt chế độ điều chỉnh khi đã nhấn 'C' trước
	                adjust_mode = 1;
	                buffer_index = 0; // Reset buffer
	                new_green_time = 0;
	                new_yellow_time = 0;
	                input_step = 0; // Bắt đầu nhập thời gian xanh
	                c_pressed = 0; // Reset trạng thái 'C' sau khi dùng
	            }
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
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim2);

//	  setLightState(currentState);  // Gọi đúng đèn ngay từ đầu

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
  {
	if(flag_scan_keypad){
	        flag_scan_keypad = 0;
	        scan_keypad();
	        string_keypad();
	    }
	if(key_pressed=='A')
	{
		quetLed_combined(time_left_lane1, time_left_lane2);
	}
	if(key_pressed=='B')
	{
		turnoff_led();
		HAL_GPIO_TogglePin(GPIOA, Yellow_Lane1_Pin);
		HAL_GPIO_TogglePin(GPIOA, Yellow_Lane2_Pin);
		HAL_Delay(200);
		HAL_GPIO_TogglePin(GPIOA, Yellow_Lane1_Pin);
		HAL_GPIO_TogglePin(GPIOA, Yellow_Lane2_Pin);
		HAL_Delay(200);

	}
	if(key_pressed=='C')
	{
		turnoff_led();
	}
	key_adjust=0;
    /* USER CODE END WHILE */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Red_Lane1_Pin|Green_Lane2_Pin|Yellow_Lane1_Pin|Yellow_Lane2_Pin
                          |Green_Lane1_Pin|Red_Lane2_Pin|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pins : Red_Lane1_Pin Green_Lane2_Pin Yellow_Lane1_Pin Yellow_Lane2_Pin
                           Green_Lane1_Pin Red_Lane2_Pin PA8 PA9
                           PA10 PA15 */
  GPIO_InitStruct.Pin = Red_Lane1_Pin|Green_Lane2_Pin|Yellow_Lane1_Pin|Yellow_Lane2_Pin
                          |Green_Lane1_Pin|Red_Lane2_Pin|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
