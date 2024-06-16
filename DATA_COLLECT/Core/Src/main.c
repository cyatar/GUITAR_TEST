/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "crc.h"
#include "i2c.h"
#include "gpio.h"
#include "usart.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "lsm6dsl.h"
#include "lsm6dsl_reg.h"
#include <stdio.h>
#include <stdlib.h>
#include "ssd1306.h"
#include "ssd1306_tests.h"
////////////

#include "ai_platform.h"
#include "network.h"
#include "network_data.h"

/////
/////



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MINI 					5
#define THRESH				1.01
#define NOISE				 200
#define DATA_INPUT_USER 		100
#define AXIS_NUMBER 			1
#define INT_MIN 3

#define DRAWLENGTH 15

///////////////////
uint8_t id = 0x00;
int start = 0;
lsm6dsl_axis3bit16_t data_raw;
volatile  uint32_t dataRdyIntReceived; // IT
stmdev_ctx_t dev_ctx;
static int32_t platform_write(void *handle, uint8_t Reg,
                              uint8_t *Bufp,
                              uint16_t len)
{
  if (handle == &hi2c1) {
    HAL_I2C_Mem_Write(handle, LSM6DSL_I2C_ADD_H, Reg,
                      I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }

  return 0;
}

static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len)
{
  if (handle == &hi2c1) {
    HAL_I2C_Mem_Read(handle, LSM6DSL_I2C_ADD_H, Reg,
                     I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }

  return 0;
}
int fputc(int ch, FILE *fp)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1,0xffff);
  return ch;
}
/////////////////////////






///////
////DRAW THE PICTURE
int times = 0;
int DRAW_POINT[DRAWLENGTH] = {63,63,63,63,63,63,63,63,63,63,63,63,63,63,63}; //127 * 63
int DRAW = -1;
int L_DRAW = -1;
////
///////AI PART


	
ai_handle network;
float aiInData[AI_NETWORK_IN_1_SIZE];
float aiOutData[AI_NETWORK_OUT_1_SIZE];
int output = 0;
uint32_t write_index = 0;
ai_buffer * ai_input;
ai_buffer * ai_output;
ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
char* OUT[5] = {
  "AM", "C", "DM","G","..."
};
int count[5] ={0};
uint32_t cls;
	
///////////
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

void get_values(void); // get value 
int strum_trigger(void); // trigger
static uint32_t argmax(const float * values, uint32_t len);
int argmax_threshold(int* array, int length, int threshold);
void normalize(float * array, int length);
/////////
void drawLineWithYCoords(int* yCoords, uint8_t length);
void shiftArray(int* arr, int length, int newHead) ;
int scaleToRange(int num);
//////////
static void AI_Init(void);
static void AI_Run(float *pIn, float *pOut);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_I2C3_Init();
  MX_I2C1_Init();
  MX_CRC_Init();
	MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	///////// LSM PART //////////////
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &hi2c1;
	lsm6dsl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	lsm6dsl_xl_data_rate_set(&dev_ctx,LSM6DSL_XL_ODR_6k66Hz);
	lsm6dsl_xl_full_scale_set(&dev_ctx,LSM6DSL_4g);
	int status = 0;
	lsm6dsl_int1_route_t reg;
	status = lsm6dsl_pin_int1_route_get(&dev_ctx, &reg);
	reg.int1_drdy_xl = 1;
	status = lsm6dsl_pin_int1_route_set(&dev_ctx, reg);
	lsm6dsl_acceleration_raw_get(&dev_ctx,data_raw.i16bit);
	lsm6dsl_device_id_get(&dev_ctx, &id);
	////////////////////////////////////
	ssd1306_Init();
//	AI_Init();

		
  drawLineWithYCoords(DRAW_POINT,DRAWLENGTH);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		start = strum_trigger();
		
		if (start != 0) {
		
			while(write_index < DATA_INPUT_USER){
			
					get_values();
					aiInData[write_index ] = data_raw.i16bit[0];
					
					write_index ++ ;
			}
			
//			normalize(aiInData, AI_NETWORK_IN_1_SIZE);
			for(int i=0;i<DATA_INPUT_USER;i++){
					
					printf("%8.6f\n",aiInData[DATA_INPUT_USER*0 + i]);
			}

			if(write_index == AI_NETWORK_IN_1_SIZE ){
					write_index = 0;
//					AI_Run(aiInData, aiOutData);
//				  cls = argmax(aiOutData, AI_NETWORK_OUT_1_SIZE);			
//					count[cls] +=1 ;	
				}
		}
		else{
			
//				output = argmax_threshold(count,AI_NETWORK_OUT_1_SIZE,0);
				
//			if(output == -1){
//				

//				for (uint32_t i = 0; i < AI_NETWORK_OUT_1_SIZE; i++) {
//						count[i] =0 ;
//                 }
//			 if(DRAW != -1){
//					L_DRAW --;
//				  if(L_DRAW ==-3){
//					
//					DRAW =-1;
//					L_DRAW = -1;
//					}
//			 }
//								 

//			}
//			
//			/////////////////////////
//			if(output != -1){
//			  DRAW = output;
//				for (uint32_t i = 0; i < AI_NETWORK_OUT_1_SIZE; i++) {
//						count[i] =0 ;
//                 }
//				
//			}
			
		}
		 
		
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


//////////////////////// PART NOAI//////////////////
void normalize(float * array, int length){
    double sum = 0.0, mean, standardDeviation = 0.0;

    int i;

    for(i = 0; i < length; ++i) {
        sum += array[i];
    }

    mean = sum/length;

    for(i = 0; i < length; ++i)
        standardDeviation += pow(array[i] - mean, 2);

    standardDeviation = sqrt(standardDeviation / length);

    /* normalization */
    for(i = 0; i < length; ++i)
        array[i] = (array[i] - mean) / standardDeviation;
}


int argmax_threshold(int* array, int length, int threshold) {
    int max_value = INT_MIN;
    int max_index = -1;
    for (int i = 0; i < length; i++) {
        if (array[i] > max_value) {
            max_value = array[i];
            max_index = i;
        }
    }
    if (max_value > threshold) {
        return max_index;
    } else {
        return -1;
    }
}

static uint32_t argmax(const float * values, uint32_t len)
{
  float max_value = values[0];
  uint32_t max_index = 0;
  for (uint32_t i = 1; i < len; i++) {
    if (values[i] > max_value) {
      max_value = values[i];
      max_index = i;
    }
  }
  return max_index;
}
int strum_trigger() {
    
			double ref_sum_x = 0.0, ref_sum_y = 0.0, ref_sum_z = 0.0;
			double new_sum_x = 0.0, new_sum_y = 0.0, new_sum_z = 0.0;
			double ref_avg_x = 0.0, ref_avg_y = 0.0, ref_avg_z = 0.0;
			double new_avg_x = 0.0, new_avg_y = 0.0, new_avg_z = 0.0;



	
    for(int i = 0; i < MINI ; i++)
		{
				get_values();
        ref_sum_x += data_raw.i16bit[0];
        ref_sum_y += data_raw.i16bit[1];
        ref_sum_z += data_raw.i16bit[2];
    
		}

    for (uint16_t i = 0; i < MINI ; i++) {
        get_values();
        new_sum_x += data_raw.i16bit[0];
        new_sum_y += data_raw.i16bit[1];
        new_sum_z += data_raw.i16bit[2];
    }
    
    new_avg_x = fabs(new_sum_x/MINI);
    new_avg_y = fabs(new_sum_y/MINI);
    new_avg_z = fabs(new_sum_z/MINI);
    ref_avg_x = fabs(ref_sum_x/MINI);
    ref_avg_y = fabs(ref_sum_y/MINI);
    ref_avg_z = fabs(ref_sum_z/MINI);
    
    if (new_avg_x > NOISE && new_avg_y > NOISE && new_avg_z > NOISE && new_avg_x > NOISE && new_avg_y > NOISE && new_avg_z > NOISE) {
        if (new_avg_x > ref_avg_x*THRESH || new_avg_y > ref_avg_y*THRESH || new_avg_z > ref_avg_z*THRESH) {
            return 1; 
        } else {
            return 0;
        }
    } else {
        return 0;
    }
}

void get_values(){
	
	
	while(1){
		if (dataRdyIntReceived != 0) {
      dataRdyIntReceived = 0;
			lsm6dsl_acceleration_raw_get(&dev_ctx,data_raw.i16bit);
			
		
			if(start != 1){
			shiftArray(DRAW_POINT,DRAWLENGTH,scaleToRange(data_raw.i16bit[0]));
			drawLineWithYCoords(DRAW_POINT,DRAWLENGTH);
			}
			
			break;
		}
	}
	
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){


		if(GPIO_Pin ==GPIO_PIN_1){
		
		
		 dataRdyIntReceived ++;
	
			
		}
		
	
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
}
///////////////////////////////

void drawLineWithYCoords(int* yCoords, uint8_t length) {
    // Calculate the horizontal scaling factor
    uint8_t scaleX = SSD1306_WIDTH / (length - 1);
	
	
//		ssd1306_Fill_18(Black);
		ssd1306_Fill(Black);
	
	
		if(DRAW != -1){
			 ssd1306_SetCursor(2,0);
			 ssd1306_WriteString(OUT[DRAW], Font_11x18, White);
//			printf(OUT[DRAW]);
//			printf("\r\n");
//			 ssd1306_UpdateScreen();
		}
		else{
         ssd1306_SetCursor(2,0);
				 ssd1306_WriteString("...", Font_11x18, White);
//			 ssd1306_UpdateScreen();
		
		}
    // Set the initial position for drawing the line
    uint8_t startX = 0;
    uint8_t startY = yCoords[0];

    // Draw the line
    for (uint8_t i = 1; i < length; i++) {
        uint8_t endX = i * scaleX;
        uint8_t endY = yCoords[i];

        // Draw a line between the previous and current point
        ssd1306_Line(startX, startY, endX, endY,White);

        // Update the start position for the next line segment
        startX = endX;
        startY = endY;
    }

    // Update the screen to display the line
    ssd1306_UpdateScreen();
}


void shiftArray(int* arr, int length, int newHead) {
    // Shift the elements of the array one position to the right
    for (int i = length - 1; i > 0; i--) {
        arr[i] = arr[i - 1];
    }

    // Fill the head of the array with the given integer
    arr[0] = newHead;
}

int scaleToRange(int num) {
    // Ensure the number is within the range 0 to 63
	
		if( num < 200 && num > -200  ){
				num = 127 ;
		
		}
		else{
		
		 num = 127 - num;
		}
    int scaledNum = num % 64;

    // If the number was negative, adjust it to be within the range 0 to 63
    if (scaledNum < 0) {
        scaledNum += 64;
    }
		if (scaledNum < 18){
		
				 scaledNum += 18;
		
		}

    return scaledNum;
}
///////////DISPLAY////////////////

static void AI_Init(void)
{
  ai_error err;

  /* Create a local array with the addresses of the activations buffers */
  const ai_handle act_addr[] = { activations };
  /* Create an instance of the model */
  err = ai_network_create_and_init(&network, act_addr, NULL);
  if (err.type != AI_ERROR_NONE) {
    printf("ai_network_create error - type=%d code=%d\r\n", err.type, err.code);
    Error_Handler();
  }
  ai_input = ai_network_inputs_get(network, NULL);
  ai_output = ai_network_outputs_get(network, NULL);
	
}
static void AI_Run(float *pIn, float *pOut)
{
  ai_i32 batch;
  ai_error err;

  /* Update IO handlers with the data payload */
  ai_input[0].data = AI_HANDLE_PTR(pIn);
  ai_output[0].data = AI_HANDLE_PTR(pOut);

  batch = ai_network_run(network, ai_input, ai_output);
  if (batch != 1) {
    err = ai_network_get_error(network);
    printf("AI ai_network_run error - type=%d code=%d\r\n", err.type, err.code);
    Error_Handler();
  }
}



/////////////////////////
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
