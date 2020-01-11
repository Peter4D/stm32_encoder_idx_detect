/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "sw_timer.h"
#include "num_str_convert.h"
#include <string.h>

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

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t const uart_test_msg[] = "hello indexFind: \n\r";
static uint8_t const uart_msg_err_cmd2long[] = " $Err:cmd to long: \n\r";


static sw_timer_t xTaskTimer;
static sw_timer_t xLed_tm;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void blink_onboard_led(uint32_t time_ms);
void led_tm_cb(void);

void taskTimer_cb(void);

void ch0_swTmr_cb(void);
void ch1_swTmr_cb(void);
void ch2_swTmr_cb(void);
void ch3_swTmr_cb(void);


//void handle_sensors(index_ch_desc_t *index_ch_desc);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct _gpio_io_desc_t
{
    GPIO_TypeDef    *port;
    uint16_t        pin;
}gpio_io_desc_t;

typedef enum{
    IDLE,
    WAIT_IDX
}idx_ch_stateMchn_t;

struct _idx_ch_desc_t
{
    uint8_t             ch_name[15];
    gpio_io_desc_t      gpio_io_input;
    gpio_io_desc_t      gpio_io_out;
    idx_ch_stateMchn_t  stateMchine;
    sw_timer_t          swtimer;
    uint32_t            delayTm;
    void(*sw_tmr_cb)(void);
}idx_ch_array[] = {
    {
        "ch_0",
        {END_SW0_GPIO_Port, END_SW0_Pin},
        {out0_GPIO_Port, out0_Pin},
        IDLE,
        {0},
        DELAY_AFTER_IDX_CH0,
        &ch0_swTmr_cb
    },
    {
        "ch_1",
        {END_SW1_GPIO_Port, END_SW1_Pin},
        {out1_GPIO_Port, out1_Pin},
        IDLE,
        {0},
        DELAY_AFTER_IDX_CH1,
        &ch1_swTmr_cb
    },
    {
        "ch_2",
        {END_SW2_GPIO_Port, END_SW2_Pin},
        {out2_GPIO_Port, out2_Pin},
        IDLE,
        {0},
        DELAY_AFTER_IDX_CH2,
        &ch2_swTmr_cb
    },
    {
        "ch_3",
        {END_SW3_GPIO_Port, END_SW3_Pin},
        {out3_GPIO_Port, out3_Pin},
        IDLE,
        {0},
        DELAY_AFTER_IDX_CH3,
        &ch3_swTmr_cb
    }
};

typedef enum{
    SM_RX_IDLE,
    SM_RX_RECEIVE,
    SM_RX_EXE,  // @todo remove: a the moment redundant state 
}SM_Rx_sm_enum_t;

struct _SM_Rx_desc_t{
    SM_Rx_sm_enum_t state;
    uint8_t Rx_buff[RX_CMD_BUFF_SIZE];
    uint8_t Rx_cnt;
    uint8_t Rx_char;
}SM_Rx = {
    SM_RX_IDLE,
    {0},
    0,
    0
};

void handle_channels(struct _idx_ch_desc_t *ch);
void SM_goToIdle(struct _idx_ch_desc_t *ch);

void TASK_serial_cmd_decode(void);


void taskTimer_cb(void) {
    
    #if ( DEBUG_LED_HEART_BEAT_EN == 1)
        HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
    #endif

    #if ( DEBUG_MSG_ENABLE == 1 )
        /* debug */
        HAL_UART_Transmit_IT(&huart1, uart_test_msg, sizeof(uart_test_msg));
    #endif
    /* set timer for new cycle */
    swTimer.set(&xTaskTimer, TASK_PERIODE);
}

void blink_onboard_led(uint32_t time_ms) {

    HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, LED_SET);
    
    swTimer.set(&xLed_tm, time_ms);
}

void led_tm_cb(void) {
    HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, LED_RESET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    /* drive software timers: resolition is 0.1 ms */
    swTimer_tick();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    /* test external interrupt */
    static uint8_t ext_int_msg[15] = {0};
    static uint8_t pinNum_str[6] = {0};

    struct _idx_ch_desc_t *p_ch;

    switch (GPIO_Pin)
    {
        case idx_int0_Pin:
        {
            #if ( DEBUG_MSG_ENABLE == 1 )
                /* #debug */
                strcpy(pinNum_str, "int_0");
            #endif
            p_ch = &idx_ch_array[0];
            break;
        }
        case idx_int1_Pin:
        {
            #if ( DEBUG_MSG_ENABLE == 1 )
                /* #debug */
                strcpy(pinNum_str, "int_1");
            #endif
            p_ch = &idx_ch_array[1];
            break;
        }
        case idx_int2_Pin:
        {
            #if ( DEBUG_MSG_ENABLE == 1 )
                /* #debug */
                strcpy(pinNum_str, "int_2");
            #endif
            p_ch = &idx_ch_array[2];
            break;
        }
        case idx_int3_Pin:
        {
            #if ( DEBUG_MSG_ENABLE == 1 )
                /* #debug */
                strcpy(pinNum_str, "int_3");
            #endif
            p_ch = &idx_ch_array[3];
            break;
        }
        default:
        {
            p_ch = &idx_ch_array[0];
            assert(0);
            break;
        }
    }

    swTimer.set(&p_ch->swtimer, p_ch->delayTm);

    //==========================================================================
    #if ( DEBUG_MSG_ENABLE == 1 )
        /* #debug */
        strcpy(ext_int_msg, "Pin: ");
        //num2str(i, pinNum_str);
        strcat(ext_int_msg, pinNum_str);
        strcat(ext_int_msg, "\n\r");

        HAL_UART_Transmit_IT(&huart1, ext_int_msg, sizeof(ext_int_msg));
    #endif
    //==========================================================================

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    static uint32_t tick_old = 0;
    uint8_t nCh = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

    /* set application heartBeat and serial debug printout task */
    swTimer_init(&xTaskTimer);
    swTimer.attach_callBack(&xTaskTimer, taskTimer_cb);
    swTimer.set(&xTaskTimer, TASK_PERIODE);

    /* sw timer that control how long will led be on */
    swTimer_init(&xLed_tm);
    swTimer.attach_callBack(&xLed_tm, led_tm_cb);

    /* calculate numner of channels used in application */
    nCh = sizeof(idx_ch_array)/sizeof(idx_ch_array[0]);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);

  swTimer_init(&idx_ch_array[0].swtimer);
  swTimer_init(&idx_ch_array[1].swtimer);
  swTimer_init(&idx_ch_array[2].swtimer);
  swTimer_init(&idx_ch_array[3].swtimer);
  
  swTimer.attach_callBack(&idx_ch_array[0].swtimer, idx_ch_array[0].sw_tmr_cb);
  swTimer.attach_callBack(&idx_ch_array[1].swtimer, idx_ch_array[1].sw_tmr_cb);
  swTimer.attach_callBack(&idx_ch_array[2].swtimer, idx_ch_array[2].sw_tmr_cb);
  swTimer.attach_callBack(&idx_ch_array[3].swtimer, idx_ch_array[3].sw_tmr_cb);

  /* enable Serial receive */
  HAL_UART_Receive_IT(&huart1, &SM_Rx.Rx_char, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        if( (HAL_GetTick() - tick_old) > CH_PULL_READ_PER){
            uint8_t i = 0;
            
            /* channels polling read of limit switches */
            for(;i < nCh; ++i) {
                handle_channels(&idx_ch_array[i]);
            }

            tick_old = HAL_GetTick();
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void TASK_serial_cmd_decode(void) {

}

void handle_channels(struct _idx_ch_desc_t *ch){
    uint32_t limit_sw_state = 0;

    limit_sw_state = HAL_GPIO_ReadPin(ch->gpio_io_input.port, ch->gpio_io_input.pin);
    if(limit_sw_state == INPUT_ON && ch->stateMchine == IDLE){
        ch->stateMchine = WAIT_IDX;
        /*set output; external interrupt will set off channel ! */
        HAL_GPIO_WritePin(ch->gpio_io_out.port, ch->gpio_io_out.pin, GPIO_PIN_SET);
    }
}

//=========================================================
/* software timers callback functions */
void ch0_swTmr_cb(void){
    
    SM_goToIdle(&idx_ch_array[0]);
}

void ch1_swTmr_cb(void){

    SM_goToIdle(&idx_ch_array[1]);
}

void ch2_swTmr_cb(void){

    SM_goToIdle(&idx_ch_array[2]);
}

void ch3_swTmr_cb(void){
    
    SM_goToIdle(&idx_ch_array[3]);
}

//=========================================================

void SM_goToIdle(struct _idx_ch_desc_t *ch) {
    HAL_GPIO_WritePin(ch->gpio_io_out.port, ch->gpio_io_out.pin, GPIO_PIN_RESET);
    ch->stateMchine = IDLE;
}

void Rx_cmd_decode(struct _SM_Rx_desc_t *SM_Rx){
    uint8_t *p_str = &SM_Rx->Rx_buff[0];
    uint8_t ch_sel = 0;
    int32_t delay_val = 0;
    static uint8_t ack_msg[10] = {0};
    uint8_t ack_msg_size = 0;

    if(p_str[0] == 'c'){
        if(p_str[1] >= '0' && p_str[1] <= '3') {
            ch_sel = p_str[1] - '0';
            if(str2num(&p_str[3], &delay_val) == 0) {
                /* value is number OK*/
                /* @todo in final version this need to be written into NVM */
                idx_ch_array[ch_sel].delayTm = delay_val;
                /*#debug*/
                blink_onboard_led(1000);
                /* #debug message */
                num2str(delay_val, ack_msg);
                ack_msg_size = strlen(ack_msg);
                ack_msg[++ack_msg_size] = '\n';
                HAL_UART_Transmit_IT(&huart1, ack_msg, ack_msg_size);
            }else {
                /* value is NaN error*/
            }
        }else{
            /* channel select error */
        }
    }else{
        /* command error */
    }
}
//idx_ch_array


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    
    switch (SM_Rx.state) {
        case SM_RX_IDLE:
        {
            SM_Rx.state = SM_RX_RECEIVE;
            SM_Rx.Rx_buff[SM_Rx.Rx_cnt++] = SM_Rx.Rx_char;
            break;
        }
        case SM_RX_RECEIVE:
        {
            
            if(SM_Rx.Rx_cnt >= RX_CMD_BUFF_SIZE) {
                /* Rx cmd buffer overflow error go to idle inform user */
                HAL_UART_Transmit_IT(&huart1, uart_msg_err_cmd2long, sizeof(uart_msg_err_cmd2long));
                SM_Rx.Rx_cnt = 0;
                SM_Rx.state = SM_RX_IDLE;
                break;
            }

            /* check for temination character */
            if(SM_Rx.Rx_char == '\r') {
                /* decode */
                SM_Rx.Rx_buff[SM_Rx.Rx_cnt] = 0x00; // zero terminate the string 
                Rx_cmd_decode(&SM_Rx);
                /* return to idle state */
                SM_Rx.Rx_cnt = 0;
                SM_Rx.state = SM_RX_IDLE;

            }else{
                SM_Rx.Rx_buff[SM_Rx.Rx_cnt++] = SM_Rx.Rx_char;
            }
            break;
        }
    }

    HAL_UART_Receive_IT(&huart1, &SM_Rx.Rx_char, 1);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
