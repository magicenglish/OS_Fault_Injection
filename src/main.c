/**
 ******************************************************************************
 * @file    main.c
 * @author  Andrea Casalino
 * @version V1.0
 * @date    2017/ 05 / 24
 * @brief   Default main function.
 ******************************************************************************
 */
#include "stm32f3xx.h"
#include "stm32f3_discovery.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_tim.h"

/* Use this to decide which fault inject */
#define FAULT 0                 // from 1 to 6 -> faults, others -> no fault

/* Use this to set TEST3 (1) or TEST1/2 (0) */
#define TEST3 0

#if TEST3
  #define TEST_CONFIG 1           // don't modify this value
#else
  /* Use this to set TEST1 (0) or TEST1 (1) */
  #define TEST_CONFIG 0           // from 0 to 1 -> different test configs
#endif


#define TASK_DELAY    500         // Task Delay [ms]
#define START_ADDRESS 0x08032000  // Points to a byte in memory
/* The page used is the 100th and it ends at addr 0x080327FF */
#define MEMORY_CELLS  100         // Number of writings on Flash memory
#define OFFSET        2           // Minimum data width

/*
 * Time interrupt
 * System Clock = 1 us => dividing it by Prescaler (1 us / 10000 = 10 ms).
 * Interrupt will rise every 500 periods = 10 ms * 500 = 5 seconds.
 */
#define PERIOD      500   // Time to count 500 periods
#define PRESCALER   10000   // Prescaler 10000

/*------------------------- GLOBAL variables -------------------------*/
int  button_counter = 0;    // Times the button has been pressed
char button_pushed  = 0;    // States button pressed

/* Priorities
 * This is used to set the different task priorities for each configuration.
 */
char priorities[] = {
  1,1,1,1,1,        // TEST1 priorities
  6,2,5,3,1         // TEST2 (and part of TEST3) priorities
};

/* Memory */
uint32_t address;   // Flash memory address in use
uint32_t ending_address = START_ADDRESS + MEMORY_CELLS * 2;

/* Timer */
TIM_HandleTypeDef timer7;
uint8_t timer_counter = 0;

/*---------------------------- Prototypes ----------------------------*/
/* System Clock Configuration */
void SystemClock_Config(void);
/* Set NVIC */
void NVIC_Setup(void);
/* Turn ON all LEDs  */
void LED_ON(void);
/* Turn OFF all LEDs */
void LED_OFF(void);
/* Enable LEDs, Gyroscope and Accelerometer */
void Peripherals_enable(void);
/* ISR of TIM7 */
void TIM7_IRQHandler(void);
/* Error Handler */
void Error_Handler(void);
/* Reset memory and starting address */
void MemoryReset();

/* Tasks */
void MAIN_Task (void *pvParameters);  // Task to handle other tasks
void vTask1    (void *pvParameters);  // Accelerometer and Gyroscope + LEDs
void vTask2    (void *pvParameters);  // All LEDs
void vTask3    (void *pvParameters);  // IDLE task
void vTask4    (void *pvParameters);  // FLASH memory + LEDs


int main(void) {
  /* Configure the system clock */
  SystemClock_Config();

  /* NVIC configuration */
  NVIC_Setup();

  /* Initialize LEDs and "user" button */
  Peripherals_enable();

  /* Create Main Task */
  xTaskCreate(MAIN_Task, "main task", 300, NULL, priorities[TEST_CONFIG * 5], NULL);

  /* Create Idle Task */
  xTaskCreate(vTask3, "Task 3", 100, NULL, priorities[TEST_CONFIG * 5 + 3], NULL);

  /* Start the scheduler so the tasks start executing. */
  vTaskStartScheduler();

  /* If all is well then main() will never reach here as the scheduler will
  now be running the tasks. */
  while (1);
}

/**
 * @brief  System Clock Configuration
 * @note   This function sets SysCLK, HCLK, Systick, PLCK1, PLCK2
 * @param  None
 * @retval None
 */
void SystemClock_Config(void){

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // gives the error of the HSI
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /* Initializes the CPU, AHB and APB busses clocks (also TIM clk) */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
      |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }

  /* Configure the Systick interrupt time */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  /* Configure the Systick */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/**
 * @brief  Initialize TIM7
 * @note   Set TIM7 in up-count mode
 * @param  @param1 Value that will be counted
 *         @param2 Ideal Prescaler value
 * @retval None
 */
void TIM7_Configuration (uint32_t value_to_count, uint32_t Prescaler_ideal) {

  __TIM7_CLK_ENABLE();
  timer7.Instance               = TIM7;
  timer7.Init.Prescaler         = Prescaler_ideal - 1;
  timer7.Init.CounterMode       = TIM_COUNTERMODE_UP;
  timer7.Init.Period            = value_to_count;
  timer7.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; // no division
  timer7.Init.RepetitionCounter = 0; // IRQ must happen only once
  timer7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  HAL_NVIC_SetPriority(TIM7_IRQn, 0x05, 0x00); // NOT ROBUST: if you put more than 15 @arg2 error
  /* Enable the TIM7 global Interrupt*/
  HAL_NVIC_EnableIRQ(TIM7_IRQn);

  HAL_TIM_Base_Init     (&timer7);
  HAL_TIM_Base_Start_IT (&timer7);
  return;
}

/**
 * @brief  This function Set NVIC
 * @param  None
 * @retval None
 */
void NVIC_Setup(void){
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  return;
}

/**
 * @brief  Enable all LEDs and Peripherals
 * @param  None
 * @retval None
 */
void Peripherals_enable(void){
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI); // set user button as Input
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);
  BSP_LED_Init(LED7);
  BSP_LED_Init(LED8);
  BSP_LED_Init(LED9);
  BSP_LED_Init(LED10);
  BSP_GYRO_Init();
  BSP_ACCELERO_Init();

  return;
}

/**
 * @brief  ISR of TIM7
 * @note   This function is called when TIM7 interrupt took place, inside HAL_TIM_IRQHandler().
 * @param  None
 * @retval None
 */
void TIM7_IRQHandler(void) {
  /* Deactivate pending interrupt */
  HAL_TIM_IRQHandler (&timer7);

  /* Time to work */
  if (timer_counter == 1) {
    HAL_TIM_Base_Stop_IT (&timer7);

    /* FAULT INJECTION */
    /* Function returning the SRAM addresses of the target variables */
    Pointer_to_SRAM privileged_data_addresses = GET_SRAM_scheduler_pointers ();

    switch (FAULT) {
      case 1:
        /*pxReadyTasksLists <= Prioritized ready tasks.
         *  Change the number of Prioritized ready tasks.
         *  From 1 to 0 */
        ((List_t *) (privileged_data_addresses.point_pxReadyTasksLists))->uxNumberOfItems  ^= 0x01;
        break;
      case 2: //no fault
        /* pxDelayedTaskList <= Points to the delayed task list currently being used.
         *  From 1 to 0 */
        ((List_t *) (privileged_data_addresses.point_pxDelayedTaskList))->uxNumberOfItems  ^= 0x01;
        break;
      case 3:
        /*xPendingReadyList <= Tasks that have been readied while the scheduler was suspended.
         * They will be moved to the ready list when the scheduler is resumed. */
        ((List_t *) (privileged_data_addresses.point_xPendingReadyList))->uxNumberOfItems ^= 0x01;
        break;
      case 4:
        /* Injection of fault #4 */
        *((UBaseType_t *)(privileged_data_addresses.point_uxTopReadyPriority)) ^= 0x01;
        break;
      case 5:
        /* Injection of fault #5*/
        *((UBaseType_t *)(privileged_data_addresses.point_xSchedulerRunning)) ^= 0x01;
        break;
      case 6://no fault
        /* Injection of fault #6*/
        *((UBaseType_t *)(privileged_data_addresses.point_uXPriority)) ^= 0x01;
        break;
      default:
        break;
    }

    /* SAVING INTERRUPT HAPPENING ONTO FLASH MEMORY
    /* Save Led position in flash memory * /
    HAL_FLASH_Unlock(); // Unlock Flash Memory
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, 0xAAAA); // Program Flash address
    HAL_FLASH_Lock(); // Lock Flash Memory
    address += OFFSET;  // Increment address
    */
  }
  timer_counter ++;
  return;
}

/**
 * @brief  ISR of BUTTON INTERRUPT
 * @note   This function is called when USER BUTTON is pressed
 * @param  None
 * @retval None
*/
 void EXTI0_IRQHandler (void) {
  /* Disable pending USER BUTTON IRQ request */
  HAL_GPIO_EXTI_IRQHandler (GPIO_PIN_0);

  /* Counter logic */
  if (button_counter < 4) {
    button_counter++;
    button_pushed = 1;
  } else {
    button_counter = 1;
    button_pushed = 1;
  }
  return;
} // end BUTTON IRQ

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
  while(1) {}
}

/**
 * @brief  Turn on LEDs showing board's grade
 * @param  None
 * @retval None
 */
void vTask1(void *pvParameters) {

  /* Accelerometer */
  long long acc[3];     // averaged value
  int16_t aDataXYZ[3];  // acquired value

  /* Gyroscope */
  long long   gyr[3];   // averaged value
  float   gDataXYZ[3];  // acquired value

  /* Memory */
  uint16_t ledData;     // Data containing LED position

  /* Counter */
  int i, j;
  /* Number of averaged acquisitions */
  int numOfSamples = 10;

  /* Loop */
  while (1) {
    /* Check to avoid writing outside the desired memory space */
    if ((address >= START_ADDRESS) && (address < ending_address)) {
      /* Reset average */
      for (i = 0; i < 3; i++) {
        acc[i] = 0;
        gyr[i] = 0;
      }

      /* Acquisition of numOfSamples values */
      for (i = 0; i < numOfSamples; i++) {
        BSP_ACCELERO_GetXYZ(aDataXYZ);
        BSP_GYRO_GetXYZ(gDataXYZ);
        for (j = 0; j < 3; j++) {
          acc[j] += aDataXYZ[j];
          gyr[j] += gDataXYZ[j];
        }
      }
      /* Averaging */
      for (i = 0; i < 3; i++) {
        acc[i] /= numOfSamples;
        gyr[i] /= numOfSamples;
      }
      /* Turn LEDs OFF */
      LED_OFF();
      /* Turn LED ON */
      if      ((acc[0] >  3000) && (acc[1] <  3000) && (acc[1] > -3000)) {
        BSP_LED_On(LED3);
        ledData = 3;
      }
      else if ((acc[0] >  3000) && (acc[1] >  3000)) {
        BSP_LED_On(LED4);
        ledData = 4;
      }
      else if ((acc[1] >  3000) && (acc[0] <  3000) && (acc[0] > -3000)) {
        BSP_LED_On(LED6);
        ledData = 6;
      }
      else if ((acc[0] < -3000) && (acc[1] >  3000)) {
        BSP_LED_On(LED8);
        ledData = 8;
      }
      else if ((acc[0] < -3000) && (acc[1] <  3000) && (acc[1] > -3000)) {
        BSP_LED_On(LED10);
        ledData = 10;
      }
      else if ((acc[0] < -3000) && (acc[1] < -3000)) {
        BSP_LED_On(LED9);
        ledData = 9;
      }
      else if ((acc[1] < -3000) && (acc[0] < 3000) && (acc[0] > -3000)) {
        BSP_LED_On(LED7);
        ledData = 7;
      }
      else if ((acc[0] >  3000) && (acc[1] < -3000)) {
        BSP_LED_On(LED5);
        ledData = 5;
      }
      else ledData = 1;

      /* Save Led position in flash memory */
      HAL_FLASH_Unlock(); // Unlock Flash Memory
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, (ledData + ((int)pvParameters << 8)) ); // Program Flash address
      HAL_FLASH_Lock();   // Lock Flash Memory
      address += OFFSET;  // Increment address


      /* Task Delay */
      vTaskDelay(TASK_DELAY);
    }
    else {
      /* No more space to write into */
      LED_OFF();
      BSP_LED_On(LED3);
      BSP_LED_On(LED6);
      BSP_LED_On(LED7);
      BSP_LED_On(LED10);
    } // end if (check for available memory space)
  } // end while loop
  vTaskDelete (NULL);
} // end xTask1

/**
 * @brief  Turn on all LEDs
 * @param  None
 * @retval None
 */
void vTask2 (void *pvParameters){
  /* Loop */
  while (1) {

    /* Check to avoid writing outside the desired memory space */
    if ((address >= START_ADDRESS) && (address < ending_address)) {
      /* Save Led position in flash memory */
      HAL_FLASH_Unlock(); // Unlock Flash Memory
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, (2 + ((int)pvParameters << 8)) ); // Program Flash address
      HAL_FLASH_Lock(); // Lock Flash Memory
      address += OFFSET;  // Increment address
    }
    /* Task Delay */
    vTaskDelay (TASK_DELAY*5);
  } // end while loop
  vTaskDelete (NULL);
} // end xTask2

/**
 * @brief  Idle state where LEDs are on and Accelerometer is not read
 * @param  None
 * @retval None
 */
void vTask3 (void *pvParameters){
  /* Switch all LEDs on */
  LED_ON();
  vTaskDelete (NULL);
} // end xTask3

/**
 * @brief  Read from flash and properly turn on LEDs
 * @param  None
 * @retval None
 */
void vTask4 (void *pvParameters){

  uint16_t *i = (uint16_t *)START_ADDRESS;

  for (; i < address; i++) {
    /* Switch all LEDs off */
    LED_OFF();

    /* Turn on the correct led */
    switch (*i % 16) {
      case 1: // horizontal position
        LED_OFF();
        break;
      case 2: // vTask2
        BSP_LED_On(LED3);
        BSP_LED_On(LED10);
        break;
      case 3:
        BSP_LED_On(LED3);
        break;
      case 4:
        BSP_LED_On(LED4);
        break;
      case 5:
        BSP_LED_On(LED5);
        break;
      case 6:
        BSP_LED_On(LED6);
        break;
      case 7:
        BSP_LED_On(LED7);
        break;
      case 8:
        BSP_LED_On(LED8);
        break;
      case 9:
        BSP_LED_On(LED9);
        break;
      case 10:
        BSP_LED_On(LED10);
        break;
      default: // error
        BSP_LED_On(LED4);
        BSP_LED_On(LED5);
        BSP_LED_On(LED8);
        BSP_LED_On(LED9);
        break;
    } // end of switch case
    vTaskDelay (TASK_DELAY/2); // delay necessary to see the LED
  } // end of for loop

  LED_OFF();

  /* End of reading from Flash */
  BSP_LED_On(LED3);
  BSP_LED_On(LED6);
  BSP_LED_On(LED7);
  BSP_LED_On(LED10);

  vTaskDelay (TASK_DELAY); // delay necessary to see the LEDs

  while (1);
  vTaskDelete (NULL);
} // end of xTask4


/**
 * @brief  Main Task, Organizing other tasks
 * @param  None
 * @retval None
 */
void MAIN_Task (void *pvParameters) {
  /* TaskHandle used to kill task */
  TaskHandle_t task1, task2, task3, task4;

#if TEST3
  TaskHandle_t task1_1, task2_1
  	  	  	 , task1_2, task2_2
             , task1_3, task2_3, task1_4, task2_4;
#endif
  /* Loop */
  while (1) {
    if (button_pushed) {
      button_pushed = 0;
      /* First button pressure */
      if (button_counter == 1) {
        /* Reset Flash memory space used to save data */
        MemoryReset();

        BSP_ACCELERO_Reset(); // Reset
        BSP_GYRO_Reset();     // Reset

#if TEST3
        /* Accelerometer task */
        xTaskCreate (vTask1, "Task 1_0", 100, (void *) 1, 1, &task1  );
        /* Blink all LEDs task */
        xTaskCreate (vTask2, "Task 2_0", 100, (void *) 1, 1, &task2  );
        /* Accelerometer task */
        xTaskCreate (vTask1, "Task 1_1", 100, (void *) 2, 2, &task1_1);
        /* Blink all LEDs task */
        xTaskCreate (vTask2, "Task 2_1", 100, (void *) 2, 2, &task2_1);
        /* Accelerometer task */
        xTaskCreate (vTask1, "Task 1_2", 100, (void *) 3, 3, &task1_2);
        /* Blink all LEDs task */
        xTaskCreate (vTask2, "Task 2_2", 100, (void *) 3, 3, &task2_2);
        /* Accelerometer task */
        xTaskCreate (vTask1, "Task 1_3", 100, (void *) 4, 4, &task1_3);
        /* Blink all LEDs task */
        xTaskCreate (vTask2, "Task 2_3", 100, (void *) 4, 4, &task2_3);
        /* Accelerometer task */
        xTaskCreate (vTask1, "Task 1_4", 100, (void *) 5, 5, &task1_4);
        /* Blink all LEDs task */
        xTaskCreate (vTask2, "Task 2_4", 100, (void *) 5, 5, &task2_4);
#else
        /* Accelerometer task */
        xTaskCreate (vTask1, "Task 1", 100, (void *) 0, priorities[TEST_CONFIG * 5 + 1], &task1);
        /* Blink all LEDs task */
        xTaskCreate (vTask2, "Task 2", 100, (void *) 0, priorities[TEST_CONFIG * 5 + 2], &task2);
#endif

        /* TIM7 initialization @arg1 "Time To Count"  @arg2 "Prescaler" */
        TIM7_Configuration(PERIOD, PRESCALER);  // interrupt every 5s
      }
      /* Second button pressure */
      else if (button_counter == 2) {
        /* Kill task 1 and 2 */
        vTaskDelete (task1);
        vTaskDelete (task2);

#if TEST3
        vTaskDelete (task1_1);
        vTaskDelete (task2_1);
        vTaskDelete (task1_2);
        vTaskDelete (task2_2);
        vTaskDelete (task1_3);
        vTaskDelete (task2_3);
        vTaskDelete (task1_4);
        vTaskDelete (task2_4);
#endif

        /* Create Idle task */
        xTaskCreate (vTask3, "Task 3", 100, NULL, priorities[TEST_CONFIG * 5 + 3], &task3);
      }      /* Third button pressure */
      else if (button_counter == 3) {
        /* Read Flash Memory task */
        xTaskCreate (vTask4, "Task 4", 100, NULL, priorities[TEST_CONFIG * 5 + 4], &task4);
      }
      /* Fourth button pressure */
      else if (button_counter == 4) {
        /* Kill task 4 */
      vTaskDelete (task4);
      /* Create Idle task */
      xTaskCreate (vTask3, "Task 3", 100, NULL, priorities[TEST_CONFIG * 5 + 3], &task3);
      }
      else {
        /* ERROR, this part shouldn't be reached */
        while (1) {
          BSP_LED_On(LED3);
          BSP_LED_On(LED5);
          BSP_LED_On(LED7);
          BSP_LED_On(LED9);
        } // end while
      } // end if
    }
    else {
      vTaskDelay (TASK_DELAY*5);
    }
  }
  vTaskDelete (NULL);
}

/**
 * @brief  ALL LEDs ON
 * @param  None
 * @retval None
 */
void LED_ON () {
  BSP_LED_On(LED3);
  BSP_LED_On(LED4);
  BSP_LED_On(LED5);
  BSP_LED_On(LED6);
  BSP_LED_On(LED7);
  BSP_LED_On(LED8);
  BSP_LED_On(LED9);
  BSP_LED_On(LED10);
}

/**
 * @brief  ALL LEDs OFF
 * @param  None
 * @retval None
 */
void LED_OFF () {
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED5);
  BSP_LED_Off(LED6);
  BSP_LED_Off(LED7);
  BSP_LED_Off(LED8);
  BSP_LED_Off(LED9);
  BSP_LED_Off(LED10);
}

/**
 * @brief  Reset memory and starting address
 * @param  None
 * @retval None
 */
void MemoryReset() {
  address = START_ADDRESS;

  FLASH_EraseInitTypeDef pEraseInit;
  pEraseInit.NbPages = 1;
  pEraseInit.PageAddress = START_ADDRESS;
  pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  uint32_t PageError;

  /* Erase page of memory that will be used */
  HAL_FLASH_Unlock(); // Unlock Flash Memory
  HAL_FLASHEx_Erase(&pEraseInit, &PageError);
  HAL_FLASH_Lock();   // Lock Flash Memory
  /* Memory erasing check */
  if (PageError != 0xFFFFFFFF)
    Error_Handler();
}

