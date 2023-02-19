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
#include "gpio.h"
#include "ipcc.h"
#include "openamp.h"
#include "spi.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DIO.h"
#include "rpmsg_hdr.h"
#include "virt_uart.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    INITIALIZING, /*  0 */
    DDR_BUFFERS_OK, /*  1 */
    SAMPLING, /*  2 */
} Machine_State_t;

typedef struct __HDR_DdrBuffTypeDef {
    uint32_t physAddr;
    uint32_t physSize;
} HDR_DdrBuffTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_DDR_BUFF 16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static struct rpmsg_virtio_device rvdev;
RPMSG_HDR_HandleTypeDef hsdb0;
VIRT_UART_HandleTypeDef huart0;

uint8_t VirtUart0ChannelBuffRx[100];
uint16_t VirtUart0ChannelRxSize = 0;
uint8_t SDB0ChannelBuffRx[100];
uint16_t SDB0ChannelRxSize = 0;
char mSdbBuffTx[512];

__IO FlagStatus VirtUart0RxMsg = RESET;
__IO FlagStatus SDB0RxMsg = RESET;

uint8_t mDdrBuffCount = 1; // number of modules

volatile static HDR_DdrBuffTypeDef mArrayDdrBuff[MAX_DDR_BUFF]; // used to store DDR buff allocated by Linux driver
volatile uint8_t mArrayDdrBuffCount = 0;
volatile uint8_t mArrayDdrBuffIndex = 0; // will vary from 0 to mArrayDdrBuffCount-1

Machine_State_t mMachineState = INITIALIZING;

uint8_t spi_transmit_buffer[DIO_SPI_MSG_LEN] = { [0 ... 15] = 0x00 };
uint8_t spi_receive_buffer[DIO_SPI_MSG_LEN] = { [0 ... 15] = 0xFF };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  CallBack function which will be called when UART0 data is received from A7.
 * @retval none
 */
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef* huart)
{
    // copy received msg in a buffer
    VirtUart0ChannelRxSize = huart->RxXferSize < 100 ? huart->RxXferSize : 99;
    memcpy(VirtUart0ChannelBuffRx, huart->pRxBuffPtr, VirtUart0ChannelRxSize);
    VirtUart0ChannelBuffRx[VirtUart0ChannelRxSize] = 0; // insure end of String
    VirtUart0RxMsg = SET;
}

/**
 * @brief  CallBack function which will be called when a DDR buffers is allocated by Linux.
 * @retval none
 */
void SDB0_RxCpltCallback(RPMSG_HDR_HandleTypeDef* huart)
{
    // copy received msg in a buffer
    SDB0ChannelRxSize = huart->RxXferSize < 100 ? huart->RxXferSize : 99;
    memcpy(SDB0ChannelBuffRx, huart->pRxBuffPtr, SDB0ChannelRxSize);
    SDB0ChannelBuffRx[SDB0ChannelRxSize] = 0; // insure end of String
    SDB0RxMsg = SET;
}

/**
 * @brief  Function which treats a command coming from A7.
 * @retval boolean command OK or not
 */
uint8_t treatRxCommand2()
{
    // example command: S => Start
    if (VirtUart0ChannelBuffRx[0] == 'S') {
        return 'S';
    } else if (VirtUart0ChannelBuffRx[0] == 'B') {
        for (int i = 0; i < 2; i++) {
            if (VirtUart0ChannelBuffRx[1 + i] < '0' || VirtUart0ChannelBuffRx[1 + i] > '9') {
                return false;
            }
        }
        mDdrBuffCount = (int)(VirtUart0ChannelBuffRx[1] - '0') * 10
            + (int)(VirtUart0ChannelBuffRx[2] - '0') * 1;
        return 'B';
    } else if (VirtUart0ChannelBuffRx[0] == 'E') {
        // exit requested
        return 'E';
    }
    return false;
}

/**
 * @brief  Function which treats a buffer allocation.
 * @retval none
 */
void treatSDBEvent()
{
    // example command: B0AxxxxxxxxLyyyyyyyy => Buff0 @:xx..x Length:yy..y
    if (SDB0ChannelBuffRx[0] != 'B') {
        return;
    }
    if (!((SDB0ChannelBuffRx[1] >= '0' && SDB0ChannelBuffRx[1] <= '9'))) {
        if (!((SDB0ChannelBuffRx[1] >= 'A' && SDB0ChannelBuffRx[1] <= 'F'))) {
            return;
        } else {
            if (mArrayDdrBuffCount != (SDB0ChannelBuffRx[1] - 'A' + 10)) {
            }
        }
    }
    if (mArrayDdrBuffCount != (SDB0ChannelBuffRx[1] - '0')) {
        return;
    }
    if (SDB0ChannelBuffRx[2] != 'A') {
        return;
    }
    if (SDB0ChannelBuffRx[11] != 'L') {
        return;
    }
    for (int i = 0; i < 8; i++) {
        if (!((SDB0ChannelBuffRx[3 + i] >= '0' && SDB0ChannelBuffRx[3 + i] <= '9') || (SDB0ChannelBuffRx[3 + i] >= 'a' && SDB0ChannelBuffRx[3 + i] <= 'f'))) {
            return;
        }
        if (!((SDB0ChannelBuffRx[12 + i] >= '0' && SDB0ChannelBuffRx[12 + i] <= '9') || (SDB0ChannelBuffRx[12 + i] >= 'a' && SDB0ChannelBuffRx[12 + i] <= 'f'))) {
            return;
        }
    }
    // save DDR buff @ and size
    mArrayDdrBuff[mArrayDdrBuffCount].physAddr = (uint32_t)strtoll((char*)&SDB0ChannelBuffRx[3], NULL, 16);
    mArrayDdrBuff[mArrayDdrBuffCount].physSize = (uint32_t)strtoll((char*)&SDB0ChannelBuffRx[12], NULL, 16);

    mArrayDdrBuffCount++;
}

void config_GPIOs()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOZ, &GPIO_InitStruct);
}

/**
 * @brief  Function which implements tha machine state.
 * @retval none
 */
void StateMachine(void)
{
    OPENAMP_check_for_message();
    switch (mMachineState) {
    case INITIALIZING:
        // waiting for SDB driver events, with at least 3 DDR buffers
        if (SDB0RxMsg) {
            SDB0RxMsg = RESET;
            treatSDBEvent();
            if (mArrayDdrBuffCount >= mDdrBuffCount) { // is number of buffer in line with received command ?
                mMachineState = DDR_BUFFERS_OK;
            }
        }
        if (VirtUart0RxMsg) {
            VirtUart0RxMsg = RESET;
            treatRxCommand2(); // to receive DDR BUFFER command
        }
        break;
    case DDR_BUFFERS_OK:
        // ready to accept a sampling command
        if (VirtUart0RxMsg) {
            VirtUart0RxMsg = RESET;
            if (treatRxCommand2() == 'S') {
                config_GPIOs();
                mMachineState = SAMPLING;
            }
        }
        break;
    case SAMPLING:
        setDO(spi_transmit_buffer);

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)spi_transmit_buffer, (uint8_t*)spi_receive_buffer, DIO_SPI_MSG_LEN, DIO_SPI_TIMEOUT);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

        getDI(spi_receive_buffer);

        uint32_t* pSource = (uint32_t*)&spi_receive_buffer[0];
        uint32_t* pDest = (uint32_t*)(mArrayDdrBuff[mArrayDdrBuffIndex].physAddr);
        // we have a buffer of mSramPacketSize/2 bytes, to send by packet of 4 => loop on mSramPacketSize/8
        for (int i = 0; i < 4; i++) {
            *(pDest + i) = *(pSource + i) & 0x7F7F7F7F; // 76µs to perform this operation, a memcpy takes 104µs
        }

        for (int i = 0; i < 16; i++) {
            spi_transmit_buffer[i] = 0x00;
            spi_receive_buffer[i] = 0xFF;
        }

        // time to change DDR buffer and send MSG to Linux
        sprintf(mSdbBuffTx, "B%dL%08x", 0, 16);
        RPMSG_HDR_Transmit(&hsdb0, (uint8_t*)mSdbBuffTx, strlen(mSdbBuffTx));

        if (VirtUart0RxMsg) {
            VirtUart0RxMsg = RESET;
            if (treatRxCommand2() == 'E') {
                mMachineState = DDR_BUFFERS_OK;
            }
        }
        break;
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

    if (IS_ENGINEERING_BOOT_MODE()) {
        /* Configure the system clock */
        SystemClock_Config();
    }

    if (IS_ENGINEERING_BOOT_MODE()) {
        /* Configure the peripherals common clocks */
        PeriphCommonClock_Config();
    } else {
        /* IPCC initialisation */
        MX_IPCC_Init();
        /* OpenAmp initialisation ---------------------------------*/
        MX_OPENAMP_Init(RPMSG_REMOTE, NULL);
    }

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_SPI1_Init();
    /* USER CODE BEGIN 2 */
    /*
     * Create HDR device
     * defined by a rpmsg channel attached to the remote device
     */
    hsdb0.rvdev = &rvdev;
    if (RPMSG_HDR_Init(&hsdb0) != RPMSG_HDR_OK) {
        Error_Handler();
    }

    /*
     * Create Virtual UART devices
     */
    if (VIRT_UART_Init(&huart0) != VIRT_UART_OK) {
        Error_Handler();
    }

    /*Need to register callback for message reception by channels*/
    if (RPMSG_HDR_RegisterCallback(&hsdb0, RPMSG_HDR_RXCPLT_CB_ID, SDB0_RxCpltCallback) != RPMSG_HDR_OK) {
        Error_Handler();
    }

    /*Need to register callback for message reception by channels (not for UART1 as used only for trace outpout) */
    if (VIRT_UART_RegisterCallback(&huart0, VIRT_UART_RXCPLT_CB_ID, VIRT_UART0_RxCpltCallback) != VIRT_UART_OK) {
        Error_Handler();
    }

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        StateMachine();
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure LSE Drive Capability
     */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI
        | RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_DIG;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDivValue = RCC_HSI_DIV1;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLL12SOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 3;
    RCC_OscInitStruct.PLL.PLLN = 81;
    RCC_OscInitStruct.PLL.PLLP = 1;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLFRACV = 2048;
    RCC_OscInitStruct.PLL.PLLMODE = RCC_PLL_FRACTIONAL;
    RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL2.PLLSource = RCC_PLL12SOURCE_HSE;
    RCC_OscInitStruct.PLL2.PLLM = 3;
    RCC_OscInitStruct.PLL2.PLLN = 66;
    RCC_OscInitStruct.PLL2.PLLP = 2;
    RCC_OscInitStruct.PLL2.PLLQ = 2;
    RCC_OscInitStruct.PLL2.PLLR = 1;
    RCC_OscInitStruct.PLL2.PLLFRACV = 5120;
    RCC_OscInitStruct.PLL2.PLLMODE = RCC_PLL_FRACTIONAL;
    RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL3.PLLSource = RCC_PLL3SOURCE_HSE;
    RCC_OscInitStruct.PLL3.PLLM = 2;
    RCC_OscInitStruct.PLL3.PLLN = 34;
    RCC_OscInitStruct.PLL3.PLLP = 2;
    RCC_OscInitStruct.PLL3.PLLQ = 17;
    RCC_OscInitStruct.PLL3.PLLR = 37;
    RCC_OscInitStruct.PLL3.PLLRGE = RCC_PLL3IFRANGE_1;
    RCC_OscInitStruct.PLL3.PLLFRACV = 6660;
    RCC_OscInitStruct.PLL3.PLLMODE = RCC_PLL_FRACTIONAL;
    RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** RCC Clock Config
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_ACLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
        | RCC_CLOCKTYPE_PCLK3 | RCC_CLOCKTYPE_PCLK4
        | RCC_CLOCKTYPE_PCLK5 | RCC_CLOCKTYPE_MPU;
    RCC_ClkInitStruct.MPUInit.MPU_Clock = RCC_MPUSOURCE_PLL1;
    RCC_ClkInitStruct.MPUInit.MPU_Div = RCC_MPU_DIV2;
    RCC_ClkInitStruct.AXISSInit.AXI_Clock = RCC_AXISSOURCE_PLL2;
    RCC_ClkInitStruct.AXISSInit.AXI_Div = RCC_AXI_DIV1;
    RCC_ClkInitStruct.MCUInit.MCU_Clock = RCC_MCUSSOURCE_PLL3;
    RCC_ClkInitStruct.MCUInit.MCU_Div = RCC_MCU_DIV1;
    RCC_ClkInitStruct.APB4_Div = RCC_APB4_DIV2;
    RCC_ClkInitStruct.APB5_Div = RCC_APB5_DIV4;
    RCC_ClkInitStruct.APB1_Div = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2_Div = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB3_Div = RCC_APB3_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Set the HSE division factor for RTC clock
     */
    __HAL_RCC_RTC_HSEDIV(24);
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

    /** Initializes the common periph clock
     */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
    PeriphClkInit.CkperClockSelection = RCC_CKPERCLKSOURCE_HSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM7 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM7) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
