#include "Hardware.h"
#include "Adafruit_NeoPixel.h"
#include "time64.h"

// Import necessary headers for register access
// (Assumed standard headers available in the environment)

extern "C" {
    // If needed for ISRs
}

// LED Configuration
#define LED_PA11_NUM 2
#define LED_PA8_NUM 2
#define LED_PB1_NUM 2
#define LED_PB0_NUM 2
#define LED_PD1_NUM 1

Adafruit_NeoPixel strip_channel[4] = {
    Adafruit_NeoPixel(LED_PA11_NUM, PA11, NEO_GRB + NEO_KHZ800),
    Adafruit_NeoPixel(LED_PA8_NUM, PA8, NEO_GRB + NEO_KHZ800),
    Adafruit_NeoPixel(LED_PB1_NUM, PB1, NEO_GRB + NEO_KHZ800),
    Adafruit_NeoPixel(LED_PB0_NUM, PB0, NEO_GRB + NEO_KHZ800)
};
Adafruit_NeoPixel strip_PD1(LED_PD1_NUM, PD1, NEO_GRB + NEO_KHZ800);

// ADC Configuration
int16_t ADC_Calibrattion_Val = 0;
#define ADC_filter_n_pow 8
constexpr int mypow(int a, int b) {
    int x = 1;
    while (b--) x *= a;
    return x;
}
constexpr const int ADC_filter_n = mypow(2, ADC_filter_n_pow);
uint16_t ADC_data[ADC_filter_n][8];
float ADC_V[8];

// DMA for UART
DMA_InitTypeDef Bambubus_DMA_InitStructure;

namespace Hardware {

    void Init() {
        Watchdog_Disable(); // Disable WWDG first!
        System_Init();
        LED_Init();    // Init LEDs 
        UART_Init();   // Init UART/DMA
        ADC_Init();    
        PWM_Init();
        
        // Debug Flash: Red -> Blue -> Green
        LED_SetColor(4, 0, 50, 0, 0); LED_Show(); DelayMS(200);
        LED_SetColor(4, 0, 0, 0, 50); LED_Show(); DelayMS(200);
        LED_SetColor(4, 0, 0, 50, 0); LED_Show(); DelayMS(200);
        LED_SetColor(4, 0, 0, 0, 0);  LED_Show();
    }

    void Watchdog_Disable() {
        WWDG_DeInit();
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, DISABLE);
    }

    void System_Init() {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
    }

    void DelayUS(uint32_t time) {
        delayMicroseconds(time);
    }

    void DelayMS(uint32_t time) {
        delay(time);
    }

    uint64_t GetTime() {
        return get_time64();
    }

    // --- UART ---
    static void (*uart_rx_callback)(uint8_t) = nullptr;

    void UART_SetRxCallback(void (*callback)(uint8_t)) {
        uart_rx_callback = callback;
    }

    void UART_Init() {
        GPIO_InitTypeDef GPIO_InitStructure = {0};
        USART_InitTypeDef USART_InitStructure = {0};
        NVIC_InitTypeDef NVIC_InitStructure = {0};

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

        /* USART1 TX-->A.9   RX-->A.10 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // TX
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // RX
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; // DE
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        GPIOA->BCR = GPIO_Pin_12;

#if defined(STANDARD_SERIAL) || defined(KLIPPER_SERIAL)
        USART_InitStructure.USART_BaudRate = 250000;
        USART_InitStructure.USART_Parity = USART_Parity_No;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
#else
        USART_InitStructure.USART_BaudRate = 1250000;
        USART_InitStructure.USART_Parity = USART_Parity_Even;
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
#endif
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

        USART_Init(USART1, &USART_InitStructure);
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
        USART_ITConfig(USART1, USART_IT_TC, ENABLE);

        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        // Configure DMA1 channel 4 for USART1 TX
        Bambubus_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DATAR;
        Bambubus_DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;
        Bambubus_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
        Bambubus_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
        Bambubus_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        Bambubus_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        Bambubus_DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        Bambubus_DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
        Bambubus_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        Bambubus_DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        Bambubus_DMA_InitStructure.DMA_BufferSize = 0;

        USART_Cmd(USART1, ENABLE);
    }
    
    extern "C" void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
    void USART1_IRQHandler(void)
    {
        volatile uint32_t sr = USART1->STATR; // Read Status Register
        volatile uint32_t dr;
        
        // Check for Read Data Register Not Empty (RXNE) OR Overrun Error (ORE)
        // Note: Reading SR then DR clears ORE, NE, FE, PE
        if ((sr & USART_FLAG_RXNE) || (sr & USART_FLAG_ORE)) 
        {
            dr = USART1->DATAR; // Read Data Register to clear flags
            // Only convert to byte and callback if it was a valid RXNE
            // (ORE might set RXNE too, or just ORE)
            if (sr & USART_FLAG_RXNE) {
                if(uart_rx_callback) uart_rx_callback((uint8_t)dr);
            }
        }

        // Check for Transmission Complete (TC)
        if (sr & USART_FLAG_TC)
        {
            USART_ClearITPendingBit(USART1, USART_IT_TC);
            GPIOA->BCR = GPIO_Pin_12; // Disable DE
        }
    }

    void UART_SendByte(uint8_t data) {
         uint32_t timeout = 10000;
         while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET && timeout--) {
             DelayUS(1);
         }
         if (timeout > 0) USART_SendData(USART1, data);
    }

    void UART_Send(const uint8_t *data, uint16_t length) {
#if defined(STANDARD_SERIAL)
        // Blocking mode for CLI interactivity (safe with ISR echo)
        
        // Disable TC interrupt to prevent ISR from disabling DE prematurely
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
        
        GPIOA->BSHR = GPIO_Pin_12; // Set High (TX) for RS485
        for (uint16_t i = 0; i < length; i++) {
             UART_SendByte(data[i]);
        }
        
        // Re-enable TC interrupt so ISR can disable DE when the *last byte* is actually done
        USART_ITConfig(USART1, USART_IT_TC, ENABLE);
        
        // ISR handles DE pin Disable on TC interrupt.


        
#else
        // DMA mode for High Speed Bus (BambuBus)
        
        // Check if DMA is currently enabled (transfer active)
        if (DMA1_Channel4->CFGR & 0x01) {
             // Wait for data transfer to complete
             uint32_t timeout = 100000;
             while (DMA_GetCurrDataCounter(DMA1_Channel4) != 0 && timeout--) {
                 DelayUS(1);
             }
             // Ensure Transmission Complete flag is set (buffer clear)
             timeout = 100000;
             while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET && timeout--) {
                 DelayUS(1);
             }
             // Create a small gap or ensure Disable is clean
             DMA_Cmd(DMA1_Channel4, DISABLE);
        }

        DMA_DeInit(DMA1_Channel4);
        Bambubus_DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)data;
        Bambubus_DMA_InitStructure.DMA_BufferSize = length;
        DMA_Init(DMA1_Channel4, &Bambubus_DMA_InitStructure);
        DMA_Cmd(DMA1_Channel4, ENABLE);
        GPIOA->BSHR = GPIO_Pin_12;
        USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
#endif
    }

    // --- ADC ---
    void ADC_Init() {
        // GPIO
        {
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_InitTypeDef GPIO_InitStructure;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }

        // DMA
        {
            DMA_InitTypeDef DMA_InitStructure;
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
            DMA_DeInit(DMA1_Channel1);
            DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->RDATAR;
            DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_data;
            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
            DMA_InitStructure.DMA_BufferSize = ADC_filter_n * 8;
            DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
            DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
            DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
            DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
            DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
            DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
            DMA_Init(DMA1_Channel1, &DMA_InitStructure);
            DMA_Cmd(DMA1_Channel1, ENABLE);
        }

        // ADC
        {
            ADC_DeInit(ADC1);
            RCC_ADCCLKConfig(RCC_PCLK2_Div8);
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
            ADC_InitTypeDef ADC_InitStructure;
            ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
            ADC_InitStructure.ADC_ScanConvMode = ENABLE;
            ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
            ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
            ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
            ADC_InitStructure.ADC_NbrOfChannel = 8;
            ADC_Init(ADC1, &ADC_InitStructure);

            ADC_Cmd(ADC1, ENABLE);
            ADC_BufferCmd(ADC1, DISABLE);

            ADC_ResetCalibration(ADC1);
            while (ADC_GetResetCalibrationStatus(ADC1));
            ADC_StartCalibration(ADC1);
            while (ADC_GetCalibrationStatus(ADC1));
            ADC_Calibrattion_Val = Get_CalibrationValue(ADC1);
            for (int i = 0; i < 8; i++)
                ADC_RegularChannelConfig(ADC1, i, i + 1, ADC_SampleTime_239Cycles5);
            ADC_DMACmd(ADC1, ENABLE);
            ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        }
        DelayMS(256); // Wait for buffer fill (ADC_filter_n = 256)
    }

    float* ADC_GetValues() {
        for (int i = 0; i < 8; i++) {
            int data_sum = 0;
            for (int j = 0; j < ADC_filter_n; j++) {
                uint16_t val = ADC_data[j][i];
                int sum = val + ADC_Calibrattion_Val;
                if (sum < 0 || val == 0) ;
                else if (sum > 4095 || val == 4095) data_sum += 4095;
                else data_sum += sum;
            }
            data_sum >>= ADC_filter_n_pow;
            ADC_V[i] = ((float)data_sum) / 4096 * 3.3;
        }
        return ADC_V;
    }

    // --- PWM ---
    void PWM_Init() {
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                                      GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

        // Remap Pins for Motor Control
        GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);    // TIM2 CH1/CH2 -> PA15/PB3
        GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); // TIM3 CH1/CH2 -> PB4/PB5

        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
        TIM_OCInitTypeDef TIM_OCInitStructure;

        TIM_TimeBaseStructure.TIM_Period = 999;
        TIM_TimeBaseStructure.TIM_Prescaler = 1;
        TIM_TimeBaseStructure.TIM_ClockDivision = 0;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
        TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
        TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_Pulse = 0;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
        TIM_OC1Init(TIM2, &TIM_OCInitStructure);
        TIM_OC2Init(TIM2, &TIM_OCInitStructure);
        TIM_OC1Init(TIM3, &TIM_OCInitStructure);
        TIM_OC2Init(TIM3, &TIM_OCInitStructure);
        TIM_OC3Init(TIM4, &TIM_OCInitStructure); // Note: Original looked like OC1/2 for TIM4 too? Let's check Motion_control.cpp.
        // Motion_control.cpp: case 1: TIM_SetCompare1(TIM4, set1); ... case 0: TIM_SetCompare3(TIM4, set1); TIM_SetCompare4(TIM4, set2);
        // So TIM4 uses Ch1, Ch2, Ch3, Ch4.
        
        // Re-checking Motion_control.cpp lines 440+:
        // Case 3: TIM2 Ch1/Ch2 (PA15 / PB3? Remap dependent?)
        // Case 2: TIM3 Ch1/Ch2 (PB4 / PB5?)
        // Case 1: TIM4 Ch1/Ch2 (PB6 / PB7)
        // Case 0: TIM4 Ch3/Ch4 (PB8 / PB9)
        
        // So we need to init all those channels.
        TIM_OC1Init(TIM4, &TIM_OCInitStructure);
        TIM_OC2Init(TIM4, &TIM_OCInitStructure);
        TIM_OC3Init(TIM4, &TIM_OCInitStructure);
        TIM_OC4Init(TIM4, &TIM_OCInitStructure);

        TIM_Cmd(TIM2, ENABLE);
        TIM_Cmd(TIM3, ENABLE);
        TIM_Cmd(TIM4, ENABLE);
    }

    void PWM_Set(uint8_t channel, int PWM) {
        uint16_t set1 = 0, set2 = 0;
        if (PWM > 0) {
            set1 = PWM;
        } else if (PWM < 0) {
            set2 = -PWM;
        } else {
            set1 = 1000;
            set2 = 1000;
        }
        switch (channel) {
        case 3:
            TIM_SetCompare1(TIM2, set1);
            TIM_SetCompare2(TIM2, set2);
            break;
        case 2:
            TIM_SetCompare1(TIM3, set1);
            TIM_SetCompare2(TIM3, set2);
            break;
        case 1:
            TIM_SetCompare1(TIM4, set1);
            TIM_SetCompare2(TIM4, set2);
            break;
        case 0:
            TIM_SetCompare3(TIM4, set1);
            TIM_SetCompare4(TIM4, set2);
            break;
        }
    }

    // --- LED ---
    void LED_Init() {
        strip_PD1.begin();
        for(int i=0; i<4; i++) strip_channel[i].begin();
    }

    void LED_SetColor(uint8_t channel, int led_idx, uint8_t r, uint8_t g, uint8_t b) {
        if (channel < 4) {
             strip_channel[channel].setPixelColor(led_idx, strip_channel[channel].Color(r, g, b));
        } else if (channel == 4) { // Mainboard
             strip_PD1.setPixelColor(led_idx, strip_PD1.Color(r, g, b));
        }
    }

    void LED_Show() {
        strip_PD1.show();
        for(int i=0; i<4; i++) strip_channel[i].show();
    }

    void LED_SetBrightness(uint8_t brightness) {
         // This is global brightness scaling, usually we set it once.
         // Original code had different brightness for PD1 (35) and channels (15).
         
         // If this function is intended to set a global brightness level, it might need more logic
         // or separate functions. For now let's reproduce original setup if brightness == 0 (init).
         
         // Allow custom brightness
         strip_PD1.setBrightness(brightness > 0 ? brightness : 35);
         for(int i=0; i<4; i++) strip_channel[i].setBrightness(brightness > 0 ? brightness : 15);
    }
}
