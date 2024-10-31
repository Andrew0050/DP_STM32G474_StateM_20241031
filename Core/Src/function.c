/* USER CODE BEGIN Header */
	
/* USER CODE END Header */

#include "function.h"
#include "CtlLoop.h"

#include "stm32g4xx_hal_def.h"

//#include "hrtim.h"
// Soft-start state flag
SState_M STState = SSInit;

// OLED refresh counter, increments every 5ms in the 5ms interrupt
uint16_t OLEDShowCnt = 0;

// Define control modes
#define MODE_OPEN 0
#define MODE_CLOSE 1

// Minimum and maximum frequencies (unit: Hz)
#define FREQ_MIN 70000.0f    // 70 kHz
#define FREQ_MAX 130000.0f   // 130 kHz
#define FREQ_STEP_PERCENT 0.1f  // Adjustment step per key press, 0.1%

// Dead time parameters (unit: 0.1%, each step adjusts by 0.1%)
#define DEADTIME_MIN_PX1000 0     // 0%
#define DEADTIME_MAX_PX1000 20    // 2%
#define DEADTIME_STEP_PX1000 1    // 0.1%

// Duty cycle parameters (unit: 0.1%, each step adjusts by 0.1%)
#define DUTY_MIN_PX10 50    // 5.0%
#define DUTY_MAX_PX10 500   // 50.0%
#define DUTY_STEP_PX10 1     // 0.1%

// Assume CNTR_MIN and CNTR_MAX are defined as follows
#define CNTR_MIN 1000
#define CNTR_MAX 16000

// Global variables
extern HRTIM_HandleTypeDef hhrtim1;
extern HRTIM_TimeBaseCfgTypeDef pGlobalTimeBaseCfg;

volatile float currentPWMFreq = 100000.0f;        // Initial frequency 100 kHz
volatile uint8_t gCurrentDeadTimePercent = 2;     // Initial dead time 2%

//volatile uint32_t OPN_DEADTIMEPX1000 = 20;         // Initial dead time 2%
//volatile int32_t vOPNDutyPercentageX10 = 250;      // Initial duty cycle 25.0%
//volatile uint8_t currentMode = MODE_OPEN;           // Initial mode open-loop

// Initialize duty cycle
volatile uint8_t gCurrentDutyPercent_TA1_TB1 = 48; // TA1/TB1 initial duty cycle 48%
volatile uint8_t gCurrentDutyPercent_TA2_TB2 = 48; // TA2/TB2 initial duty cycle 48%

// Time base configuration structure
HRTIM_TimeBaseCfgTypeDef pGlobalTimeBaseCfg;

// Current PLL frequency
volatile uint32_t currentPLLFreq = 160000000;      // 160 MHz, based on fHRCK setting



/** ===================================================================
**     Funtion Name :void Key_Scan(void)
**     Description : The state of two buttons
**       Default state of KEYFlag is 0. When pressed, Flag becomes 1, and when pressed again, Flag becomes 0, cycling in this way.
**       When the machine is running normally or during startup, pressing the button will turn off the output and enter standby mode.
**     Parameters  :
**     Returns     :
** ===================================================================*/
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{			
	/* Check if the button is pressed */
	if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == KEY_OFF)  
	{	 
		/* Wait for the button to be released */
		while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_OFF);   
		return 	KEY_ON;	 
	}
	else
		return KEY_OFF;

}


/** ===================================================================
**     Function Name : void Button_Task(void)
**     Description    : The state of seven buttons

PWM functions:

TA1 -> Positive duty cycle 48%
TB1 -> Positive duty cycle 48%
TA2 -> Positive duty cycle 48%
TB2 -> Positive duty cycle 48%

TA1 and TB1 waveforms are complementary, TA2 and TB2 waveforms are complementary.
-> When TA1 duty cycle is 48%, TB1 duty cycle is 52%
-> When TA2 duty cycle is 48%, TB2 duty cycle is 52%

TA1 and TA2 waveforms are identical, TB1 and TB2 waveforms are identical.
TA1 and TB1 waveforms are complementary, TA2 and TB2 waveforms are complementary.

1. TA1 and TB1 are complementary.
2. TA2 and TB2 are complementary.
3. Dead time for TA1 and TB1 ranges from 2% to 50%, minimum is 2%.
4. Dead time for TA2 and TB2 ranges from 2% to 50%, minimum is 2%.
5. Initial frequency is 100KHz.
6. Duty cycle is 50%.

;---
Button functions:

1. PA6 -> Increases frequency of T1, TB1, TA2, TB2 simultaneously
          -> Upper limit 125K

2. PA7 -> Decreases frequency of T1, TB1, TA2, TB2 simultaneously
          -> Lower limit 77K

3. PB4 -> Increases dead time for TA1, TB1
           -> Upper limit 2%, 50-48 = 2%

4. PB5 -> Decreases dead time for TA1, TB1
           -> Lower limit 10%, 50-40 = 10%

5. PB6 -> Increases duty cycle for TA2, TB2 simultaneously
            -> Upper limit 45%

6. PB7  -> Decreases duty cycle for TA2, TB2 simultaneously
            -> Minimum 5%

7. PB9  -> Switches operating mode
            -> open loop / closed loop

Button function descriptions:
1. KEY1 increases frequency by 1% each time it is pressed.
2. KEY2 decreases frequency by 1% each time it is pressed.
3. KEY3 increases dead time by 1% each time it is pressed.
4. KEY4 decreases dead time by 1% each time it is pressed.
5. KEY5 increases duty cycle by 1% each time it is pressed.
6. KEY6 decreases duty cycle by 1% each time it is pressed.
7. KEY7 switches mode / open-loop or closed-loop.

**     Parameters  :
**     Returns     :
** ===================================================================*/

void Button_Task(void)
{
	// KEY1/PA6 : Increase frequency of T1, TB1, TA2, and TB2 simultaneously
	if (Key_Scan(KEY1_INC_Freq_GPIO_Port, KEY1_INC_Freq_Pin) == KEY_ON)
	{
		if (currentPWMFreq < FREQ_MAX)
		{
			currentPWMFreq *= (1.0f + (FREQ_STEP_PERCENT / 100.0f)); // Increase by 0.1%
			if (currentPWMFreq > FREQ_MAX)
				currentPWMFreq = FREQ_MAX;
	
			// Update HRTIM frequency
			if (SetPWMFrequency((uint32_t)currentPWMFreq) != HAL_OK) {
				// Handle error
				Error_Handler();
			}
			HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin);
		}
	}
	
	// KEY2/PA7 : Decrease frequency of T1, TB1, TA2, and TB2 simultaneously
	if (Key_Scan(KEY2_DEC_Freq_GPIO_Port, KEY2_DEC_Freq_Pin) == KEY_ON)
	{
		if (currentPWMFreq > FREQ_MIN)
		{
			currentPWMFreq *= (1.0f - (FREQ_STEP_PERCENT / 100.0f)); // Decrease by 0.1%
			if (currentPWMFreq < FREQ_MIN)
				currentPWMFreq = FREQ_MIN;
	
			// Update HRTIM frequency
			if (SetPWMFrequency((uint32_t)currentPWMFreq) != HAL_OK) {
				// Handle error
				Error_Handler();
			}
			HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin);
		}
	}

    // KEY3/PB4: Increase dead time of TA1/TB1
    if (Key_Scan(KEY3_INC_DT_GPIO_Port, KEY3_INC_DT_Pin) == KEY_ON)
    {
		if (gCurrentDeadTimePercent < 50)
		{
			gCurrentDeadTimePercent += 1; // Increase by 1%
	
			if (gCurrentDeadTimePercent > 50)
				gCurrentDeadTimePercent = 50;
	
				// Manually adjust dead time
				SetDeadTimeManual(gCurrentDeadTimePercent);
	
				HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); 
		}
    }

	// KEY4/PB5: Decrease dead time of TA1/TB1
    if (Key_Scan(KEY4_DEC_DT_GPIO_Port, KEY4_DEC_DT_Pin) == KEY_ON)
    {
		if (gCurrentDeadTimePercent > 10)
		{
			gCurrentDeadTimePercent -= 1; // Decrease by 1%
	
			if (gCurrentDeadTimePercent < 10)
				gCurrentDeadTimePercent = 10;
	
					// Manually adjust dead time
					SetDeadTimeManual(gCurrentDeadTimePercent);
					HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); 
			}
    }

	// KEY5/PB6: Increase duty cycle of TA2/TB2 simultaneously
    if (Key_Scan(KEY5_INC_DUTY_GPIO_Port, KEY5_INC_DUTY_Pin) == KEY_ON)
    {
		if (gCurrentDutyPercent_TA2_TB2 < 45)
		{
			gCurrentDutyPercent_TA2_TB2 += 1; // Increase by 1%
			if (gCurrentDutyPercent_TA2_TB2 > 45)
				gCurrentDutyPercent_TA2_TB2 = 45;
	
				// Update duty cycle
				if (SetDutyCycle_TA2_TB2(gCurrentDutyPercent_TA2_TB2) != HAL_OK) {
					// Handle error
					Error_Handler();
				}
					HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); 
			}
    }
	// KEY6/PB7: Decrease duty cycle of TA2/TB2 simultaneously
    if (Key_Scan(KEY6_DEC_DUTY_GPIO_Port, KEY6_DEC_DUTY_Pin) == KEY_ON)
    {
			if (gCurrentDutyPercent_TA2_TB2 > 5)
			{
				gCurrentDutyPercent_TA2_TB2 -= 1; // Decrease by 1%
				if (gCurrentDutyPercent_TA2_TB2 < 5)
					gCurrentDutyPercent_TA2_TB2 = 5;
	
				// Update duty cycle
				if (SetDutyCycle_TA2_TB2(gCurrentDutyPercent_TA2_TB2) != HAL_OK) {
					// Handle error
					Error_Handler();
				}
					HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); // Toggle LED
			}
    }

    if (Key_Scan(KEY7_SWITCH_MODE_GPIO_Port, KEY7_SWITCH_MODE_Pin) == KEY_ON)
    {
		Mode_Switch(); // Switch mode
    }
	// Update OLED display
		UpdateDisplay();

}



/** ===================================================================
**     Function Name : Set_HRTIM_CompareValue
**     Description : 

Currently using 100MHz for frequency division, timebase = 16000 = 100KHz
Midpoint of period 50% = 8000

1.  Duty cycle cannot exceed 50%
2.  Frequency upper limit is 130KHz, lower limit is 70KHz
3.  Dead time upper limit is 2%

**     Parameters  : req_tim_freq - Requested PWM frequency (Hz).
**     Returns     : HAL_StatusTypeDef - Returns HAL_OK on success, HAL_ERROR on failure.
** ===================================================================*/
HAL_StatusTypeDef SetPWMFrequency(uint32_t req_tim_freq) {
    uint32_t prescaler_value;
    uint32_t fHRCK;
    uint32_t period;

    // Define frequency range
    if (req_tim_freq < FREQ_MIN || req_tim_freq > FREQ_MAX) {
        return HAL_ERROR; // Frequency out of range
    }

    if (req_tim_freq >= 100000) {
        // Use prescaler = MUL16
        prescaler_value = 16;
        fHRCK = 100000000UL * prescaler_value; // 100MHz * 16 = 1.6GHz
    } else {
        // Use prescaler = MUL8
        prescaler_value = 8;
        fHRCK = 100000000UL * prescaler_value; // 100MHz * 8 = 800MHz (adjust according to actual clock configuration)
    }

    // Calculate the new period
    period = fHRCK / req_tim_freq;

    // Verify if the period is within the allowable range
    if (period < CNTR_MIN || period > CNTR_MAX) {
        return HAL_ERROR;
    }

    // Update the period and prescaler ratio in the global time base configuration
    pGlobalTimeBaseCfg.Period = period;
    if (prescaler_value == 16) {
        pGlobalTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL16;
    } else if (prescaler_value == 8) {
        pGlobalTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL8;
    } else {
        return HAL_ERROR; // Unsupported prescaler
    }

    // Configure the time base for Timer A
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pGlobalTimeBaseCfg) != HAL_OK) {
        return HAL_ERROR;
    }

    // Configure the time base for Timer B
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pGlobalTimeBaseCfg) != HAL_OK) {
        return HAL_ERROR;
    }

    // Set compare unit 2 to midpoint value (50% duty cycle)
    HRTIM_CompareCfgTypeDef pCompareCfg = {0};
    pCompareCfg.CompareValue = (period / 2) - 1;
    pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    pCompareCfg.AutoDelayedTimeout = 0x0000;

    // Configure compare unit 2 for Timer A
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK) {
        return HAL_ERROR;
    }

    // Configure compare unit 2 for Timer B
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK) {
        return HAL_ERROR;
    }

    // Software reset for Timer A and Timer B of HRTIM to apply new settings
    if (HAL_HRTIM_SoftwareReset(&hhrtim1, HRTIM_TIMERRESET_TIMER_A | HRTIM_TIMERRESET_TIMER_B) != HAL_OK) {
        return HAL_ERROR;
    }

    // Update global frequency variables
    currentPWMFreq = req_tim_freq;
    currentPLLFreq = fHRCK;

    return HAL_OK;
}



/**
  * @brief  Manually set dead time.
  * @param  dead_time_percent - New dead time percentage
  * @retval None
  */
void SetDeadTimeManual(uint8_t dead_time_percent)
{
    // Ensure dead time is within allowable range
    if (dead_time_percent < 2 || dead_time_percent > 50) {
        return;
    }

    uint32_t period = pGlobalTimeBaseCfg.Period;
    uint32_t dead_time_ticks = (period * dead_time_percent) / 100;

    // Adjust Timer A Compare Unit 2 (midpoint) for TA1 ResetSource
    HRTIM_CompareCfgTypeDef compareConfigA = {0};
    compareConfigA.CompareValue = dead_time_ticks;
    compareConfigA.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    compareConfigA.AutoDelayedTimeout = 0x0000;

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &compareConfigA) != HAL_OK) {
        Error_Handler();
    }

    // Adjust Timer B Compare Unit 2 (midpoint) for TB1 ResetSource
    HRTIM_CompareCfgTypeDef compareConfigB = {0};
    compareConfigB.CompareValue = period - dead_time_ticks;
    compareConfigB.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    compareConfigB.AutoDelayedTimeout = 0x0000;

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &compareConfigB) != HAL_OK) {
        Error_Handler();
    }

    // Restart timers to apply the new dead time
    if (HAL_HRTIM_SoftwareReset(&hhrtim1, HRTIM_TIMERRESET_TIMER_A | HRTIM_TIMERRESET_TIMER_B) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B) != HAL_OK) {
        Error_Handler();
    }
}


/**
  * @brief  Set the PWM duty cycle for TA1/TB1 of HRTIM.
  * @param  duty_percent - New duty cycle (percentage).
  * @retval HAL_StatusTypeDef - Returns HAL_OK on success, HAL_ERROR on failure.
  */
HAL_StatusTypeDef SetDutyCycle_TA1_TB1(uint8_t duty_percent)
{
    if (duty_percent < 5 || duty_percent > 95)
    {
        return HAL_ERROR;
    }

    uint32_t period = pGlobalTimeBaseCfg.Period;
    uint32_t compare_value = (period * duty_percent) / 100;

    // Update Timer A Compare Unit 2 (midpoint) corresponding to TA1 ResetSource
    HRTIM_CompareCfgTypeDef compareConfig = {0};
    compareConfig.CompareValue = compare_value;
    compareConfig.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    compareConfig.AutoDelayedTimeout = 0x0000;

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &compareConfig) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &compareConfig) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
  * @brief  Set the PWM duty cycle for TA2/TB2 of HRTIM.
  * @param  duty_percent - New duty cycle (percentage).
  * @retval HAL_StatusTypeDef - Returns HAL_OK on success, HAL_ERROR on failure.
  */
HAL_StatusTypeDef SetDutyCycle_TA2_TB2(uint8_t duty_percent)
{
    if (duty_percent < 5 || duty_percent > 45)
    {
        return HAL_ERROR;
    }

    uint32_t period = pGlobalTimeBaseCfg.Period;
    uint32_t compare_value = (period * duty_percent) / 100;

    // Update Timer A Compare Unit 2 (midpoint) corresponding to TA2 ResetSource
    HRTIM_CompareCfgTypeDef compareConfig = {0};
    compareConfig.CompareValue = compare_value;
    compareConfig.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    compareConfig.AutoDelayedTimeout = 0x0000;

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &compareConfig) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &compareConfig) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}




/*
** ===================================================================
**     Function Name :   void ADCSample(void)
**     Description :    Samples Vin, Iin, Vout, and Iout
**     Parameters  :
**     Returns     :
** ===================================================================
*/

struct _ADI SADC={2048,2048,0,0,2048,2048,0,0,0,0}; // Input and output parameter sampling values and average values
struct _Ctr_value CtrValue={0,0,0,MIN_BUKC_DUTY,0,0,0}; // Control parameters
struct _FLAG DF={0,0,0,0,0,0,0,0}; // Control flag bits
uint16_t ADC1_RESULT[4]={0,0,0,0}; // DMA data storage register for transferring ADC samples from peripheral to memory




CCMRAM void ADCSample(void)
{
	// Declare variables for averaging Vin, Iin, Vout, and Iout
	static uint32_t VinAvgSum=0, IinAvgSum=0, VoutAvgSum=0, IoutAvgSum=0;
	
	// Convert ADC readings using calibration factors (Q15 format), including offset compensation
	SADC.Vin  = ((uint32_t)ADC1_RESULT[0] * CAL_VIN_K >> 12) + CAL_VIN_B;
	SADC.Iin  = ((uint32_t)ADC1_RESULT[1] * CAL_IIN_K >> 12) + CAL_IIN_B;
	SADC.Vout = ((uint32_t)ADC1_RESULT[2] * CAL_VOUT_K >> 12) + CAL_VOUT_B;
	SADC.Iout = ((uint32_t)ADC1_RESULT[3] * CAL_IOUT_K >> 12) + CAL_IOUT_B;

	// Check for invalid readings; if Vin is below the threshold, set it to 0
	if(SADC.Vin < 100) 
		SADC.Vin = 0;
	
	// If Iin is less than 2048, set it to 2048 (0A)
	if(SADC.Iin < 2048) 
		SADC.Iin = 2048;
	
	if(SADC.Vout < 100)
		SADC.Vout = 0;
	
	if(SADC.Iout < 2048)
		SADC.Iout = 2048;

	// Calculate average values of Vin, Iin, Vout, and Iout using moving average
	VinAvgSum = VinAvgSum + SADC.Vin - (VinAvgSum >> 2); // Add current Vin value and subtract the oldest value
	SADC.VinAvg = VinAvgSum >> 2; // Update Vin average
	
	IinAvgSum = IinAvgSum + SADC.Iin - (IinAvgSum >> 2); // Add current Iin value and subtract the oldest value
	SADC.IinAvg = IinAvgSum >> 2; // Update Iin average
	
	VoutAvgSum = VoutAvgSum + SADC.Vout - (VoutAvgSum >> 2); // Add current Vout value and subtract the oldest value
	SADC.VoutAvg = VoutAvgSum >> 2; // Update Vout average
	
	IoutAvgSum = IoutAvgSum + SADC.Iout - (IoutAvgSum >> 2); // Add current Iout value and subtract the oldest value
	SADC.IoutAvg = IoutAvgSum >> 2; // Update Iout average
}



#define MODE_OPEN_LOOP 0
#define MODE_CLOSED_LOOP 1
// Mode variable, default to open-loop mode
volatile uint8_t currentMode = MODE_OPEN_LOOP;

/**
  * @brief  Mode switch function
  * @retval None
  */
void Mode_Switch(void)
{
    if (currentMode == MODE_OPEN_LOOP)
    {
        currentMode = MODE_CLOSED_LOOP;

        // Initialize frequency to 100 kHz
        currentPWMFreq = 100000.0f;
        if (SetPWMFrequency((uint32_t)currentPWMFreq) != HAL_OK)
        {
            Error_Handler();
        }

        // Display mode change
        OLED_ShowStr(55, 0, "Close", 2);
    }
    else
    {
        currentMode = MODE_OPEN_LOOP;

        // Initialize frequency to 100 kHz
        currentPWMFreq = 100000.0f;
        if (SetPWMFrequency((uint32_t)currentPWMFreq) != HAL_OK)
        {
            Error_Handler();
        }


        OLED_ShowStr(55, 0, "      ", 2); // Clear previous display
        OLED_ShowStr(55, 0, "Open", 2);   // Display mode change
    }

    HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); // Toggle LED to indicate mode change
}




/**
  * @brief  Update OLED display function
  * @retval None
  */
void UpdateDisplay(void)
{
    // Display different information based on mode
    if (currentMode == MODE_CLOSED_LOOP)
    {
        // Closed-loop mode: display frequency converted from ADC voltage

        // Ensure ADC has sampled
        ADCSample();

        // Convert ADC voltage to 0-3.3V
        float adc_voltage = (SADC.VinAvg / 4095.0f) * 3.3f;

        // Calculate corresponding frequency
        // Midpoint 1.65V corresponds to 100 kHz
        // Set frequency range from 50 kHz to 150 kHz
        float frequency = 100000.0f + ((adc_voltage - 1.65f) / 1.65f) * 50000.0f; // ¡Ó50 kHz


        // Limit frequency range
        if (frequency < FREQ_MIN)
            frequency = FREQ_MIN;
        if (frequency > FREQ_MAX)
            frequency = FREQ_MAX;

		// Update global frequency variable
		currentPWMFreq = frequency;
		//currentPLLFreq = fHRCK; // Ensure fHRCK is correctly calculated and defined
		
		// Set PWM frequency
		if (SetPWMFrequency((uint32_t)currentPWMFreq) != HAL_OK)
		{
			Error_Handler();
		}
		
		// Display frequency with two decimal places
		char freqStr[10];
		sprintf(freqStr, "%.2f", currentPWMFreq / 1000.0f); // e.g., "100.00KHz"
		
		// Clear previous display area (adjust blank spaces as needed)
		//OLED_ShowStr(45, 2, "		 ", 2); // Clear previous display at (45,2)
		// Display new frequency
		OLED_ShowStr(45, 2, freqStr, 2); // Display current frequency at (45,2)


	 	// Display ADC voltage
		// Convert ADC voltage to 0-3.3V
		float adc_voltage_display = (SADC.VinAvg / 4095.0f) * 3.3f;
	
		// Convert voltage to display format (mV)
		uint8_t Vtemp[4] = {0};
		uint32_t Vdisplay = (uint32_t)(adc_voltage_display * 1000.0f); // Convert to mV
	
		Vtemp[0] = (uint8_t)(Vdisplay / 1000);
		Vtemp[1] = (uint8_t)((Vdisplay % 1000) / 100);
		Vtemp[2] = (uint8_t)((Vdisplay % 100) / 10);
		Vtemp[3] = (uint8_t)(Vdisplay % 10);
	
		// Display ADC voltage
		OLEDShowData(50, 6, Vtemp[0]);
		OLEDShowData(65, 6, Vtemp[1]);
		OLEDShowData(75, 6, Vtemp[2]);
		OLEDShowData(85, 6, Vtemp[3]);

    }
    else
    {
        // Open-loop mode: display the currently manually set frequency

        // Display frequency
        //unsigned char freqStr[10];
        //sprintf(freqStr, "%.1fKHz", currentPWMFreq / 1000.0f);
        //OLED_ShowStr(60, 6, "     ", 2); // Clear previous display
        //OLED_ShowStr(60, 6, freqStr, 2);

		//OLED_ShowStr(50, 6, "	     ", 2); // Clear previous display
		//OLED_ShowStr(30, 6, " STOP   ", 2);   // Display mode change

		//OLED_ShowStr(0, 6, "ADC:", 2);
		//OLED_ShowStr(60, 6, ".", 2);
		//OLED_ShowStr(95, 6, "V", 2);

		// Display frequency with two decimal places
		char freqStr[10];
		sprintf(freqStr, "%.2f", currentPWMFreq / 1000.0f);
		//OLED_ShowStr(45, 2, "		 ", 2); // Clear previous display
		OLED_ShowStr(45, 2, freqStr, 2); // Display current frequency
	


    }


    // Optional: Display other data, such as Iout, etc.
}


/** ===================================================================
**     Function Name : void MX_OLED_Init(void)
**     Description : OLED initialization routine
**     Initialize the OLED display interface
**     Parameters  :
**     Returns     :
** ===================================================================*/
void OLED_Open_Mode_Init(void)
{
	// Initialize the OLED
	OLED_Init();
	OLED_CLS(); // Clear the OLED display
	
	// Display initial text labels on the OLED
	OLED_ShowStr(0, 0, "Mode:", 2);

	OLED_ShowStr(0, 2, "Freq:", 2);
	OLED_ShowStr(68, 2, ".", 2);
	OLED_ShowStr(100, 2, "KHz", 2);

	OLED_ShowStr(0, 4, "Du/DT:", 2);
	OLED_ShowStr(85, 4, "/", 2);
	OLED_ShowStr(120, 4, "%", 2);

	OLED_ShowStr(0, 6, "ADC:", 2);
	OLED_ShowStr(60, 6, ".", 2);
	OLED_ShowStr(95, 6, "V", 2);

	OLED_ON(); // Turn on the OLED display
}



/** ===================================================================
**     Function Name : void MX_OLED_Init(void)
**     Description : OLED initialization routine
**     Initialize the OLED display interface
**     Parameters  :
**     Returns     :
** ===================================================================*/
void MX_OLED_Init(void)
{
	// Initialize the OLED
	OLED_Init();
	OLED_CLS(); // Clear the OLED display
	
	// Display initial text labels on the OLED
	OLED_ShowStr(0, 0, "Mode:", 2);

	OLED_ShowStr(0, 2, "ADC:", 2);
	OLED_ShowStr(60, 2, ".", 2);
	OLED_ShowStr(95, 2, "V", 2);

	OLED_ShowStr(0, 4, "Duty:", 2);
	OLED_ShowStr(68, 4, ".", 2);
	OLED_ShowStr(95, 4, "%", 2);

	OLED_ShowStr(0, 6, "Freq:", 2);
	OLED_ShowStr(68, 6, ".", 2);
	OLED_ShowStr(95, 6, "KHz", 2);

	OLED_ON(); // Turn on the OLED display
}




/** ===================================================================
**     Function Name : void StateM(void)
**     Description   : State machine function, runs in a 5ms interrupt,
**                     executes every 5ms.
**     Initialization state
**     Waiting for soft start state
**     Start state
**     Running state
**     Fault state
**     Parameters    :
**     Returns       :
** ===================================================================*/
void StateM(void)
{
    // Determine the state type
    switch(DF.SMFlag)
    {
        // Initialization state
        case Init: StateMInit();
        break;
        
        // Waiting state
        case Wait: StateMWait();
        break;
        
        // Soft start state
        case Rise: StateMRise();
        break;
        
        // Running state
        case Run: StateMRun();
        break;
        
        // Fault state
        case Err: StateMErr();
        break;
    }
}

/** ===================================================================
**     Function Name : void StateMInit(void)
**     Description   : Initialization state function, parameter initialization
**     Parameters    :
**     Returns       :
** ===================================================================*/
void StateMInit(void)
{
    // Relevant parameter initialization
    ValInit();
    // State machine transitions to waiting for soft start state
    DF.SMFlag = Wait;
}


/** ===================================================================
**     Funtion Name :void StateMWait(void)
**     Description :   
**     Parameters  :
**     Returns     :
** ===================================================================*/
void StateMWait(void)
{


}
/*
** ===================================================================
**     Funtion Name : void StateMRise(void)
**     Description :ÈíÆô
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_SSCNT       20/
void StateMRise(void)
{
	
	
}
/*
** ===================================================================
**     Funtion Name :void StateMRun(void)
**     Description :
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
void StateMRun(void)
{

}

/*
** ===================================================================
**     Funtion Name :void StateMErr(void)
**     Description :
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
void StateMErr(void)
{

}

/** ===================================================================
**     Funtion Name :void ValInit(void)
**     Description :   
**     Parameters  :
**     Returns     :
** ===================================================================*/
void ValInit(void)
{

}

/** ===================================================================
**     Funtion Name :void VrefGet(void)
**     Description :   
**     Parameters  :
**     Returns     :
** ===================================================================*/
void VrefGet(void)
{

}
/*
** ===================================================================
**     Funtion Name :void ShortOff(void)
**     Description :
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_SHORT_I     3444
#define MIN_SHORT_V     289
void ShortOff(void)
{

}
/*
** ===================================================================
**     Funtion Name :void SwOCP(void)
**     Description :Èí
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_OCP_VAL     3165
void SwOCP(void)
{

}

/*
** ===================================================================
**     Funtion Name :void SwOVP(void)
**     Description :Èí
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_VOUT_OVP_VAL    3012
void VoutSwOVP(void)
{

}

/*
** ===================================================================
**     Funtion Name :void VinSwUVP(void)
**     Description :
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MIN_UVP_VAL    686
#define MIN_UVP_VAL_RE  795
void VinSwUVP(void)
{

}

/*
** ===================================================================
**     Funtion Name :void VinSwOVP(void)
**     Description :
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_VIN_OVP_VAL    3012
void VinSwOVP(void)
{

}

/** ===================================================================
**     Funtion Name :void LEDShow(void)
**     Description :  LED
**     Parameters  :
**     Returns     :
** ===================================================================*/
void LEDShow(void)
{

}





/** ===================================================================
**     Funtion Name :void BBMode(void)
**     Description :
**     Parameters  :
**     Returns     :
** ===================================================================*/
void BBMode(void)
{
	DF.BBFlag = Buck;//buck mode
}


/** ===================================================================
**     Funtion Name :void KEYFlag(void)
**     Description : The state of two buttons
**       Default state of KEYFlag is 0. When pressed, Flag becomes 1, and when pressed again, Flag becomes 0, cycling in this way.
**       When the machine is running normally or during startup, pressing the button will turn off the output and enter standby mode.
**     Parameters  :
**     Returns     :
** ===================================================================*/
#define READ_KEY1_INC_Freq() HAL_GPIO_ReadPin(GPIOA, KEY1_INC_Freq_Pin)
#define READ_KEY2_DEC_Freq() HAL_GPIO_ReadPin(GPIOA, KEY2_DEC_Freq_Pin)
void KEYFlag(void)
{
    // Timer, used for button debounce
    static uint16_t KeyDownCnt1 = 0, KeyDownCnt2 = 0;
    
    // Button pressed
    if (READ_KEY1_INC_Freq() == 0)
    {
        // Timing, button press is valid after 150 ms
        KeyDownCnt1++;
        if (KeyDownCnt1 > 30)
        {
            KeyDownCnt1 = 0; // Reset timer
            // Button state has changed
            if (DF.KeyFlag1 == 0)
                DF.KeyFlag1 = 1;
            else
                DF.KeyFlag1 = 0;
        }
    }
    else
        KeyDownCnt1 = 0; // Reset timer
    
    // Button pressed
    if (READ_KEY2_DEC_Freq() == 0)
    {
        // Timing, button press is valid after 150 ms
        KeyDownCnt2++;
        if (KeyDownCnt2 > 30)
        {
            KeyDownCnt2 = 0; // Reset timer
            // Button state has changed
            if (DF.KeyFlag2 == 0)
                DF.KeyFlag2 = 1;
            else
                DF.KeyFlag2 = 0;
        }
    }
    else
        KeyDownCnt2 = 0; // Reset timer

    // When the machine is running normally or during startup, pressing the button will turn off the output and enter standby mode
    if ((DF.KeyFlag1 == 0) && ((DF.SMFlag == Rise) || (DF.SMFlag == Run)))
    {
        DF.SMFlag = Wait;
        // Turn off PWM
        DF.PWMENFlag = 0;
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2); // Turn off
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2); // Turn off    
    }
}





