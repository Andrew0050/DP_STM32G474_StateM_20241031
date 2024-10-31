/* USER CODE BEGIN Header */
	
/* USER CODE END Header */

#include "function.h"//功能函数头文件
#include "CtlLoop.h"//控制环路头文件

#include "stm32g4xx_hal_def.h"

//#include "hrtim.h"
// Soft-start state flag
SState_M STState = SSInit;

// OLED refresh counter, increments every 5ms in the 5ms interrupt
uint16_t OLEDShowCnt = 0;




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
**     Description :   等待状态机，等待1S后无故障则软启
**     Parameters  :
**     Returns     :
** ===================================================================*/
void StateMWait(void)
{
	//计数器定义
	static uint16_t CntS = 0;
	
	//关PWM
	DF.PWMENFlag=0;
	//计数器累加
	CntS ++;
	//等待1S，无故障情况,切按键按下，启动，则进入启动状态
	if(CntS>200)
	{
		CntS=200;
		HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //开启PWM输出和PWM计时器
		if((DF.ErrFlag==F_NOERR)&&(DF.KeyFlag1==1))
		{
			//计数器清0
			CntS=0;
			//状态标志位跳转至等待状态
			DF.SMFlag  = Rise;
			//软启动子状态跳转至初始化状态
			STState = SSInit;
		}
	}
}
/*
** ===================================================================
**     Funtion Name : void StateMRise(void)
**     Description :软启阶段
**     软启初始化
**     软启等待
**     开始软启
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_SSCNT       20//等待100ms
void StateMRise(void)
{
	//计时器
	static  uint16_t  Cnt = 0;
	//最大占空比限制计数器
	static  uint16_t	BUCKMaxDutyCnt=0,BoostMaxDutyCnt=0;

	//判断软启状态
	switch(STState)
	{
		//初始化状态
		case    SSInit:
		{
			//关闭PWM
			DF.PWMENFlag=0;
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //关闭
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //关闭				
			//软启中将运行限制占空比启动，从最小占空比开始启动
			CtrValue.BUCKMaxDuty  = MIN_BUKC_DUTY;
			CtrValue.BoostMaxDuty = MIN_BOOST_DUTY;
			//环路计算变量初始化
			VErr0=0;
			VErr1=0;
			VErr2=0;
			u0 = 0;
			u1 = 0;
			//跳转至软启等待状态
			STState = SSWait;

			break;
		}
		//等待软启动状态
		case    SSWait:
		{
			//计数器累加
			Cnt++;
			//等待100ms
			if(Cnt> MAX_SSCNT)
			{
				//计数器清0
				Cnt = 0;
				//限制启动占空比
				CtrValue.BuckDuty = MIN_BUKC_DUTY;
				CtrValue.BUCKMaxDuty= MIN_BUKC_DUTY;
				CtrValue.BoostDuty = MIN_BOOST_DUTY;
				CtrValue.BoostMaxDuty = MIN_BOOST_DUTY;
				//环路计算变量初始化
				VErr0=0;
				VErr1=0;
				VErr2=0;
				u0 = 0;
				u1 = 0;
				//CtrValue.Voref输出参考电压从一半开始启动，避免过冲，然后缓慢上升
				CtrValue.Voref  = CtrValue.Voref >>1;			
				STState = SSRun;	//跳转至软启状态		
			}
			break;
		}
		//软启动状态
		case    SSRun:
		{
			if(DF.PWMENFlag==0)//正式发波前环路变量清0
			{
				//环路计算变量初始化
				VErr0=0;
				VErr1=0;
				VErr2=0;
				u0 = 0;
				u1 = 0;	
				HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //开启PWM输出和PWM计时器
				HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //开启PWM输出和PWM计时器					
			}
			//发波标志位置位
			DF.PWMENFlag=1;
			//最大占空比限制逐渐增加
			BUCKMaxDutyCnt++;
			BoostMaxDutyCnt++;
			//最大占空比限制累加
			CtrValue.BUCKMaxDuty = CtrValue.BUCKMaxDuty + BUCKMaxDutyCnt*5;
			CtrValue.BoostMaxDuty = CtrValue.BoostMaxDuty + BoostMaxDutyCnt*5;
			//累加到最大值
			if(CtrValue.BUCKMaxDuty > MAX_BUCK_DUTY)
				CtrValue.BUCKMaxDuty  = MAX_BUCK_DUTY ;
			if(CtrValue.BoostMaxDuty > MAX_BOOST_DUTY)
				CtrValue.BoostMaxDuty  = MAX_BOOST_DUTY ;
			
			if((CtrValue.BUCKMaxDuty==MAX_BUCK_DUTY)&&(CtrValue.BoostMaxDuty==MAX_BOOST_DUTY))			
			{
				//状态机跳转至运行状态
				DF.SMFlag  = Run;
				//软启动子状态跳转至初始化状态
				STState = SSInit;	
			}
			break;
		}
		default:
		break;
	}
}
/*
** ===================================================================
**     Funtion Name :void StateMRun(void)
**     Description :正常运行，主处理函数在中断中运行
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
**     Description :故障状态
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
void StateMErr(void)
{
	//关闭PWM
	DF.PWMENFlag=0;
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //关闭
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //关闭		
	//若故障消除跳转至等待重新软启
	if(DF.ErrFlag==F_NOERR)
			DF.SMFlag  = Wait;
}

/** ===================================================================
**     Funtion Name :void ValInit(void)
**     Description :   相关参数初始化函数
**     Parameters  :
**     Returns     :
** ===================================================================*/
void ValInit(void)
{
	//关闭PWM
	DF.PWMENFlag=0;
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //关闭
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //关闭		
	//清除故障标志位
	DF.ErrFlag=0;
	//初始化电压参考量
	CtrValue.Voref=0;
	//限制占空比
	CtrValue.BuckDuty = MIN_BUKC_DUTY;
	CtrValue.BUCKMaxDuty= MIN_BUKC_DUTY;
	CtrValue.BoostDuty = MIN_BOOST_DUTY;
	CtrValue.BoostMaxDuty = MIN_BOOST_DUTY;	
	//环路计算变量初始化
	VErr0=0;
	VErr1=0;
	VErr2=0;
	u0 = 0;
	u1 = 0;
}

/** ===================================================================
**     Funtion Name :void VrefGet(void)
**     Description :   从滑动变阻器值获取输出电压参考值，调整滑动电位器，CtrValue.Voref变换，当电压参考值变化时，缓慢增加
**     Parameters  :
**     Returns     :
** ===================================================================*/
#define MAX_VREF    2921//输出最大参考电压48V  0.5V的余量   48.5V/68V*Q12
#define MIN_VREF    271//最低电压参考值5V   0.5V的余量   4.5V/68V*2^Q12
#define VREF_K      10//递增或递减步长
void VrefGet(void)
{
	//电压参考值中间变量
	int32_t VTemp = 0;	
	//滑动平均求和中间变量
	static int32_t VadjSum = 0;

	//获取ADC采样值-读滑动电位器上的电压
	SADC.Vadj = HAL_ADC_GetValue(&hadc1);
	//对采样值滑动求平均
	VadjSum = VadjSum + SADC.Vadj -(VadjSum>>8);
	SADC.VadjAvg = VadjSum>>8;
	
	//参考电压 = MIN_VREF+滑动变阻器采样值，MIN_VREF为最低输出电压。
	VTemp = MIN_VREF + SADC.Vadj;
	
	//缓慢递增或缓慢递减电压参考值
	if( VTemp> ( CtrValue.Voref + VREF_K))
			CtrValue.Voref = CtrValue.Voref + VREF_K;
	else if( VTemp < ( CtrValue.Voref - VREF_K ))
			CtrValue.Voref =CtrValue.Voref - VREF_K;
	else
			CtrValue.Voref = VTemp ;

	//BUCK 模式下调压限制，输出电压最大达到输入电压的0.85倍
	if(CtrValue.Voref >((SADC.VinAvg*3482)>>12))//输出限制在输入的0.85*vin 
		CtrValue.Voref =((SADC.VinAvg*3482)>>12);
}

/*
** ===================================================================
**     Funtion Name :void ShortOff(void)
**     Description :短路保护，可以重启10次
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_SHORT_I     3444//短路电流判据
#define MIN_SHORT_V     289//短路电压判据
void ShortOff(void)
{
	static int32_t RSCnt = 0;
	static uint8_t RSNum =0 ;

	//当输出电流大于 *A，且电压小于*V时，可判定为发生短路保护
	if((SADC.Iout> MAX_SHORT_I)&&(SADC.Vout <MIN_SHORT_V))
	{
		//关闭PWM
		DF.PWMENFlag=0;
		HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //关闭
		HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //关闭	
		//故障标志位
		setRegBits(DF.ErrFlag,F_SW_SHORT);
		//跳转至故障状态
		DF.SMFlag  =Err;
	}
	//输出短路保护恢复
	//当发生输出短路保护，关机后等待4S后清楚故障信息，进入等待状态等待重启
	if(getRegBits(DF.ErrFlag,F_SW_SHORT))
	{
		//等待故障清楚计数器累加
		RSCnt++;
		//等待2S
		if(RSCnt >400)
		{
			//计数器清零
			RSCnt=0;
			//短路重启只重启10次，10次后不重启
			if(RSNum > 10)
			{
				//确保不清除故障，不重启
				RSNum =11;
				//关闭PWM
				DF.PWMENFlag=0;
				HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //关闭
				HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //关闭	
			}
			else
			{
				//短路重启计数器累加
				RSNum++;
				//清除过流保护故障标志位
				clrRegBits(DF.ErrFlag,F_SW_SHORT);
			}
		}
	}
}
/*
** ===================================================================
**     Funtion Name :void SwOCP(void)
**     Description :软件过流保护，可重启
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_OCP_VAL     3165//*A过流保护点 
void SwOCP(void)
{
	//过流保护判据保持计数器定义
	static  uint16_t  OCPCnt=0;
	//故障清楚保持计数器定义
	static  uint16_t  RSCnt=0;
	//保留保护重启计数器
	static  uint16_t  RSNum=0;

	//当输出电流大于*A，且保持500ms
	if((SADC.Iout > MAX_OCP_VAL)&&(DF.SMFlag  ==Run))
	{
		//条件保持计时
		OCPCnt++;
		//条件保持50ms，则认为过流发生
		if(OCPCnt > 10)
		{
			//计数器清0
			OCPCnt  = 0;
			//关闭PWM
			DF.PWMENFlag=0;
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //关闭
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //关闭	
			//故障标志位
			setRegBits(DF.ErrFlag,F_SW_IOUT_OCP);
			//跳转至故障状态
			DF.SMFlag  =Err;
		}
	}
	else
		//计数器清0
		OCPCnt  = 0;

	//输出过流后恢复
	//当发生输出软件过流保护，关机后等待4S后清楚故障信息，进入等待状态等待重启
	if(getRegBits(DF.ErrFlag,F_SW_IOUT_OCP))
	{
		//等待故障清楚计数器累加
		RSCnt++;
		//等待2S
		if(RSCnt > 400)
		{
			//计数器清零
			RSCnt=0;
			//过流重启计数器累加
			RSNum++;
			//过流重启只重启10次，10次后不重启（严重故障）
			if(RSNum > 10 )
			{
				//确保不清除故障，不重启
				RSNum =11;
				//关闭PWM
				DF.PWMENFlag=0;
				HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //关闭
				HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //关闭		
			}
			else
			{
			 //清除过流保护故障标志位
				clrRegBits(DF.ErrFlag,F_SW_IOUT_OCP);
			}
		}
	}
}

/*
** ===================================================================
**     Funtion Name :void SwOVP(void)
**     Description :软件输出过压保护，不重启
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_VOUT_OVP_VAL    3012//50V过压保护	（50/68）*Q12
void VoutSwOVP(void)
{
	//过压保护判据保持计数器定义
	static  uint16_t  OVPCnt=0;

	//当输出电压大于50V，且保持100ms
	if (SADC.Vout > MAX_VOUT_OVP_VAL)
	{
		//条件保持计时
		OVPCnt++;
		//条件保持10ms
		if(OVPCnt > 2)
		{
			//计时器清零
			OVPCnt=0;
			//关闭PWM
			DF.PWMENFlag=0;
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //关闭
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //关闭		
			//故障标志位
			setRegBits(DF.ErrFlag,F_SW_VOUT_OVP);
			//跳转至故障状态
			DF.SMFlag  =Err;
		}
	}
	else
		OVPCnt = 0;
}

/*
** ===================================================================
**     Funtion Name :void VinSwUVP(void)
**     Description :输入软件欠压保护，低压输入保护,可恢复
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MIN_UVP_VAL    686//11.4V欠压保护 （11.4/68 ）*Q12
#define MIN_UVP_VAL_RE  795//13.2V欠压保护恢复 （13.2/68）*Q12
void VinSwUVP(void)
{
	//过压保护判据保持计数器定义
	static  uint16_t  UVPCnt=0;
	static  uint16_t	RSCnt=0;

	//当输出电流小于于11.4V，且保持200ms
	if ((SADC.Vin < MIN_UVP_VAL) && (DF.SMFlag != Init ))
	{
		//条件保持计时
		UVPCnt++;
		//条件保持10ms
		if(UVPCnt > 2)
		{
			//计时器清零
			UVPCnt=0;
			RSCnt=0;
			//关闭PWM
			DF.PWMENFlag=0;
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //关闭
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //关闭		
			//故障标志位
			setRegBits(DF.ErrFlag,F_SW_VIN_UVP);
			//跳转至故障状态
			DF.SMFlag  =Err;
		}
	}
	else
		UVPCnt = 0;
	
	//输入欠压保护恢复
	//当发生输入欠压保护，等待输入电压恢复至正常水平后清楚故障标志位，重启
	if(getRegBits(DF.ErrFlag,F_SW_VIN_UVP))
	{
		if(SADC.Vin > MIN_UVP_VAL_RE) 
		{
			//等待故障清楚计数器累加
			RSCnt++;
			//等待1S
			if(RSCnt > 200)
			{
				RSCnt=0;
				UVPCnt=0;
				//清楚故障标志位
				clrRegBits(DF.ErrFlag,F_SW_VIN_UVP);
			}	
		}
		else	
			RSCnt=0;	
	}
	else
		RSCnt=0;
}

/*
** ===================================================================
**     Funtion Name :void VinSwOVP(void)
**     Description :软件输入过压保护，不重启
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_VIN_OVP_VAL    3012//50V过压保护	（50/68）*Q12
void VinSwOVP(void)
{
	//过压保护判据保持计数器定义
	static  uint16_t  OVPCnt=0;

	//当输出电压大于50V，且保持100ms
	if (SADC.Vin > MAX_VIN_OVP_VAL )
	{
		//条件保持计时
		OVPCnt++;
		//条件保持10ms
		if(OVPCnt > 2)
		{
			//计时器清零
			OVPCnt=0;
			//关闭PWM
			DF.PWMENFlag=0;
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //关闭
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //关闭		
			//故障标志位
			setRegBits(DF.ErrFlag,F_SW_VIN_OVP);
			//跳转至故障状态
			DF.SMFlag  =Err;
		}
	}
	else
		OVPCnt = 0;
}

/** ===================================================================
**     Funtion Name :void LEDShow(void)
**     Description :  LED显示函数
**     初始化与等待启动状态，红黄绿全亮
**     启动状态，黄绿亮
**     运行状态，绿灯亮
**     故障状态，红灯亮
**     Parameters  :
**     Returns     :
** ===================================================================*/
//输出状态灯宏定义
 #define SET_LED_G()	HAL_GPIO_WritePin(GPIOB, LED_G_Pin,GPIO_PIN_SET)//绿灯亮
 #define SET_LED_Y()	HAL_GPIO_WritePin(GPIOB, LED_Y_Pin,GPIO_PIN_SET)//绿灯亮
 #define SET_LED_R()	HAL_GPIO_WritePin(GPIOB, LED_R_Pin,GPIO_PIN_SET)//绿灯亮
 #define CLR_LED_G()	HAL_GPIO_WritePin(GPIOB, LED_G_Pin,GPIO_PIN_RESET)//绿灯灭
 #define CLR_LED_Y()	HAL_GPIO_WritePin(GPIOB, LED_Y_Pin,GPIO_PIN_RESET)//黄灯灭
 #define CLR_LED_R()	HAL_GPIO_WritePin(GPIOB, LED_R_Pin,GPIO_PIN_RESET)//红灯灭
void LEDShow(void)
{
	switch(DF.SMFlag)
	{
		//初始化状态，红黄绿全亮
		case  Init :
		{
			SET_LED_G();
			SET_LED_Y();
			SET_LED_R();
			break;
		}
		//等待状态，红黄绿全亮
		case  Wait :
		{
			SET_LED_G();
			SET_LED_Y();
			SET_LED_R();
			break;
		}
		//软启动状态，黄绿亮
		case  Rise :
		{
			SET_LED_G();
			SET_LED_Y();
			CLR_LED_R();
			break;
		}
		//运行状态，绿灯亮
		case  Run :
		{
			SET_LED_G();
			CLR_LED_Y();
			CLR_LED_R();
			break;
		}
		//故障状态，红灯亮
		case  Err :
		{
			CLR_LED_G();
			CLR_LED_Y();
			SET_LED_R();
			break;
		}
	}
}





/** ===================================================================
**     Funtion Name :void BBMode(void)
**     Description :运行模式判断
** 		 BUCK模式：输出参考电压<0.8倍输入电压
** 		 BOOST模式：输出参考电压>1.2倍输入电压
**		 MIX模式：1.15倍输入电压>输出参考电压>0.85倍输入电压
**		 当进入MIX（buck-boost）模式后，退出到BUCK或者BOOST时需要滞缓，防止在临界点来回振荡
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

PWM 

TA1 -> タゑ 48%
TB1 -> タゑ 48%
TA2 -> タゑ 48%
TB2 -> タゑ 48%

TA1籔TB1猧が干TA2 籔 TB2 猧が干
-> 讽TA1ゑ48%TB1ゑ52%
-> 讽TA2 ゑ48%TB2ゑ52%

TA1 籔 TA2 猧璓 TB1 籔 TB2 猧璓
TA1 籔 TB1 猧が干TA2 籔 TB2 猧が干

1. TA1 籔 TB1 琌が干
2. TA2 籔 TB2 琌が干 
3. TA1 籔 TB1 跋丁 2%50%ぃ2%
4. TA2 籔 TB2 跋丁 2%50%ぃ2%
5. ﹍繵瞯 100KHz
6. duty  50%

;---
龄弧

1. PA6 -> T1, TB1, TA2, TB2 秸蔼繵瞯
          -> 125K

2. PA7 -> T1, TB1, TA2, TB2 秸繵瞯
          -> 77K

3. PB4 -> TA1, TB1 糤跋丁
           ->  2% 50-48 = 2%

4. PB5 -> TA1, TB1 搭ぶ跋丁
           ->  10% 50-40 = 10%

5. PB6 -> TA2, TB2 秸蔼ゑ
            -> 45%

6. PB7  -> TA2, TB2 ゑ
            -> 程5%
7. PB9  -> ち传家Α
            -> open loop / close loop

龄弧
1. KEY1 increases frequency by 1% each time it is pressed
2. KEY2 decreases frequency by 1% each time it is pressed
3. KEY3 increases dead time by 1% each time it is pressed
4. KEY4 decreases dead time by 1% each time it is pressed
5. KEY5 increases duty cycle by 1% each time it is pressed
6. KEY6 decreases duty cycle by 1% each time it is pressed
7. KEY7 switches mode / open-loop or closed-loop

**     Parameters  :
**     Returns     :
** ===================================================================*/

// ﹚竡北家Α
#define MODE_OPEN 0
#define MODE_CLOSE 1

// 程籔程繵瞯虫Hz
#define FREQ_MIN 70000.0f    // 70 kHz
#define FREQ_MAX 130000.0f   // 130 kHz
#define FREQ_STEP_PERCENT 0.1f  // –Ω龄秸俱κだゑ0.1%

// 跋丁把计虫0.1%–˙秸俱 0.1%
#define DEADTIME_MIN_PX1000 0     // 0%
#define DEADTIME_MAX_PX1000 20    // 2%
#define DEADTIME_STEP_PX1000 1    // 0.1%

// ゑ把计虫0.1%–˙秸俱 0.1%
#define DUTY_MIN_PX10 50    // 5.0%
#define DUTY_MAX_PX10 500   // 50.0%
#define DUTY_STEP_PX10 1     // 0.1%

// 安砞 CNTR_MIN ㎝ CNTR_MAX ﹚竡
#define CNTR_MIN 1000
#define CNTR_MAX 16000


// Ы跑秖
extern HRTIM_HandleTypeDef hhrtim1;
extern HRTIM_TimeBaseCfgTypeDef pGlobalTimeBaseCfg;

volatile float currentPWMFreq = 100000.0f;        // ﹍繵瞯 100 kHz
volatile uint8_t gCurrentDeadTimePercent = 2;     // ﹍跋丁 2%

//volatile uint32_t OPN_DEADTIMEPX1000 = 20;         // ﹍跋丁 2%
//volatile int32_t vOPNDutyPercentageX10 = 250;      // ﹍ゑ 25.0%
//volatile uint8_t currentMode = MODE_OPEN;           // ﹍家Α秨吏

// ﹍てゑ
volatile uint8_t gCurrentDutyPercent_TA1_TB1 = 48; // TA1/TB1 ﹍ゑ 48%
volatile uint8_t gCurrentDutyPercent_TA2_TB2 = 48; // TA2/TB2 ﹍ゑ 48%

// 膀皌竚挡篶
HRTIM_TimeBaseCfgTypeDef pGlobalTimeBaseCfg;

// 讽玡 PLL 繵瞯
volatile uint32_t currentPLLFreq = 160000000;      // 160 MHz沮 fHRCK 砞﹚

void Button_Task(void)
{
	// KEY1/PA6 : T1, TB1, TA2, TB2 秸蔼繵瞯
	if (Key_Scan(KEY1_INC_Freq_GPIO_Port, KEY1_INC_Freq_Pin) == KEY_ON)
	{
		if (currentPWMFreq < FREQ_MAX)
		{
			currentPWMFreq *= (1.0f + (FREQ_STEP_PERCENT / 100.0f)); // 糤 0.1%
			if (currentPWMFreq > FREQ_MAX)
				currentPWMFreq = FREQ_MAX;
	
			// 穝 HRTIM 繵瞯
			if (SetPWMFrequency((uint32_t)currentPWMFreq) != HAL_OK) {
				// 矪瞶岿粇
				Error_Handler();
			}
			HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin);
		}
	}
	
	// KEY2/PA7 : T1, TB1, TA2, TB2 秸繵瞯
	if (Key_Scan(KEY2_DEC_Freq_GPIO_Port, KEY2_DEC_Freq_Pin) == KEY_ON)
	{
		if (currentPWMFreq > FREQ_MIN)
		{
			currentPWMFreq *= (1.0f - (FREQ_STEP_PERCENT / 100.0f)); // 搭ぶ 0.1%
			if (currentPWMFreq < FREQ_MIN)
				currentPWMFreq = FREQ_MIN;
	
			// 穝 HRTIM 繵瞯
			if (SetPWMFrequency((uint32_t)currentPWMFreq) != HAL_OK) {
				// 矪瞶岿粇
				Error_Handler();
			}
			HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin);
		}
	}

    // KEY3/PB4: 糤 TA1/TB1 跋丁
    if (Key_Scan(KEY3_INC_DT_GPIO_Port, KEY3_INC_DT_Pin) == KEY_ON)
    {
		if (gCurrentDeadTimePercent < 50)
		{
			gCurrentDeadTimePercent += 1; // 糤 1%
	
			if (gCurrentDeadTimePercent > 50)
				gCurrentDeadTimePercent = 50;
	
				// も笆秸俱跋丁
				SetDeadTimeManual(gCurrentDeadTimePercent);
	
				HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); 
		}
    }

	// KEY4/PB5: 搭ぶ TA1/TB1 跋丁
    if (Key_Scan(KEY4_DEC_DT_GPIO_Port, KEY4_DEC_DT_Pin) == KEY_ON)
    {
		if (gCurrentDeadTimePercent > 10)
		{
			gCurrentDeadTimePercent -= 1; // 搭ぶ 1%
	
			if (gCurrentDeadTimePercent < 10)
				gCurrentDeadTimePercent = 10;
	
					// も笆秸俱跋丁
					SetDeadTimeManual(gCurrentDeadTimePercent);
					HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); 
			}
    }

	// KEY5/PB6: 秸蔼 TA2/TB2 ゑ
    if (Key_Scan(KEY5_INC_DUTY_GPIO_Port, KEY5_INC_DUTY_Pin) == KEY_ON)
    {
		if (gCurrentDutyPercent_TA2_TB2 < 45)
		{
			gCurrentDutyPercent_TA2_TB2 += 1; // 糤 1%
			if (gCurrentDutyPercent_TA2_TB2 > 45)
				gCurrentDutyPercent_TA2_TB2 = 45;
	
				// 穝ゑ
				if (SetDutyCycle_TA2_TB2(gCurrentDutyPercent_TA2_TB2) != HAL_OK) {
					// 矪瞶岿粇
					Error_Handler();
				}
					HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); 
			}
    }
	// KEY6/PB7: 秸 TA2/TB2 ゑ
    if (Key_Scan(KEY6_DEC_DUTY_GPIO_Port, KEY6_DEC_DUTY_Pin) == KEY_ON)
    {
			if (gCurrentDutyPercent_TA2_TB2 > 5)
			{
				gCurrentDutyPercent_TA2_TB2 -= 1; // 搭ぶ 1%
				if (gCurrentDutyPercent_TA2_TB2 < 5)
					gCurrentDutyPercent_TA2_TB2 = 5;
	
				// 穝ゑ
				if (SetDutyCycle_TA2_TB2(gCurrentDutyPercent_TA2_TB2) != HAL_OK) {
					// 矪瞶岿粇
					Error_Handler();
				}
					HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); // 皗脅 LED
			}
    }

    if (Key_Scan(KEY7_SWITCH_MODE_GPIO_Port, KEY7_SWITCH_MODE_Pin) == KEY_ON)
    {
		Mode_Switch(); // ち传家Α
    }
	// 穝 OLED 陪ボ
		UpdateDisplay();

}


/** ===================================================================
**     Funtion Name : Set_HRTIM_CompareVale
**     Description : 

ヘ玡琌ㄏノ 100MHz秈︽埃繵 timebase = 16000 = 100KHz
秅戳い丁翴 50% = 8000

1.  duty ぃ禬筁 50%
2.  Frequency  130KHz  70KHz
3.  dead time2%

**     Parameters  :req_tim_freq - 叫― PWM 繵瞯Hz
**     Returns     :HAL_StatusTypeDef - Θ HAL_OKア毖 HAL_ERROR
** ===================================================================*/
HAL_StatusTypeDef SetPWMFrequency(uint32_t req_tim_freq) {
    uint32_t prescaler_value;
    uint32_t fHRCK;
    uint32_t period;

    // ﹚竡繵瞯絛瞅
    if (req_tim_freq < FREQ_MIN || req_tim_freq > FREQ_MAX) {
        return HAL_ERROR; // 繵瞯ぃ絛瞅ず
    }

    if (req_tim_freq >= 100000) {
        // ㄏノ prescaler = MUL16
        prescaler_value = 16;
        fHRCK = 100000000UL * prescaler_value; // 100MHz * 16 = 1.6GHz
    } else {
        // ㄏノ prescaler = MUL8
        prescaler_value = 8;
        fHRCK = 100000000UL * prescaler_value; // 100MHz * 8 = 800MHz (沮龟悔牧皌竚秸俱)
    }

    // 璸衡穝 Period
    period = fHRCK / req_tim_freq;

    // 喷靡 Period 琌す砛絛瞅ず
    if (period < CNTR_MIN || period > CNTR_MAX) {
        return HAL_ERROR;
    }

    // 穝Ы膀皌竚 Period ㎝ PrescalerRatio
    pGlobalTimeBaseCfg.Period = period;
    if (prescaler_value == 16) {
        pGlobalTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL16;
    } else if (prescaler_value == 8) {
        pGlobalTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL8;
    } else {
        return HAL_ERROR; // ぃや箇だ繵竟
    }

    // 砞﹚ Timer A 膀
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pGlobalTimeBaseCfg) != HAL_OK) {
        return HAL_ERROR;
    }

    // 砞﹚ Timer B 膀
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pGlobalTimeBaseCfg) != HAL_OK) {
        return HAL_ERROR;
    }

    // 砞﹚ゑ耕虫じ 2 い翴50% ゑ
    HRTIM_CompareCfgTypeDef pCompareCfg = {0};
    pCompareCfg.CompareValue = (period / 2) - 1;
    pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    pCompareCfg.AutoDelayedTimeout = 0x0000;

    // 砞﹚ Timer A ゑ耕虫じ 2
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK) {
        return HAL_ERROR;
    }

    // 砞﹚ Timer B ゑ耕虫じ 2
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK) {
        return HAL_ERROR;
    }

    // 硁ン竚 HRTIM  Timer A ㎝ Timer B莱ノ穝砞﹚
    if (HAL_HRTIM_SoftwareReset(&hhrtim1, HRTIM_TIMERRESET_TIMER_A | HRTIM_TIMERRESET_TIMER_B) != HAL_OK) {
        return HAL_ERROR;
    }

    // 穝Ы繵瞯跑秖
    currentPWMFreq = req_tim_freq;
    currentPLLFreq = fHRCK;

    return HAL_OK;
}


/**
  * @brief  も笆砞﹚跋丁
  * @param  dead_time_percent - 穝跋丁
  * @retval None
  */
void SetDeadTimeManual(uint8_t dead_time_percent)
{
    // 絋玂跋丁す砛絛瞅ず
    if (dead_time_percent < 2 || dead_time_percent > 50) {
        return;
    }

    uint32_t period = pGlobalTimeBaseCfg.Period;
    uint32_t dead_time_ticks = (period * dead_time_percent) / 100;

    // 秸俱 Timer A Compare Unit 2 (い翴) ノ TA1  ResetSource
    HRTIM_CompareCfgTypeDef compareConfigA = {0};
    compareConfigA.CompareValue = dead_time_ticks;
    compareConfigA.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    compareConfigA.AutoDelayedTimeout = 0x0000;

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &compareConfigA) != HAL_OK) {
        Error_Handler();
    }

    // 秸俱 Timer B Compare Unit 2 (い翴) ノ TB1  ResetSource
    HRTIM_CompareCfgTypeDef compareConfigB = {0};
    compareConfigB.CompareValue = period - dead_time_ticks;
    compareConfigB.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    compareConfigB.AutoDelayedTimeout = 0x0000;

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &compareConfigB) != HAL_OK) {
        Error_Handler();
    }

    // 穝币笆璸竟莱ノ穝跋丁
    if (HAL_HRTIM_SoftwareReset(&hhrtim1, HRTIM_TIMERRESET_TIMER_A | HRTIM_TIMERRESET_TIMER_B) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B) != HAL_OK) {
        Error_Handler();
    }
}


/**
  * @brief  砞﹚ HRTIM  PWM ゑ TA1/TB1
  * @param  duty_percent - 穝ゑκだゑ
  * @retval HAL_StatusTypeDef - Θ HAL_OKア毖 HAL_ERROR
  */
HAL_StatusTypeDef SetDutyCycle_TA1_TB1(uint8_t duty_percent)
{
    if (duty_percent < 5 || duty_percent > 95)
    {
        return HAL_ERROR;
    }

    uint32_t period = pGlobalTimeBaseCfg.Period;
    uint32_t compare_value = (period * duty_percent) / 100;

    // 穝 Timer A Compare Unit 2 (い翴) 癸莱 TA1  ResetSource
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
  * @brief  砞﹚ HRTIM  PWM ゑ TA2/TB2
  * @param  duty_percent - 穝ゑκだゑ
  * @retval HAL_StatusTypeDef - Θ HAL_OKア毖 HAL_ERROR
  */
HAL_StatusTypeDef SetDutyCycle_TA2_TB2(uint8_t duty_percent)
{
    if (duty_percent < 5 || duty_percent > 45)
    {
        return HAL_ERROR;
    }

    uint32_t period = pGlobalTimeBaseCfg.Period;
    uint32_t compare_value = (period * duty_percent) / 100;

    // 穝 Timer A Compare Unit 2 (い翴) 癸莱 TA2  ResetSource
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
	OLED_ShowStr(68, 2, ".", 2);
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
**     Function Name : void OLEDShow(void)
**     Description : OLED display routine		 
**     Display operating modes - BUCK MODE, BOOST MODE, MIX MODE
**     Display states: IDLE, RISING, RUNNING, ERROR
**     Display output voltage: converted values
**     Display output current: converted values
**     Parameters  :
**     Returns     :
** ===================================================================*/
void OLEDShow(void)
{
	u8 Vtemp[4] = {0, 0, 0, 0};
	u8 Itemp[4] = {0, 0, 0, 0};
	uint32_t VoutT = 0, IoutT = 0;
	//uint32_t VinT = 0, IinT = 0, VadjT = 0;
	static uint16_t BBFlagTemp = 10, SMFlagTemp = 10;
	
	// Calculate the output voltage and current, converted to 100x (for display adjustment) 
	VoutT = SADC.VoutAvg * 6800 >> 12;
	IoutT = (SADC.IoutAvg - 2048) * 2200 >> 12;
	//VinT = SADC.VinAvg * 6800 >> 12;
	//IinT = (SADC.IinAvg - 2048) * 2200 >> 12;
	//VadjT = CtrValue.Voref * 6800 >> 12;
	
	// Convert calculated output voltage and current to display format
	// Output voltage
	Vtemp[0] = (u8)(VoutT / 1000);
	Vtemp[1] = (u8)((VoutT - (uint8_t)Vtemp[0] * 1000) / 100);
	Vtemp[2] = (u8)((VoutT - (uint16_t)Vtemp[0] * 1000 - (uint16_t)Vtemp[1] * 100) / 10);
	Vtemp[3] = (u8)(VoutT - (uint16_t)Vtemp[0] * 1000 - (uint16_t)Vtemp[1] * 100 - (uint16_t)Vtemp[2] * 10);	
	// Input voltage
/*	Vtemp[0] = (u8)(VinT / 1000);
	Vtemp[1] = (u8)((VinT - (uint8_t)Vtemp[0] * 1000) / 100);
	Vtemp[2] = (u8)((VinT - (uint16_t)Vtemp[0] * 1000 - (uint16_t)Vtemp[1] * 100) / 10);
	Vtemp[3] = (u8)(VinT - (uint16_t)Vtemp[0] * 1000 - (uint16_t)Vtemp[1] * 100 - (uint16_t)Vtemp[2] * 10); */
	// Output current
	Itemp[0] = (u8)(IoutT / 1000);
	Itemp[1] = (u8)((IoutT - (uint8_t)Itemp[0] * 1000) / 100);
	Itemp[2] = (u8)((IoutT - (uint16_t)Itemp[0] * 1000 - (uint16_t)Itemp[1] * 100) / 10);
	Itemp[3] = (u8)(IoutT - (uint16_t)Itemp[0] * 1000 - (uint16_t)Itemp[1] * 100 - (uint16_t)Itemp[2] * 10);
	// Input current
/*	Itemp[0] = (u8)(IinT / 1000);
	Itemp[1] = (u8)((IinT - (uint8_t)Itemp[0] * 1000) / 100);
	Itemp[2] = (u8)((IinT - (uint16_t)Itemp[0] * 1000 - (uint16_t)Itemp[1] * 100) / 10);
	Itemp[3] = (u8)(IinT - (uint16_t)Itemp[0] * 1000 - (uint16_t)Itemp[1] * 100 - (uint16_t)Itemp[2] * 10); */
	// Reference voltage
/*	Vtemp[0] = (u8)(VadjT / 1000);
	Vtemp[1] = (u8)((VadjT - (uint8_t)Vtemp[0] * 1000) / 100);
	Vtemp[2] = (u8)((VadjT - (uint16_t)Vtemp[0] * 1000 - (uint16_t)Vtemp[1] * 100) / 10);
	Vtemp[3] = (u8)(VadjT - (uint16_t)Vtemp[0] * 1000 - (uint16_t)Vtemp[1] * 100 - (uint16_t)Vtemp[2] * 10); */
	
	// If the operating mode has changed, update display
	if(BBFlagTemp != DF.BBFlag)
	{
		// Save current flag status
		BBFlagTemp = DF.BBFlag;
		// Display operating mode
		switch(DF.BBFlag)
		{
			// NA
			case NA :		
			{
				OLED_ShowStr(55, 0, "Open Loop", 2);
				break;
			}
			// BUCK mode
			case Buck :		
			{
				OLED_ShowStr(25, 0, "MODE:BUCK ", 2);
				break;
			}
			// Boost mode
			case Boost :		
			{
				OLED_ShowStr(25, 0, "MODE:BOOST", 2);
				break;
			}
			// Mix mode
			case Mix :		
			{
				OLED_ShowStr(25, 0, "MODE:MIX ", 2);
				break;
			}
		}
	}
	
	// If the state has changed, update display
	if(SMFlagTemp != DF.SMFlag)
	{	
		SMFlagTemp = DF.SMFlag;
		// Display state
		switch(DF.SMFlag)
		{
			// Initialization state
			case Init :
			{
				//OLED_ShowStr(55, 2, "Init  ", 2);
				break;
			}
			// Waiting state
			case Wait :
			{
				//OLED_ShowStr(55, 2, "Waiting", 2);
				break;
			}
			// Rising state
			case Rise :
			{
				//OLED_ShowStr(55, 2, "Rising", 2);
				break;
			}
			// Running state
			case Run :
			{
				//OLED_ShowStr(55, 2, "Running", 2);
				break;
			}
			// Error state
			case Err :
			{
				//OLED_ShowStr(55, 2, "Error  ", 2);
				break;
			}
		}	
	}
	
	// Display voltage and current values
	OLEDShowData(50, 2, Vtemp[0]);
	OLEDShowData(60, 2, Vtemp[1]);
	OLEDShowData(75, 2, Vtemp[2]);
	OLEDShowData(85, 2, Vtemp[3]);

	OLEDShowData(50, 4, Vtemp[0]);
	OLEDShowData(60, 4, Vtemp[1]);
	OLEDShowData(75, 4, Vtemp[2]);
	OLEDShowData(85, 4, Vtemp[3]);

	OLEDShowData(50, 6, Itemp[0]);
	OLEDShowData(60, 6, Itemp[1]);
	OLEDShowData(75, 6, Itemp[2]);
	OLEDShowData(85, 6, Itemp[3]);
}





#define MODE_OPEN_LOOP 0
#define MODE_CLOSED_LOOP 1
// 家Α跑秖纐粄秨吏家Α
volatile uint8_t currentMode = MODE_OPEN_LOOP;

/**
  * @brief  ち传家Αㄧ计
  * @retval None
  */
void Mode_Switch(void)
{
    if (currentMode == MODE_OPEN_LOOP)
    {
        currentMode = MODE_CLOSED_LOOP;
        // ﹍て繵瞯 100 kHz
        currentPWMFreq = 100000.0f;
        if (SetPWMFrequency((uint32_t)currentPWMFreq) != HAL_OK)
        {
            Error_Handler();
        }

        // 陪ボ家Α跑
        OLED_ShowStr(60, 0, "Closed", 2);
    }
    else
    {
        currentMode = MODE_OPEN_LOOP;
        // 陪ボ家Α跑
        OLED_ShowStr(60, 0, "Open", 2);
    }

    HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); // 皗脅 LED ボ家Α跑
}




/**
  * @brief  穝 OLED 陪ボㄧ计
  * @retval None
  */
void UpdateDisplay(void)
{
    // 沮家Α陪ボぃ獺
    if (currentMode == MODE_CLOSED_LOOP)
    {
        // 超吏家Α陪ボ ADC 筿溃锣传繵瞯

        // 絋玂 ADC 竒蹦妓
        ADCSample();

        // 盢 ADC 筿溃锣传 0-3.3V
        float adc_voltage = (SADC.VinAvg / 4095.0f) * 3.3f;

        // 璸衡癸莱繵瞯
        // い丁 1.65V 癸莱 100 kHz
        // 砞﹚繵瞯絛瞅 50 kHz  150 kHz
        float frequency = 100000.0f + ((adc_voltage - 1.65f) / 1.65f) * 50000.0f; // ∮50 kHz

        // 繵瞯絛瞅
        if (frequency < 50000.0f)
            frequency = 50000.0f;
        if (frequency > 150000.0f)
            frequency = 150000.0f;

        // 穝 PWM 繵瞯
        currentPWMFreq = frequency;
        //if (SetPWMFrequency((uint32_t)currentPWMFreq) != HAL_OK)
        //{
        //    Error_Handler();
        //}

        // 陪ボ繵瞯
        unsigned char freqStr[10];
        //sprintf(freqStr, "%.1fKHz", frequency / 1000.0f);
        OLED_ShowStr(60, 6, "     ", 2); // 睲埃Τ陪ボ
        OLED_ShowStr(60, 6, freqStr, 2);
    }
    else
    {
        // 秨吏家Α陪ボ讽玡も笆砞竚繵瞯

        // 陪ボ繵瞯
        unsigned char freqStr[10];
        //sprintf(freqStr, "%.1fKHz", currentPWMFreq / 1000.0f);
        OLED_ShowStr(60, 6, "     ", 2); // 睲埃Τ陪ボ
        OLED_ShowStr(60, 6, freqStr, 2);
    }

    // 陪ボ ADC 筿溃
    // 盢 ADC 筿溃锣传 0-3.3V
    float adc_voltage_display = (SADC.VinAvg / 4095.0f) * 3.3f;

    // 盢筿溃锣传陪ボΑ (mV)
    uint8_t Vtemp[4] = {0};
    uint32_t Vdisplay = (uint32_t)(adc_voltage_display * 1000.0f); // 锣传 mV

    Vtemp[0] = (uint8_t)(Vdisplay / 1000);
    Vtemp[1] = (uint8_t)((Vdisplay % 1000) / 100);
    Vtemp[2] = (uint8_t)((Vdisplay % 100) / 10);
    Vtemp[3] = (uint8_t)(Vdisplay % 10);

    // 陪ボ ADC 筿溃
    OLEDShowData(50, 2, Vtemp[0]);
    OLEDShowData(60, 2, Vtemp[1]);
    OLEDShowData(75, 2, Vtemp[2]);
    OLEDShowData(85, 2, Vtemp[3]);

    // 匡陪ボㄤ计沮 Iout 单
}











