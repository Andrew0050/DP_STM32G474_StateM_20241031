/* USER CODE BEGIN Header */
	
/* USER CODE END Header */

#include "function.h"//���ܺ���ͷ�ļ�
#include "CtlLoop.h"//���ƻ�·ͷ�ļ�

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
**     Description :   �ȴ�״̬�����ȴ�1S���޹���������
**     Parameters  :
**     Returns     :
** ===================================================================*/
void StateMWait(void)
{
	//����������
	static uint16_t CntS = 0;
	
	//��PWM
	DF.PWMENFlag=0;
	//�������ۼ�
	CntS ++;
	//�ȴ�1S���޹������,�а������£����������������״̬
	if(CntS>200)
	{
		CntS=200;
		HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //����PWM�����PWM��ʱ��
		if((DF.ErrFlag==F_NOERR)&&(DF.KeyFlag1==1))
		{
			//��������0
			CntS=0;
			//״̬��־λ��ת���ȴ�״̬
			DF.SMFlag  = Rise;
			//��������״̬��ת����ʼ��״̬
			STState = SSInit;
		}
	}
}
/*
** ===================================================================
**     Funtion Name : void StateMRise(void)
**     Description :�����׶�
**     ������ʼ��
**     �����ȴ�
**     ��ʼ����
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_SSCNT       20//�ȴ�100ms
void StateMRise(void)
{
	//��ʱ��
	static  uint16_t  Cnt = 0;
	//���ռ�ձ����Ƽ�����
	static  uint16_t	BUCKMaxDutyCnt=0,BoostMaxDutyCnt=0;

	//�ж�����״̬
	switch(STState)
	{
		//��ʼ��״̬
		case    SSInit:
		{
			//�ر�PWM
			DF.PWMENFlag=0;
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //�ر�
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //�ر�				
			//�����н���������ռ�ձ�����������Сռ�ձȿ�ʼ����
			CtrValue.BUCKMaxDuty  = MIN_BUKC_DUTY;
			CtrValue.BoostMaxDuty = MIN_BOOST_DUTY;
			//��·���������ʼ��
			VErr0=0;
			VErr1=0;
			VErr2=0;
			u0 = 0;
			u1 = 0;
			//��ת�������ȴ�״̬
			STState = SSWait;

			break;
		}
		//�ȴ�������״̬
		case    SSWait:
		{
			//�������ۼ�
			Cnt++;
			//�ȴ�100ms
			if(Cnt> MAX_SSCNT)
			{
				//��������0
				Cnt = 0;
				//��������ռ�ձ�
				CtrValue.BuckDuty = MIN_BUKC_DUTY;
				CtrValue.BUCKMaxDuty= MIN_BUKC_DUTY;
				CtrValue.BoostDuty = MIN_BOOST_DUTY;
				CtrValue.BoostMaxDuty = MIN_BOOST_DUTY;
				//��·���������ʼ��
				VErr0=0;
				VErr1=0;
				VErr2=0;
				u0 = 0;
				u1 = 0;
				//CtrValue.Voref����ο���ѹ��һ�뿪ʼ������������壬Ȼ��������
				CtrValue.Voref  = CtrValue.Voref >>1;			
				STState = SSRun;	//��ת������״̬		
			}
			break;
		}
		//������״̬
		case    SSRun:
		{
			if(DF.PWMENFlag==0)//��ʽ����ǰ��·������0
			{
				//��·���������ʼ��
				VErr0=0;
				VErr1=0;
				VErr2=0;
				u0 = 0;
				u1 = 0;	
				HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //����PWM�����PWM��ʱ��
				HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //����PWM�����PWM��ʱ��					
			}
			//������־λ��λ
			DF.PWMENFlag=1;
			//���ռ�ձ�����������
			BUCKMaxDutyCnt++;
			BoostMaxDutyCnt++;
			//���ռ�ձ������ۼ�
			CtrValue.BUCKMaxDuty = CtrValue.BUCKMaxDuty + BUCKMaxDutyCnt*5;
			CtrValue.BoostMaxDuty = CtrValue.BoostMaxDuty + BoostMaxDutyCnt*5;
			//�ۼӵ����ֵ
			if(CtrValue.BUCKMaxDuty > MAX_BUCK_DUTY)
				CtrValue.BUCKMaxDuty  = MAX_BUCK_DUTY ;
			if(CtrValue.BoostMaxDuty > MAX_BOOST_DUTY)
				CtrValue.BoostMaxDuty  = MAX_BOOST_DUTY ;
			
			if((CtrValue.BUCKMaxDuty==MAX_BUCK_DUTY)&&(CtrValue.BoostMaxDuty==MAX_BOOST_DUTY))			
			{
				//״̬����ת������״̬
				DF.SMFlag  = Run;
				//��������״̬��ת����ʼ��״̬
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
**     Description :�������У������������ж�������
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
**     Description :����״̬
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
void StateMErr(void)
{
	//�ر�PWM
	DF.PWMENFlag=0;
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //�ر�
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //�ر�		
	//������������ת���ȴ���������
	if(DF.ErrFlag==F_NOERR)
			DF.SMFlag  = Wait;
}

/** ===================================================================
**     Funtion Name :void ValInit(void)
**     Description :   ��ز�����ʼ������
**     Parameters  :
**     Returns     :
** ===================================================================*/
void ValInit(void)
{
	//�ر�PWM
	DF.PWMENFlag=0;
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //�ر�
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //�ر�		
	//������ϱ�־λ
	DF.ErrFlag=0;
	//��ʼ����ѹ�ο���
	CtrValue.Voref=0;
	//����ռ�ձ�
	CtrValue.BuckDuty = MIN_BUKC_DUTY;
	CtrValue.BUCKMaxDuty= MIN_BUKC_DUTY;
	CtrValue.BoostDuty = MIN_BOOST_DUTY;
	CtrValue.BoostMaxDuty = MIN_BOOST_DUTY;	
	//��·���������ʼ��
	VErr0=0;
	VErr1=0;
	VErr2=0;
	u0 = 0;
	u1 = 0;
}

/** ===================================================================
**     Funtion Name :void VrefGet(void)
**     Description :   �ӻ���������ֵ��ȡ�����ѹ�ο�ֵ������������λ����CtrValue.Voref�任������ѹ�ο�ֵ�仯ʱ����������
**     Parameters  :
**     Returns     :
** ===================================================================*/
#define MAX_VREF    2921//������ο���ѹ48V  0.5V������   48.5V/68V*Q12
#define MIN_VREF    271//��͵�ѹ�ο�ֵ5V   0.5V������   4.5V/68V*2^Q12
#define VREF_K      10//������ݼ�����
void VrefGet(void)
{
	//��ѹ�ο�ֵ�м����
	int32_t VTemp = 0;	
	//����ƽ������м����
	static int32_t VadjSum = 0;

	//��ȡADC����ֵ-��������λ���ϵĵ�ѹ
	SADC.Vadj = HAL_ADC_GetValue(&hadc1);
	//�Բ���ֵ������ƽ��
	VadjSum = VadjSum + SADC.Vadj -(VadjSum>>8);
	SADC.VadjAvg = VadjSum>>8;
	
	//�ο���ѹ = MIN_VREF+��������������ֵ��MIN_VREFΪ��������ѹ��
	VTemp = MIN_VREF + SADC.Vadj;
	
	//�������������ݼ���ѹ�ο�ֵ
	if( VTemp> ( CtrValue.Voref + VREF_K))
			CtrValue.Voref = CtrValue.Voref + VREF_K;
	else if( VTemp < ( CtrValue.Voref - VREF_K ))
			CtrValue.Voref =CtrValue.Voref - VREF_K;
	else
			CtrValue.Voref = VTemp ;

	//BUCK ģʽ�µ�ѹ���ƣ������ѹ���ﵽ�����ѹ��0.85��
	if(CtrValue.Voref >((SADC.VinAvg*3482)>>12))//��������������0.85*vin 
		CtrValue.Voref =((SADC.VinAvg*3482)>>12);
}

/*
** ===================================================================
**     Funtion Name :void ShortOff(void)
**     Description :��·��������������10��
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_SHORT_I     3444//��·�����о�
#define MIN_SHORT_V     289//��·��ѹ�о�
void ShortOff(void)
{
	static int32_t RSCnt = 0;
	static uint8_t RSNum =0 ;

	//������������� *A���ҵ�ѹС��*Vʱ�����ж�Ϊ������·����
	if((SADC.Iout> MAX_SHORT_I)&&(SADC.Vout <MIN_SHORT_V))
	{
		//�ر�PWM
		DF.PWMENFlag=0;
		HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //�ر�
		HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //�ر�	
		//���ϱ�־λ
		setRegBits(DF.ErrFlag,F_SW_SHORT);
		//��ת������״̬
		DF.SMFlag  =Err;
	}
	//�����·�����ָ�
	//�����������·�������ػ���ȴ�4S�����������Ϣ������ȴ�״̬�ȴ�����
	if(getRegBits(DF.ErrFlag,F_SW_SHORT))
	{
		//�ȴ���������������ۼ�
		RSCnt++;
		//�ȴ�2S
		if(RSCnt >400)
		{
			//����������
			RSCnt=0;
			//��·����ֻ����10�Σ�10�κ�����
			if(RSNum > 10)
			{
				//ȷ����������ϣ�������
				RSNum =11;
				//�ر�PWM
				DF.PWMENFlag=0;
				HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //�ر�
				HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //�ر�	
			}
			else
			{
				//��·�����������ۼ�
				RSNum++;
				//��������������ϱ�־λ
				clrRegBits(DF.ErrFlag,F_SW_SHORT);
			}
		}
	}
}
/*
** ===================================================================
**     Funtion Name :void SwOCP(void)
**     Description :�������������������
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_OCP_VAL     3165//*A���������� 
void SwOCP(void)
{
	//���������оݱ��ּ���������
	static  uint16_t  OCPCnt=0;
	//����������ּ���������
	static  uint16_t  RSCnt=0;
	//������������������
	static  uint16_t  RSNum=0;

	//�������������*A���ұ���500ms
	if((SADC.Iout > MAX_OCP_VAL)&&(DF.SMFlag  ==Run))
	{
		//�������ּ�ʱ
		OCPCnt++;
		//��������50ms������Ϊ��������
		if(OCPCnt > 10)
		{
			//��������0
			OCPCnt  = 0;
			//�ر�PWM
			DF.PWMENFlag=0;
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //�ر�
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //�ر�	
			//���ϱ�־λ
			setRegBits(DF.ErrFlag,F_SW_IOUT_OCP);
			//��ת������״̬
			DF.SMFlag  =Err;
		}
	}
	else
		//��������0
		OCPCnt  = 0;

	//���������ָ�
	//�����������������������ػ���ȴ�4S�����������Ϣ������ȴ�״̬�ȴ�����
	if(getRegBits(DF.ErrFlag,F_SW_IOUT_OCP))
	{
		//�ȴ���������������ۼ�
		RSCnt++;
		//�ȴ�2S
		if(RSCnt > 400)
		{
			//����������
			RSCnt=0;
			//���������������ۼ�
			RSNum++;
			//��������ֻ����10�Σ�10�κ����������ع��ϣ�
			if(RSNum > 10 )
			{
				//ȷ����������ϣ�������
				RSNum =11;
				//�ر�PWM
				DF.PWMENFlag=0;
				HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //�ر�
				HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //�ر�		
			}
			else
			{
			 //��������������ϱ�־λ
				clrRegBits(DF.ErrFlag,F_SW_IOUT_OCP);
			}
		}
	}
}

/*
** ===================================================================
**     Funtion Name :void SwOVP(void)
**     Description :��������ѹ������������
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_VOUT_OVP_VAL    3012//50V��ѹ����	��50/68��*Q12
void VoutSwOVP(void)
{
	//��ѹ�����оݱ��ּ���������
	static  uint16_t  OVPCnt=0;

	//�������ѹ����50V���ұ���100ms
	if (SADC.Vout > MAX_VOUT_OVP_VAL)
	{
		//�������ּ�ʱ
		OVPCnt++;
		//��������10ms
		if(OVPCnt > 2)
		{
			//��ʱ������
			OVPCnt=0;
			//�ر�PWM
			DF.PWMENFlag=0;
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //�ر�
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //�ر�		
			//���ϱ�־λ
			setRegBits(DF.ErrFlag,F_SW_VOUT_OVP);
			//��ת������״̬
			DF.SMFlag  =Err;
		}
	}
	else
		OVPCnt = 0;
}

/*
** ===================================================================
**     Funtion Name :void VinSwUVP(void)
**     Description :�������Ƿѹ��������ѹ���뱣��,�ɻָ�
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MIN_UVP_VAL    686//11.4VǷѹ���� ��11.4/68 ��*Q12
#define MIN_UVP_VAL_RE  795//13.2VǷѹ�����ָ� ��13.2/68��*Q12
void VinSwUVP(void)
{
	//��ѹ�����оݱ��ּ���������
	static  uint16_t  UVPCnt=0;
	static  uint16_t	RSCnt=0;

	//���������С����11.4V���ұ���200ms
	if ((SADC.Vin < MIN_UVP_VAL) && (DF.SMFlag != Init ))
	{
		//�������ּ�ʱ
		UVPCnt++;
		//��������10ms
		if(UVPCnt > 2)
		{
			//��ʱ������
			UVPCnt=0;
			RSCnt=0;
			//�ر�PWM
			DF.PWMENFlag=0;
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //�ر�
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //�ر�		
			//���ϱ�־λ
			setRegBits(DF.ErrFlag,F_SW_VIN_UVP);
			//��ת������״̬
			DF.SMFlag  =Err;
		}
	}
	else
		UVPCnt = 0;
	
	//����Ƿѹ�����ָ�
	//����������Ƿѹ�������ȴ������ѹ�ָ�������ˮƽ��������ϱ�־λ������
	if(getRegBits(DF.ErrFlag,F_SW_VIN_UVP))
	{
		if(SADC.Vin > MIN_UVP_VAL_RE) 
		{
			//�ȴ���������������ۼ�
			RSCnt++;
			//�ȴ�1S
			if(RSCnt > 200)
			{
				RSCnt=0;
				UVPCnt=0;
				//������ϱ�־λ
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
**     Description :��������ѹ������������
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_VIN_OVP_VAL    3012//50V��ѹ����	��50/68��*Q12
void VinSwOVP(void)
{
	//��ѹ�����оݱ��ּ���������
	static  uint16_t  OVPCnt=0;

	//�������ѹ����50V���ұ���100ms
	if (SADC.Vin > MAX_VIN_OVP_VAL )
	{
		//�������ּ�ʱ
		OVPCnt++;
		//��������10ms
		if(OVPCnt > 2)
		{
			//��ʱ������
			OVPCnt=0;
			//�ر�PWM
			DF.PWMENFlag=0;
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //�ر�
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //�ر�		
			//���ϱ�־λ
			setRegBits(DF.ErrFlag,F_SW_VIN_OVP);
			//��ת������״̬
			DF.SMFlag  =Err;
		}
	}
	else
		OVPCnt = 0;
}

/** ===================================================================
**     Funtion Name :void LEDShow(void)
**     Description :  LED��ʾ����
**     ��ʼ����ȴ�����״̬�������ȫ��
**     ����״̬��������
**     ����״̬���̵���
**     ����״̬�������
**     Parameters  :
**     Returns     :
** ===================================================================*/
//���״̬�ƺ궨��
 #define SET_LED_G()	HAL_GPIO_WritePin(GPIOB, LED_G_Pin,GPIO_PIN_SET)//�̵���
 #define SET_LED_Y()	HAL_GPIO_WritePin(GPIOB, LED_Y_Pin,GPIO_PIN_SET)//�̵���
 #define SET_LED_R()	HAL_GPIO_WritePin(GPIOB, LED_R_Pin,GPIO_PIN_SET)//�̵���
 #define CLR_LED_G()	HAL_GPIO_WritePin(GPIOB, LED_G_Pin,GPIO_PIN_RESET)//�̵���
 #define CLR_LED_Y()	HAL_GPIO_WritePin(GPIOB, LED_Y_Pin,GPIO_PIN_RESET)//�Ƶ���
 #define CLR_LED_R()	HAL_GPIO_WritePin(GPIOB, LED_R_Pin,GPIO_PIN_RESET)//�����
void LEDShow(void)
{
	switch(DF.SMFlag)
	{
		//��ʼ��״̬�������ȫ��
		case  Init :
		{
			SET_LED_G();
			SET_LED_Y();
			SET_LED_R();
			break;
		}
		//�ȴ�״̬�������ȫ��
		case  Wait :
		{
			SET_LED_G();
			SET_LED_Y();
			SET_LED_R();
			break;
		}
		//������״̬��������
		case  Rise :
		{
			SET_LED_G();
			SET_LED_Y();
			CLR_LED_R();
			break;
		}
		//����״̬���̵���
		case  Run :
		{
			SET_LED_G();
			CLR_LED_Y();
			CLR_LED_R();
			break;
		}
		//����״̬�������
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
**     Description :����ģʽ�ж�
** 		 BUCKģʽ������ο���ѹ<0.8�������ѹ
** 		 BOOSTģʽ������ο���ѹ>1.2�������ѹ
**		 MIXģʽ��1.15�������ѹ>����ο���ѹ>0.85�������ѹ
**		 ������MIX��buck-boost��ģʽ���˳���BUCK����BOOSTʱ��Ҫ�ͻ�����ֹ���ٽ��������
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

PWM �\��G

TA1 -> �����Ť� 48%
TB1 -> �����Ť� 48%
TA2 -> �����Ť� 48%
TB2 -> �����Ť� 48%

TA1�PTB1�i�Τ��ɡATA2 �P TB2 �i�Τ���
-> ��TA1�e�Ť�48%�ɡATB1�e�Ť�52%
-> ��TA2 �e�Ť�48%�ɡATB2�e�Ť�52%

TA1 �P TA2 �i�Τ@�P�A TB1 �P TB2 �i�Τ@�P
TA1 �P TB1 �i�Τ��ɡATA2 �P TB2 �i�Τ���

1. TA1 �P TB1 �O����
2. TA2 �P TB2 �O���� 
3. TA1 �P TB1 �����Ϯɶ��� 2%��50%�A����C��2%
4. TA2 �P TB2 �����Ϯɶ��� 2%��50%�A����C��2%
5. ��l�W�v�� 100KHz
6. duty �� 50%

;---
����\�໡���G

1. PA6 -> T1, TB1, TA2, TB2 �P�ɽհ��W�v
          -> �W��125K

2. PA7 -> T1, TB1, TA2, TB2 �P�ɽէC�W�v
          -> �U��77K

3. PB4 -> TA1, TB1 �W�[���Ϯɶ�
           -> �W�� 2%�A 50-48 = 2%

4. PB5 -> TA1, TB1 ��֦��Ϯɶ�
           -> �U�� 10%�A 50-40 = 10%

5. PB6 -> TA2, TB2 �P�ɽհ��e�Ť�
            -> �W��45%

6. PB7  -> TA2, TB2 �P�ɭ��C�e�Ť�
            -> �̧C��5%
7. PB9  -> �����u�@�Ҧ�
            -> open loop / close loop

����\�໡���G
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

// �w�q����Ҧ�
#define MODE_OPEN 0
#define MODE_CLOSE 1

// �̤p�P�̤j�W�v�]���GHz�^
#define FREQ_MIN 70000.0f    // 70 kHz
#define FREQ_MAX 130000.0f   // 130 kHz
#define FREQ_STEP_PERCENT 0.1f  // �C������վ㪺�ʤ���0.1%

// ���Ϯɶ��Ѽơ]���G0.1%�A�Y�C�B�վ� 0.1%�^
#define DEADTIME_MIN_PX1000 0     // 0%
#define DEADTIME_MAX_PX1000 20    // 2%
#define DEADTIME_STEP_PX1000 1    // 0.1%

// ���Ť�Ѽơ]���G0.1%�A�Y�C�B�վ� 0.1%�^
#define DUTY_MIN_PX10 50    // 5.0%
#define DUTY_MAX_PX10 500   // 50.0%
#define DUTY_STEP_PX10 1     // 0.1%

// ���] CNTR_MIN �M CNTR_MAX �w�q�p�U
#define CNTR_MIN 1000
#define CNTR_MAX 16000


// �����ܶq
extern HRTIM_HandleTypeDef hhrtim1;
extern HRTIM_TimeBaseCfgTypeDef pGlobalTimeBaseCfg;

volatile float currentPWMFreq = 100000.0f;        // ��l�W�v 100 kHz
volatile uint8_t gCurrentDeadTimePercent = 2;     // ��l���Ϯɶ� 2%

//volatile uint32_t OPN_DEADTIMEPX1000 = 20;         // ��l���Ϯɶ� 2%
//volatile int32_t vOPNDutyPercentageX10 = 250;      // ��l���Ť� 25.0%
//volatile uint8_t currentMode = MODE_OPEN;           // ��l�Ҧ��}��

// ��l�Ʀ��Ť�
volatile uint8_t gCurrentDutyPercent_TA1_TB1 = 48; // TA1/TB1 ��l���Ť� 48%
volatile uint8_t gCurrentDutyPercent_TA2_TB2 = 48; // TA2/TB2 ��l���Ť� 48%

// �ɰ�t�m���c
HRTIM_TimeBaseCfgTypeDef pGlobalTimeBaseCfg;

// ��e PLL �W�v
volatile uint32_t currentPLLFreq = 160000000;      // 160 MHz�A�ھ� fHRCK �]�w

void Button_Task(void)
{
	// KEY1/PA6 : T1, TB1, TA2, TB2 �P�ɽհ��W�v
	if (Key_Scan(KEY1_INC_Freq_GPIO_Port, KEY1_INC_Freq_Pin) == KEY_ON)
	{
		if (currentPWMFreq < FREQ_MAX)
		{
			currentPWMFreq *= (1.0f + (FREQ_STEP_PERCENT / 100.0f)); // �W�[ 0.1%
			if (currentPWMFreq > FREQ_MAX)
				currentPWMFreq = FREQ_MAX;
	
			// ��s HRTIM ���W�v
			if (SetPWMFrequency((uint32_t)currentPWMFreq) != HAL_OK) {
				// �B�z���~
				Error_Handler();
			}
			HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin);
		}
	}
	
	// KEY2/PA7 : T1, TB1, TA2, TB2 �P�ɽէC�W�v
	if (Key_Scan(KEY2_DEC_Freq_GPIO_Port, KEY2_DEC_Freq_Pin) == KEY_ON)
	{
		if (currentPWMFreq > FREQ_MIN)
		{
			currentPWMFreq *= (1.0f - (FREQ_STEP_PERCENT / 100.0f)); // ��� 0.1%
			if (currentPWMFreq < FREQ_MIN)
				currentPWMFreq = FREQ_MIN;
	
			// ��s HRTIM ���W�v
			if (SetPWMFrequency((uint32_t)currentPWMFreq) != HAL_OK) {
				// �B�z���~
				Error_Handler();
			}
			HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin);
		}
	}

    // KEY3/PB4: �W�[ TA1/TB1 �����Ϯɶ�
    if (Key_Scan(KEY3_INC_DT_GPIO_Port, KEY3_INC_DT_Pin) == KEY_ON)
    {
		if (gCurrentDeadTimePercent < 50)
		{
			gCurrentDeadTimePercent += 1; // �W�[ 1%
	
			if (gCurrentDeadTimePercent > 50)
				gCurrentDeadTimePercent = 50;
	
				// ��ʽվ㦺�Ϯɶ�
				SetDeadTimeManual(gCurrentDeadTimePercent);
	
				HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); 
		}
    }

	// KEY4/PB5: ��� TA1/TB1 �����Ϯɶ�
    if (Key_Scan(KEY4_DEC_DT_GPIO_Port, KEY4_DEC_DT_Pin) == KEY_ON)
    {
		if (gCurrentDeadTimePercent > 10)
		{
			gCurrentDeadTimePercent -= 1; // ��� 1%
	
			if (gCurrentDeadTimePercent < 10)
				gCurrentDeadTimePercent = 10;
	
					// ��ʽվ㦺�Ϯɶ�
					SetDeadTimeManual(gCurrentDeadTimePercent);
					HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); 
			}
    }

	// KEY5/PB6: �P�ɽհ� TA2/TB2 ���Ť�
    if (Key_Scan(KEY5_INC_DUTY_GPIO_Port, KEY5_INC_DUTY_Pin) == KEY_ON)
    {
		if (gCurrentDutyPercent_TA2_TB2 < 45)
		{
			gCurrentDutyPercent_TA2_TB2 += 1; // �W�[ 1%
			if (gCurrentDutyPercent_TA2_TB2 > 45)
				gCurrentDutyPercent_TA2_TB2 = 45;
	
				// ��s���Ť�
				if (SetDutyCycle_TA2_TB2(gCurrentDutyPercent_TA2_TB2) != HAL_OK) {
					// �B�z���~
					Error_Handler();
				}
					HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); 
			}
    }
	// KEY6/PB7: �P�ɽէC TA2/TB2 ���Ť�
    if (Key_Scan(KEY6_DEC_DUTY_GPIO_Port, KEY6_DEC_DUTY_Pin) == KEY_ON)
    {
			if (gCurrentDutyPercent_TA2_TB2 > 5)
			{
				gCurrentDutyPercent_TA2_TB2 -= 1; // ��� 1%
				if (gCurrentDutyPercent_TA2_TB2 < 5)
					gCurrentDutyPercent_TA2_TB2 = 5;
	
				// ��s���Ť�
				if (SetDutyCycle_TA2_TB2(gCurrentDutyPercent_TA2_TB2) != HAL_OK) {
					// �B�z���~
					Error_Handler();
				}
					HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); // �{�{ LED
			}
    }

    if (Key_Scan(KEY7_SWITCH_MODE_GPIO_Port, KEY7_SWITCH_MODE_Pin) == KEY_ON)
    {
		Mode_Switch(); // �����Ҧ�
    }
	// ��s OLED ���
		UpdateDisplay();

}


/** ===================================================================
**     Funtion Name : Set_HRTIM_CompareVale
**     Description : 

�ثe�O�ϥ� 100MHz�i�氣�W�A timebase = 16000 = 100KHz
�g�������I 50% = 8000

1.  duty ����W�L 50%
2.  Frequency �W���� 130KHz�A �U���� 70KHz
3.  dead time���W����2%

**     Parameters  :req_tim_freq - �ШD�� PWM �W�v�]Hz�^�C
**     Returns     :HAL_StatusTypeDef - ���\��^ HAL_OK�A���Ѫ�^ HAL_ERROR�C
** ===================================================================*/
HAL_StatusTypeDef SetPWMFrequency(uint32_t req_tim_freq) {
    uint32_t prescaler_value;
    uint32_t fHRCK;
    uint32_t period;

    // �w�q�W�v�d��
    if (req_tim_freq < FREQ_MIN || req_tim_freq > FREQ_MAX) {
        return HAL_ERROR; // �W�v���b�d��
    }

    if (req_tim_freq >= 100000) {
        // �ϥ� prescaler = MUL16
        prescaler_value = 16;
        fHRCK = 100000000UL * prescaler_value; // 100MHz * 16 = 1.6GHz
    } else {
        // �ϥ� prescaler = MUL8
        prescaler_value = 8;
        fHRCK = 100000000UL * prescaler_value; // 100MHz * 8 = 800MHz (�ھڹ�ڮ����t�m�վ�)
    }

    // �p��s�� Period
    period = fHRCK / req_tim_freq;

    // ���� Period �O�_�b���\�d��
    if (period < CNTR_MIN || period > CNTR_MAX) {
        return HAL_ERROR;
    }

    // ��s�����ɰ�t�m�� Period �M PrescalerRatio
    pGlobalTimeBaseCfg.Period = period;
    if (prescaler_value == 16) {
        pGlobalTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL16;
    } else if (prescaler_value == 8) {
        pGlobalTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL8;
    } else {
        return HAL_ERROR; // ��������w���W��
    }

    // �]�w Timer A ���ɰ�
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pGlobalTimeBaseCfg) != HAL_OK) {
        return HAL_ERROR;
    }

    // �]�w Timer B ���ɰ�
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pGlobalTimeBaseCfg) != HAL_OK) {
        return HAL_ERROR;
    }

    // �]�w����椸 2 �����I�ȡ]50% ���Ť�^
    HRTIM_CompareCfgTypeDef pCompareCfg = {0};
    pCompareCfg.CompareValue = (period / 2) - 1;
    pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    pCompareCfg.AutoDelayedTimeout = 0x0000;

    // �]�w Timer A ������椸 2
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK) {
        return HAL_ERROR;
    }

    // �]�w Timer B ������椸 2
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK) {
        return HAL_ERROR;
    }

    // �n�󭫸m HRTIM �� Timer A �M Timer B�A�H���ηs���]�w
    if (HAL_HRTIM_SoftwareReset(&hhrtim1, HRTIM_TIMERRESET_TIMER_A | HRTIM_TIMERRESET_TIMER_B) != HAL_OK) {
        return HAL_ERROR;
    }

    // ��s�����W�v�ܶq
    currentPWMFreq = req_tim_freq;
    currentPLLFreq = fHRCK;

    return HAL_OK;
}


/**
  * @brief  ��ʳ]�w���Ϯɶ��C
  * @param  dead_time_percent - �s�����Ϯɶ�
  * @retval None
  */
void SetDeadTimeManual(uint8_t dead_time_percent)
{
    // �T�O���Ϯɶ��b���\�d��
    if (dead_time_percent < 2 || dead_time_percent > 50) {
        return;
    }

    uint32_t period = pGlobalTimeBaseCfg.Period;
    uint32_t dead_time_ticks = (period * dead_time_percent) / 100;

    // �վ� Timer A Compare Unit 2 (���I) �Ω� TA1 �� ResetSource
    HRTIM_CompareCfgTypeDef compareConfigA = {0};
    compareConfigA.CompareValue = dead_time_ticks;
    compareConfigA.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    compareConfigA.AutoDelayedTimeout = 0x0000;

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &compareConfigA) != HAL_OK) {
        Error_Handler();
    }

    // �վ� Timer B Compare Unit 2 (���I) �Ω� TB1 �� ResetSource
    HRTIM_CompareCfgTypeDef compareConfigB = {0};
    compareConfigB.CompareValue = period - dead_time_ticks;
    compareConfigB.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    compareConfigB.AutoDelayedTimeout = 0x0000;

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &compareConfigB) != HAL_OK) {
        Error_Handler();
    }

    // ���s�Ұʭp�ɾ��H���ηs�����Ϯɶ�
    if (HAL_HRTIM_SoftwareReset(&hhrtim1, HRTIM_TIMERRESET_TIMER_A | HRTIM_TIMERRESET_TIMER_B) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B) != HAL_OK) {
        Error_Handler();
    }
}


/**
  * @brief  �]�w HRTIM �� PWM ���Ť� TA1/TB1�C
  * @param  duty_percent - �s�����Ť�]�ʤ���^�C
  * @retval HAL_StatusTypeDef - ���\��^ HAL_OK�A���Ѫ�^ HAL_ERROR�C
  */
HAL_StatusTypeDef SetDutyCycle_TA1_TB1(uint8_t duty_percent)
{
    if (duty_percent < 5 || duty_percent > 95)
    {
        return HAL_ERROR;
    }

    uint32_t period = pGlobalTimeBaseCfg.Period;
    uint32_t compare_value = (period * duty_percent) / 100;

    // ��s Timer A Compare Unit 2 (���I) ���� TA1 �� ResetSource
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
  * @brief  �]�w HRTIM �� PWM ���Ť� TA2/TB2�C
  * @param  duty_percent - �s�����Ť�]�ʤ���^�C
  * @retval HAL_StatusTypeDef - ���\��^ HAL_OK�A���Ѫ�^ HAL_ERROR�C
  */
HAL_StatusTypeDef SetDutyCycle_TA2_TB2(uint8_t duty_percent)
{
    if (duty_percent < 5 || duty_percent > 45)
    {
        return HAL_ERROR;
    }

    uint32_t period = pGlobalTimeBaseCfg.Period;
    uint32_t compare_value = (period * duty_percent) / 100;

    // ��s Timer A Compare Unit 2 (���I) ���� TA2 �� ResetSource
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
// �Ҧ��ܶq�A�q�{���}���Ҧ�
volatile uint8_t currentMode = MODE_OPEN_LOOP;

/**
  * @brief  �����Ҧ����
  * @retval None
  */
void Mode_Switch(void)
{
    if (currentMode == MODE_OPEN_LOOP)
    {
        currentMode = MODE_CLOSED_LOOP;
        // ��l���W�v�� 100 kHz
        currentPWMFreq = 100000.0f;
        if (SetPWMFrequency((uint32_t)currentPWMFreq) != HAL_OK)
        {
            Error_Handler();
        }

        // ��ܼҦ��ܧ�
        OLED_ShowStr(60, 0, "Closed", 2);
    }
    else
    {
        currentMode = MODE_OPEN_LOOP;
        // ��ܼҦ��ܧ�
        OLED_ShowStr(60, 0, "Open", 2);
    }

    HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); // �{�{ LED �H���ܼҦ��ܧ�
}




/**
  * @brief  ��s OLED ��ܨ��
  * @retval None
  */
void UpdateDisplay(void)
{
    // �ھڼҦ���ܤ��P�H��
    if (currentMode == MODE_CLOSED_LOOP)
    {
        // �����Ҧ��G��� ADC �q�����ഫ�᪺�W�v

        // �T�O ADC �w�g�ļ�
        ADCSample();

        // �N ADC �q���ഫ�� 0-3.3V
        float adc_voltage = (SADC.VinAvg / 4095.0f) * 3.3f;

        // �p��������W�v
        // ������ 1.65V ���� 100 kHz
        // �]�w�W�v�d�� 50 kHz �� 150 kHz
        float frequency = 100000.0f + ((adc_voltage - 1.65f) / 1.65f) * 50000.0f; // ��50 kHz

        // �����W�v�d��
        if (frequency < 50000.0f)
            frequency = 50000.0f;
        if (frequency > 150000.0f)
            frequency = 150000.0f;

        // ��s PWM �W�v
        currentPWMFreq = frequency;
        //if (SetPWMFrequency((uint32_t)currentPWMFreq) != HAL_OK)
        //{
        //    Error_Handler();
        //}

        // ����W�v
        unsigned char freqStr[10];
        //sprintf(freqStr, "%.1fKHz", frequency / 1000.0f);
        OLED_ShowStr(60, 6, "     ", 2); // �M���즳���
        OLED_ShowStr(60, 6, freqStr, 2);
    }
    else
    {
        // �}���Ҧ��G��ܷ�e��ʳ]�m���W�v

        // ����W�v
        unsigned char freqStr[10];
        //sprintf(freqStr, "%.1fKHz", currentPWMFreq / 1000.0f);
        OLED_ShowStr(60, 6, "     ", 2); // �M���즳���
        OLED_ShowStr(60, 6, freqStr, 2);
    }

    // ��� ADC �q����
    // �N ADC �q���ഫ�� 0-3.3V
    float adc_voltage_display = (SADC.VinAvg / 4095.0f) * 3.3f;

    // �N�q�����ഫ����ܮ榡 (mV)
    uint8_t Vtemp[4] = {0};
    uint32_t Vdisplay = (uint32_t)(adc_voltage_display * 1000.0f); // �ഫ�� mV

    Vtemp[0] = (uint8_t)(Vdisplay / 1000);
    Vtemp[1] = (uint8_t)((Vdisplay % 1000) / 100);
    Vtemp[2] = (uint8_t)((Vdisplay % 100) / 10);
    Vtemp[3] = (uint8_t)(Vdisplay % 10);

    // ��� ADC �q��
    OLEDShowData(50, 2, Vtemp[0]);
    OLEDShowData(60, 2, Vtemp[1]);
    OLEDShowData(75, 2, Vtemp[2]);
    OLEDShowData(85, 2, Vtemp[3]);

    // �i��G��ܨ�L�ƾڡA�p Iout ��
}











