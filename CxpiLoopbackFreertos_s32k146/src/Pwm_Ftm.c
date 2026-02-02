/*
 * Pwm_Ftm.c
 *
 *  Created on: Jan 24, 2026
 *      Author: andre
 */


#include "../include/Pwm_Ftm.h"


const FTM_Type* const FtmBasePtr[FTM_INSTANCE_COUNT] = {FTM0, FTM1, FTM2, FTM3, FTM4, FTM5};




static inline uint8_t FTM_DRV_GetClockPs(const FTM_Type * ftmBase)
{
    return (uint8_t)((((ftmBase)->SC) & FTM_SC_PS_MASK) >> FTM_SC_PS_SHIFT);
}






static void Ftm_Reset(const uint8_t Instance)
{

	uint8_t ChannelIndex;
	FTM_Type* const BasePtr = (FTM_Type* const)FtmBasePtr[Instance];


	/* WPDIS is set when WPEN bit is read as a 1 and then 1 is written to WPDIS */
	((BasePtr)->FMS) &= 0U;
	/* This is the reset value for MODE register. WPDIS bit is set to disable write protection */
	((BasePtr)->MODE) = 0x00000004U;
	((BasePtr)->SC) &= 0U;
	((BasePtr)->CNT) = 0U;
	((BasePtr)->MOD) = 0U;
	((BasePtr)->CNTIN) = 0U;
	((BasePtr)->STATUS) &= 0U;
	((BasePtr)->SYNC) = 0U;
	((BasePtr)->OUTINIT) = 0U;
	((BasePtr)->OUTMASK) = 0U;
	((BasePtr)->COMBINE) = 0U;
	((BasePtr)->DEADTIME) = 0U;
	((BasePtr)->EXTTRIG) &= 0U;
	((BasePtr)->POL) = 0U;
	((BasePtr)->FILTER) = 0U;
	((BasePtr)->FLTCTRL) = 0U;
	((BasePtr)->QDCTRL) = 0U;
	((BasePtr)->CONF) = 0U;
	((BasePtr)->FLTPOL) = 0U;
	((BasePtr)->SYNCONF) = 0U;
	((BasePtr)->INVCTRL) = 0U;
	((BasePtr)->SWOCTRL) = 0U;
	((BasePtr)->PWMLOAD) = 0U;
	((BasePtr)->HCR) = 0U;
	#if FEATURE_FTM_HAS_SUPPORTED_DITHERING
		#if (FTM_INSTANCE_COUNT > 2U)
		if ((BasePtr == FTM1) || (BasePtr == FTM2))
		#else
		if (BasePtr == FTM1)
		#endif
		{
			((BasePtr)->MOD_MIRROR) = 0U;
		}
	#endif
	/* Set to reset value all CnV and CnSC registers */
	for (ChannelIndex = 0; ChannelIndex < FEATURE_FTM_CHANNEL_COUNT; ChannelIndex++)
	{
		((BasePtr)->CONTROLS[ChannelIndex].CnSC) &= 0U;
		((BasePtr)->CONTROLS[ChannelIndex].CnV) = 0U;
		#if FEATURE_FTM_HAS_SUPPORTED_DITHERING
			#if (FTM_INSTANCE_COUNT > 2U)
			if ((BasePtr == FTM1) || (BasePtr == FTM2))
			#else
			if (BasePtr == IP_FTM1)
			#endif
			{
				((BasePtr)->CV_MIRROR[ChannelIndex]) = 0U;
			}
		#endif
	}
}

static inline void FTM_DRV_SetTimerOverflowInt(FTM_Type * const ftmBase, bool state)
{
    FTM_RMW_SC(ftmBase, FTM_SC_TOIE_MASK, FTM_SC_TOIE(state));
}


static inline void FTM_Enable(FTM_Type* const ftmBase, bool enable)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_FTMEN_MASK, FTM_MODE_FTMEN(enable));
}


static inline void FTM_SetClockPrescaler(FTM_Type * const ftmBase, FtmClockPrescaler ClockPrescaler)
{
    FTM_RMW_SC(ftmBase, FTM_SC_PS_MASK, FTM_SC_PS(ClockPrescaler));
}


static inline void FTM_DRV_SetInitTriggerCmd(FTM_Type * const ftmBase, bool enable)
{
    ftmBase->EXTTRIG = (ftmBase->EXTTRIG & ~FTM_EXTTRIG_INITTRIGEN_MASK) | FTM_EXTTRIG_INITTRIGEN(enable);
}

static inline void FTM_DRV_SetBdmMode(FTM_Type * const ftmBase, FtmBdmMode val)
{
    FTM_RMW_CONF(ftmBase, FTM_CONF_BDMMODE_MASK, FTM_CONF_BDMMODE(val));
}



static inline void FTM_SoftwareSync(FTM_Type* const BasePtr)
{
	BasePtr->SYNCONF = (BasePtr->SYNCONF & ~FTM_SYNCONF_SWOM_MASK) | FTM_SYNCONF_SWOM(1u);
	BasePtr->SYNCONF = (BasePtr->SYNCONF & ~FTM_SYNCONF_SWINVC_MASK) | FTM_SYNCONF_SWINVC(1u);
	BasePtr->SYNCONF = (BasePtr->SYNCONF & ~FTM_SYNCONF_SWSOC_MASK) | FTM_SYNCONF_SWSOC(1u);
	BasePtr->SYNCONF = (BasePtr->SYNCONF & ~FTM_SYNCONF_SWWRBUF_MASK) | FTM_SYNCONF_SWWRBUF(1u);
	BasePtr->SYNCONF = (BasePtr->SYNCONF & ~FTM_SYNCONF_SWRSTCNT_MASK) | FTM_SYNCONF_SWRSTCNT(1u);
}


void Pwm_Ftm_Init(const uint8_t Instance, const FtmClockPrescaler ClockPrescaler)
{

}

/* Function prototypes */
void FTM0_PWM_Init(void);



void FTM0_PWM_Init(void)
{
    /* Enable PORTD clock */
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;

    /* Enable FTM0 clock + select SPLL_DIV1 */
    PCC->PCCn[PCC_FTM0_INDEX] =
        PCC_PCCn_PCS(3) |
        PCC_PCCn_CGC_MASK;

    /* PTD15 -> FTM0_CH1 */
    PORTD->PCR[15] = PORT_PCR_MUX(2);

    /* Disable write protection */
    FTM0->MODE |= FTM_MODE_WPDIS_MASK;

    /* Reset counter */
    FTM0->CNTIN = 0;
    FTM0->CNT   = 0;

    FTM_DRV_SetBdmMode(FTM0, 1u);

    /* Set period */
    FTM0->MOD = PWM_MOD;

    /* Edge-aligned PWM, high true */
    FTM0->CONTROLS[1].CnSC =
        FTM_CnSC_MSB_MASK |
        FTM_CnSC_ELSB_MASK;

    /* Duty cycle */
    FTM0->CONTROLS[1].CnV = PWM_DUTY;

    /* Start FTM */
    FTM0->SC =
        FTM_SC_CLKS(1) |   // System clock
        FTM_SC_PS(0);      // Prescaler = 1
}



void CxpiPWMCenterAlignInit(void)
{
	/* Enable clock for PORTD */
	PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;
	/* Select and enable clock for FTM0 */
	PCC->PCCn[PCC_FTM0_INDEX] = PCC_PCCn_PCS(1) | PCC_PCCn_CGC_MASK;

	/* Set PORTD pins for FTM0 */
	PORTD->PCR[16] = PORT_PCR_MUX(2);

    FTM_DRV_SetBdmMode(FTM0, 3u);

    /* Disable write protection */
    FTM0->MODE |= FTM_MODE_WPDIS_MASK;

	/* IP_FTM0->SC |= FTM_SC_PS(2); */

	/* Select up-down counter for Center-Align PWM */
	FTM0->SC |= (uint32_t)FTM_SC_CPWMS_MASK;

	/* Set Modulo (10kHz PWM frequency @112MHz system clock) */
	FTM0->MOD = FTM_MOD_MOD(1200-1);
	/* Set CNTIN */
	FTM0->CNTIN = FTM_CNTIN_INIT(0);
	/* High-true pulses of PWM signals */
	FTM0->CONTROLS[1].CnSC = FTM_CnSC_ELSB_MASK;
	/* Set Channel Value */
	FTM0->CONTROLS[1].CnV = FTM_CnV_VAL(600); // 50% duty cycle
	/* FTM counter reset */
	FTM0->CNT = 0;

	/* Clock selection */
	FTM0->SC |= FTM_SC_CLKS(1);
}

void CxpiPWMStop(void)
{
	/* Clock selection stop and disable PWM generation */
	FTM0->SC &= ~(FTM_SC_PWMEN1_MASK);
}



void CxpiPWMStart(void)
{
	/* Clock selection and enabling PWM generation */
	FTM0->SC |= FTM_SC_PWMEN1_MASK;
}


void CxpiPWMCenterAlignInitMotor1(void)
{
	PCC->PCCn[PCC_FTM2_INDEX] &= ~PCC_PCCn_CGC_MASK;  /* Ensure clk disabled for config  */
	/* Select and enable clock for FTM2 */
	PCC->PCCn[PCC_FTM2_INDEX] = PCC_PCCn_PCS(1) | PCC_PCCn_CGC_MASK;

	/* Enable clock for PORTD */
	PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;

	/* Set PORTD pins for FTM2 */
	PORTD->PCR[0] = PORT_PCR_MUX(2);

    FTM_DRV_SetBdmMode(FTM2, 3u);

    /* Disable write protection */
    FTM2->MODE |= FTM_MODE_WPDIS_MASK;

	/* IP_FTM0->SC |= FTM_SC_PS(2); */

	/* Select up-down counter for Center-Align PWM */
	FTM2->SC |= (uint32_t)FTM_SC_CPWMS_MASK;

	/* Set Modulo (10kHz PWM frequency @112MHz system clock) */
	FTM2->MOD = FTM_MOD_MOD(1200-1);
	/* Set CNTIN */
	FTM2->CNTIN = FTM_CNTIN_INIT(0);
	/* High-true pulses of PWM signals */
	FTM2->CONTROLS[2].CnSC = FTM_CnSC_ELSB_MASK;
	/* Set Channel Value */
	FTM2->CONTROLS[2].CnV = FTM_CnV_VAL(600); // 50% duty cycle
	/* FTM counter reset */
	FTM2->CNT = 0;

	/* Clock selection */
	FTM2->SC |= FTM_SC_CLKS(1);
}

void CxpiPWMStopMotor1(void)
{
	/* Clock selection stop and disable PWM generation */
	FTM2->SC &= ~(FTM_SC_PWMEN2_MASK);
}



void CxpiPWMStartMotor1(void)
{
	/* Clock selection and enabling PWM generation */
	FTM2->SC |= FTM_SC_PWMEN2_MASK;
}


void CxpiPWMCenterAlignInitMotor(void)
{
	/* Enable clock for PORTD */
	PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;
	/* Select and enable clock for FTM0 */
	PCC->PCCn[PCC_FTM0_INDEX] = PCC_PCCn_PCS(1) | PCC_PCCn_CGC_MASK;

	/* Set PORTD pins for FTM0 */
	PORTD->PCR[15] = PORT_PCR_MUX(2);

    FTM_DRV_SetBdmMode(FTM0, 3u);

    /* Disable write protection */
    FTM0->MODE |= FTM_MODE_WPDIS_MASK;

	/* IP_FTM0->SC |= FTM_SC_PS(2); */

	/* Select up-down counter for Center-Align PWM */
	FTM0->SC |= (uint32_t)FTM_SC_CPWMS_MASK;

	/* Set Modulo (10kHz PWM frequency @112MHz system clock) */
	FTM0->MOD = FTM_MOD_MOD(1200-1);
	/* Set CNTIN */
	FTM0->CNTIN = FTM_CNTIN_INIT(0);
	/* High-true pulses of PWM signals */
	FTM0->CONTROLS[0].CnSC = FTM_CnSC_ELSB_MASK;
	/* Set Channel Value */
	FTM0->CONTROLS[0].CnV = FTM_CnV_VAL(600); // 50% duty cycle
	/* FTM counter reset */
	FTM0->CNT = 0;

	/* Clock selection */
	FTM0->SC |= FTM_SC_CLKS(1);
}


void CxpiPWMStopMotor(void)
{
	/* Clock selection stop and disable PWM generation */
	FTM0->SC &= ~ (FTM_SC_PWMEN0_MASK);
}



void CxpiPWMStartMotor(void)
{
	/* Clock selection and enabling PWM generation */
	FTM0->SC |= FTM_SC_PWMEN0_MASK;
}
