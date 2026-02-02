/*
 * Pwm_Ftm.h
 *
 *  Created on: Jan 24, 2026
 *      Author: andre
 */

#ifndef PWM_FTM_H_
#define PWM_FTM_H_


#include "S32K146.h"
#include "S32K146_features.h"
#include <stdbool.h>


#define PWM_MOD   1999U   // 20 kHz @ 40 MHz
#define PWM_DUTY  1000U   // 50%

/*!
 * @brief FTM_MODE -  Read and modify and write Counter Features Mode Selection (RW)
 */
#define FTM_RMW_MODE(base, mask, value) (((base)->MODE) = ((((base)->MODE) & ~(mask)) | (value)))

/*!
 * @brief FTM_SC - Read and modify and write to Status And Control (RW)
 */
#define FTM_RMW_SC(base, mask, value) (((base)->SC) = ((((base)->SC) & ~(mask)) | (value)))

/*!
 * @brief FTM_CONF -  Read and modify and write Configuration (RW)
 */
#define FTM_RMW_CONF(base, mask, value) (((base)->CONF) = ((((base)->CONF) & ~(mask)) | (value)))



typedef enum
{
    FTM_BDM_MODE_00 = 0x00U,    /*!< FTM counter stopped, CH(n)F bit can be set, FTM channels
                                 *   in functional mode, writes to MOD,CNTIN and C(n)V registers bypass
                                 *   the register buffers */
    FTM_BDM_MODE_01 = 0x01U,    /*!< FTM counter stopped, CH(n)F bit is not set, FTM channels
                                 *   outputs are forced to their safe value , writes to MOD,CNTIN and
                                 *   C(n)V registers bypass the register buffers */
    FTM_BDM_MODE_10 = 0x02U,    /*!< FTM counter stopped, CH(n)F bit is not set, FTM channels
                                *    outputs are frozen when chip enters in BDM mode, writes to MOD,
                                *    CNTIN and C(n)V registers bypass the register buffers */
    FTM_BDM_MODE_11 = 0x03U     /*!< FTM counter in functional mode, CH(n)F bit can be set,
                                 *   FTM channels in functional mode, writes to MOD,CNTIN and C(n)V
                                 *   registers is in fully functional mode */
} FtmBdmMode;



typedef enum
{
    FTM_CLOCK_SOURCE_NONE           = 0x00U,    /*!< None use clock for FTM  */
    FTM_CLOCK_SOURCE_SYSTEMCLK      = 0x01U,    /*!< System clock            */
    FTM_CLOCK_SOURCE_FIXEDCLK       = 0x02U,    /*!< Fixed clock             */
    FTM_CLOCK_SOURCE_EXTERNALCLK    = 0x03U     /*!< External clock          */
} FtmClockSource;


typedef enum
{
	FTM_CLOCK_CIV_BY_1      = 0x00u,
	FTM_CLOCK_CIV_BY_2      = 0x01u,
	FTM_CLOCK_CIV_BY_4      = 0x02u,
	FTM_CLOCK_CIV_BY_8      = 0x03u,
	FTM_CLOCK_CIV_BY_16     = 0x04u,
	FTM_CLOCK_CIV_BY_32     = 0x05u,
	FTM_CLOCK_CIV_BY_64     = 0x06u,
	FTM_CLOCK_CIV_BY_128    = 0x07u,
} FtmClockPrescaler;




/* PWM parameters */
#define PWM_MOD_VALUE     1999U   /* 20 kHz */
#define PWM_DUTY_50       1000U   /* 50% */

void CxpiPWMCenterAlignInit(void);
void CxpiPWMStop(void);
void CxpiPWMStart(void);
void CxpiPWMCenterAlignInitRed(void);
void CxpiPWMStopRed(void);
void CxpiPWMStartRed(void);
void CxpiPWMCenterAlignInitMotor(void);
void CxpiPWMStartMotor(void);
void CxpiPWMStopMotor(void);

#endif /* PWM_FTM_H_ */
