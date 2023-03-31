
#ifndef __MSLA001B_H__
#define __MSLA001B_H__
#include "cust_alsps.h"
#include <FreeRTOS.h>
#include <semphr.h>


/*MSLA001B ALSPS REGS*/
#define MSLA001B_REG_STATUSFLAG					0x00
#define MSLA001B_REG_THRESHOLDFLAG				0x01
#define MSLA001B_REG_ERRORFLAG					0x02
#define MSLA001B_REG_PDAT_L						0x03
#define MSLA001B_REG_PDAT_H						0x04
#define MSLA001B_REG_ADAT_0_L						0x05
#define MSLA001B_REG_ADAT_0_H						0x06
#define MSLA001B_REG_ADAT_1_L						0x07
#define MSLA001B_REG_ADAT_1_H						0x08
#define MSLA001B_REG_ADAT_2_L						0x09
#define MSLA001B_REG_ADAT_2_H						0x0A
#define MSLA001B_REG_PID							0x1E
#define MSLA001B_REG_VID							0x1F
#define MSLA001B_REG_CTC_TARGET					0x20
#define MSLA001B_REG_CTC_RECAL_CTRL				0x22
#define MSLA001B_REG_PSCTRL						0x40
#define MSLA001B_REG_PSPULSEWIDTH					0x42
#define MSLA001B_REG_PSBURSTCOUNT					0x43
#define MSLA001B_REG_LEDDRIVERCTRL				0x44
#define MSLA001B_REG_PSIINTCTRL					0x45
#define MSLA001B_REG_PSCTCCTRL					0x47
#define MSLA001B_REG_PSBASELINE_L					0x4A
#define MSLA001B_REG_PSBASELINE_H					0x4B
#define MSLA001B_REG_PSTHRESHOLD_L_L				0x4C
#define MSLA001B_REG_PSTHRESHOLD_L_H				0x4D
#define MSLA001B_REG_PSTHRESHOLD_H_L				0x4E
#define MSLA001B_REG_PSTHRESHOLD_H_H				0x4F
#define MSLA001B_REG_ALS_0_CTRL					0x50
#define MSLA001B_REG_APW_0_L						0x51
#define MSLA001B_REG_APW_0_H						0x52
#define MSLA001B_REG_ALS_1_CTRL					0x53
#define MSLA001B_REG_APW_1_L						0x54
#define MSLA001B_REG_APW_1_H						0x55
#define MSLA001B_REG_ALS_2_CTRL					0x56
#define MSLA001B_REG_APW_2_L						0x57
#define MSLA001B_REG_APW_2_H						0x58
#define MSLA001B_REG_ALS_0_THRESHOLD_L_L			0x60
#define MSLA001B_REG_ALS_0_THRESHOLD_L_H			0x61
#define MSLA001B_REG_ALS_0_THRESHOLD_H_L			0x62
#define MSLA001B_REG_ALS_0_THRESHOLD_H_H			0x63
#define MSLA001B_REG_ALS_1_THRESHOLD_L_L			0x64
#define MSLA001B_REG_ALS_1_THRESHOLD_L_H			0x65
#define MSLA001B_REG_ALS_1_THRESHOLD_H_L			0x66
#define MSLA001B_REG_ALS_1_THRESHOLD_H_H			0x67
#define MSLA001B_REG_ALS_2_THRESHOLD_L_L			0x68
#define MSLA001B_REG_ALS_2_THRESHOLD_L_H			0x69
#define MSLA001B_REG_ALS_2_THRESHOLD_H_L			0x6A
#define MSLA001B_REG_ALS_2_THRESHOLD_H_H			0x6B
#define MSLA001B_REG_ALS_INTCTRL					0x6C
#define MSLA001B_REG_SENSOR_CTRL					0x80
#define MSLA001B_REG_INTERRUPUT_CTRL_0				0x81
#define MSLA001B_REG_INTERRUPUT_CTRL_1				0x82
#define MSLA001B_REG_SOFTWARE_RST					0x84
#define MSLA001B_REG_FRAMETIME_L					0x88
#define MSLA001B_REG_FRAMETIME_H					0x89
#define MSLA001B_REG_UUID							0xC0
#define MSLA001B_REG_CTC_GAIN						0xCB
#define MSLA001B_REG_CTC_STEP						0xCC
#define MSLA001B_REG_SPEC_ADD						0xCF
#define MSLA001B_REG_SPE							0xCD
#define MSLA001B_REG_L2FQC							0xC9
#define MSLA001B_REG_DARK0							0xF4
#define MSLA001B_REG_DARK0_CH0						0xFA


#define MSLA001B_CTC_TARGET_VALUE					32
#define MSLA001B_CTC_RECAL_VALUE					0x08

#define MSLA001B_CTC_AUTO_ENABLE_VALUE				0xF1
#define MSLA001B_FRAMETIME_VALUE					30
#define MSLA001B_PSCTRL_VALUE						0x1F
#define MSLA001B_PPW_VALUE							0x80
#define MSLA001B_PBC_VALUE							0x02


#define MSLA001B_LEDDRV_VALUE						0x28
#define MSLA001B_PS_INTCTRL_STATIC_VALUE			0x50
#define MSLA001B_PS_INTCTRL_DYNA_VALUE				0x40

#define MSLA001B_ALS0_GAIN_VALUE					0x02
#define MSLA001B_ALS0_PW_VALUE						250 
#define MSLA001B_ALS1_GAIN_VALUE					0x02
#define MSLA001B_ALS1_PW_VALUE						250 
#define MSLA001B_ALS2_GAIN_VALUE					0x02
#define MSLA001B_ALS2_PW_VALUE						250 

#define MSLA001B_ALS0_ENABLE						0x02
#define MSLA001B_ALS1_ENABLE						0x04
#define MSLA001B_ALS2_ENABLE						0x08

#define MSLA001B_PID								0x02



// 2023 02 22 
#define MSLA001B_SET_SPE_0xCD						0xCB



#endif
