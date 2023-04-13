#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <seos.h>
#include <util.h>
#include <sensors.h>
#include <plat/inc/rtc.h>
#include <contexthub_core.h>
#include <mt_gpt.h>
#include <timer.h>

// 2023 02 23
#include <unistd.h> // new

#include "eint.h"
#include "msla001b.h"
#include "alsps.h"


#define ALS_NAME                         "msla001b_l"
#define PS_NAME                          "msla001b_p"

#define UNIT_MS_TO_NS   				1000000

enum msla001bState {
    STATE_SAMPLE_ALS            = CHIP_SAMPLING_ALS,
    STATE_SAMPLE_ALS_DONE       = CHIP_SAMPLING_ALS_DONE,
    STATE_SAMPLE_PS             = CHIP_SAMPLING_PS,
#ifdef CFG_ONCE_TIMER_SUPPORT
    STATE_SAMPLE_PS_ONCE = CHIP_SAMPLING_PS_ONCE,
#endif
    STATE_SAMPLE_PS_DONE        = CHIP_SAMPLING_PS_DONE,
    STATE_ALS_ENABLE            = CHIP_ALS_ENABLE,
    STATE_ALS_ENABLE_DONE       = CHIP_ALS_ENABLE_DONE,
    STATE_ALS_DISABLE           = CHIP_ALS_DISABLE,
    STATE_ALS_DISABLE_DONE      = CHIP_ALS_DISABLE_DONE,
    STATE_ALS_RATECHG           = CHIP_ALS_RATECHG,
    STATE_ALS_RATECHG_DONE      = CHIP_ALS_RATECHG_DONE,
    STATE_ALS_CALI              = CHIP_ALS_CALI,
    STATE_ALS_CALI_DONE         = CHIP_ALS_CALI_DONE,
    STATE_ALS_CFG               = CHIP_ALS_CFG,
    STATE_ALS_CFG_DONE          = CHIP_ALS_CFG_DONE,
    STATE_PS_ENABLE             = CHIP_PS_ENABLE,
    STATE_PS_ENABLE_DONE        = CHIP_PS_ENABLE_DONE,
    STATE_PS_DISABLE            = CHIP_PS_DISABLE,
    STATE_PS_DISABLE_DONE       = CHIP_PS_DISABLE_DONE,
    STATE_PS_RATECHG            = CHIP_PS_RATECHG,
    STATE_PS_RATECHG_DONE       = CHIP_PS_RATECHG_DONE,
    STATE_PS_CALI               = CHIP_PS_CALI,
    STATE_PS_CALI_DONE          = CHIP_PS_CALI_DONE,
    STATE_PS_CFG                = CHIP_PS_CFG,
    STATE_PS_CFG_DONE           = CHIP_PS_CFG_DONE,
#ifdef CFG_FAE_TIMER_SUPPORT
    STATE_SAMPLING_FAE_PS = CHIP_SAMPLING_FAE_PS,
    STATE_SAMPLING_FAE_PS_DONE = CHIP_SAMPLING_FAE_PS_DONE,
#endif
    STATE_INIT_DONE             = CHIP_INIT_DONE,
    STATE_IDEL                  = CHIP_IDLE,
    STATE_RESET                 = CHIP_RESET,
	/* Alsps sample */
    STATE_GET_ALS_DATA,
    STATE_GET_ALS_DGAIN,
    STATE_SET_INIT_DGAIN,
    STATE_ALS_SECOND_INTELLI_WAIT,
    STATE_ALS_SET_DEBOUNCE,
    STATE_GET_PS_FLG,
    STATE_GET_PS_RAW_DATA,
    STATE_PS_SET_DEBOUNCE,


    /* 0x80_Set */
    STATE_0x80_Read,
    STATE_0x80_Set0,
    STATE_0x80_Set0x81,
    STATE_0x80_Set0x01,
    STATE_Loop_5_ps,
    STATE_Read_5_ps,
	
	STATE_0x80_ReadALS,
    STATE_0x80_Set0ALS,

#ifdef CFG_ONCE_TIMER_SUPPORT
    STATE_GET_PS_STATUS_ONCE,
    STATE_GET_PS_RAW_DATA_ONCE,
#endif
    /* Power on & off */
    STATE_GET_ALSPS_STATE,
    STATE_ALS_POWER_ON,
    STATE_ALS_POWER_OFF,
    STATE_PS_POWER_ON,

    STATE_PS_POWER_OFF,
    STATE_PS_UNMASK_EINT,
    STATE_CLR_INT,
    STATE_EN_EINT,
    STATE_INTERRUPUT_CTRL1,
    /* Init state */
    STATE_CLR_POR,
    STATE_SET_SW_RST,
    STATE_SET_SW_0_RST,
    
    // 2023 02 15  l2fqc
    // STATE_SET_l2fqc_read,
    // STATE_SET_l2fqc_set,

    STATE_SET_ALSPS_CTRL,
    STATE_GET_SPEC,
    STATE_SET_SPEC,
    STATE_SET_ALS2_CTRL,
    STATE_SET_INTELL_WAIT_PS,
    STATE_SET_PS_THDH,
    STATE_SET_PS_THDL,
    STATE_SETUP_EINT,
#ifdef CFG_FAE_TIMER_SUPPORT
    //STATE_GET_INVALID_FLAG,
    //STATE_GET_BGIR_RAW_DATA,
    //STATE_GET_PS_OFF_DATA,
    STATE_TUNE_FAE,
#endif
    STATE_CORE
};

#define I2C_SPEED 			400000
#define MAX_RXBUF 			16
#define MAX_TXBUF 			16
#define MAX_UUIDUF 			8
extern int32_t sensorAlsSample;
//int32_t sensorAlsSample = 50;

enum CH_Data {
		CH0,
		CH1,
		CH2,
		CH_NUM
	};

static struct msla001bTask {
    /* txBuf for i2c operation, fill register and fill value */
    bool                        alsPowerOn;
    bool                        psPowerOn;
    unsigned int                als_debounce_time;
    unsigned int                als_debounce_on;    /* indicates if the debounce is on */
    uint64_t                    als_debounce_end;
    unsigned int                ps_debounce_time;
    unsigned int                ps_debounce_on;     /* indicates if the debounce is on */
    uint64_t                    ps_debounce_end;
    unsigned int                ps_suspend;
    uint8_t                     txBuf[MAX_TXBUF];
    /* rxBuf for i2c operation, receive rawdata */
    uint8_t                     rxBuf[MAX_RXBUF];
    uint8_t                     tmpBuf;
    uint8_t                     deviceId;
    /* data for factory */
    struct                      alsps_hw *hw;
    uint8_t                     i2c_addr;
    struct                      transferDataInfo dataInfo;
    struct                      AlsPsData data[2];
    int32_t                     psCali;
    /* data for ALSCFG */
    int32_t                     alsCali;
    /* data for PSCFG */
    uint8_t                     tran_count;
    uint32_t                    tran_ps_cali_data[4];
    char                        pscali_flag;
    int                         tran_cali_ct;
    uint32_t                    als_raw_data[CH_NUM];
    uint32_t                    prox_raw_data;
    uint32_t                    ps_threshold_high;
    uint32_t                    ps_threshold_low;
    uint32_t                    als_threshold_high;
    uint32_t                    als_threshold_low;
    uint8_t                     ledctrl_val;
    uint8_t                     state_val;
	bool						ps_first_read;
    uint32_t                    als_gain;
    uint32_t                    als_integration_time;
    uint8_t                     als_integrate_gain;
    uint8_t                     flag_report_count;
	uint8_t                     uuidBuf[MAX_UUIDUF];
	uint32_t					als_integ_ratio;
} mTask;

static struct msla001bTask *msla001bDebugPoint;

//static int getAlsSampleData(void);

static void cal_end_time(unsigned int ms, uint64_t *end_tick)
{
    *end_tick = rtcGetTime() +
                ((uint64_t)ms * UNIT_MS_TO_NS);
}

static void psGetSensorInfo(struct sensorInfo_t *data)
{
    strncpy(data->name, PS_NAME, sizeof(data->name));
}

static void alsGetSensorInfo(struct sensorInfo_t *data)
{
    strncpy(data->name, ALS_NAME, sizeof(data->name));
}

static int msla001b_read_als_data(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    if (rxTransferDataInfo(&mTask.dataInfo, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize)) {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "als rx error\n");
        return -1;
    }

    mTask.txBuf[0] = MSLA001B_REG_PDAT_L;
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1, mTask.rxBuf, 8, i2cCallBack, next_state);


}


static int msla001b_get_als_value(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int als_data = 0;


    
    mTask.rxBuf[0] = mTask.rxBuf[2];
    mTask.rxBuf[1] = mTask.rxBuf[3];

    mTask.rxBuf[2] = mTask.rxBuf[4];
    mTask.rxBuf[3] = mTask.rxBuf[5];

    mTask.rxBuf[4] = mTask.rxBuf[6];
    mTask.rxBuf[5] = mTask.rxBuf[7];
    

    mTask.als_raw_data[CH0] = ((mTask.rxBuf[1] << 8) | mTask.rxBuf[0]);
    mTask.als_raw_data[CH1] = ((mTask.rxBuf[3] << 8) | mTask.rxBuf[2]);
    mTask.als_raw_data[CH2] = ((mTask.rxBuf[5] << 8) | mTask.rxBuf[4]);
	
	/*als_data report raw data for transsion models*/
    als_data = mTask.als_raw_data[CH0];

    if (als_data < 0)
        mTask.data[0].sensType = SENS_TYPE_INVALID;
    else {
		mTask.data[0].als_data = als_data;
        mTask.data[0].sensType = SENS_TYPE_ALS;
    }
    osLog(LOG_INFO, "msla001b_get_als_value: (v1.1) 0x%x,0x%x\n", mTask.rxBuf[1], mTask.rxBuf[0]);
    osLog(LOG_INFO, "msla001b_get_als_value:0x%d\n",mTask.als_raw_data[CH0]);
	/*different integration time compensation(1st: 25ms, 2nd: 100ms)*/
    if (sensorAlsSample > 70)
    {
        if (mTask.flag_report_count == 1){
            mTask.data[0].als_data = mTask.data[0].als_data * mTask.als_integ_ratio; /* report als data when als IT = 25ms  */
            osLog(LOG_INFO, "mTask.data.als_data-1 first %d %d %d\n", mTask.data[0].als_data, mTask.flag_report_count, mTask.als_integ_ratio);
        }
		else{
			mTask.data[0].als_data = mTask.data[0].als_data ; /* report als data when als IT = 100ms  */
			osLog(LOG_INFO, "mTask.data.als_data-2 first %d %d\n", mTask.data[0].als_data, mTask.flag_report_count);
		}
    }
    else
    {
        if (mTask.flag_report_count == 1){
            mTask.data[0].als_data = mTask.data[0].als_data * mTask.als_integ_ratio; /* report als data when als IT = 25ms  */
            osLog(LOG_INFO, "mTask.data.als_data-3 first %d %d %d\n", mTask.data[0].als_data, mTask.flag_report_count, mTask.als_integ_ratio);
        }
		else{
			mTask.data[0].als_data = mTask.data[0].als_data ; /* report als data when als IT = 100ms  */
			osLog(LOG_INFO, "mTask.data.als_data-4 first %d %d\n", mTask.data[0].als_data, mTask.flag_report_count);
		}
    }
    osLog(LOG_INFO, "msla001b_report: als_data=%d\n", mTask.data[0].als_data);
    txTransferDataInfo(&mTask.dataInfo, 1, &mTask.data[0]);
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}




static int tran_msla001b_set_second_intelli(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                          void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                          void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    osLog(LOG_INFO, "tran_msla001b_set_second_intelli:mTask.rxBuf[0]=0x%x,mTask.rxBuf[1]=0x%x\n",mTask.rxBuf[0],mTask.rxBuf[1]);
    if (mTask.flag_report_count == 1)
    {
        osLog(LOG_ERROR, "tran_msla001b_set_second_intelli als report %d\n", mTask.flag_report_count);
        mTask.flag_report_count = 2;
        mTask.txBuf[0] = MSLA001B_REG_APW_0_L;
        mTask.txBuf[1] = (uint8_t)(mTask.als_integration_time & 0x00FF);
	    mTask.txBuf[2] = (uint8_t)((mTask.als_integration_time & 0xFF00) >> 8);
        osLog(LOG_INFO, "tran_msla001b_set_second_intelli:mTasktxBuf[1]=0x%x,mTask.txBuf[2]=0x%x\n",mTask.txBuf[1],mTask.txBuf[2]);
        ret = i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 3, NULL, 0, NULL, NULL);
        if (ret < 0)
        {
            osLog(LOG_ERROR, "tran_msla001b_set_second_intelli i2c err\n");
        }
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
        return ret;
    }
    else
    {
        mTask.flag_report_count++;
        if (mTask.flag_report_count > 3)
        {
            mTask.flag_report_count = 3;
        }
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
        return 0;
    }
}
#if 0
static int getAlsSampleData(void)
{
	osLog(LOG_INFO, "%s\n", __func__);
	int als_raw_data;
	
	mTask.txBuf[0] = MSLA001B_REG_PDAT_L;
    i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1, mTask.rxBuf, 8, NULL, 0);
	
	mTask.rxBuf[0] = mTask.rxBuf[2];
    mTask.rxBuf[1] = mTask.rxBuf[3];
	als_raw_data = ((mTask.rxBuf[1] << 8) | mTask.rxBuf[0]);
 	
	osLog(LOG_INFO, "%s als_raw_data(%d)\n", als_raw_data);
	return als_raw_data;
}
#endif
static void alsGetData(void *sample)
{
    struct SingleAxisDataPoint *singleSample = (struct SingleAxisDataPoint *)sample;
    //singleSample->idata = getAlsSampleData();
    singleSample->idata = mTask.als_raw_data[CH0];
}

static int msla001b_read_ps(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                          void *inBuf, uint8_t inSize, uint8_t elemInSize,
                          void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    if (rxTransferDataInfo(&mTask.dataInfo, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize)) {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "msla001b ps, rx dataInfo error\n");
        return -1;
    }

	osLog(LOG_INFO, "%s\n", __func__);
	
    mTask.txBuf[0] = MSLA001B_REG_THRESHOLDFLAG;
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         mTask.rxBuf, 1, i2cCallBack,
                         next_state);
}

static int msla001b_get_ps_status(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ps_flag = mTask.rxBuf[0] & 0x03;
    osLog(LOG_ERROR, "msla001b_get_ps_status,mTask.rxBuf[0]=0x%x,ps_flag=%d\n",mTask.rxBuf[0],ps_flag);


	if (ps_flag == 2) { 
        mTask.data[0].prox_state = PROX_STATE_FAR; /* far */
        mTask.data[0].prox_data = 1; /* far state */
    } else if (ps_flag == 1) {
        mTask.data[0].prox_state = PROX_STATE_NEAR; /* near */
        mTask.data[0].prox_data = 0; /* near state */
    } else if (ps_flag != 0 ){
			return -1;
	}

	if(mTask.ps_first_read){
		mTask.txBuf[0] = MSLA001B_REG_PSIINTCTRL;
		mTask.txBuf[1] = MSLA001B_PS_INTCTRL_DYNA_VALUE;
			
		i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
				NULL, 0);
		mTask.ps_first_read = 0;
		}
	
	
    mTask.txBuf[0] = MSLA001B_REG_PDAT_L;


    // 2023 02 17  Read 8 data
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         mTask.rxBuf, 8, i2cCallBack, next_state);
}

static int msla001b_get_ps_raw_data(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                  void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                  void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{

	osLog(LOG_INFO, "%s PDAT_Low = %d, PDAT_High = %d, pscali = %ld\n",__func__ ,mTask.rxBuf[0], mTask.rxBuf[1], mTask.psCali);

	mTask.prox_raw_data = (mTask.rxBuf[1] << 8) | mTask.rxBuf[0];
    if (mTask.prox_raw_data < mTask.psCali)
        mTask.prox_raw_data = 0;
    mTask.prox_raw_data -= mTask.psCali;
	
    osLog(LOG_INFO, "%s, mTask.prox_raw_data=%d\n", __func__,mTask.prox_raw_data);
    mTask.txBuf[0] = MSLA001B_REG_STATUSFLAG;
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                       mTask.rxBuf, 1, i2cCallBack,
                       next_state);
}

#ifdef CFG_ONCE_TIMER_SUPPORT
static int msla001b_read_ps_once(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                          void *inBuf, uint8_t inSize, uint8_t elemInSize,
                          void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    osLog(LOG_ERROR, "msla001b_read_ps_once\n");
     sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int msla001b_get_ps_status_once(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
     sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int msla001b_get_ps_raw_data_once(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                  void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                  void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;

}
#endif //CFG_ONCE_TIMER_SUPPORT

#ifdef CFG_FAE_TIMER_SUPPORT
static int msla001b_get_ps_data(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    mTask.txBuf[0] = MSLA001B_REG_PDAT_L;
    // 2023 02 17  Read 8 data
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         mTask.rxBuf, 8, i2cCallBack, next_state);
}

static int msla001b_tune_fae(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    mTask.prox_raw_data = (mTask.rxBuf[1] << 8) | mTask.rxBuf[0];
    if (mTask.prox_raw_data < mTask.psCali)
        mTask.prox_raw_data = 0;
    mTask.prox_raw_data -= mTask.psCali;
	
    osLog(LOG_INFO, "%s, mTask.ps_raw_data=%d\n", __func__,mTask.prox_raw_data);
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}
#endif //CFG_FAE_TIMER_SUPPORT

static int msla001b_clr_int(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                  void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                  void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_ERROR, "msla001b_clr_int,rxbuf=0x%x\n",mTask.rxBuf[0]);
    mTask.txBuf[0] = MSLA001B_REG_STATUSFLAG;
    mTask.txBuf[1] = 0x00;
    //osLog(LOG_ERROR, "msla001b_clr_int：0x%x,0x%x\n",mTask.rxBuf[0],mTask.txBuf[1]);
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                       i2cCallBack, next_state);
}

static int msla001b_interrupt_ctrl1(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                  void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                  void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_ERROR, "msla001b_interrupt_ctrl1\n");
    mTask.txBuf[0] = MSLA001B_REG_INTERRUPUT_CTRL_1;
    mTask.txBuf[1] = 0x01;
    //osLog(LOG_ERROR, "msla001b_clr_int：0x%x,0x%x\n",mTask.rxBuf[0],mTask.txBuf[1]);
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                       i2cCallBack, next_state);
}

static int msla001b_enable_eint(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                  void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                  void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mt_eint_unmask(mTask.hw->eint_num);
    osLog(LOG_ERROR, "msla001b_enable_eint,mTask.data[0].prox_data=%d,\n",mTask.data[0].prox_data);
    mTask.data[0].sensType = SENS_TYPE_PROX;
    txTransferDataInfo(&mTask.dataInfo, 1, &mTask.data[0]);
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static void psGetData(void *sample)
{
    //char txBuf[1];
    //static char rxBuf[2];
    struct TripleAxisDataPoint *tripleSample = (struct TripleAxisDataPoint *)sample;
    tripleSample->ix = mTask.prox_raw_data;
    tripleSample->iy = (mTask.data[0].prox_state == PROX_STATE_NEAR ? 0 : 1);

	osLog(LOG_INFO, "%s: ps raw data:%d, ps state:%d, psCali:%d \n", __func__, mTask.prox_raw_data, tripleSample->iy, mTask.psCali);
    #if 0
    void get_ps_data(void *cookie, size_t tx, size_t rx, int err)
    {
        char *rxBuf = cookie;
        if (err == 0) {
            mTask.prox_raw_data = (rxBuf[1] << 8) | rxBuf[0];
            if (mTask.prox_raw_data < mTask.psCali)
                mTask.prox_raw_data = 0;
            mTask.prox_raw_data -= mTask.psCali;
        } else
            osLog(LOG_INFO, "msla001b: read ps data i2c error (%d)\n", err);
    }
    osLog(LOG_INFO, "%s: ps raw data1:%d, ps state1:%d, psCali1:%d \n", __func__, mTask.prox_raw_data, tripleSample->iy, mTask.psCali);
    txBuf[0] = MSLA001B_REG_PDAT_L;
 
    // 2023 02 17 Read 8 Data
    i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, txBuf, 1,  rxBuf, 8 , get_ps_data, rxBuf);
    #endif
}

static void msla001b_ps_set_threshold(uint32_t threshold_high, uint32_t threshold_low)
{
    int ret;
    char txBuf[6];
    osLog(LOG_INFO, "msla001b_ps_set_threshold:threshold_high=%d,threshold_low=%d\n", threshold_high,threshold_low);
    txBuf[0] = MSLA001B_REG_PSTHRESHOLD_L_L;
	txBuf[1] = (uint8_t)(threshold_low & 0xFF);
	txBuf[2] = (uint8_t)((threshold_low & 0xFF00) >> 8);
	txBuf[3] = (uint8_t)(threshold_high & 0xFF);
	txBuf[4] = (uint8_t)((threshold_high & 0xFF00) >> 8);
	osLog(LOG_INFO, "msla001b_ps_set_threshold:1=0x%x,2=0x%x,3=0x%x,4=0x%x,)\n", txBuf[1],txBuf[2],txBuf[3],txBuf[4]);
    ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, &txBuf[0], 5,
                NULL, 0);
    if (ret) {
        osLog(LOG_INFO, "msla001b: set ps threshold i2c error (%d)\n", ret);
        return;
    }
}

static void psGetCalibration(int32_t *cali, int32_t size)
{	
	osLog(LOG_INFO, "%s \n", __func__);
    cali[0] = mTask.psCali;
}

static void psSetCalibration(int32_t *cali, int32_t size)
{
	/*MSLA001B is auto calibration*/
	int ret;
	char txBuf[2];
    
    osLog(LOG_INFO, "%s cali(0x%x) size(%d). \n", __func__, cali, size);
    if (cali != NULL)
	    mTask.psCali = cali[0];

	txBuf[0] = MSLA001B_REG_CTC_TARGET;
	txBuf[1] = MSLA001B_CTC_TARGET_VALUE;

	ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, &txBuf[0], 2, NULL, 0);
	if (ret) {
        osLog(LOG_ERROR, "%s:  write MSLA001B_REG_CTC_TARGET error (%d)\n", __func__, ret);
        return;
    }
	
	txBuf[0] = MSLA001B_REG_CTC_RECAL_CTRL;
	txBuf[1] = MSLA001B_CTC_RECAL_VALUE;

	ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, &txBuf[0], 2, NULL, 0);
	if (ret) {
        osLog(LOG_ERROR, "%s: write MSLA001B_REG_CTC_RECAL_CTRL error (%d)\n", __func__, ret);
        return;
    }

	txBuf[0] = MSLA001B_REG_PSCTCCTRL;
	txBuf[1] = MSLA001B_CTC_AUTO_ENABLE_VALUE;

	ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, &txBuf[0], 2, NULL, 0);	
	if (ret) {
        osLog(LOG_ERROR, "%s: write MSLA001B_REG_PSCTCCTRL error (%d)\n", __func__, ret);
        return;
    }
    
}

static void psGetThreshold(uint32_t *threshold_high, uint32_t *threshold_low)
{
    *threshold_high = mTask.ps_threshold_high;
    *threshold_low = mTask.ps_threshold_low;
    osLog(LOG_INFO, "%s ==>threshold_high:%ld threshold_low:%ld \n", __func__, *threshold_high, *threshold_low);
}

static void psSetThreshold(uint32_t threshold_high, uint32_t threshold_low)
{
    osLog(LOG_INFO, "%s ==>threshold_high:%ld threshold_low:%ld \n", __func__, threshold_high, threshold_low);
    mTask.ps_threshold_high = threshold_high;
    mTask.ps_threshold_low = threshold_low;
    msla001b_ps_set_threshold(mTask.ps_threshold_high, mTask.ps_threshold_low);
}

static int msla001b_mask_eint(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_INFO, "%s ==>mTask.hw->eint_num:%ld\n", __func__, mTask.hw->eint_num);
    mt_eint_mask(mTask.hw->eint_num);
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int msla001b_unmask_eint(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mt_eint_unmask(mTask.hw->eint_num);
     osLog(LOG_INFO, "%s ==>mTask.hw->eint_num:%ld\n", __func__, mTask.hw->eint_num);
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int msla001b_set_als_debounce_on(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                      void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                      void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.als_debounce_on = 1;
    cal_end_time(mTask.als_debounce_time, &mTask.als_debounce_end);
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int msla001b_set_ps_debounce_on(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                     void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                     void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.ps_debounce_on = 1;
     osLog(LOG_ERROR, "msla001b_set_ps_debounce_on\n");
    cal_end_time(mTask.ps_debounce_time, &mTask.ps_debounce_end);
    mTask.data[0].prox_state = PROX_STATE_INIT;
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}



static int msla001b_get_als2_state(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void* next_state,
    void* inBuf, uint8_t inSize, uint8_t elemInSize,
    void* outBuf, uint8_t* outSize, uint8_t* elemOutSize)
{
    int ret = 0;
    mTask.tmpBuf = mTask.rxBuf[0];

    mTask.txBuf[0] = MSLA001B_REG_PSIINTCTRL; // 0x45
    mTask.txBuf[1] = MSLA001B_PS_INTCTRL_STATIC_VALUE; //0x50

    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, 0);

    mTask.ps_first_read = 1;

    osLog(LOG_INFO, "msla001b_get_alsps_state\n");
    mTask.txBuf[0] = MSLA001B_REG_SENSOR_CTRL;
    mTask.txBuf[1] = 0;

    ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack, next_state);
    return ret;
}


static int msla001b_set_init_dgain(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                 void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                 void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) 
{
    /*speed up for the 1st time sampling*/
    mTask.flag_report_count= 1;
    mTask.txBuf[0] = MSLA001B_REG_APW_0_L;
	mTask.txBuf[1] = (uint8_t)(MSLA001B_ALS0_PW_VALUE & 0x00FF);
	mTask.txBuf[2] = (uint8_t)((MSLA001B_ALS0_PW_VALUE & 0xFF00) >> 8);
    osLog(LOG_INFO, "msla001b_set_init_dgain integration=%d\n",MSLA001B_ALS0_PW_VALUE);
    osLog(LOG_INFO, "msla001b_set_init_dgain mTask.txBuf[1]=0x%x,mTask.txBuf[2]=0x%x\n",mTask.txBuf[1],mTask.txBuf[2]);
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 3,
                         i2cCallBack, next_state);

}
#if 1
static int msla001b_get_als_dgain(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                 void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                 void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) 
{
    //mTask.flag_report_count= 1;
    int ret = 0;
    mTask.txBuf[0] = MSLA001B_REG_APW_0_L;
    osLog(LOG_INFO, "msla001b_get_als_dgain");
    ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         mTask.rxBuf, 2, i2cCallBack,
                         next_state);
    osLog(LOG_INFO, "msla001b_get_als_dgain:ret=%d\n",ret);
    if (ret < 0)
    {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
        osLog(LOG_ERROR, "msla001b_als_cfg, rx inSize and elemSize error\n");
        return -1;
    }
    return ret;
}
#endif

static int msla001b_set_als_power_on(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                   void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                   void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
	/*setup for dark count ~ 0, requested by Transsion*/
	mTask.txBuf[0] = MSLA001B_REG_SPE;
    mTask.txBuf[1] = 00;
	i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, 0);
	
	
	mTask.txBuf[0] = MSLA001B_REG_L2FQC;
	i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1, mTask.rxBuf, 1, NULL, 0);
	mTask.txBuf[0] = MSLA001B_REG_L2FQC;
    mTask.txBuf[1] = (mTask.rxBuf[0] | 0x40);/*enable l2fqc*/
	osLog(LOG_INFO, "%s MSLA001B_REG_L2FQC(0x%x).\n",__func__, mTask.txBuf[1]);
	i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, 0);


	mTask.txBuf[0] = MSLA001B_REG_DARK0_CH0;
	mTask.txBuf[1] = 0x70;
	mTask.txBuf[2] = 0x00;
	osLog(LOG_INFO, "%s MSLA001B_REG_DARK0_CH0 Write(0x%x 0x%x).\n",__func__, mTask.txBuf[1], mTask.txBuf[2]);
	i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 3, NULL, 0);	
	
	mTask.rxBuf[0] = 0;
	mTask.rxBuf[1] = 0;
	mTask.txBuf[0] = MSLA001B_REG_DARK0_CH0;
	i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1, mTask.rxBuf, 2, NULL, 0);
	osLog(LOG_INFO, "%s MSLA001B_REG_DARK0_CH0 Read(0x%x 0x%x).\n",__func__, mTask.rxBuf[0], mTask.rxBuf[1]);
	
	/*show uuid*/
	osLog(LOG_INFO, "%s UUID (0x%x 0x%x 0x%x 0x%x - 0x%x 0x%x 0x%x 0x%x).\n",__func__, mTask.uuidBuf[0],mTask.uuidBuf[1],mTask.uuidBuf[2],mTask.uuidBuf[3],mTask.uuidBuf[4],mTask.uuidBuf[5],mTask.uuidBuf[6],mTask.uuidBuf[7]);
	
	/*set FDC for linear relationship of integration time.*/
	mTask.txBuf[0] = MSLA001B_REG_ALS_CTRL3;
    mTask.txBuf[1] = 0x01;
	i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, 0);
	
	mTask.txBuf[0] = MSLA001B_REG_FDC_WTIMEL;
    mTask.txBuf[1] = 0xC8;
	i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, 0);
	
	mTask.txBuf[0] = MSLA001B_REG_FDC_WTIMEH;
    mTask.txBuf[1] = 0x00;
	i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, 0);
	
	mTask.rxBuf[0] = 0;
	mTask.rxBuf[1] = 0;
	mTask.rxBuf[2] = 0;
	mTask.txBuf[0] = MSLA001B_REG_ALS_CTRL3;
	i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1, mTask.rxBuf, 3, NULL, 0);
	osLog(LOG_INFO, "%s MSLA001B_REG_ALS_CTRL3 Read(0x%x 0x%x 0x%x).\n",__func__, mTask.rxBuf[0], mTask.rxBuf[1], mTask.rxBuf[2]);
	
	/*set sensor control*/
    mTask.flag_report_count = 1;
    mTask.txBuf[0] = MSLA001B_REG_SENSOR_CTRL;
    mTask.txBuf[1] = mTask.tmpBuf | (MSLA001B_ALS0_ENABLE);//enable CH0


    osLog(LOG_INFO, "%s MSLA001B_REG_SENSOR_CTRL Write (0x%x 0x%x ).\n", __func__, mTask.txBuf[0], mTask.txBuf[1]);


    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack, next_state);




}

static int msla001b_set_als_power_off(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                    void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
	int ret;
	osLog(LOG_INFO, "msla001b_set_als_power_off\n");
    mTask.txBuf[0] = MSLA001B_REG_SENSOR_CTRL;
    ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         mTask.rxBuf, 1, NULL, 0);
	if (ret < 0)
    {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
        osLog(LOG_ERROR, "msla001b_set_als_power_off(Read MSLA001B_REG_SENSOR_CTRL), rx inSize and elemSize error\n");
        return -1;
    }
	else{
		osLog(LOG_ERROR, "msla001b_set_als_power_off(Read MSLA001B_REG_SENSOR_CTRL) done.\n");
	}
	
	// 0x80 Set 0
    ret = 0;
    mTask.txBuf[0] = MSLA001B_REG_SENSOR_CTRL;
    mTask.txBuf[1] = 0x00;
    ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, 0);
    if (ret < 0)
    {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
        osLog(LOG_ERROR, "msla001b_set_als_power_off(Set 0), rx inSize and elemSize error\n");
        return -1;
    }
	else{
		osLog(LOG_ERROR, "msla001b_set_als_power_off(Set 0) clear zero done.\n");
	}
	
    mTask.als_debounce_on = 0;
    mTask.txBuf[0] = MSLA001B_REG_SENSOR_CTRL;
    mTask.txBuf[1] = mTask.rxBuf[0] & ~(0x0E);
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                       i2cCallBack, next_state);
}

static int msla001b_als_ratechg(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                              void *inBuf, uint8_t inSize, uint8_t elemInSize,
                              void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int msla001b_als_cali(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                              void *inBuf, uint8_t inSize, uint8_t elemInSize,
                              void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
/*temp set the cali value = 0, if enable the feature, please rewrite here*/
    int32_t alsCali[2] = {0, 0};
    alsPsSendCalibrationResult(SENS_TYPE_ALS, alsCali);
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int msla001b_als_cfg(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                              void *inBuf, uint8_t inSize, uint8_t elemInSize,
                              void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
/*Align als_cali not need work now*/
    int ret = 0;
    struct alspsCaliCfgPacket caliCfgPacket;
    // double  alsCali_mi;

    ret = rxCaliCfgInfo(&caliCfgPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0)
    {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
        osLog(LOG_ERROR, "msla001b_als_cfg, rx inSize and elemSize error\n");
        return -1;
    }

	osLog(LOG_INFO, "%s: [%d]\n",__func__, caliCfgPacket.caliCfgData[0]);

	if (caliCfgPacket.caliCfgData[0] != 0)
    {
        mTask.alsCali = caliCfgPacket.caliCfgData[0];
    }
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

/*
static int msla001b_set_ps_power_on(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                  void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                  void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_INFO,"msla001b_set_ps_power_on\n");
    mTask.txBuf[0] = MSLA001B_REG_SENSOR_CTRL; //0x80
    mTask.txBuf[1] = mTask.rxBuf[0] | 0x01;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                       i2cCallBack, next_state);
}
*/

static int msla001b_set_ps_power_off(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                   void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                   void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
	int ret;
	osLog(LOG_INFO, "msla001b_set_ps_power_off\n");
    mTask.txBuf[0] = MSLA001B_REG_SENSOR_CTRL;
    ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         mTask.rxBuf, 1, NULL, 0);
	if (ret < 0)
    {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
        osLog(LOG_ERROR, "msla001b_set_ps_power_off(Read MSLA001B_REG_SENSOR_CTRL), rx inSize and elemSize error\n");
        return -1;
    }
	else{
		osLog(LOG_ERROR, "msla001b_set_ps_power_off(Read MSLA001B_REG_SENSOR_CTRL) done.\n");
	}
	
	//0x80 Set 0
    ret = 0;
    mTask.txBuf[0] = MSLA001B_REG_SENSOR_CTRL;
    mTask.txBuf[1] = 0x00;
    ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, 0);
    if (ret < 0)
    {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
        osLog(LOG_ERROR, "msla001b_set_ps_power_off, rx inSize and elemSize error\n");
        return -1;
    }
	else{
		osLog(LOG_ERROR, "msla001b_set_ps_power_off clear zero done.\n");
	}
	
    mTask.ps_debounce_on = 0;
    osLog(LOG_INFO,"msla001b_set_ps_power_off\n");
    mTask.txBuf[0] = MSLA001B_REG_SENSOR_CTRL;
    mTask.txBuf[1] = mTask.rxBuf[0] & ~(0x01);
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                       i2cCallBack, next_state);
}

static int msla001b_ps_ratechg(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                             void *inBuf, uint8_t inSize, uint8_t elemInSize,
                             void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int msla001b_ps_cali(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                              void *inBuf, uint8_t inSize, uint8_t elemInSize,
                              void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
	/*Auto calibration by IC*/

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int msla001b_ps_cfg(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                              void *inBuf, uint8_t inSize, uint8_t elemInSize,
                              void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    
    int ret = 0;
    struct alspsCaliCfgPacket caliCfgPacket;

    ret = rxCaliCfgInfo(&caliCfgPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);

    if (ret < 0)
    {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
        osLog(LOG_ERROR, "msla001b_ps_cfg, rx inSize and elemSize error\n");
        return -1;
    }
    osLog(LOG_INFO, "%s, [high, low]: [%d, %d]\n", __func__,caliCfgPacket.caliCfgData[0], caliCfgPacket.caliCfgData[1]);

    
    if (caliCfgPacket.caliCfgData[0] != 0 && caliCfgPacket.caliCfgData[1] != 0 && caliCfgPacket.caliCfgData[0] > caliCfgPacket.caliCfgData[1])
    {
        if (mTask.tran_count == 0)
        {
            mTask.tran_count++;
            mTask.tran_ps_cali_data[0] = caliCfgPacket.caliCfgData[0];
            mTask.tran_ps_cali_data[1] = caliCfgPacket.caliCfgData[1];
            osLog(LOG_INFO, "%s: one [high, low] = [%ld, %ld]\n", __func__, mTask.tran_ps_cali_data[0], mTask.tran_ps_cali_data[1]);
            msla001b_ps_set_threshold(caliCfgPacket.caliCfgData[0], caliCfgPacket.caliCfgData[1]);
        }
        else if (mTask.tran_count == 1)
        {
            mTask.tran_count = 0;
            mTask.tran_ps_cali_data[2] = caliCfgPacket.caliCfgData[0];
            mTask.tran_ps_cali_data[3] = caliCfgPacket.caliCfgData[1];
            osLog(LOG_INFO, "%s: two [high, low] = [%ld, %ld]\n", __func__, mTask.tran_ps_cali_data[2], mTask.tran_ps_cali_data[3]);
            if (mTask.tran_ps_cali_data[0] < mTask.tran_ps_cali_data[2])
            {
                mTask.hw->ps_setting[1] = mTask.tran_ps_cali_data[0];
                mTask.hw->ps_setting[2] = mTask.tran_ps_cali_data[1];
                mTask.hw->ps_threshold_high = mTask.tran_ps_cali_data[2];
                mTask.hw->ps_threshold_low = mTask.tran_ps_cali_data[3];
            }
            else
            {
                mTask.hw->ps_setting[1] = mTask.tran_ps_cali_data[2];
                mTask.hw->ps_setting[2] = mTask.tran_ps_cali_data[3];
                mTask.hw->ps_threshold_high = mTask.tran_ps_cali_data[0];
                mTask.hw->ps_threshold_low = mTask.tran_ps_cali_data[1];
            }

            mTask.pscali_flag = 1;
            mTask.ps_threshold_high = mTask.hw->ps_threshold_high;
            mTask.ps_threshold_low = mTask.hw->ps_threshold_low;
            mTask.tran_cali_ct = mTask.hw->ps_threshold_high - mTask.hw->ps_setting[1];
            osLog(LOG_INFO, "%s: init tran_cali_ct = %d \n", __func__, mTask.tran_cali_ct);
            msla001b_ps_set_threshold((uint32_t)(mTask.ps_threshold_high), (uint32_t)(mTask.ps_threshold_low));
        }
    }
    else
    {
        mTask.pscali_flag = 0;
        caliCfgPacket.caliCfgData[0] = mTask.ps_threshold_high;
        caliCfgPacket.caliCfgData[1] = mTask.ps_threshold_low;
        msla001b_ps_set_threshold(caliCfgPacket.caliCfgData[0], caliCfgPacket.caliCfgData[1]);
    }

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int msla001b_get_alsps_state(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                               void *inBuf, uint8_t inSize, uint8_t elemInSize,
                               void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
	mTask.txBuf[0] = MSLA001B_REG_PSIINTCTRL; // 0x45
	mTask.txBuf[1] = MSLA001B_PS_INTCTRL_STATIC_VALUE; //0x50
			
	i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
				NULL, 0);

	mTask.ps_first_read = 1;
	
    osLog(LOG_INFO, "msla001b_get_alsps_state\n");
    mTask.txBuf[0] = MSLA001B_REG_SENSOR_CTRL;
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         mTask.rxBuf, 1, i2cCallBack,
                         next_state);
}
static int msla001b_set_frame_time(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
	int ret;
  
	osLog(LOG_INFO, "%s\n", __func__);
	/*read UUID for the sensor*/
	mTask.txBuf[0] = MSLA001B_REG_UUID;
    i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1, mTask.uuidBuf, 8, NULL, 0);
	
	
	
	/*set frame time for proximity sensor*/
    mTask.txBuf[0] = MSLA001B_REG_FRAMETIME_L;
	mTask.txBuf[1] = (uint8_t)(MSLA001B_FRAMETIME_VALUE & 0x00FF);
	mTask.txBuf[2] = (uint8_t)((MSLA001B_FRAMETIME_VALUE & 0xFF00) >> 8);
    	
    ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 3,
                       i2cCallBack, next_state);
	
    if (ret < 0) {
        osLog(LOG_INFO, "msla001b: set ps framtime i2c error (%d)\n", ret);
        
    }

	return ret;
}
static int msla001b_set_sw_reset(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                             void *inBuf, uint8_t inSize, uint8_t elemInSize,
                             void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_INFO, "msla001b_set_sw_reset!\n");
    mTask.txBuf[0] = MSLA001B_REG_SOFTWARE_RST;
    mTask.txBuf[1] = 0xA5;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                       i2cCallBack, next_state);

}


static int msla001b_0x80_Read(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void* next_state,
    void* inBuf, uint8_t inSize, uint8_t elemInSize,
    void* outBuf, uint8_t* outSize, uint8_t* elemOutSize)
{
    int ret = 0;

    mTask.txBuf[0] = MSLA001B_REG_SENSOR_CTRL;
    ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1, mTask.rxBuf, 1, i2cCallBack, next_state);


    return ret;
}

/*
static int msla001b_0x80_Set0(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void* next_state,
    void* inBuf, uint8_t inSize, uint8_t elemInSize,
    void* outBuf, uint8_t* outSize, uint8_t* elemOutSize)
{

    // 0x80 Set 0
    int ret = 0;
    mTask.txBuf[0] = MSLA001B_REG_SENSOR_CTRL;
    mTask.txBuf[1] = 0x00;
    ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, 0);
    if (ret < 0)
    {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
        osLog(LOG_ERROR, "msla001b_0x80_Set0, rx inSize and elemSize error\n");
        return -1;
    }


    mTask.txBuf[0] = MSLA001B_REG_SENSOR_CTRL;
    mTask.txBuf[1] = mTask.rxBuf[0];
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack, next_state);
}
*/




static int msla001b_0x80_Set0x81(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void* next_state,
    void* inBuf, uint8_t inSize, uint8_t elemInSize,
    void* outBuf, uint8_t* outSize, uint8_t* elemOutSize)
{
    mTask.tmpBuf = mTask.rxBuf[0];
    osLog(LOG_ERROR, "msla001b_0x80_ReSet :  %d , %d\n", mTask.tmpBuf, mTask.rxBuf[0]);
    // 0x80 Set 0
    int ret = 0;
    mTask.txBuf[0] = MSLA001B_REG_SENSOR_CTRL;
    mTask.txBuf[1] = 0x00;
    ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, 0);
    if (ret < 0)
    {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
        osLog(LOG_ERROR, "msla001b_0x80_Set0, rx inSize and elemSize error\n");
        return -1;
    }
    mTask.txBuf[0] = MSLA001B_REG_SENSOR_CTRL;
    mTask.txBuf[1] = 0x81;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack, next_state);
}



static int msla001b_0x80_Set0x01(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void* next_state,
    void* inBuf, uint8_t inSize, uint8_t elemInSize,
    void* outBuf, uint8_t* outSize, uint8_t* elemOutSize)
{
	// 0x80 Set 0
    int ret = 0;
    mTask.txBuf[0] = MSLA001B_REG_SENSOR_CTRL;
    mTask.txBuf[1] = 0x00;
    ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, 0);
    if (ret < 0)
    {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
        osLog(LOG_ERROR, "msla001b_0x80_Set0x01, rx inSize and elemSize error\n");
        return -1;
    }
	else{
		osLog(LOG_ERROR, "msla001b_0x80_Set0x01 clear zero done.\n");
	}
    // 0x80 Set 0
    mTask.txBuf[0] = MSLA001B_REG_SENSOR_CTRL;
    mTask.txBuf[1] = ((mTask.tmpBuf | 0x01) & 0x7F);
    osLog(LOG_INFO, "msla001b_0x80_Set0x01: 0x%x  \n", mTask.txBuf[1]);
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack, next_state);
}


static int msla001b_Loop_5_ps(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void* next_state,
    void* inBuf, uint8_t inSize, uint8_t elemInSize,
    void* outBuf, uint8_t* outSize, uint8_t* elemOutSize)
{
    int ret = 0;
    for (int i = 0;; )
    {
        mTask.txBuf[0] = MSLA001B_REG_STATUSFLAG;
		mTask.rxBuf[0] = 0;
        ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1, mTask.rxBuf, 1, NULL, 0);
		osLog(LOG_ERROR, "msla001b_Loop_5_ps call i2cMasterTxRx ret(%d) cnt(%d)\n", ret, i++);
		if (mTask.rxBuf[0] == 0)
		{
			//osLog(LOG_ERROR, "msla001b_Loop_5_ps Loop ING ");
		}
		else
		{
			osLog(LOG_ERROR, "msla001b_Loop_5_ps Loop SUCCESS ");
			break;
		}
    }
    mTask.txBuf[0] = MSLA001B_REG_PDAT_L;
    osLog(LOG_ERROR, "msla001b_Loop_5_ps : %d , %d, ret(%d) \n", mTask.rxBuf[0], mTask.rxBuf[1], ret);
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1, mTask.rxBuf, 2, i2cCallBack, next_state);
    
}


static int msla001b_Read_5_ps(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void* next_state,
    void* inBuf, uint8_t inSize, uint8_t elemInSize,
    void* outBuf, uint8_t* outSize, uint8_t* elemOutSize)
{
    //int ret = 0;
    osLog(LOG_ERROR, "msla001b_Read_5_ps : %d , %d \n", mTask.rxBuf[0], mTask.rxBuf[1]);
    mTask.txBuf[0] = MSLA001B_REG_STATUSFLAG;
    mTask.txBuf[1] = 0x00;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack, next_state);
}

static int msla001b_clr_por_flag(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                             void *inBuf, uint8_t inSize, uint8_t elemInSize,
                             void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.txBuf[0] = MSLA001B_REG_STATUSFLAG;
    mTask.txBuf[1] = 0x00;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                       i2cCallBack, next_state);
}

static int msla001b_spec_addr_state(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                               void *inBuf, uint8_t inSize, uint8_t elemInSize,
                               void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.txBuf[0] = MSLA001B_REG_SPEC_ADD;
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         mTask.rxBuf, 1, i2cCallBack,
                         next_state);
}

static int msla001b_spec_addr_cfg(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                             void *inBuf, uint8_t inSize, uint8_t elemInSize,
                             void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.txBuf[0] = MSLA001B_REG_SPEC_ADD;
    mTask.txBuf[1] = mTask.rxBuf[0] | 0x20;	   //Don't change this value!
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                       i2cCallBack, next_state);
}


static int msla001b_set_alsps_ctrl(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.txBuf[0] = MSLA001B_REG_PSCTRL;
    mTask.txBuf[1] = MSLA001B_PSCTRL_VALUE;
//     osLog(LOG_INFO, "msla001b_set_alsps_ctrl!\n");
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                       i2cCallBack, next_state);

}

static int msla001b_set_als2_ctrl(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
	
    mTask.txBuf[0] = MSLA001B_REG_ALS_0_CTRL;   //设置光感GAIN，
    mTask.txBuf[1] = mTask.als_integrate_gain & 0x07;
	mTask.txBuf[2] = (uint8_t)(MSLA001B_ALS0_PW_VALUE & 0x00FF);
	mTask.txBuf[3] = (uint8_t)((MSLA001B_ALS0_PW_VALUE & 0xFF00) >> 8);
	mTask.txBuf[4] = MSLA001B_ALS1_GAIN_VALUE;
	mTask.txBuf[5] = (uint8_t)(MSLA001B_ALS0_PW_VALUE & 0x00FF);
	mTask.txBuf[6] = (uint8_t)((MSLA001B_ALS0_PW_VALUE & 0xFF00) >> 8);

    osLog(LOG_INFO, "msla001b_set_als2_ctrl：mTask.txBuf[1]=0x%x,mTask.txBuf[2]=0x%x,mTask.txBuf[3]=0x%x\n",mTask.txBuf[1],mTask.txBuf[2],mTask.txBuf[3]);

     // 2023 02 23
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 7 ,i2cCallBack, next_state);

}

static int msla001b_set_intell_wait_ps(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.txBuf[0] = MSLA001B_REG_PSPULSEWIDTH;
	mTask.txBuf[1] = MSLA001B_PPW_VALUE; // MSLA001B_REG_PSPULSEWIDTH
    mTask.txBuf[2] = MSLA001B_PBC_VALUE; // MSLA001B_REG_PSBURSTCOUNT
    mTask.txBuf[3] = MSLA001B_LEDDRV_VALUE; // MSLA001B_REG_LEDDRIVERCTRL
    mTask.txBuf[4] = MSLA001B_PS_INTCTRL_STATIC_VALUE;
    osLog(LOG_INFO, "msla001b_set_intell_wait_ps!\n");
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 5,
                       i2cCallBack, next_state);
}
								
static int msla001b_set_ps_thdl(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    if (mTask.hw->polling_mode_ps == 0) {
        mTask.txBuf[0] = MSLA001B_REG_PSTHRESHOLD_L_L;
        mTask.txBuf[1] = (uint8_t)(mTask.ps_threshold_low  & 0x00FF);
        mTask.txBuf[2] = (uint8_t)((mTask.ps_threshold_low & 0xFF00)>> 8);
        osLog(LOG_INFO, "msla001b_set_ps_thdl!txBuf[1]=0x%x,txBuf[2]=0x%x\n",mTask.txBuf[1],mTask.txBuf[2]);
        return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 3,
                           i2cCallBack, next_state);
    } else {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
        return 0;
    }
}


static int msla001b_set_ps_thdh(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    if (mTask.hw->polling_mode_ps == 0) {
        mTask.txBuf[0] = MSLA001B_REG_PSTHRESHOLD_H_L;
        mTask.txBuf[1] = (uint8_t)(mTask.ps_threshold_high  & 0x00FF);
        mTask.txBuf[2] = (uint8_t)((mTask.ps_threshold_high & 0xFF00)>> 8);

        osLog(LOG_INFO, "msla001b_set_ps_thdh!txBuf[1]=0x%x,txBuf[2]=0x%x\n",mTask.txBuf[1],mTask.txBuf[2]);
        return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 3,
                           i2cCallBack, next_state);
    } else {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
        return 0;
    }
}

static void msla001b_eint_handler(int arg)
{
    osLog(LOG_ERROR, "msla001b msla001b_eint_handler enter!!!\n");
    alsPsInterruptOccur();
}

static int msla001b_setup_eint(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                             void *inBuf, uint8_t inSize, uint8_t elemInSize,
                             void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_INFO, "%s: \n"__func__);
    mt_eint_dis_hw_debounce(mTask.hw->eint_num);
    if(mTask.hw->polling_mode_ps == 0)
        mt_eint_registration(mTask.hw->eint_num, LEVEL_SENSITIVE, LOW_LEVEL_TRIGGER, msla001b_eint_handler, EINT_INT_UNMASK,
		EINT_INT_AUTO_UNMASK_OFF);

    psSetCalibration(NULL,0);

    mTask.txBuf[0] = MSLA001B_REG_INTERRUPUT_CTRL_1;
    mTask.txBuf[1] = 0x01;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack, next_state);
}

static void msla001bSetDebugTrace(int32_t trace)
{
    int i, ret = 0;

    for(i=0; i<11;i++){
        mTask.txBuf[0] = i;
        ret = i2cMasterTxRxSync(mTask.hw->i2c_num,mTask.i2c_addr,mTask.txBuf,1,mTask.rxBuf,1,NULL,NULL);
        if(ret < 0){
            osLog(LOG_ERROR, "msla001b i2cMasterTxRxSync FAIL!!!\n");
                  i2cMasterRelease(mTask.hw->i2c_num);
        }
        osLog(LOG_ERROR, "reg %x value %x\n",i,mTask.rxBuf[0]);
    }
}

static int msla001b_register_core(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    struct sensorCoreInfo mInfo;
    memset(&mInfo, 0x00, sizeof(struct sensorCoreInfo));

    /* Register sensor Core */
    mInfo.sensType = SENS_TYPE_ALS;
    mInfo.getData = alsGetData;
    mInfo.getSensorInfo = alsGetSensorInfo;
    sensorCoreRegister(&mInfo);
    mInfo.sensType = SENS_TYPE_PROX,
    mInfo.gain = 1;
    mInfo.sensitivity = 1;
    mInfo.getCalibration = psGetCalibration;
    mInfo.setCalibration = psSetCalibration;
    mInfo.getThreshold = psGetThreshold;
    mInfo.setThreshold = psSetThreshold;
    mInfo.getData = psGetData;
    mInfo.setDebugTrace = msla001bSetDebugTrace;
    mInfo.getSensorInfo = psGetSensorInfo;
    sensorCoreRegister(&mInfo);
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    osLog(LOG_INFO, "msla001b_register_core!\n");
    return 0;
}



static struct sensorFsm msla001bFsm[] = {
    /* sample als */
    sensorFsmCmd(STATE_SAMPLE_ALS, STATE_GET_ALS_DATA, msla001b_read_als_data),
    sensorFsmCmd(STATE_GET_ALS_DATA, STATE_GET_ALS_DGAIN, msla001b_get_als_value),
    sensorFsmCmd(STATE_GET_ALS_DGAIN, STATE_ALS_SECOND_INTELLI_WAIT, msla001b_get_als_dgain),
    sensorFsmCmd(STATE_ALS_SECOND_INTELLI_WAIT, STATE_SAMPLE_ALS_DONE, tran_msla001b_set_second_intelli),

    /* sample ps*/
    sensorFsmCmd(STATE_SAMPLE_PS, STATE_GET_PS_FLG, msla001b_read_ps),
    sensorFsmCmd(STATE_GET_PS_FLG, STATE_GET_PS_RAW_DATA, msla001b_get_ps_status),
    sensorFsmCmd(STATE_GET_PS_RAW_DATA, STATE_CLR_INT, msla001b_get_ps_raw_data),
    sensorFsmCmd(STATE_CLR_INT, STATE_INTERRUPUT_CTRL1, msla001b_clr_int),
    sensorFsmCmd(STATE_INTERRUPUT_CTRL1, STATE_EN_EINT, msla001b_interrupt_ctrl1),
    sensorFsmCmd(STATE_EN_EINT, STATE_SAMPLE_PS_DONE, msla001b_enable_eint),

    /* als enable state */
    sensorFsmCmd(STATE_ALS_ENABLE, STATE_0x80_Set0, msla001b_0x80_Read),            // new set
    sensorFsmCmd(STATE_0x80_Set0, STATE_SET_INIT_DGAIN, msla001b_get_als2_state),  // new set 
    sensorFsmCmd(STATE_SET_INIT_DGAIN, STATE_ALS_POWER_ON, msla001b_set_init_dgain),
    sensorFsmCmd(STATE_ALS_POWER_ON, STATE_ALS_SET_DEBOUNCE, msla001b_set_als_power_on),
    sensorFsmCmd(STATE_ALS_SET_DEBOUNCE, STATE_ALS_ENABLE_DONE, msla001b_set_als_debounce_on),

    /* als disable state */
    sensorFsmCmd(STATE_ALS_DISABLE, STATE_ALS_POWER_OFF, msla001b_get_alsps_state),
    sensorFsmCmd(STATE_ALS_POWER_OFF, STATE_ALS_DISABLE_DONE, msla001b_set_als_power_off),



    /* ps enable state */
    sensorFsmCmd(STATE_PS_ENABLE,STATE_0x80_Read, msla001b_get_alsps_state), // new set
    sensorFsmCmd(STATE_0x80_Read, STATE_0x80_Set0x81, msla001b_0x80_Read),
    sensorFsmCmd(STATE_0x80_Set0x81, STATE_Read_5_ps , msla001b_0x80_Set0x81),
    sensorFsmCmd(STATE_Read_5_ps  , STATE_Loop_5_ps  , msla001b_Read_5_ps),         // new set
    sensorFsmCmd(STATE_Loop_5_ps  , STATE_0x80_Set0x01  , msla001b_Loop_5_ps),      // new set
    sensorFsmCmd(STATE_0x80_Set0x01,STATE_PS_SET_DEBOUNCE, msla001b_0x80_Set0x01),  // new set
    sensorFsmCmd(STATE_PS_SET_DEBOUNCE, STATE_PS_UNMASK_EINT, msla001b_set_ps_debounce_on),
    sensorFsmCmd(STATE_PS_UNMASK_EINT, STATE_PS_ENABLE_DONE, msla001b_unmask_eint),

    /* ps disable state */
    sensorFsmCmd(STATE_PS_DISABLE, STATE_GET_ALSPS_STATE, msla001b_mask_eint),
    sensorFsmCmd(STATE_GET_ALSPS_STATE, STATE_PS_POWER_OFF, msla001b_get_alsps_state),
    sensorFsmCmd(STATE_PS_POWER_OFF, STATE_PS_DISABLE_DONE, msla001b_set_ps_power_off),

    /* als rate change */
    sensorFsmCmd(STATE_ALS_RATECHG, STATE_ALS_RATECHG_DONE, msla001b_als_ratechg),
    /* als cali state */
    sensorFsmCmd(STATE_ALS_CALI, CHIP_ALS_CALI_DONE, msla001b_als_cali),
    /* als cfg state */
    sensorFsmCmd(STATE_ALS_CFG, CHIP_ALS_CFG_DONE, msla001b_als_cfg),
    /* ps rate change */
    sensorFsmCmd(STATE_PS_RATECHG, STATE_PS_RATECHG_DONE, msla001b_ps_ratechg),
    /* ps cali state */
    sensorFsmCmd(STATE_PS_CALI, CHIP_PS_CALI_DONE, msla001b_ps_cali),
    /* ps cfg state */
    sensorFsmCmd(STATE_PS_CFG, CHIP_PS_CFG_DONE, msla001b_ps_cfg),
    
#ifdef CFG_ONCE_TIMER_SUPPORT
    /* ps sample data once, for interrupt mode */
    sensorFsmCmd(STATE_SAMPLE_PS_ONCE, STATE_GET_PS_STATUS_ONCE, msla001b_read_ps_once),
    sensorFsmCmd(STATE_GET_PS_STATUS_ONCE, STATE_GET_PS_RAW_DATA_ONCE, msla001b_get_ps_status_once),
    sensorFsmCmd(STATE_GET_PS_RAW_DATA_ONCE, STATE_SAMPLE_PS_DONE, msla001b_get_ps_raw_data_once),
#endif // CFG_ONCE_TIMER_SUPPORT

#ifdef CFG_FAE_TIMER_SUPPORT
	sensorFsmCmd(STATE_SAMPLING_FAE_PS, STATE_TUNE_FAE, msla001b_get_ps_data),
	//sensorFsmCmd(STATE_GET_INVALID_FLAG, STATE_GET_BGIR_RAW_DATA, msla001b_get_invalid_flag),
	//sensorFsmCmd(STATE_GET_BGIR_RAW_DATA, STATE_GET_PS_OFF_DATA, msla001b_get_bgir_raw_data),
	//sensorFsmCmd(STATE_GET_PS_OFF_DATA, STATE_TUNE_FAE, msla001b_get_ps_off_data),
	sensorFsmCmd(STATE_TUNE_FAE, STATE_SAMPLING_FAE_PS_DONE, msla001b_tune_fae),
#endif

    /* init state */
    sensorFsmCmd(STATE_RESET, STATE_SET_SW_RST, msla001b_set_sw_reset),
    sensorFsmCmd(STATE_SET_SW_RST, STATE_CLR_POR, msla001b_set_frame_time),
    sensorFsmCmd(STATE_CLR_POR, STATE_GET_SPEC, msla001b_clr_por_flag),
    sensorFsmCmd(STATE_GET_SPEC, STATE_SET_SPEC, msla001b_spec_addr_state),
    sensorFsmCmd(STATE_SET_SPEC, STATE_SET_ALSPS_CTRL,msla001b_spec_addr_cfg),
    sensorFsmCmd(STATE_SET_ALSPS_CTRL, STATE_SET_PS_THDH, msla001b_set_alsps_ctrl),
    sensorFsmCmd(STATE_SET_PS_THDH, STATE_SET_PS_THDL, msla001b_set_ps_thdh),
    sensorFsmCmd(STATE_SET_PS_THDL, STATE_SET_ALS2_CTRL, msla001b_set_ps_thdl),
    sensorFsmCmd(STATE_SET_ALS2_CTRL, STATE_SET_INTELL_WAIT_PS, msla001b_set_als2_ctrl),
	sensorFsmCmd(STATE_SET_INTELL_WAIT_PS, STATE_SETUP_EINT, msla001b_set_intell_wait_ps),
    sensorFsmCmd(STATE_SETUP_EINT, STATE_CORE, msla001b_setup_eint),
    sensorFsmCmd(STATE_CORE, STATE_INIT_DONE, msla001b_register_core),
};

static int msla001bInit(void)
{
    int ret = 0;
    osLog(LOG_INFO, "%s: task starting\n", __func__);
    msla001bDebugPoint = &mTask;

    mTask.hw = get_cust_alsps("msla001b");
    if (NULL == mTask.hw) {
        osLog(LOG_ERROR, "get_cust_acc_hw fail\n");
        return -1;
    }
    mTask.i2c_addr = mTask.hw->i2c_addr[0];
    mTask.als_debounce_time = 200;
    mTask.als_debounce_on = 0;
    mTask.ps_debounce_time = 10;
    mTask.ps_debounce_on = 0;
    mTask.psCali = 0;
    mTask.tran_count = 0;
    mTask.alsCali = 1000;
    mTask.als_threshold_high = mTask.hw->als_threshold_high;
    mTask.als_threshold_low = mTask.hw->als_threshold_low;
    mTask.ps_threshold_high = mTask.hw->ps_threshold_high;
    mTask.ps_threshold_low = mTask.hw->ps_threshold_low;
    mTask.state_val = 0; /* Standby */
	mTask.ps_first_read = 1;
    mTask.als_gain = mTask.hw->als_gain;
    mTask.als_integration_time = MSLA001B_ALS0_PW_VALUE; //mTask.hw->als_integration_time;
   // if(mTask.hw->als_integrate_gain == 0)
   //    mTask.hw->als_integrate_gain = 4;
    mTask.als_integrate_gain = MSLA001B_ALS0_GAIN_VALUE;
    mTask.als_integ_ratio = 1; //  mTask.als_integrate_gain;
    i2cMasterRequest(mTask.hw->i2c_num, I2C_SPEED);

    mTask.txBuf[0] = MSLA001B_REG_PID;
    ret = i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
        &mTask.deviceId, 1, NULL, NULL);
    if (ret < 0) {
        osLog(LOG_ERROR, "msla001b i2cMasterTxRxSync fail!!!\n");
        ret = -1;
        i2cMasterRelease(mTask.hw->i2c_num);
        goto err_out;
    }
    if (mTask.deviceId == MSLA001B_PID) {  
        mTask.ledctrl_val = MSLA001B_LEDDRV_VALUE;
        osLog(LOG_INFO, "msla001b: auto detect success:0x%x\n", mTask.deviceId);
        goto success_out;
    } else {
        mTask.ledctrl_val = MSLA001B_LEDDRV_VALUE;
        i2cMasterRelease(mTask.hw->i2c_num);
        osLog(LOG_ERROR, "msla001b: read id fail!!!\n");
        ret = -1;
        goto err_out;
    }
success_out:
    alsSensorRegister();
    psSensorRegister();
    registerAlsPsDriverFsm(msla001bFsm, ARRAY_SIZE(msla001bFsm));
     if(mTask.hw->polling_mode_ps)
     {
        osLog(LOG_ERROR, "PS_POLLING_MODE!\n");
        registerPsInterruptMode(PS_POLLING_MODE);
     }
    else
    {
         osLog(LOG_ERROR, "PS_INTERRUPT_MODE!\n");
        registerPsInterruptMode(PS_INTERRUPT_MODE);
    }
err_out:
    return ret;
}

#ifndef CFG_OVERLAY_INIT_SUPPORT
MODULE_DECLARE(msla001b, SENS_TYPE_ALS, msla001bInit);
#else
#include "mtk_overlay_init.h"
OVERLAY_DECLARE(msla001b, OVERLAY_ID_ALSPS, msla001bInit);
#endif
