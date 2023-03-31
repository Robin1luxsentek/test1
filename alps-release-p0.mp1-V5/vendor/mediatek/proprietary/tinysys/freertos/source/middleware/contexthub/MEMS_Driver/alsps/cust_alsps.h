#ifndef __CUST_ALSPS_H__
#define __CUST_ALSPS_H__

#define C_CUST_ALS_LEVEL    16
#define C_CUST_I2C_ADDR_NUM 2


#define C_CUST_PS_SET_NUM 3


#define MAX_THRESHOLD_HIGH 0xffff
#define MIN_THRESHOLD_LOW 0x0

struct alsps_hw {
    const char *name;
    int i2c_num;                                    /*!< the i2c bus used by ALS/PS */
    int polling_mode_ps;                               /*!< 1: polling mode ; 0:interrupt mode*/
    int polling_mode_als;                               /*!< 1: polling mode ; 0:interrupt mode*/
    unsigned char   i2c_addr[C_CUST_I2C_ADDR_NUM];  /*!< i2c address list, some chip will have multiple address */
    unsigned int    als_level[C_CUST_ALS_LEVEL -
                              1]; /*!< (C_CUST_ALS_LEVEL-1) levels divides all range into C_CUST_ALS_LEVEL levels*/
    unsigned int    als_value[C_CUST_ALS_LEVEL];    /*!< the value reported in each level */
    unsigned int    ps_threshold_high;
    unsigned int    ps_threshold_low;
    unsigned int    als_threshold_high;
    unsigned int    als_threshold_low;
    unsigned int    eint_num;

    unsigned int   als_gain;
    unsigned int   als_integration_time;
    unsigned int   als_integrate_gain;

    unsigned int    ps_setting[C_CUST_PS_SET_NUM];

};
struct alsps_hw *get_cust_alsps(const char *name);
#endif