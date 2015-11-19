 /*
 * NT99142_A20_driver
 * 20150304_Derek
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-mediabus.h>//linux-3.0
#include <linux/io.h>
//#include <mach/gpio_v2.h>
#include <mach/sys_config.h>
#include <linux/regulator/consumer.h>
#include <mach/system.h>
//#include "../../../../power/axp_power/axp-gpio.h"
#if defined CONFIG_ARCH_SUN4I
#include "../include/sun4i_csi_core.h"
#include "../include/sun4i_dev_csi.h"
#elif defined CONFIG_ARCH_SUN5I
#include "../include/sun5i_csi_core.h"
#include "../include/sun5i_csi_core.h"
#include "../include/sun5i_dev_csi.h"
#endif


#include "../include/sunxi_csi_core.h"
#include "../include/sunxi_dev_csi.h"
#include <linux/gpio.h>


MODULE_AUTHOR("raymonxiu");
MODULE_DESCRIPTION("A low-level driver for samsung nt99142 sensors");
MODULE_LICENSE("GPL");

//for internel driver debug
#define DEV_DBG_EN   		1
#if(DEV_DBG_EN == 1)		
#define csi_dev_dbg(x,arg...) printk(KERN_INFO"[CSI_DEBUG][nt99142]"x,##arg)
#else
#define csi_dev_dbg(x,arg...) 
#endif
#define csi_dev_err(x,arg...) printk(KERN_INFO"[CSI_ERR][nt99142]"x,##arg)
#define csi_dev_print(x,arg...) printk(KERN_INFO"[CSI][nt99142]"x,##arg)


///add by Yuyali
#define LOG_ERR_RET(x)  { \
							int ret; \ 
							ret = x; \    
							if(ret < 0){\
								csi_dev_err("error at %s\n",__func__);  \
								return ret; \ 
								} \ 
						}



#define MCLK (24*1000*1000)
#define VREF_POL	CSI_HIGH
#define HREF_POL	CSI_HIGH
#define CLK_POL		CSI_RISING
#define IO_CFG		0						//0 for csi0
#define V4L2_IDENT_SENSOR 0x141

//define the voltage level of control signal
#define CSI_STBY_ON			1
#define CSI_STBY_OFF 		0
#define CSI_RST_ON			0
#define CSI_RST_OFF			1
#define CSI_PWR_ON			1
#define CSI_PWR_OFF			0
#define CSI_AF_PWR_ON		0
#define CSI_AF_PWR_OFF	1

#define REG_TERM 0xff
#define VAL_TERM 0xff
//#define GAIN_V 0x27//------------GAIN的最大值
#define GAIN_V 0x20

#define SHI_DA  0

#define  FPS25  1

#define MIRROR 0//0 = normal, 1 = mirror
#define FLIP   0//0 = normal, 1 = flip


#define REG_ADDR_STEP 2
#define REG_DATA_STEP 1
#define REG_STEP 			(REG_ADDR_STEP+REG_DATA_STEP)

#define CONTINUEOUS_AF

/*
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */
#define QSXGA_WIDTH		2560
#define QSXGA_HEIGHT	1920
#define QXGA_WIDTH 		2048
#define QXGA_HEIGHT		1536
#define HD1080_WIDTH	1920
#define HD1080_HEIGHT	1080
#define UXGA_WIDTH		1600
#define UXGA_HEIGHT		1200
#define SXGA_WIDTH		1280
#define SXGA_HEIGHT		960
#define HD720_WIDTH 	1280
#define HD720_HEIGHT	720
#define XGA_WIDTH			1024
#define XGA_HEIGHT 		768
#define SVGA_WIDTH		800
#define SVGA_HEIGHT 	600
#define VGA_WIDTH			640
#define VGA_HEIGHT		480
#define QVGA_WIDTH		320
#define QVGA_HEIGHT		240
#define CIF_WIDTH			352
#define CIF_HEIGHT		288
#define QCIF_WIDTH		176
#define	QCIF_HEIGHT		144
#define HE 360



/*
 * Our nominal (default) frame rate.
 */
#define SENSOR_FRAME_RATE 30

/*
 * The nt99142 sits on i2c with ID 0x54
 */
#define I2C_ADDR 0x54

/* Registers */

struct cfg_array {
/* coming later */	
struct regval_list * regs;
int size;

};

static int sensor_s_band_filter(struct v4l2_subdev *sd, 
		enum v4l2_power_line_frequency value);

		
/*
 * Information we maintain about a known sensor.
 */
struct sensor_format_struct;  /* coming later */
struct snesor_colorfx_struct; /* coming later */
__csi_subdev_info_t ccm_info_con = 
{
	.mclk 	= MCLK,
	.vref 	= VREF_POL,
	.href 	= HREF_POL,
	.clock	= CLK_POL,
	.iocfg	= IO_CFG,
};

struct sensor_info {
	struct v4l2_subdev sd;
	struct sensor_format_struct *fmt;  /* Current format */
	__csi_subdev_info_t *ccm_info;
	int	width;
	int	height;
	unsigned int capture_mode;		//0:video 1:capture
	unsigned int coarse_af_pd;		//0:busy 1:ok 2:failed
	unsigned int focus_status;		//0:idle 1:busy
	int brightness;
	int	contrast;
	int saturation;
	int hue;
	int sharpness;
	int hflip;
	int vflip;
	int gain;
	int autogain;
	int exp;
	enum v4l2_exposure_auto_type autoexp;
	int autowb;
	enum v4l2_whiteblance wb;
	enum v4l2_colorfx clrfx;
	enum v4l2_flash_mode flash_mode;
	enum v4l2_power_line_frequency band_filter;
	enum v4l2_autofocus_mode af_mode;
	enum v4l2_autofocus_ctrl af_ctrl;
	struct v4l2_fract tpf;			
};

static inline struct sensor_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sensor_info, sd);
}



/*
 * The default register settings
 *
 */

struct regval_list {
	unsigned short reg_num;
	unsigned char value;
};




static struct regval_list sensor_default_regs[] = {
    //**************************************
    //common
    //NT99142DVRSetting
    //20150304
    //**************************************
	0x3069, 0x01, //Driving
	0x306A, 0x03,
	0x3012, 0x02,                                                        
	0x3013, 0x00, 
	0x30A0, 0x03, 
	0x30A1, 0x23,                                                        
	0x30A2, 0x70,                                                        
	0x30A3, 0x01,                                                                                                               
	0x303E, 0x04,                                                        
	0x303F, 0x32, 
	0x3051, 0x3A,                                                        
	0x3052, 0x0F, 
	0x3055, 0x00,                                                        
	0x3056, 0x18,                                                        
	0x305F, 0x33, 
	0x308B, 0x2F,   // HG_BLC_Thr_U
	0x308C, 0x28,   // HG_BLC_Thr_L
	0x308D, 0x30,   // HG_ABLC_Ofs_Cst
	0x308E, 0x3A,   // HG_ABLC_Ofs_Wgt
	0x308F, 0x0E,   // HG_OB_Mul	
	0x3100, 0x03,                                                        
	0x3101, 0x00,                                                        
	0x3102, 0x0A,                                                        
	0x3103, 0x00,                                                        
	0x3105, 0x03,                                                        
	0x3106, 0x04,                                                        
	0x3107, 0x10,                                                        
	0x3108, 0x00,                                                        
	0x3109, 0x00,                                                        
	0x307D, 0x00,                                                        
	0x310A, 0x05,                                                        
	0x310C, 0x00,                                                        
	0x310D, 0x80,                                                        
	0x3110, 0x33,                                                        
	0x3111, 0x59,                                                        
	0x3112, 0x44,                                                        
	0x3113, 0x66,                                                        
	0x3114, 0x66,                                                        
	0x311D, 0x40,                                                        
	0x3127, 0x01,                                                        
	0x3129, 0x44,                                                        
	0x3136, 0x59,                                                        
	0x313F, 0x02,                                                                                                              
	0x3268, 0x01,   
	0x32B3, 0x80,
	0x32B4, 0x20,
	0x32B8, 0x06,
	0x32B9, 0x06,
	0x32BC, 0x3A,
	0x32BD, 0x04,
	0x32BE, 0x04,	
	0x32CB, 0x20,                                                        
	0x32CC, 0x70,                                                        
	0x32CD, 0xA0,                                                        
	0x3297, 0x03,                                                        
	0x324F, 0x01,                                                        
	0x3210, 0x16,                                                        
	0x3211, 0x16,                                                        
	0x3212, 0x16,                                                        
	0x3213, 0x16,                                                        
	0x3230, 0x00,                                                        
	0x3233, 0x00,                                                        
	0x3234, 0x00,                                                        
	0x3237, 0x00,                                                        
	0x324F, 0x00,                                                        
	0x3210, 0x15,                                                        
	0x3211, 0x15,                                                        
	0x3212, 0x15,                                                        
	0x3213, 0x14,                                                        
	0x3214, 0x0F,                                                        
	0x3215, 0x0F,                                                        
	0x3216, 0x0F,                                                        
	0x3217, 0x0F,                                                        
	0x3218, 0x0F,                                                        
	0x3219, 0x0F,                                                        
	0x321A, 0x0F,                                                        
	0x321B, 0x0F,                                                        
	0x321C, 0x0F,                                                        
	0x321D, 0x0F,                                                        
	0x321E, 0x10,                                                        
	0x321F, 0x11,                                                        
	0x3230, 0x03,                                                        
	0x3231, 0x00,                                                        
	0x3232, 0x00,                                                        
	0x3233, 0x06,                                                        
	0x3234, 0x00,                                                        
	0x3235, 0x00,                                                        
	0x3236, 0x00,                                                        
	0x3237, 0x00,                                                        
	0x3238, 0x24,                                                        
	0x3239, 0x24,                                                        
	0x323A, 0x24,                                                        
	0x3243, 0xC2,                                                        
	0x3244, 0x00,                                                        
	0x3245, 0x00,                                                        
	0x3302, 0x00,	   //Color correction and transform matrix                
	0x3303, 0x4F,                                                        
	0x3304, 0x00,                                                        
	0x3305, 0x89,                                                        
	0x3306, 0x00,                                                        
	0x3307, 0x27,                                                        
	0x3308, 0x07,                                                        
	0x3309, 0xCC,                                                        
	0x330A, 0x06,                                                        
	0x330B, 0xDE,                                                        
	0x330C, 0x01,                                                        
	0x330D, 0x56,                                                        
	0x330E, 0x00,                                                        
	0x330F, 0xCF,                                                        
	0x3310, 0x07,                                                        
	0x3311, 0x3E,                                                        
	0x3312, 0x07,                                                        
	0x3313, 0xF3,                                                        
	0x3270, 0x04,    //Gamma                                                
	0x3271, 0x10,                                                        
	0x3272, 0x1C,                                                        
	0x3273, 0x32,                                                        
	0x3274, 0x44,                                                        
	0x3275, 0x54,                                                        
	0x3276, 0x70,                                                        
	0x3277, 0x88,                                                        
	0x3278, 0x9D,                                                        
	0x3279, 0xB0,                                                        
	0x327A, 0xCE,                                                        
	0x327B, 0xE2,                                                        
	0x327C, 0xEF,                                                        
	0x327D, 0xF7,                                                        
	0x327E, 0xFF,                                                        
	0x3250, 0x28,   //CA window                                           
	0x3251, 0x20,                                                        
	0x3252, 0x2F,                                                        
	0x3253, 0x20,                                                        
	0x3254, 0x33,                                                        
	0x3255, 0x26,                                                        
	0x3256, 0x25,                                                        
	0x3257, 0x19,                                                        
	0x3258, 0x44,                                                        
	0x3259, 0x35,                                                        
	0x325A, 0x24,                                                        
	0x325B, 0x16,                                                        
	0x325C, 0xA0,                                                        
	0x3326, 0x10,	  //Eext_Mul                                             
	0x3368, 0x38,   //Edge_Enhance_0                                     
	0x3369, 0x30,   //Edge_Enhance_1                                     
	0x336B, 0x14,   //Edge_Enhance_2                                     
	0x3327, 0x00,   //Eext_Sel                                           
	0x3332, 0x80,   //EMapB                                              
	0x3363, 0x31,   //IQ_Auto_Ctrl                                       
	0x3360, 0x10,   //IQ_Param_Sel_0 2x                                  
	0x3361, 0x20,   //IQ_Param_Sel_1 4x                                  
	0x3362, 0x28,   //IQ_Param_Sel_2 6x                                  
	0x3365, 0x06,   //EMapA_0                                            
	0x3366, 0x08,   //EMapA_1                                            
	0x3367, 0x0A,   //EMapA_2                                            
	0x336D, 0x1e,   //NR_DPC_Ratio_0                                     
	0x336E, 0x12,   //NR_DPC_Ratio_1                                     
	0x3370, 0x09,   //NR_DPC_Ratio_2                                     
	0x3371, 0x20,   //NR_Wgt parameter 0  1  2                           
	0x3372, 0x28,                                                        
	0x3374, 0x38,                                                        
	0x3379, 0x04,	  //NR_Comp_Max parameter 0  1  2                        
	0x337A, 0x06,                                                        
	0x337C, 0x08,                                                        
	0x33A9, 0x02,   //NR_Post_Thr parameter 0  1  2                      
	0x33AA, 0x04,                                                        
	0x33AC, 0x06,                                                        
	0x33AD, 0x04,   //NR_Post_EThr parameter 0  1  2                     
	0x33AE, 0x06,                                                        
	0x33B0, 0x06,                                                        
	0x33A1, 0x80,                                                        
	0x33A2, 0x00,                                                        
	0x33A3, 0x38,                                                        
	0x33A4, 0x00,                                                        
	0x33A5, 0x80,                                                        
	0x33A7, 0x12,                                                        
	0x33C0, 0x01,                                                        
	0x33B1, 0x44,                                                        
	0x33B4, 0x8A,                                                        
	0x33B9, 0x03,                                                        
	0x3710, 0x07,                                                        
	0x371E, 0x02,                                                        
	0x371F, 0x02,                                                        
	0x3364, 0x09,                                                        
	0x33BD, 0x00,                                                        
	0x33BE, 0x08,                                                        
	0x33BF, 0x10,  
	0x3700, 0x10,  //Gamma  
	0x3701, 0x1B,
	0x3702, 0x25,
	0x3703, 0x37,
	0x3704, 0x48,
	0x3705, 0x56,
	0x3706, 0x6F,
	0x3707, 0x85,
	0x3708, 0x98,
	0x3709, 0xAA,
	0x370A, 0xC6,
	0x370B, 0xD7,
	0x370C, 0xE3,
	0x370D, 0xEB,
	0x370E, 0xF0,                                                      
	0x32F1, 0x05,                                                        
	0x33B6, 0xA0,                                                        
	0x3813, 0x07,                                                        
	0x3262, 0x51,                                                        
	0x3290, 0x77,                                                        
	0x3292, 0x73,                                                        
	0x3022, 0x03,  
    0x3800, 0x00,
    0x3813, 0x07,	
	0x32B0, 0x58,                                                        
	0x32B1, 0xAF,                                                        
	0x32B2, 0x14,                                                        
	0x32FC, 0x00,   
};
                                                       	    
//static struct regval_list sensor_qsxga_regs[] = {
//};
//
//static struct regval_list sensor_qxga_regs[] = { //qxga: 2048*1536
//};                                      



static struct regval_list sensor_720p_30fps_regs[] = {
    //MCLK:      24.00MHz 
    //PCLK:      72.00MHz 
    //Size:      1280x720 
    //FPS:       25.00~30.02 
    //Line:      1586 
    //Frame:      756 
    //Flicker:   50Hz 
    0x32BB, 0x67,  //AE Start
    0x32B8, 0x06, 
    0x32B9, 0x06, 
    0x32BC, 0x3A, 
    0x32BD, 0x04, 
    0x32BE, 0x04, 
    0x32BF, 0x60, 
    0x32C0, 0x60, 
    0x32C1, 0x60, 
    0x32C2, 0x60, 
    0x32C3, 0x00, 
    0x32C4, 0x2F, 
    0x32C5, 0x20, 
    0x32C6, 0x20, 
    0x32D3, 0x00, 
    0x32D4, 0xE3, 
    0x32D5, 0x7C, 
    0x32D6, 0x00, 
    0x32D7, 0xBD, 
    0x32D8, 0x77,  //AE End
    0x32F0, 0x01,  //Output Format
    0x3200, 0x7E,  //Mode Control
    0x3201, 0x3D,  //Mode Control
    0x302A, 0x80,  //PLL Start
    0x302C, 0x17, 
    0x302D, 0x11,  //PLL End
    0x3022, 0x03,  //Timing Start
    0x300A, 0x06, 
    0x300B, 0x32, 
    0x300C, 0x02, 
    0x300D, 0xF4,  //Timing End
    0x320A, 0x00, 
    0x3021, 0x02, 
    0x3060, 0x01, 
};


static struct regval_list sensor_720p_25fps_regs[] = {
    //MCLK:      24.00MHz 
    //PCLK:      60.00MHz 
    //Size:      1280x720 
    //FPS:       20.00~25.02 
    //Line:      1586 
    //Frame:      756 
    //Flicker:   50Hz 
    0x32BB, 0x67,  //AE Start
    0x32B8, 0x06, 
    0x32B9, 0x06, 
    0x32BC, 0x3A, 
    0x32BD, 0x04, 
    0x32BE, 0x04,
    0x32BF, 0x60, 
    0x32C0, 0x64, 
    0x32C1, 0x64, 
    0x32C2, 0x64, 
    0x32C3, 0x00, 
    0x32C4, 0x2F, 
    0x32C5, 0x20, 
    0x32C6, 0x20,
    0x32D3, 0x00, 
    0x32D4, 0xBD, 
    0x32D5, 0x77, 
    0x32D6, 0x00, 
    0x32D7, 0x9E, 
    0x32D8, 0x73,  //AE End
    0x32F0, 0x01,  //Output Format
    0x3200, 0x7E,  //Mode Control
    0x3201, 0x3D,  //Mode Control
    0x302A, 0x80,  //PLL Start
    0x302C, 0x13, 
    0x302D, 0x11,  //PLL End
    0x3022, 0x03,  //Timing Start
    0x300A, 0x06, 
    0x300B, 0x32, 
    0x300C, 0x02, 
    0x300D, 0xF4,  //Timing End
    0x320A, 0x00, 
    0x3021, 0x02, 
    0x3060, 0x01, 
};

static struct regval_list sensor_720p_15fps_regs[] = { 
    //MCLK:      24.00MHz 
    //PCLK:      40.00MHz 
    //Size:      1280x720 
    //FPS:       15.00~15.01 
    //Line:      1586 
    //Frame:      840 
    //Flicker:   50Hz 
    0x32BB, 0x67,  //AE Start
    0x32B8, 0x06, 
    0x32B9, 0x06, 
    0x32BC, 0x3A, 
    0x32BD, 0x04, 
    0x32BE, 0x04, 
    0x32BF, 0x60, 
    0x32C0, 0x6A, 
    0x32C1, 0x6A, 
    0x32C2, 0x6A, 
    0x32C3, 0x00, 
    0x32C4, 0x2F, 
    0x32C5, 0x20, 
    0x32C6, 0x20,
    0x32D3, 0x00, 
    0x32D4, 0x7E, 
    0x32D5, 0x6F, 
    0x32D6, 0x00, 
    0x32D7, 0x69, 
    0x32D8, 0x6A,  //AE End
    0x32F0, 0x01,  //Output Format
    0x3200, 0x7E,  //Mode Control
    0x3201, 0x3D,  //Mode Control
    0x302A, 0x80,  //PLL Start
    0x302C, 0x13, 
    0x302D, 0x21,  //PLL End
    0x3022, 0x03,  //Timing Start
    0x300A, 0x06, 
    0x300B, 0x32, 
    0x300C, 0x03, 
    0x300D, 0x48,  //Timing End
    0x320A, 0x00, 
    0x3021, 0x02, 
    0x3060, 0x01, 
};

static struct regval_list sensor_720p_5fps_regs[] = { 
    //MCLK:      24.00MHz 
    //PCLK:      24.00MHz 
    //Size:      1280x720 
    //FPS:       5.00~5.00 
    //Line:      1586 
    //Frame:     1513 
    //Flicker:   50Hz 
    0x32BB, 0x67,  //AE Start
    0x32B8, 0x06, 
    0x32B9, 0x06, 
    0x32BC, 0x3A, 
    0x32BD, 0x04, 
    0x32BE, 0x04,
    0x32BF, 0x60, 
    0x32C0, 0x84, 
    0x32C1, 0x84, 
    0x32C2, 0x84, 
    0x32C3, 0x00, 
    0x32C4, 0x2F, 
    0x32C5, 0x20, 
    0x32C6, 0x20,
    0x32D3, 0x00, 
    0x32D4, 0x4C, 
    0x32D5, 0x63, 
    0x32D6, 0x00, 
    0x32D7, 0x3F, 
    0x32D8, 0x5F,  //AE End
    0x32F0, 0x01,  //Output Format
    0x3200, 0x7E,  //Mode Control
    0x3201, 0x3D,  //Mode Control
    0x302A, 0x80,  //PLL Start
    0x302C, 0x0F, 
    0x302D, 0x13,  //PLL End
    0x3022, 0x03,  //Timing Start
    0x300A, 0x06, 
    0x300B, 0x32, 
    0x300C, 0x05, 
    0x300D, 0xE9,  //Timing End
    0x320A, 0x00, 
    0x3021, 0x02, 
    0x3060, 0x01, 

};


static struct regval_list sensor_vga_regs[] = { //VGA:  640*480
    //MCLK:      24.00MHz 
    //PCLK:      72.00MHz 
    //Size:      640x480 
    //FPS:       30.00~30.02 
    //Line:      1586 
    //Frame:      756 
    //Flicker:   50Hz 
    0x32BB, 0x67,  //AE Start
    0x32B8, 0x06, 
    0x32B9, 0x06, 
    0x32BC, 0x3A, 
    0x32BD, 0x04, 
    0x32BE, 0x04, 
    0x32BF, 0x60, 
    0x32C0, 0x5A, 
    0x32C1, 0x5A, 
    0x32C2, 0x5A, 
    0x32C3, 0x00, 
    0x32C4, 0x2F, 
    0x32C5, 0x20, 
    0x32C6, 0x20,
    0x32D3, 0x00, 
    0x32D4, 0xE3, 
    0x32D5, 0x7C, 
    0x32D6, 0x00, 
    0x32D7, 0xBD, 
    0x32D8, 0x77,  //AE End
    0x32E0, 0x02,  //Scale Start
    0x32E1, 0x80, 
    0x32E2, 0x01, 
    0x32E3, 0xE0, 
    0x32E4, 0x00, 
    0x32E5, 0x80, 
    0x32E6, 0x00, 
    0x32E7, 0x80,  //Scale End
    0x32F0, 0x11,  //Output Format
    0x3200, 0x7E,  //Mode Control
    0x3201, 0x7D,  //Mode Control
    0x302A, 0x80,  //PLL Start
    0x302C, 0x17, 
    0x302D, 0x11,  //PLL End
    0x3022, 0x03,  //Timing Start
    0x300A, 0x06, 
    0x300B, 0x32, 
    0x300C, 0x02, 
    0x300D, 0xF4,  //Timing End
    0x320A, 0x38, 
    0x3021, 0x02, 
    0x3060, 0x01, 
	

};

#if 0
//misc
static struct regval_list sensor_oe_disable_regs[] = {
};
#endif

#if 0
//stuff about auto focus
static struct regval_list sensor_af_fw_regs[] = {
};
#endif

static struct regval_list sensor_ae_awb_lockon_regs[] = {

};

static struct regval_list sensor_ae_awb_lockoff_regs[] = {

};

static struct regval_list sensor_af_single_trig_regs[] = {

};

static struct regval_list sensor_af_continueous_regs[] = {

};

static struct regval_list sensor_af_infinity_regs[] = {

};

static struct regval_list sensor_af_macro_regs[] = {

};



/*
 * The white balance settings
 * Here only tune the R G B channel gain. 
 * The white balance enalbe bit is modified in sensor_s_autowb and sensor_s_wb
 */
static struct regval_list sensor_wb_auto_regs[] = {
	0x3206, 0x03,
	0x3060, 0x04,

};

static struct regval_list sensor_wb_daylight_regs[] = {
	////WB sunny
	0x3206, 0x02,
	0x3290, 0x4E,
	0x3292, 0x5A,
	0x3060, 0x04,
};

static struct regval_list sensor_wb_cloud_regs[] = {	
	////WB Cloudy
	0x3206, 0x02,
	0x3290, 0x45,
	0x3292, 0x40,
	0x3060, 0x04,

};



static struct regval_list sensor_wb_incandescence_regs[] = {
	////bai chi deng
	0x3206, 0x02,
	0x3290, 0x4C,
	0x3292, 0x72,
	0x3060, 0x04,


};

static struct regval_list sensor_wb_fluorescent_regs[] = {
	////WB Office
	0x3206, 0x02,
	0x3290, 0x5C,
	0x3292, 0x7F,
	0x3060, 0x04,

};

static struct regval_list sensor_wb_tungsten_regs[] = {
	////WB HOME
	0x3206, 0x02,
	0x3290, 0x40,
	0x3292, 0x8C,
	0x3060, 0x04,
};


/************************************
*******By Yuyali,20140913***********
*************************************/
static struct cfg_array sensor_wb[] = {
  
  {
  	.regs = sensor_wb_auto_regs,          //V4L2_WHITE_BALANCE_AUTO      
    .size = ARRAY_SIZE(sensor_wb_auto_regs),
  },
  {
  	.regs = sensor_wb_cloud_regs,         //V4L2_WHITE_BALANCE_CLOUDY       
    .size = ARRAY_SIZE(sensor_wb_cloud_regs),
  },
  {
  	.regs = sensor_wb_daylight_regs,      //V4L2_WHITE_BALANCE_DAYLIGHT     
    .size = ARRAY_SIZE(sensor_wb_daylight_regs),
  },
  
  {
  	.regs = sensor_wb_incandescence_regs, //V4L2_WHITE_BALANCE_INCANDESCENT 
    .size = ARRAY_SIZE(sensor_wb_incandescence_regs),
  },
  {
  	.regs = sensor_wb_fluorescent_regs,   //V4L2_WHITE_BALANCE_FLUORESCENT  
    .size = ARRAY_SIZE(sensor_wb_fluorescent_regs),
  },

/*	
  {
  	.regs = sensor_wb_tungsten_regs,      //V4L2_WHITE_BALANCE_FLUORESCENT_H
    .size = ARRAY_SIZE(sensor_wb_tungsten_regs),
  },
  {
  	.regs = sensor_wb_horizon,            //V4L2_WHITE_BALANCE_HORIZON    
    .size = ARRAY_SIZE(sensor_wb_horizon),
  },  
  
  {
  	.regs = sensor_wb_flash,              //V4L2_WHITE_BALANCE_FLASH        
    .size = ARRAY_SIZE(sensor_wb_flash),
  },
  
  {
  	.regs = sensor_wb_shade,              //V4L2_WHITE_BALANCE_SHADE  
    .size = ARRAY_SIZE(sensor_wb_shade),
  },
  */
//  {
//  	.regs = NULL,
//    .size = 0,
//  },
};






/*********************
 * The color effect settings
   By Yuyali,20140913
 **********************************/
static struct regval_list sensor_colorfx_none_regs[] = {
	0x32f1,0x00,
	0x3060, 0x01 ,

};

static struct regval_list sensor_colorfx_bw_regs[] = {
	////sensor_Effect_WandB
	0x32f1, 0x01, 
	0x3060, 0x01,

};

static struct regval_list sensor_colorfx_sepia_regs[] = {

    ////sensor_Effect_Sepia
    0x32f1, 0x02,	
	0x3060, 0x01,

};

static struct regval_list sensor_colorfx_negative_regs[] = {
	////sensor_Effect_Negative	
	0x32f1, 0x03, 
	0x3060, 0x01,

};

static struct regval_list sensor_colorfx_emboss_regs[] = {
//NULL
};

static struct regval_list sensor_colorfx_sketch_regs[] = {
	//NULL
};

static struct regval_list sensor_colorfx_sky_blue_regs[] = {
	////sensor_Effect_Bluish
	0x32f1, 0x05,
	0x32f4, 0xF0,
	0x32f5, 0x80,
	0x3060, 0x04,

};

static struct regval_list sensor_colorfx_grass_green_regs[] = { 
	0x32f1, 0x05,
	0x32f4, 0x90,
	0x32f5, 0x80,
	0x3060, 0x04,
};

static struct regval_list sensor_colorfx_skin_whiten_regs[] = {
//NULL
};

static struct regval_list sensor_colorfx_vivid_regs[] = {
//NULL
};

/*
 * The power frequency
 */
static struct regval_list sensor_flicker_50hz_regs[] = {

	0x32BB, 0x67,
	0x3060, 0x01,
};

static struct regval_list sensor_flicker_60hz_regs[] = {
	0x32BB, 0x77,
	0x3060, 0x01,
};

/************************
 * The brightness setttings
    By Yuyali,20140913
 *****************************/
static struct regval_list sensor_brightness_neg4_regs[] = {
	//// Brightness -4
	0x32fc , 0x80,
	0x3060 , 0x01,

};

static struct regval_list sensor_brightness_neg3_regs[] = {
	//// Brightness -3
	0x32fc , 0xa0,
	0x3060 , 0x01,


};

static struct regval_list sensor_brightness_neg2_regs[] = {
	//// Brightness -2
	0x32fc , 0xc0,
	0x3060 , 0x01,


};

static struct regval_list sensor_brightness_neg1_regs[] = {
	//// Brightness -1
	0x32fc , 0xe0,
	0x3060 , 0x01,


};

static struct regval_list sensor_brightness_zero_regs[] = {

    ////  Brightness 0
	0x32fc , 0x00,
	0x3060 , 0x01,

};

static struct regval_list sensor_brightness_pos1_regs[] = {

    //// Brightness +1
    0x32fc , 0x1c,
    0x3060 , 0x01,

};

static struct regval_list sensor_brightness_pos2_regs[] = {
    ////  Brightness +2
	0x32fc , 0x40,
	0x3060 , 0x01,
};

static struct regval_list sensor_brightness_pos3_regs[] = {

    ////  Brightness +3
	0x32fc,0x60,
	0x3060,0x01,

};

static struct regval_list sensor_brightness_pos4_regs[] = {

    0x32fc , 0x7f,
	0x3060 , 0x01,

};
static struct cfg_array sensor_brightness[] = {
  {
  	.regs = sensor_brightness_neg4_regs,
  	.size = ARRAY_SIZE(sensor_brightness_neg4_regs),
  },
  {
  	.regs = sensor_brightness_neg3_regs,
  	.size = ARRAY_SIZE(sensor_brightness_neg3_regs),
  },
  {
  	.regs = sensor_brightness_neg2_regs,
  	.size = ARRAY_SIZE(sensor_brightness_neg2_regs),
  },
  {
  	.regs = sensor_brightness_neg1_regs,
  	.size = ARRAY_SIZE(sensor_brightness_neg1_regs),
  },
  {
  	.regs = sensor_brightness_zero_regs,
  	.size = ARRAY_SIZE(sensor_brightness_zero_regs),
  },
  {
  	.regs = sensor_brightness_pos1_regs,
  	.size = ARRAY_SIZE(sensor_brightness_pos1_regs),
  },
  {
  	.regs = sensor_brightness_pos2_regs,
  	.size = ARRAY_SIZE(sensor_brightness_pos2_regs),
  },
  {
  	.regs = sensor_brightness_pos3_regs,
  	.size = ARRAY_SIZE(sensor_brightness_pos3_regs),
  },
  {
  	.regs = sensor_brightness_pos4_regs,
  	.size = ARRAY_SIZE(sensor_brightness_pos4_regs),
  },
};


/****************
 * 20150304
 ************************/


static struct regval_list sensor_contrast_zero_regs[] = {
	0x32F2, 0x90,
	0x32fc, 0xe0,
	0x3060, 0x01, 

};

static struct regval_list sensor_contrast_pos1_regs[] = {
	0x32F2, 0x88,
	0x32fc, 0xf0,
	0x3060, 0x01, 


};

static struct regval_list sensor_contrast_pos2_regs[] = {
	0x32F2, 0x80,
	0x32fc, 0x00,
	0x3060, 0x01, 

};

static struct regval_list sensor_contrast_pos3_regs[] = {
	0x32F2, 0x78,
	0x32fc, 0x90,
	0x3060, 0x01, 

};

static struct regval_list sensor_contrast_pos4_regs[] = {
	0x32F2, 0x70,
	0x32fc, 0xA0,
	0x3060, 0x01, 
};

static struct regval_list sensor_contrast_pos5_regs[] = {
	0x32F2, 0x68,
	0x32fc, 0xB0,
	0x3060, 0x01, 
};

static struct cfg_array sensor_contrast[] = { 
	
	
	{  
		.regs = sensor_contrast_zero_regs, 
		.size = ARRAY_SIZE(sensor_contrast_zero_regs), 
	},
	{  	
		.regs = sensor_contrast_pos1_regs,
		.size = ARRAY_SIZE(sensor_contrast_pos1_regs),
	},
	{  	
		.regs = sensor_contrast_pos2_regs,  
		.size = ARRAY_SIZE(sensor_contrast_pos2_regs), 
	},
	{  	
		.regs = sensor_contrast_pos3_regs,  
		.size = ARRAY_SIZE(sensor_contrast_pos3_regs),
	},
	{  
		.regs = sensor_contrast_pos4_regs,
		.size = ARRAY_SIZE(sensor_contrast_pos4_regs),
	}, 
	{  	
		.regs = sensor_contrast_pos5_regs, 
		.size = ARRAY_SIZE(sensor_contrast_pos5_regs),

	},
};
/*
 * The saturation setttings
 */
static struct regval_list sensor_saturation_neg4_regs[] = {

};

static struct regval_list sensor_saturation_neg3_regs[] = {

};

static struct regval_list sensor_saturation_neg2_regs[] = {

};

static struct regval_list sensor_saturation_neg1_regs[] = {

};

static struct regval_list sensor_saturation_zero_regs[] = {

};

static struct regval_list sensor_saturation_pos1_regs[] = {

};

static struct regval_list sensor_saturation_pos2_regs[] = {

};

static struct regval_list sensor_saturation_pos3_regs[] = {

};

static struct regval_list sensor_saturation_pos4_regs[] = {

};

/*
 * 20150304
 */

static struct regval_list sensor_ev_neg3_regs[] = {
	0x32F2,0x20,
	0x3060,0x01,

};

static struct regval_list sensor_ev_neg2_regs[] = {
	0x32F2,0x40,
	0x3060,0x01,

};

static struct regval_list sensor_ev_neg1_regs[] = {
    //
	0x32F2,0x60,
	0x3060,0x01,

};                     

static struct regval_list sensor_ev_zero_regs[] = {
	0x32F2,0x80,
	0x3060,0x01,
};

static struct regval_list sensor_ev_pos1_regs[] = {
	0x32F2,0xA0,
	0x3060,0x01,
};

static struct regval_list sensor_ev_pos2_regs[] = {
	0x32F2,0xC0,
	0x3060,0x01,

};

static struct regval_list sensor_ev_pos3_regs[] = {
	0x32F2,0xE0,
	0x3060,0x01,

};
/*********
****By Yuyali, 20140913
**********/
static struct cfg_array sensor_exposure[] = { 
	
	
	{  
		.regs = sensor_ev_neg3_regs, 
		.size = ARRAY_SIZE(sensor_ev_neg3_regs), 
	},
	{  	
		.regs = sensor_ev_neg2_regs,
		.size = ARRAY_SIZE(sensor_ev_neg2_regs),
	},
	{  	
		.regs = sensor_ev_neg1_regs,  
		.size = ARRAY_SIZE(sensor_ev_neg1_regs), 
	},
	{  	
		.regs = sensor_ev_zero_regs,  
		.size = ARRAY_SIZE(sensor_ev_zero_regs),
	},
	{  
		.regs = sensor_ev_pos1_regs,
		.size = ARRAY_SIZE(sensor_ev_pos1_regs),
	}, 
	{  	
		.regs = sensor_ev_pos2_regs, 
		.size = ARRAY_SIZE(sensor_ev_pos2_regs),

	},
	{  	
		.regs = sensor_ev_pos3_regs, 
		.size = ARRAY_SIZE(sensor_ev_pos3_regs),

	},
};



/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 * 
 */


static struct regval_list sensor_fmt_yuv422_yuyv[] = {	
{0xffff,0x10}
   
};

static struct regval_list sensor_fmt_yuv422_yvyu[] = {
{0xffff,0x10}
};

static struct regval_list sensor_fmt_yuv422_vyuy[] = {
{0xffff,0x10}
};

static struct regval_list sensor_fmt_yuv422_uyvy[] = {
{0xffff,0x10}
};

//static struct regval_list sensor_fmt_raw[] = {
//	
//};


/*
 * The IPC setttings
 * add by chenzhixiang
 * 	brightness ->address:0x32FC default:0x00
 *  contrast   ->address:0x32F2 default:0x80
 * 	saturation ->address:0x32F3 default:0x80
 * 	hue        ->address:0x32FB default:0x5A
 */
static struct regval_list sensor_brightness_neg_regs[] = {
	{0x32FC, 0x12}, //default:0x00  bright(0x00-0x40-0x7f) dark(0x00-0xc0-0x80)
	{0x3060, 0x01}, //Activate setting for specified registers
};

static struct regval_list sensor_contrast_neg_regs[] = {
	{0x32F1, 0x05},//User define and Hue adjust enable
	{0x32F2, 0x73},//Y_Component default:0x80 Max(0xFF),Min(0x00) 
	{0x3060, 0x01}, //Activate setting for specified registers
};

static struct regval_list sensor_saturation_neg_regs[] = {
	{0x32F1, 0x05},//User define and Hue adjust enable
	{0x32F3, 0x00},//Chroma_Component default:0x80 Max(0xFF),Min(0x00) 
	{0x3060, 0x01}, //Activate setting for specified registers
};

static struct regval_list sensor_hue_neg_regs[] = {
	{0x32F1, 0x05}, //User define and Hue adjust enable
	{0x32FB, 0x00}, //Hue_Angle_Shift default:0x5A, Max(0xB4),Min(0x00) 
};

static struct regval_list sensor_sharpness_neg_regs[] = {
	{0x3326, 0x05}, 
};




/*
 * Low-level register I/O.
 *
 */


/*
 * On most platforms, we'd rather do straight i2c I/O.
 */
#if 0
static int sensor_read(struct v4l2_subdev *sd, unsigned char *reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 data[REG_STEP];
	struct i2c_msg msg;
	int ret,i;
	
	for(i = 0; i < REG_ADDR_STEP; i++)
		data[i] = reg[i];
	
	for(i = REG_ADDR_STEP; i < REG_STEP; i++)
		data[i] = 0xff;
	/*
	 * Send out the register address...
	 */
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = REG_ADDR_STEP;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		csi_dev_err("Error %d on register write\n", ret);
		return ret;
	}
	/*
	 * ...then read back the result.
	 */
	
	msg.flags = I2C_M_RD;//#define I2C_M_RD		0x0001	/* read data, from slave to master */
	msg.len = REG_DATA_STEP;
	msg.buf = &data[REG_ADDR_STEP];
	
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0) {
		for(i = 0; i < REG_DATA_STEP; i++)
			value[i] = data[i+REG_ADDR_STEP];
		ret = 0;
	}
	else {
		csi_dev_err("Error %d on register read\n", ret);
	}
	return ret;
}
#endif

static int sensor_read_im(struct v4l2_subdev *sd, unsigned int addr,
		unsigned int *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 data[REG_STEP];
	struct i2c_msg msg;
	int ret,i,j;
	
	for(i = 0, j = REG_ADDR_STEP-1; i < REG_ADDR_STEP; i++,j--)
		data[i] = (addr&(0xff<<(j*8)))>>(j*8);
	
	for(i = REG_ADDR_STEP; i < REG_STEP; i++)
		data[i] = 0xff;
	/*
	 * Send out the register address...
	 */
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = REG_ADDR_STEP;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		csi_dev_err("Error %d on register write\n", ret);
		return ret;
	}
	/*
	 * ...then read back the result.
	 */
	
	msg.flags = I2C_M_RD;
	msg.len = REG_DATA_STEP;
	msg.buf = &data[REG_ADDR_STEP];
	
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0) {
		for(i = 0,j = REG_DATA_STEP-1; i < REG_DATA_STEP; i++,j--)
			*((unsigned char*)(value)+j) = data[i+REG_ADDR_STEP];
//		*value = data[2]*256+data[3];
		ret = 0;
	}
	else {
		csi_dev_err("Error %d on register read\n", ret);
	}
	return ret;
}


static int sensor_write(struct v4l2_subdev *sd, unsigned char *reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg;
	unsigned char data[REG_STEP];
	int ret,i;
	
	for(i = 0; i < REG_ADDR_STEP; i++)
			data[i] = reg[i];
	for(i = REG_ADDR_STEP; i < REG_STEP; i++)
			data[i] = value[i-REG_ADDR_STEP];
	
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = REG_STEP;
	msg.buf = data;

	
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0) {
		ret = 0;
	}
	else if (ret < 0) {
		csi_dev_err("addr = 0x%4x, value = 0x%4x\n ",reg[0]*256+reg[1],value[0]*256+value[1]);
		csi_dev_err("sensor_write error!\n");
	}
	return ret;
}

static int sensor_write_im(struct v4l2_subdev *sd, unsigned int addr,
		unsigned int value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg;
	unsigned char data[REG_STEP];
	int ret=0;
	unsigned i,j;
	
	for(i = 0, j = REG_ADDR_STEP-1; i < REG_ADDR_STEP; i++,j--) {
			data[i] = (addr&(0xff<<(j*8)))>>(j*8);
	}
			
	for(i = REG_ADDR_STEP,j = REG_DATA_STEP-1; i < REG_STEP; i++,j--)
			data[i] = *((unsigned char*)(&value)+j);
	
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = REG_STEP;
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0) {
		ret = 0;
	}
	else if (ret < 0) {
		csi_dev_err("addr = 0x%4x, value = 0x%4x\n ",addr,value);
		csi_dev_err("sensor_write error!\n");
	}
	return ret;
}


/*
 * Write a list of register settings;
 */
static int sensor_write_array(struct v4l2_subdev *sd, struct regval_list *vals , uint size)
{
	int i,ret;
//	unsigned char rd;
	if (size == 0)
		return -EINVAL;

	for(i = 0; i < size ; i++)
	{
		if(vals->reg_num == 0xffff) {
			msleep(vals->value);
		}	
		else {	
			ret = sensor_write_im(sd, vals->reg_num, vals->value);
			if (ret < 0)
			{
				csi_dev_err("sensor_write_err!\n");
				return ret;
			}
		}
		vals++;
	}
	
	return 0;
}

#if 0
static int sensor_write_continuous(struct v4l2_subdev *sd, int addr, char vals[] , uint size)
{
	int i,ret;
	struct regval_list reg_addr;
	
	if (size == 0)
		return -EINVAL;
	
	for(i = 0; i < size ; i++)
	{
		reg_addr.reg_num[0] = (addr&0xff00)>>8;
		reg_addr.reg_num[1] = (addr&0x00ff);
		
		ret = sensor_write(sd, reg_addr.reg_num, &vals[i]);
		if (ret < 0)
		{
			csi_dev_err("sensor_write_err!\n");
			return ret;
		}
		addr++;
	}
	
	return 0;
}
#endif

/* stuff about exposure when capturing image */

static int sensor_set_exposure(struct v4l2_subdev *sd)
{	
	return 0;
}

/* stuff about auto focus */



static int sensor_download_af_fw(struct v4l2_subdev *sd)
{
  return 0;
}

static int sensor_ae_awb_lockon(struct v4l2_subdev *sd)
{
	int ret=0;
	
	ret = sensor_write_array(sd, sensor_ae_awb_lockon_regs , ARRAY_SIZE(sensor_ae_awb_lockon_regs));
	if(ret < 0)
		csi_dev_err("sensor_ae_awb_lockon error!\n"); 
	
	return ret;
};

static int sensor_ae_awb_lockoff(struct v4l2_subdev *sd)
{
	int ret=0;
	
	ret = sensor_write_array(sd, sensor_ae_awb_lockoff_regs , ARRAY_SIZE(sensor_ae_awb_lockoff_regs));
	if(ret < 0)
		csi_dev_err("sensor_ae_awb_lockoff error!\n"); 
	
	return ret;
};

static int sensor_g_single_af(struct v4l2_subdev *sd)
{
	unsigned int rdval;
	int ret=0;
	struct sensor_info *info = to_state(sd);
	
	csi_dev_dbg("sensor_g_single_af\n");
	csi_dev_dbg("info->coarse_af_pd = %d\n",info->coarse_af_pd);
	if(info->coarse_af_pd != 1) {
		//wait for 1st af complete
	  rdval = 0xffff;
	  sensor_write_im(sd, 0x002c , 0x7000);
		sensor_write_im(sd, 0x002e , 0x2eee);
	  ret =  sensor_read_im(sd, 0x0f12 , &rdval);
		if (ret < 0)
		{
			csi_dev_err("sensor_g_single_af read error !\n");
			ret = -EAGAIN;
			goto af_out;
		}
	
	  if(rdval == 0x0001 ) {
			csi_dev_dbg("Single AF 1st is busy,value = 0x%4x\n",rdval);
			msleep(50);
			info->coarse_af_pd = 0;
			return EBUSY;
		} else if (rdval == 0x0002) {
			//focus ok
			info->coarse_af_pd = 1;
			csi_dev_print("Single AF 1st is complete,value = 0x%4x\n",rdval);
		} else {
			csi_dev_print("Single AF 1st is failed,value = 0x%4x\n",rdval);
			info->focus_status = 0;	//idle
			info->coarse_af_pd = 2;
			ret = EFAULT;
			goto af_out;
		}
	}
	
	//wait for 2nd af complete
	rdval = 0xffff;
	sensor_write_im(sd, 0x002c , 0x7000);
	sensor_write_im(sd, 0x002e , 0x2207);
	ret =  sensor_read_im(sd, 0x0f12 , &rdval);
	if (ret < 0)
	{
		csi_dev_err("sensor_g_single_af read error !\n");
		goto af_out;
	}
	
//	while((rdval&0xff00)!=0x0000){
//		sensor_read_im(sd, 0x0f12 , &rdval);
//		msleep(50);
//	}

	if((rdval&0xff00)!=0x0000) {
		csi_dev_dbg("Single AF 2nd is busy,value = 0x%4x\n",rdval);
		return EBUSY;
	}
	
	csi_dev_print("Single AF 2nd is complete,value = 0x%4x\n",rdval);
	info->focus_status = 0;	//idle
	
	ret = 0;
af_out:	
	
	//ae lock off
	sensor_ae_awb_lockoff(sd);
	
	return ret;
}

static int sensor_s_single_af(struct v4l2_subdev *sd)
{
  int ret=0;
//	struct sensor_info *info = to_state(sd);
//#if 0
//	unsigned int rdval,cnt;	
//#endif

	csi_dev_print("sensor_s_single_af\n");
	
//	info->focus_status = 0;	//idle
//	info->coarse_af_pd = 0;
//	//ae lock on
//	sensor_ae_awb_lockon(sd);
//	
//	//single trigger	
//	ret = sensor_write_array(sd, sensor_af_single_trig_regs , ARRAY_SIZE(sensor_af_single_trig_regs));
//	if(ret < 0) {
//		csi_dev_err("sensor_s_single_af error!\n"); 
//    //ae lock off
//		sensor_ae_awb_lockoff(sd);
//  } else if(ret == 0) {
//  	info->focus_status = 1;	//busy
//  }
//	
//	//msleep(100);
//	
//#if 0	
//	//wait for 1st af complete
//
//  rdval = 0xffff;
//	cnt = 0;
//	while(rdval!=0x0002)
//	{
//		ret = sensor_write_im(sd, 0x002c , 0x7000);
//		ret = sensor_write_im(sd, 0x002e , 0x2eee);
//		if(ret < 0)
//		{
//			csi_dev_err("sensor_s_single_af error!\n"); 
//	    goto af_out;
//	  }
//		ret =  sensor_read_im(sd, 0x0f12 , &rdval);
//  	if (ret < 0)
//  	{
//  		csi_dev_err("sensor_s_single_af read error !\n");
//  		goto af_out;
//  	}
//  	csi_dev_dbg("Single AF 1st ,value = 0x%4x\n",rdval);
//  	
//  	if(rdval!=0x0001 && rdval!=0x0002) {
//  		csi_dev_err("Single AF 1st is failed,value = 0x%4x\n",rdval);
//			ret = -EFAULT;
//  		goto af_out;
//  	} else {
//			mdelay(15);
//			cnt++;
//			if(cnt>60) {
//				csi_dev_err("Single AF 1st is timeout,value = 0x%4x\n",rdval);
//				ret = -EFAULT;
//	  		goto af_out;
//			}
//		}
//	}
//	csi_dev_print("Single AF 1st is complete,value = 0x%4x\n",rdval);
//	
//	//wait for 2nd af complete
//	rdval = 0xffff;
//  cnt = 0;
//	while((rdval&0xff00)!=0x0000)
//	{
//		ret = sensor_write_im(sd, 0x002c , 0x7000);
//		ret = sensor_write_im(sd, 0x002e , 0x2207);
//		if(ret < 0)
//		{
//			csi_dev_err("sensor_s_single_af error!\n"); 
//	    goto af_out;
//	  }
//	  
//		ret =  sensor_read_im(sd, 0x0f12 , &rdval);
//  	if (ret < 0)
//  	{
//  		csi_dev_err("sensor_s_single_af read error !\n");
//  		goto af_out;
//  	}
//  	mdelay(5);
//  	
//		cnt++;
//		if(cnt>300) {
//			csi_dev_err("Single AF 2nd is timeout,value = 0x%4x\n",rdval);
//			ret = -EFAULT;
//  		goto af_out;
//		}
//	}
//	csi_dev_print("Single AF 2nd is complete,value = 0x%4x\n",rdval);
//	
//af_out:	
//	//ae lock off
//	sensor_ae_awb_lockoff(sd);
//#endif	

	return ret;
}

static int sensor_s_continueous_af(struct v4l2_subdev *sd)
{
  int ret=0;

  csi_dev_print("sensor_s_continueous_af\n");
//  ret = sensor_write_array(sd,sensor_af_continueous_regs,ARRAY_SIZE(sensor_af_continueous_regs));
//  if(ret < 0)
//    csi_dev_err("sensor_s_continueous_af error\n");
   
  return ret;
}

static int sensor_s_infinity_af(struct v4l2_subdev *sd)
{
  int ret=0;
  
  csi_dev_print("sensor_s_infinity_af\n");
//  ret = sensor_write_array(sd,sensor_af_infinity_regs,ARRAY_SIZE(sensor_af_infinity_regs));
//  if(ret < 0)
//    csi_dev_err("sensor_s_infinity_af error\n");
   
  return ret;
}

static int sensor_s_macro_af(struct v4l2_subdev *sd)
{
  int ret=0;
  
  csi_dev_print("sensor_s_macro_af\n");
//  ret = sensor_write_array(sd,sensor_af_macro_regs,ARRAY_SIZE(sensor_af_macro_regs));
//  if(ret < 0)
//    csi_dev_err("sensor_af_macro_regs error\n");
   
  return ret;
}

static int sensor_s_pause_af(struct v4l2_subdev *sd)
{
	return 0;
}

static int sensor_s_release_af(struct v4l2_subdev *sd)
{
	return 0;
}

static int sensor_s_af_zone(struct v4l2_subdev *sd, int xc, int yc)
{
//	struct sensor_info *info = to_state(sd);
//	int fst_win_start_x,fst_win_start_y;
//	int fst_disp_start_x,fst_disp_start_y;
//	int fst_win_size_x = 512,fst_win_size_y = 568;
//	int fst_disp_size_x,fst_disp_size_y;
//	int scnd_win_start_x,scnd_win_start_y;
//	int scnd_disp_start_x,scnd_disp_start_y;
//	int scnd_win_size_x = 116,scnd_win_size_y = 306;
//	int scnd_disp_size_x,scnd_disp_size_y;
//	
//	if(info->width == 0 || info->height == 0) {
//		csi_dev_err("current width or height is zero!\n");
//		return -EINVAL;
//	}
//	
//	if(info->focus_status == 1)	//can not set af zone when focus is busy
//		return 0;
//	
//	csi_dev_print("sensor_s_af_zone\n");
//	csi_dev_dbg("af zone input xc=%d,yc=%d\n",xc,yc);	
//	//first window
//	
//	fst_disp_size_x = fst_win_size_x * info->width /1024;
//	
//	if(xc + fst_disp_size_x/2 > info->width) {
//		fst_disp_start_x = info->width - fst_disp_size_x;
//	} else if(xc - (fst_disp_size_x/2) < 0) {
//		fst_disp_start_x = 0;
//	} else {
//		fst_disp_start_x = xc - (fst_disp_size_x/2);
//	}
//	
//	fst_disp_size_y = fst_win_size_y * info->height /1024;
//	if(yc + fst_disp_size_y/2 > info->height) {
//		fst_disp_start_y = info->height - fst_disp_size_y;
//	} else if(yc - fst_disp_size_y/2 < 0) {
//		fst_disp_start_y = 0;
//	} else {
//		fst_disp_start_y = yc - fst_disp_size_y/2;
//	}
//	
//	fst_win_start_x = fst_disp_start_x * 1024 / info->width;
//	fst_win_start_y = fst_disp_start_y * 1024 / info->height;
//	
//	//second window
//	
//	scnd_disp_size_x = scnd_win_size_x * info->width /1024;
//	if(xc + scnd_disp_size_x/2 > info->width) {
//		scnd_disp_start_x = info->width - scnd_disp_size_x;
//	} else if(xc - scnd_disp_size_x/2 < 0) {
//		scnd_disp_start_x = 0;
//	} else {
//		scnd_disp_start_x = xc - scnd_disp_size_x/2;
//	}
//	
//	scnd_disp_size_y = scnd_win_size_y * info->height /1024;
//	if(yc + scnd_disp_size_y/2 > info->height) {
//		scnd_disp_start_y = info->height - scnd_disp_size_y;
//	} else if(yc - scnd_disp_size_y/2 < 0) {
//		scnd_disp_start_y = 0;
//	} else {
//		scnd_disp_start_y = yc - scnd_disp_size_y/2;
//	}
//	
//	scnd_win_start_x = scnd_disp_start_x * 1024 / info->width;
//	scnd_win_start_y = scnd_disp_start_y * 1024 / info->height;
//	
//	sensor_write_im(sd,0x0028,0x7000);
//	sensor_write_im(sd,0x002a,0x0294);
//	sensor_write_im(sd,0x0f12,fst_win_start_x);	//REG_TC_AF_FstWinStartX 
//	sensor_write_im(sd,0x0f12,fst_win_start_y);	//REG_TC_AF_FstWinStartY 
//	sensor_write_im(sd,0x0f12,fst_win_size_x); 	//REG_TC_AF_FstWinSizeX  
//	sensor_write_im(sd,0x0f12,fst_win_size_y); 	//REG_TC_AF_FstWinSizeY  
//	sensor_write_im(sd,0x0f12,scnd_win_start_x); //REG_TC_AF_ScndWinStartX
//	sensor_write_im(sd,0x0f12,scnd_win_start_y);	//REG_TC_AF_ScndWinStartY
//	sensor_write_im(sd,0x0f12,scnd_win_size_x);	//REG_TC_AF_ScndWinSizeX 
//	sensor_write_im(sd,0x0f12,scnd_win_size_y);	//REG_TC_AF_ScndWinSizeY 
//	sensor_write_im(sd,0x0f12,0x0001);						//REG_TC_AF_WinSizesUpdated;
//	
//	csi_dev_dbg("af zone 1st window stx=%d,sty=%d,width=%d,height=%d\n",fst_win_start_x,fst_win_start_y,fst_win_size_x,fst_win_size_y);
//	csi_dev_dbg("af zone 2nd window stx=%d,sty=%d,width=%d,height=%d\n",scnd_win_start_x,scnd_win_start_y,scnd_win_size_x,scnd_win_size_y);
//	
//	mdelay(30);
	return 0;
}

#if 0
static int sensor_s_relaunch_af_zone(struct v4l2_subdev *sd)
{
	return 0;
}
#endif
/*
 * CSI GPIO control
 */
static void csi_gpio_write(struct v4l2_subdev *sd, struct gpio_config *gpio, int level)
{
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
/**		
  if(gpio->port == 0xffff) {
//    axp_gpio_set_io(gpio->port_num, 1);
//    axp_gpio_set_value(gpio->port_num, status); 
  } else {
    gpio_write_one_pin_value(dev->csi_pin_hd,status,(char *)&gpio->gpio_name);
  }
  **/
   if(gpio->gpio==GPIO_INDEX_INVALID)
  {
    //printk("gpio_p=%p,gpio->gpio=%d\n",gpio,gpio->gpio);
    csi_dev_dbg("invalid gpio\n");
    return;
  }

	  gpio_direction_output(gpio->gpio, level);
	  gpio->data=level;
  
}


static void csi_gpio_set_status(struct v4l2_subdev *sd, struct gpio_config *gpio, int status)
{
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
		
		/***
  if(gpio->port == 0xffff) {
//    axp_gpio_set_io(gpio->port_num, status);
  } else {
    gpio_set_one_pin_io_status(dev->csi_pin_hd,status,(char *)&gpio->gpio_name);
  }
  ***/
  	if(1 == status) {  /* output */
		if(0 != gpio_direction_output(gpio->gpio, gpio->data))
			csi_dev_dbg("gpio_direction_output failed\n");
	} else if(0 == status) {  /* input */
	  if(0 != gpio_direction_input(gpio->gpio) )
	    csi_dev_dbg("gpio_direction_input failed\n");
	}
	gpio->mul_sel=status;
}


/*
 * Stuff that knows about the sensor.
 */
 
static int sensor_power(struct v4l2_subdev *sd, int on)
{
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
//	int ret=0;
  
  //insure that clk_disable() and clk_enable() are called in pair 
  //when calling CSI_SUBDEV_STBY_ON/OFF and CSI_SUBDEV_PWR_ON/OFF
  switch(on)
	{
		case CSI_SUBDEV_STBY_ON:
			csi_dev_dbg("CSI_SUBDEV_STBY_ON\n");
			//disable io oe
//			csi_dev_print("disalbe oe!\n");
//			ret = sensor_write_array(sd, sensor_oe_disable_regs , ARRAY_SIZE(sensor_oe_disable_regs));
//			if(ret < 0)
//				csi_dev_err("disalbe oe falied!\n");
			//make sure that no device can access i2c bus during sensor initial or power down
			//when using i2c_lock_adpater function, the following codes must not access i2c bus before calling i2c_unlock_adapter
//			i2c_lock_adapter(client->adapter);
			//reset on io
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			msleep(10);
			//standby on io
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_ON);
			msleep(10);
			//inactive mclk after stadby in
			clk_disable(dev->csi_module_clk);
//			//remember to unlock i2c adapter, so the device can access the i2c bus again
//			i2c_unlock_adapter(client->adapter);	
			break;
		case CSI_SUBDEV_STBY_OFF:
			csi_dev_dbg("CSI_SUBDEV_STBY_OFF\n");
			//make sure that no device can access i2c bus during sensor initial or power down
			//when using i2c_lock_adpater function, the following codes must not access i2c bus before calling i2c_unlock_adapter
//			i2c_lock_adapter(client->adapter);
			//active mclk before stadby out
			clk_enable(dev->csi_module_clk);
			msleep(10);
			//standby off io
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_OFF);
			msleep(10);
			//reset on io
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			msleep(10);
//			//remember to unlock i2c adapter, so the device can access the i2c bus again
//			i2c_unlock_adapter(client->adapter);	
			break;
		case CSI_SUBDEV_PWR_ON:
			csi_dev_dbg("CSI_SUBDEV_PWR_ON\n");
			//make sure that no device can access i2c bus during sensor initial or power down
			//when using i2c_lock_adpater function, the following codes must not access i2c bus before calling i2c_unlock_adapter
			i2c_lock_adapter(client->adapter);
			csi_gpio_set_status(sd,&dev->standby_io,1);//set the gpio to output
			csi_gpio_set_status(sd,&dev->reset_io,1);//set the gpio to output
			//power supply
			csi_gpio_write(sd,&dev->power_io,CSI_PWR_ON);
			if(dev->iovdd) {
				regulator_enable(dev->iovdd);
			}
			if(dev->dvdd) {
				regulator_enable(dev->dvdd);
			}
			if(dev->avdd) {
				regulator_enable(dev->avdd);
			}
//			csi_gpio_write(sd,&dev->af_power_io,CSI_AF_PWR_ON);
//			mdelay(10);
			//power on reset
			csi_gpio_write(sd,&dev->reset_io,0);
			csi_gpio_write(sd,&dev->standby_io,1);
                        	printk("11 00000000000000 \n");
			//active mclk power on reset
			clk_enable(dev->csi_module_clk);
			msleep(30);
			csi_gpio_write(sd,&dev->reset_io,1);
			msleep(30);
		        csi_gpio_set_status(sd,&dev->standby_io,0);//

			printk(" 00000000000000 \n");
			//remember to unlock i2c adapter, so the device can access the i2c bus again
			i2c_unlock_adapter(client->adapter);	
			break;
		case CSI_SUBDEV_PWR_OFF:
			csi_dev_dbg("CSI_SUBDEV_PWR_OFF\n");
			//make sure that no device can access i2c bus during sensor initial or power down
			//when using i2c_lock_adpater function, the following codes must not access i2c bus before calling i2c_unlock_adapter
			i2c_lock_adapter(client->adapter);
			//reset on
			csi_gpio_write(sd,&dev->reset_io,0);
			msleep(10);
			//inactive mclk before power off
			clk_disable(dev->csi_module_clk);
			msleep(10);
			//standy on
			csi_gpio_write(sd,&dev->standby_io,0);		
			//power supply off
//			csi_gpio_write(sd,&dev->af_power_io,CSI_AF_PWR_OFF);
			if(dev->dvdd) {
				regulator_disable(dev->dvdd);
			}
			if(dev->avdd) {
				regulator_disable(dev->avdd);
			}
			if(dev->iovdd) {
				regulator_disable(dev->iovdd);
			}
			csi_gpio_write(sd,&dev->power_io,CSI_PWR_OFF);
			//set the io to hi-z
			csi_gpio_set_status(sd,&dev->reset_io,0);//set the gpio to input
			csi_gpio_set_status(sd,&dev->standby_io,0);//set the gpio to input
			//remember to unlock i2c adapter, so the device can access the i2c bus again
			i2c_unlock_adapter(client->adapter);	
			break;
		default:
			return -EINVAL;
	}		

	return 0;
}
 
static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);

	switch(val)
	{
		case CSI_SUBDEV_RST_OFF:
			csi_dev_dbg("CSI_SUBDEV_RST_OFF\n");
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(5);
			break;
		case CSI_SUBDEV_RST_ON:
			csi_dev_dbg("CSI_SUBDEV_RST_ON\n");
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(5);
			break;
		case CSI_SUBDEV_RST_PUL:
			csi_dev_dbg("CSI_SUBDEV_RST_PUL\n");
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(5);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(5);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(5);
			break;
		default:
			return -EINVAL;
	}
		
	return 0;
}

static int sensor_detect(struct v4l2_subdev *sd)
{
	int ret=0;
	unsigned int rdval=0xffff;
	#if 0
	ret = sensor_write_im(sd, 0x002c, 0x7000);
	ret = sensor_write_im(sd, 0x002e, 0x015e);
	if (ret < 0)
	{
		csi_dev_err("sensor_write err at sensor_detect!\n");
		return ret;
	}

	ret = sensor_read_im(sd, 0x0f12, &rdval);
  csi_dev_dbg("id read from sensor is 0x%4x\n",rdval);
	if(rdval != 0x04a0 )
	{
		csi_dev_err("id read from sensor is 0x%4x,not 0x4ec0\n",rdval);
		return -ENODEV;
	}
	#endif
	return 0;
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret=0;
	struct sensor_info *info = to_state(sd);
	
	csi_dev_dbg("sensor_init\n");
	
	/*Make sure it is a target sensor*/
	ret = sensor_detect(sd);
	if (ret) {
		csi_dev_err("chip found is not an target chip.\n");
		return ret;
	}
	
	ret = sensor_write_array(sd, sensor_default_regs , ARRAY_SIZE(sensor_default_regs));	
	if(ret < 0) {
		csi_dev_err("write sensor_default_regs error\n");
		return ret;
	}
			
	sensor_s_band_filter(sd, V4L2_CID_POWER_LINE_FREQUENCY_50HZ);
	
	info->focus_status = 0;
	info->width = 0;
	info->height = 0;
	info->brightness = 128;
	//info->contrast = 128;
	info->saturation = 128;
	info->hue = 90;
	info->sharpness = 4;
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 0;
	info->autogain = 1;
	info->exp = 0;
	info->autoexp = 0;
	info->autowb = 1;
	info->wb = V4L2_WB_AUTO;
	info->clrfx = V4L2_COLORFX_NONE;
	info->band_filter = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;
	info->af_mode = V4L2_AF_FIXED;
	info->af_ctrl = V4L2_AF_RELEASE;
	info->tpf.numerator = 1;            
	info->tpf.denominator = 30;    /* 30fps */
	info->width = 0;
	info->height = 0;

#if 1	
	unsigned int rdval;
	//sensor_write_im(sd, 0xfffe , 0x0026);
	//sensor_write_im(sd, 0x002e , 0x2207);
	ret =  sensor_read_im(sd, 0x302a , &rdval);
	csi_dev_dbg("---------------nt99142---------init_sensor----%x---\n",rdval);
	ret =  sensor_read_im(sd, 0x3326 , &rdval);
	csi_dev_dbg("---------------nt99142---------init_sensor----%x---\n",rdval);
	ret =  sensor_read_im(sd, 0x3069 , &rdval);
	csi_dev_dbg("---------------nt99142---------init_sensor----%x---\n",rdval);
	ret =  sensor_read_im(sd, 0x306A , &rdval);
	csi_dev_dbg("---------------nt99142---------init_sensor----%x---\n",rdval);
#endif


	
	return 0;
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret=0;
	
	switch(cmd){
		case CSI_SUBDEV_CMD_GET_INFO: 
		{
			struct sensor_info *info = to_state(sd);
			__csi_subdev_info_t *ccm_info = arg;
			
			csi_dev_dbg("CSI_SUBDEV_CMD_GET_INFO\n");
			
			ccm_info->mclk 	=	info->ccm_info->mclk ;
			ccm_info->vref 	=	info->ccm_info->vref ;
			ccm_info->href 	=	info->ccm_info->href ;
			ccm_info->clock	=	info->ccm_info->clock;
			ccm_info->iocfg	=	info->ccm_info->iocfg;
	
			csi_dev_dbg("ccm_info.mclk=%d\n ",info->ccm_info->mclk);
			csi_dev_dbg("ccm_info.vref=%x\n ",info->ccm_info->vref);
			csi_dev_dbg("ccm_info.href=%x\n ",info->ccm_info->href);
			csi_dev_dbg("ccm_info.clock=%x\n ",info->ccm_info->clock);
			csi_dev_dbg("ccm_info.iocfg=%x\n ",info->ccm_info->iocfg);
			break;
		}
		case CSI_SUBDEV_CMD_SET_INFO:
		{
			struct sensor_info *info = to_state(sd);
			__csi_subdev_info_t *ccm_info = arg;
			
			csi_dev_dbg("CSI_SUBDEV_CMD_SET_INFO\n");
			
			info->ccm_info->mclk 	=	ccm_info->mclk 	;
			info->ccm_info->vref 	=	ccm_info->vref 	;
			info->ccm_info->href 	=	ccm_info->href 	;
			info->ccm_info->clock	=	ccm_info->clock	;
			info->ccm_info->iocfg	=	ccm_info->iocfg	;
			
			csi_dev_dbg("ccm_info.mclk=%d\n ",info->ccm_info->mclk);
			csi_dev_dbg("ccm_info.vref=%x\n ",info->ccm_info->vref);
			csi_dev_dbg("ccm_info.href=%x\n ",info->ccm_info->href);
			csi_dev_dbg("ccm_info.clock=%x\n ",info->ccm_info->clock);
			csi_dev_dbg("ccm_info.iocfg=%x\n ",info->ccm_info->iocfg);
			
			break;
		}
		default:
			return -EINVAL;
	}		
		return ret;
}


/*
 * Store information about the video data format. 
 */
static struct sensor_format_struct {
	__u8 *desc;
	//__u32 pixelformat;
	enum v4l2_mbus_pixelcode mbus_code;//linux-3.0
	struct regval_list *regs;
	int	regs_size;
	int bpp;   /* Bytes per pixel */
} sensor_formats[] = {
	{
		.desc		= "YUYV 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YUYV8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_yuyv,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_yuyv),
		.bpp		= 2,
	},
	{
		.desc		= "YVYU 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YVYU8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_yvyu,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_yvyu),
		.bpp		= 2,
	},
	{
		.desc		= "UYVY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_UYVY8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_uyvy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_uyvy),
		.bpp		= 2,
	},
	{
		.desc		= "VYUY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_VYUY8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_vyuy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_vyuy),
		.bpp		= 2,
	},
//	{
//		.desc		= "Raw RGB Bayer",
//		.mbus_code	= V4L2_MBUS_FMT_SBGGR8_1X8,//linux-3.0
//		.regs 		= sensor_fmt_raw,
//		.regs_size = ARRAY_SIZE(sensor_fmt_raw),
//		.bpp		= 1
//	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

	

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */


static struct sensor_win_size {
	int	width;
	int	height;
	int	hstart;		/* Start/stop values for the camera.  Note */
	int	hstop;		/* that they do not always make complete */
	int	vstart;		/* sense to humans, but evidently the sensor */
	int	vstop;		/* will do the right thing... */
	struct regval_list *regs; /* Regs to tweak */
	int regs_size;
	int (*set_size) (struct v4l2_subdev *sd);
/* h/vref stuff */
} sensor_win_sizes[] = {
	/* qsxga: 2592*1936 */
//	{
//		.width			= QSXGA_WIDTH,
//		.height 		= QSXGA_HEIGHT,
//		.regs			  = sensor_qsxga_regs,
//		.regs_size	= ARRAY_SIZE(sensor_qsxga_regs),
//		.set_size		= NULL,
//	},
	/* qxga: 2048*1536 */
//	{
//		.width			= QXGA_WIDTH,
//		.height 		= QXGA_HEIGHT,
//		.regs			  = sensor_qxga_regs,
//		.regs_size	= ARRAY_SIZE(sensor_qxga_regs),
//		.set_size		= NULL,
//	},
	/* 1080P */
//	{
//		.width			= HD1080_WIDTH,
//		.height			= HD1080_HEIGHT,
//		.regs 			= sensor_1080p_regs,
//		.regs_size	= ARRAY_SIZE(sensor_1080p_regs),
//		.set_size		= NULL,
//	},
	/* sxga */
	#if 1
	{
		.width			= HD720_WIDTH,
		.height			= HD720_HEIGHT,
		.regs 			= sensor_720p_30fps_regs,
		.regs_size	= ARRAY_SIZE(sensor_720p_30fps_regs),
		.set_size		= NULL,
	},
	#endif
	/* 720p */
	{
		.width			= HD720_WIDTH,
		.height			= HD720_HEIGHT,
		.regs 			= sensor_720p_25fps_regs,
		.regs_size	= ARRAY_SIZE(sensor_720p_25fps_regs),
		.set_size		= NULL,
	},

	/* VGA */
	{
		.width			= VGA_WIDTH,
		.height			= VGA_HEIGHT,
		.regs				= sensor_vga_regs,
		.regs_size	= ARRAY_SIZE(sensor_vga_regs),
		.set_size		= NULL,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))




static int sensor_enum_fmt(struct v4l2_subdev *sd, unsigned index,
                 enum v4l2_mbus_pixelcode *code)//linux-3.0
{
//	struct sensor_format_struct *ofmt;

	if (index >= N_FMTS)//linux-3.0
		return -EINVAL;

	*code = sensor_formats[index].mbus_code;//linux-3.0
//	ofmt = sensor_formats + fmt->index;
//	fmt->flags = 0;
//	strcpy(fmt->description, ofmt->desc);
//	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}


static int sensor_try_fmt_internal(struct v4l2_subdev *sd,
		//struct v4l2_format *fmt,
		struct v4l2_mbus_framefmt *fmt,//linux-3.0
		struct sensor_format_struct **ret_fmt,
		struct sensor_win_size **ret_wsize)
{
	int index;
	struct sensor_win_size *wsize;
//	struct v4l2_pix_format *pix = &fmt->fmt.pix;//linux-3.0

	for (index = 0; index < N_FMTS; index++)
		if (sensor_formats[index].mbus_code == fmt->code)//linux-3.0
			break;
	
	if (index >= N_FMTS) {
		/* default to first format */
		index = 0;
		fmt->code = sensor_formats[0].mbus_code;//linux-3.0
	}
	
	if (ret_fmt != NULL)
		*ret_fmt = sensor_formats + index;
		
	/*
	 * Fields: the sensor devices claim to be progressive.
	 */
	fmt->field = V4L2_FIELD_NONE;//linux-3.0
	
	
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	for (wsize = sensor_win_sizes; wsize < sensor_win_sizes + N_WIN_SIZES;
	     wsize++)
		if (fmt->width >= wsize->width && fmt->height >= wsize->height)//linux-3.0
			break;
		
	if (wsize >= sensor_win_sizes + N_WIN_SIZES)
		wsize--;   /* Take the smallest one */
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	/*
	 * Note the size we'll actually handle.
	 */
	fmt->width = wsize->width;//linux-3.0
	fmt->height = wsize->height;//linux-3.0
	//pix->bytesperline = pix->width*sensor_formats[index].bpp;//linux-3.0
	//pix->sizeimage = pix->height*pix->bytesperline;//linux-3.0
	
	return 0;
}

static int sensor_try_fmt(struct v4l2_subdev *sd, 
             struct v4l2_mbus_framefmt *fmt)//linux-3.0
{
	return sensor_try_fmt_internal(sd, fmt, NULL, NULL);
}


/*
 * Set a format.
 */
static int sensor_s_fmt(struct v4l2_subdev *sd, 
             struct v4l2_mbus_framefmt *fmt)//linux-3.0
{
	int ret=0;
	struct sensor_format_struct *sensor_fmt;
	struct sensor_win_size *wsize;
	struct sensor_info *info = to_state(sd);
	
	csi_dev_dbg("sensor_s_fmt\n");
	
	ret = sensor_try_fmt_internal(sd, fmt, &sensor_fmt, &wsize);
	if (ret)
		return ret;
	
	if(info->fmt == sensor_fmt && info->width == wsize->width && info->height == wsize->height)
	{
		csi_dev_print("format and size remain the same\n");
		goto update;
	}
	
  
	if(info->capture_mode == V4L2_MODE_VIDEO)
	{
		//video
//		if(info->af_mode != V4L2_AF_FIXED) {
//			ret = sensor_s_release_af(sd);
//			if (ret < 0)
//			{
//				csi_dev_err("sensor_s_release_af err !\n");
//				return ret;
//			}	
//		}
	}
	else if(info->capture_mode == V4L2_MODE_IMAGE)
	{
		//capture
//		ret = sensor_set_exposure(sd);
//		if (ret < 0)
//		{
//			csi_dev_err("sensor_set_exposure err !\n");
//			return ret;
//		}	
	}
	
	ret =  sensor_write_array(sd, sensor_fmt->regs , sensor_fmt->regs_size);
	if (ret < 0)		
		return ret;
	
	ret = 0;
	if (wsize->regs)
	{
		ret = sensor_write_array(sd, wsize->regs , wsize->regs_size);
		if (ret < 0)
			return ret;
	}
	
	if (wsize->set_size)
	{
		ret = wsize->set_size(sd);
		if (ret < 0)
			return ret;
	}
  
//	if(info->capture_mode == V4L2_MODE_VIDEO)
//	{
//		//video
//		if(info->af_mode != V4L2_AF_FIXED) {
//			
//#if 0			
//			if(info->af_mode != V4L2_AF_TOUCH && info->af_mode != V4L2_AF_FACE) {				
//				ret = sensor_s_relaunch_af_zone(sd);	//set af zone to default zone
//				if (ret < 0) {
//					csi_dev_err("sensor_s_relaunch_af_zone err !\n");
//					return ret;
//				}	
//			}
//#endif
//
//#ifdef CONTINUEOUS_AF				
//			if(info->af_mode != V4L2_AF_INFINITY) {
//				ret = sensor_s_continueous_af(sd);		//set continueous af
//				if (ret < 0) {
//					csi_dev_err("sensor_s_continueous_af err !\n");
//					return ret;
//				}
//			}
//		}
//#endif
//	}

update:
	info->fmt = sensor_fmt;
	info->width = wsize->width;
	info->height = wsize->height;
	
	csi_dev_print("s_fmt set width = %d, height = %d\n",wsize->width,wsize->height);
	
	if(1){//fix voffset for 720p
    unsigned int tmpval = 0;
    unsigned int voffset;
    struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
    
	  if(wsize->width==1280&&wsize->height==720)
	    voffset=0x78;
	  else
	    voffset=0x00;
    
    tmpval = *(unsigned int*)(dev->regs+0x44);
    tmpval = (tmpval&0xffff0000)|voffset;
    *(unsigned int*)(dev->regs+0x44)=tmpval;
    
    
    csi_print("patch voffset!");
	
	csi_dev_dbg(" s_fmt  frame rate %d\n",info->tpf.denominator);
	switch(info->tpf.denominator)
	{
		case 30:
			ret = sensor_write_array(sd, sensor_720p_30fps_regs , ARRAY_SIZE(sensor_720p_30fps_regs));
			if(ret>=0)
			{
				csi_dev_dbg("ser rate 30 done\n");
			}
			else
			{
				csi_dev_dbg("ser rate 30 fail\n");
			}
			break;
		case 25:
			ret = sensor_write_array(sd, sensor_720p_25fps_regs , ARRAY_SIZE(sensor_720p_25fps_regs));
			if(ret>=0)
			{
				csi_dev_dbg("ser rate 25 done\n");
			}
			break;
		case 15:
			ret = sensor_write_array(sd, sensor_720p_15fps_regs , ARRAY_SIZE(sensor_720p_15fps_regs));
			if(ret>=0)
			{
				csi_dev_dbg("ser rate 15 done\n");
			}
			break;
		case 5:
			ret = sensor_write_array(sd, sensor_720p_5fps_regs , ARRAY_SIZE(sensor_720p_5fps_regs));
			if(ret>=0)
			{
				csi_dev_dbg("ser rate 5 done\n");
			}
			break;
		default:
			csi_dev_dbg("ser rate default\n");
			break;
	}
	
	}
	return 0;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int sensor_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct sensor_info *info = to_state(sd);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	
	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->capturemode = info->capture_mode;
	
	cp->timeperframe.numerator = info->tpf.numerator;
	cp->timeperframe.denominator = info->tpf.denominator;
	 
	return 0;
}

static int sensor_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;
	struct sensor_info *info = to_state(sd);
	int ret;
	csi_dev_dbg("sensor_s_parm\n");

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (info->tpf.numerator == 0)
		return -EINVAL;

	if (tpf->numerator == 0 || tpf->denominator == 0)	{
		tpf->numerator = 1;
		tpf->denominator = 30;/* Reset to full rate */
		csi_dev_err("sensor frame rate reset to full rate!\n");
	}

	csi_dev_dbg("set frame rate %d\n",tpf->denominator/tpf->numerator);

	switch(tpf->denominator)
	{
		case 30:
			ret = sensor_write_array(sd, sensor_720p_30fps_regs , ARRAY_SIZE(sensor_720p_30fps_regs));
			if(ret>0)
			{
				csi_dev_dbg("ser rate 30 done\n");
			}
			break;
		case 25:
			ret = sensor_write_array(sd, sensor_720p_25fps_regs , ARRAY_SIZE(sensor_720p_25fps_regs));
			if(ret>0)
			{
				csi_dev_dbg("ser rate 25 done\n");
			}
			break;
		case 15:
			ret = sensor_write_array(sd, sensor_720p_15fps_regs , ARRAY_SIZE(sensor_720p_15fps_regs));
			if(ret>0)
			{
				csi_dev_dbg("ser rate 15 done\n");
			}
			break;
		case 5:
			ret = sensor_write_array(sd, sensor_720p_5fps_regs , ARRAY_SIZE(sensor_720p_5fps_regs));
			if(ret>0)
			{
				csi_dev_dbg("ser rate 5 done\n");
			}
			break;
		default:
			csi_dev_dbg("ser rate default\n");
			break;
	}

	info->tpf.denominator = tpf->denominator;
	csi_dev_dbg(" s_parm  frame rate %d\n",info->tpf.denominator);
	return 0;
}


/* 
 * Code for dealing with controls.
 * fill with different sensor module
 * different sensor module has different settings here
 * if not support the follow function ,retrun -EINVAL
 */

/* *********************************************begin of ******************************************** */
static int sensor_queryctrl(struct v4l2_subdev *sd,
		struct v4l2_queryctrl *qc)
{
	/* Fill in min, max, step and default value for these controls. */
	/* see include/linux/videodev2.h for details */
	/* see sensor_s_parm and sensor_g_parm for the meaning of value */

	switch (qc->id) {
	case V4L2_CID_BRIGHTNESS:
		return v4l2_ctrl_query_fill(qc, 0, 255, 1, 0);
	case V4L2_CID_CONTRAST:
		return v4l2_ctrl_query_fill(qc, 0, 255, 1, 0);
	case V4L2_CID_SATURATION:
		return v4l2_ctrl_query_fill(qc, 0, 255, 1, 0);
	case V4L2_CID_HUE:
		return v4l2_ctrl_query_fill(qc, 0, 180, 1, 0);
	case V4L2_CID_SHARPNESS:
		return v4l2_ctrl_query_fill(qc, 0, 7, 1, 0);


////	case V4L2_CID_VFLIP:
////	case V4L2_CID_HFLIP:
////		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
//	case V4L2_CID_GAIN:
//		return v4l2_ctrl_query_fill(qc, 0, 255, 1, 128);
//	case V4L2_CID_AUTOGAIN:
//		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
	case V4L2_CID_EXPOSURE:
		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 0);
//	case V4L2_CID_EXPOSURE_AUTO:
//		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_DO_WHITE_BALANCE:
		return v4l2_ctrl_query_fill(qc, 0, 5, 1, 0);
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
	case V4L2_CID_COLORFX:
		return v4l2_ctrl_query_fill(qc, 0, 9, 1, 0);
////	case V4L2_CID_CAMERA_FLASH_MODE:
////	  return v4l2_ctrl_query_fill(qc, 0, 4, 1, 0);	
////	case V4L2_CID_CAMERA_AF_MODE:
////	  return v4l2_ctrl_query_fill(qc, 0, 5, 1, 0);
////	case V4L2_CID_CAMERA_AF_CTRL:
////	  return v4l2_ctrl_query_fill(qc, 0, 5, 1, 0);
		
	 default:
		 	return 0;
	}
	return 0;
}

static int sensor_g_hflip(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->hflip;
	return 0;
}

static int sensor_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret=0;
//	struct sensor_info *info = to_state(sd);
//	unsigned int pre_val,cap_val;
//	struct regval_list regs[] = {
//		////==================================================================================
//		//// 20.Preview & Capture Configration Setting
//		////==================================================================================
//		//{{0xFC,0xFC},{0xD0,0x00}},
//		//{{0x00,0x28},{0x70,0x00}},
//		//{{0x00,0x2A},{0x02,0xD0}},	
//		//{{0x0F,0x12},{0x00,0x00}},	//REG_0TC_PCFG_uPrevMirror     Original 00 / H Mirror 05 / V Mirror 0A / HV Mirror 0F
//		//{{0x0F,0x12},{0x00,0x00}},	//REG_0TC_PCFG_uCaptureMirror  Original 00 / H Mirror 05 / V Mirror 0A / HV Mirror 0F
//		
//		//==================================================================================
//		// 21.Select Cofigration Display
//		//==================================================================================
//		//PREVIEW
//		{{0xFC,0xFC},{0xD0,0x00}},
//		{{0x00,0x28},{0x70,0x00}},
//		{{0x00,0x2A},{0x02,0x66}},
//		{{0x0f,0x12},{0x00,0x00}},	//REG_TC_GP_ActivePrevConfig
//		{{0x00,0x2A},{0x02,0x6A}},
//		{{0x0F,0x12},{0x00,0x01}},	//REG_TC_GP_PrevOpenAfterChange
//		{{0x00,0x2A},{0x02,0x4E}},
//		{{0x0F,0x12},{0x00,0x01}},	//REG_TC_GP_NewConfigSync
//		{{0x00,0x2A},{0x02,0x68}},
//		{{0x0F,0x12},{0x00,0x01}},	//REG_TC_GP_PrevConfigChanged
//		{{0x00,0x2A},{0x02,0x3E}},
//		{{0x0F,0x12},{0x00,0x01}},	//REG_TC_GP_EnablePreview
//		{{0x0F,0x12},{0x00,0x01}},	//REG_TC_GP_EnablePreviewChanged;
//  };
//
//	if(info->hflip == value)
//		return 0;
//
//	sensor_write_im(sd, 0x002c , 0x7000);
//	sensor_write_im(sd, 0x002e , 0x02d0);
//	sensor_read_im(sd, 0x0f12, &pre_val);
//	sensor_read_im(sd, 0x0f12, &cap_val);
//	sensor_write_im(sd, 0x0028, 0x7000);
//	sensor_write_im(sd, 0x002a, 0x02d0);
//	
//	switch (value) {
//		case 0:
//		  pre_val &= 0xfa;
//		  cap_val &= 0xfa;
//			break;
//		case 1:
//		  pre_val |= 0x05;
//		  cap_val |= 0x05;
//			break;
//		default:
//			return -EINVAL;
//	}
//	
//	sensor_write_im(sd, 0x0f12, pre_val);
//  sensor_write_im(sd, 0x0f12, cap_val);
//  
//	ret = sensor_write_array(sd, regs, ARRAY_SIZE(regs));
//	if (ret < 0) {
//		csi_dev_err("sensor_write err at sensor_s_hflip!\n");
//		return ret;
//	}
//
//	info->hflip = value;
	
	return 0;
}

static int sensor_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->vflip;
	return 0;
}

static int sensor_s_vflip(struct v4l2_subdev *sd, int value)
{
	int ret=0;
//	struct sensor_info *info = to_state(sd);
//	unsigned int pre_val,cap_val;
//	struct regval_list regs[] = {
//		////==================================================================================
//		//// 20.Preview & Capture Configration Setting
//		////==================================================================================
//		//{{0xFC,0xFC},{0xD0,0x00}},
//		//{{0x00,0x28},{0x70,0x00}},
//		//{{0x00,0x2A},{0x02,0xD0}},	
//		//{{0x0F,0x12},{0x00,0x00}},	//REG_0TC_PCFG_uPrevMirror     Original 00 / H Mirror 05 / V Mirror 0A / HV Mirror 0F
//		//{{0x0F,0x12},{0x00,0x00}},	//REG_0TC_PCFG_uCaptureMirror  Original 00 / H Mirror 05 / V Mirror 0A / HV Mirror 0F
//		
//		//==================================================================================
//		// 21.Select Cofigration Display
//		//==================================================================================
//		//PREVIEW
//		{{0xFC,0xFC},{0xD0,0x00}},
//		{{0x00,0x28},{0x70,0x00}},
//		{{0x00,0x2A},{0x02,0x66}},
//		{{0x0f,0x12},{0x00,0x00}},	//REG_TC_GP_ActivePrevConfig
//		{{0x00,0x2A},{0x02,0x6A}},
//		{{0x0F,0x12},{0x00,0x01}},	//REG_TC_GP_PrevOpenAfterChange
//		{{0x00,0x2A},{0x02,0x4E}},
//		{{0x0F,0x12},{0x00,0x01}},	//REG_TC_GP_NewConfigSync
//		{{0x00,0x2A},{0x02,0x68}},
//		{{0x0F,0x12},{0x00,0x01}},	//REG_TC_GP_PrevConfigChanged
//		{{0x00,0x2A},{0x02,0x3E}},
//		{{0x0F,0x12},{0x00,0x01}},	//REG_TC_GP_EnablePreview
//		{{0x0F,0x12},{0x00,0x01}},	//REG_TC_GP_EnablePreviewChanged;
//  };
//
//	if(info->vflip == value)
//		return 0;
//	
//	sensor_write_im(sd, 0x0028, 0x7000);
//	sensor_write_im(sd, 0x002a, 0x02d0);
//	sensor_write_im(sd, 0x002c , 0x7000);
//	sensor_write_im(sd, 0x002e , 0x02d0);
//	sensor_read_im(sd, 0x0f12, &pre_val);
//	sensor_read_im(sd, 0x0f12, &cap_val);
//	
//	switch (value) {
//		case 0:
//		  pre_val &= 0xf5;
//		  cap_val &= 0xf5;
//			break;
//		case 1:
//		  pre_val |= 0x0A;
//		  cap_val |= 0x0A;
//			break;
//		default:
//			return -EINVAL;
//	}
//
//	sensor_write_im(sd, 0x0f12, pre_val);
//  sensor_write_im(sd, 0x0f12, cap_val);
//
//	ret = sensor_write_array(sd, regs, ARRAY_SIZE(regs));
//	if (ret < 0) {
//		csi_dev_err("sensor_write err at sensor_s_vflip!\n");
//		return ret;
//	}
//
//	info->vflip = value;
	
	return 0;
}

static int sensor_g_autogain(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_autogain(struct v4l2_subdev *sd, int value)
{
	return -EINVAL;
}

static int sensor_g_autoexp(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_autoexp(struct v4l2_subdev *sd,
		enum v4l2_exposure_auto_type value)
{
	return -EINVAL;
}

static int sensor_g_autowb(struct v4l2_subdev *sd, int *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->autowb;
	return 0;
}

static int sensor_s_autowb(struct v4l2_subdev *sd, int value)
{
	int ret=0;
	struct sensor_info *info = to_state(sd);
	
	ret = sensor_write_array(sd, sensor_wb_auto_regs, ARRAY_SIZE(sensor_wb_auto_regs));
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_autowb!\n");
		return ret;
	}
	
	//mdelay(10);
	info->autowb = value;
	csi_dev_dbg("set AutoWhiteBalance successfully!!\n");
	return 0;
}



static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_gain(struct v4l2_subdev *sd, int value)
{
	return -EINVAL;
}

static int sensor_g_band_filter(struct v4l2_subdev *sd, 
		__s32 *value)
{
	struct sensor_info *info = to_state(sd);
		
	*value = info->band_filter;
	return 0;
}

static int sensor_s_band_filter(struct v4l2_subdev *sd, 
		enum v4l2_power_line_frequency value)
{
	struct sensor_info *info = to_state(sd);

	int ret = 0;
	
	switch(value) {
		case V4L2_CID_POWER_LINE_FREQUENCY_DISABLED:
			break;			
		case V4L2_CID_POWER_LINE_FREQUENCY_50HZ:
			ret = sensor_write_array(sd, sensor_flicker_50hz_regs, ARRAY_SIZE(sensor_flicker_50hz_regs));
			if (ret < 0)
				csi_dev_err("sensor_write_array err at sensor_s_band_filter!\n");
			break;
		case V4L2_CID_POWER_LINE_FREQUENCY_60HZ:
			ret = sensor_write_array(sd, sensor_flicker_60hz_regs, ARRAY_SIZE(sensor_flicker_60hz_regs));
			if (ret < 0)
				csi_dev_err("sensor_write_array err at sensor_s_band_filter!\n");
		  break;
	}
	//mdelay(10);
	info->band_filter = value;
	return ret;
}
/* *********************************************end of ******************************************** */
//ipc setting
static int sensor_g_brightness(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	csi_dev_dbg("sensor_g_brightness\n");
	*value = info->brightness;
	return 0;
}

//yuyali--20140915
static int sensor_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret=0;
	int usrvalue = value;
	//value = value - 128;
	struct sensor_info *info = to_state(sd);
	if(usrvalue != info->brightness)
	{
		/****
			sensor_brightness_neg_regs[0].value = value;
			ret=sensor_write_array(sd,sensor_brightness_neg_regs,
	             ARRAY_SIZE(sensor_brightness_neg_regs));
			if (ret < 0) {
				csi_dev_dbg("sensor_write_array err at sensor_s_brightness!\n");
			return ret;
			}
			*****/

	if(usrvalue < -4 || usrvalue > 4)
		{			
			csi_dev_dbg("set brightness value is not correct!!\n");
			return -EINVAL;
		}
		
	
	LOG_ERR_RET(sensor_write_array(sd, sensor_brightness[value+4].regs, sensor_brightness[value+4].size))

		
	//	mdelay(10);
		info->brightness = usrvalue;
	csi_dev_dbg("set brightness value successfully!!\n");
	}
	else
	{
		csi_dev_dbg("same value\n");
	}

	//csi_dev_dbg("nt99142 sensor_0x32FC,%4x\n",sensor_brightness_neg_regs[0].value);
	return 0;
}

static int sensor_g_contrast(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	csi_dev_dbg("sensor_g_contrast\n");
	*value = info->contrast;
	return 0;
}

static int sensor_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret=0;
	
	struct sensor_info *info = to_state(sd);
	
	
	csi_dev_dbg("nt99142 sensor_s_contrast,%d\n",value);
	
	if(value != info->contrast)
	{
		/*****
		sensor_contrast_neg_regs[1].value = value;
		
		ret=sensor_write_array(sd,sensor_contrast_neg_regs,
       ARRAY_SIZE(sensor_contrast_neg_regs));
       *****/
       
      /********************************
        *****By Yuyali, 201409013******
       
        *********************************/
        if(value < 0 || value > 5) 
        {			
			csi_dev_dbg("set contrast value is not correct!!\n");
			return -EINVAL;
		}
		
        LOG_ERR_RET(sensor_write_array(sd, sensor_contrast[value].regs, sensor_contrast[value].size))
			
		
	//	mdelay(10);
		info->contrast = value;
		csi_dev_dbg("set contrast value successfully!!\n");
	}
	else
	{
		csi_dev_dbg("same value\n");
	}
		
	//csi_dev_dbg("nt99142 sensor_ 0x32F2,%4x\n",sensor_contrast_neg_regs[1].value);
	return 0;
}

static int sensor_g_saturation(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	csi_dev_dbg("sensor_g_saturation\n");
	*value = info->saturation;
	return 0;
}

static int sensor_s_saturation(struct v4l2_subdev *sd, int value)
{
	int ret=0;
	struct sensor_info *info = to_state(sd);
	if(value != info->saturation)
	{
		sensor_saturation_neg_regs[1].value= value;
		ret=sensor_write_array(sd,sensor_saturation_neg_regs,
             ARRAY_SIZE(sensor_saturation_neg_regs));
		if (ret < 0) {
			csi_dev_dbg("sensor_write_array err at sensor_s_saturation!\n");
			return ret;
		}
	//	mdelay(10);
		info->saturation = value;
		csi_dev_dbg("set saturation value successfully!!\n");
	}
	else
	{
		csi_dev_dbg("same value\n");
	}
	return 0;
}

static int sensor_g_hue(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	csi_dev_dbg("sensor_g_hue\n");
	*value = info->hue;
	return 0;
}

static int sensor_s_hue(struct v4l2_subdev *sd, int value)
{
	int ret=0;
	struct sensor_info *info = to_state(sd);
	sensor_hue_neg_regs[1].value= value;
	ret=sensor_write_array(sd,sensor_hue_neg_regs,
             ARRAY_SIZE(sensor_hue_neg_regs));
	if (ret < 0) {
		csi_dev_dbg("sensor_write_array err at sensor_s_hue!\n");
		return ret;
	}
//	mdelay(10);

	info->hue = value;
	csi_dev_dbg("set hue successfully!!\n");

	return 0;
}

static int sensor_g_sharpness(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	csi_dev_dbg("sensor_g_sharpness\n");
	*value = info->sharpness;
	return 0;
}


static int sensor_s_sharpness(struct v4l2_subdev *sd, int value)
{
	csi_dev_dbg("nt99142 sensor_s_sharpness,%d\n",value);
	int ret=0;
	struct sensor_info *info = to_state(sd);
	if(value != info->sharpness)
	{
		sensor_sharpness_neg_regs[0].value= 7 - value;
		ret=sensor_write_array(sd,sensor_sharpness_neg_regs,
             ARRAY_SIZE(sensor_sharpness_neg_regs));
		if (ret < 0) {
			csi_dev_dbg("sensor_write_array err at sensor_s_sharpness!\n");
			return ret;
		}
		mdelay(10);
		info->sharpness = value;
		csi_dev_dbg("set sharpness value successfully!! \n");
	}
	else
	{
		csi_dev_dbg("same value\n");
	}
	return 0;
}


static int sensor_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->exp;
	return 0;
}

static int sensor_s_exp(struct v4l2_subdev *sd, int value)
{
	int ret=0;
	int usrvalue = value;
	struct sensor_info *info = to_state(sd);
	#if 0
	switch (value) {
		case -4:
		  ret = sensor_write_array(sd, sensor_ev_neg4_regs, ARRAY_SIZE(sensor_ev_neg4_regs));
			break;
		case -3:
			ret = sensor_write_array(sd, sensor_ev_neg3_regs, ARRAY_SIZE(sensor_ev_neg3_regs));
			break;
		case -2:
			ret = sensor_write_array(sd, sensor_ev_neg2_regs, ARRAY_SIZE(sensor_ev_neg2_regs));
			break;   
		case -1:
			ret = sensor_write_array(sd, sensor_ev_neg1_regs, ARRAY_SIZE(sensor_ev_neg1_regs));
			break;
		case 0:   
			ret = sensor_write_array(sd, sensor_ev_zero_regs, ARRAY_SIZE(sensor_ev_zero_regs));
			break;
		case 1:
			ret = sensor_write_array(sd, sensor_ev_pos1_regs, ARRAY_SIZE(sensor_ev_pos1_regs));
			break;
		case 2:
			ret = sensor_write_array(sd, sensor_ev_pos2_regs, ARRAY_SIZE(sensor_ev_pos2_regs));
			break;	
		case 3:
			ret = sensor_write_array(sd, sensor_ev_pos3_regs, ARRAY_SIZE(sensor_ev_pos3_regs));
			break;
		case 4:
			ret = sensor_write_array(sd, sensor_ev_pos4_regs, ARRAY_SIZE(sensor_ev_pos4_regs));
			break;
		default:
			return -EINVAL;
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_exp!\n");
		return ret;
	}
	#endif
	 
      /********************************
        *****By Yuyali, 201409013******
       
        *********************************/
        if(usrvalue <-3 || usrvalue > 3) 
        {
			csi_dev_dbg("set exposure value is not correct!!\n");
			
			return -EINVAL;
		}
			
        LOG_ERR_RET(sensor_write_array(sd, sensor_exposure[usrvalue+3].regs, sensor_exposure[usrvalue+3].size))
//	mdelay(10);
	info->exp = value;
	csi_dev_dbg("set exposure value successfully!!\n");
	return 0;
}

static int sensor_g_wb(struct v4l2_subdev *sd, int *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_whiteblance *wb_type = (enum v4l2_whiteblance*)value;
	
	*wb_type = info->wb;
	
	return 0;
}
/*******************************
*******By yuyali, 20140916*****
*******************************/
static int sensor_s_wb(struct v4l2_subdev *sd,
		enum v4l2_whiteblance value)
{
	/****
	int ret=0;
	struct sensor_info *info = to_state(sd);
	
	if (value == V4L2_WB_AUTO) {
		ret = sensor_s_autowb(sd, 1);
		return ret;
	} 
	else {
		
		//ret = sensor_s_autowb(sd, 0);
		//if(ret < 0) {
		//	csi_dev_err("sensor_s_autowb error, return %x!\n",ret);
		//	return ret;
		}
		switch (value) {
			case V4L2_WB_CLOUD:
			  ret = sensor_write_array(sd, sensor_wb_cloud_regs, ARRAY_SIZE(sensor_wb_cloud_regs));
				break;
			case V4L2_WB_DAYLIGHT:
				ret = sensor_write_array(sd, sensor_wb_daylight_regs, ARRAY_SIZE(sensor_wb_daylight_regs));
				break;
			case V4L2_WB_INCANDESCENCE:
				ret = sensor_write_array(sd, sensor_wb_incandescence_regs, ARRAY_SIZE(sensor_wb_incandescence_regs));
				break;    
			case V4L2_WB_FLUORESCENT:
				ret = sensor_write_array(sd, sensor_wb_fluorescent_regs, ARRAY_SIZE(sensor_wb_fluorescent_regs));
				break;
			case V4L2_WB_TUNGSTEN:   
				ret = sensor_write_array(sd, sensor_wb_tungsten_regs, ARRAY_SIZE(sensor_wb_tungsten_regs));
				break;
			default:
				return -EINVAL;
		} 
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_s_wb error, return %x!\n",ret);
		return ret;
	}
	
//	mdelay(10);
	info->wb = value;
	csi_dev_dbg("set white balance value successfully!!\n");
***********/

	csi_dev_dbg("set white balance value is starting!!\n");

	struct sensor_info *info = to_state(sd);
	int usrvalue=value;
	if(info->capture_mode == V4L2_MODE_IMAGE)
		return 0;
	if(usrvalue <0 || usrvalue > 4) 
        {
			csi_dev_dbg("set white balance value is not correct!!\n");
			
			return -EINVAL;
		}
	LOG_ERR_RET(sensor_write_array(sd, sensor_wb[value].regs ,sensor_wb[value].size) )

	info->wb = value;
	csi_dev_dbg("set white balance value successfully!!\n");
	return 0;

}

static int sensor_g_colorfx(struct v4l2_subdev *sd,
		__s32 *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_colorfx *clrfx_type = (enum v4l2_colorfx*)value;
	
	*clrfx_type = info->clrfx;
	return 0;
}

static int sensor_s_colorfx(struct v4l2_subdev *sd,
		enum v4l2_colorfx value)
{
	int ret=0;
	struct sensor_info *info = to_state(sd);
	
	switch (value) {
	case V4L2_COLORFX_NONE:
	  ret = sensor_write_array(sd, sensor_colorfx_none_regs, ARRAY_SIZE(sensor_colorfx_none_regs));
		break;
	case V4L2_COLORFX_BW:
		ret = sensor_write_array(sd, sensor_colorfx_bw_regs, ARRAY_SIZE(sensor_colorfx_bw_regs));
		break;  
	case V4L2_COLORFX_SEPIA:
		ret = sensor_write_array(sd, sensor_colorfx_sepia_regs, ARRAY_SIZE(sensor_colorfx_sepia_regs));
		break;   
	case V4L2_COLORFX_NEGATIVE:
		ret = sensor_write_array(sd, sensor_colorfx_negative_regs, ARRAY_SIZE(sensor_colorfx_negative_regs));
		break;
	case V4L2_COLORFX_EMBOSS:   
		ret = sensor_write_array(sd, sensor_colorfx_emboss_regs, ARRAY_SIZE(sensor_colorfx_emboss_regs));
		break;
	case V4L2_COLORFX_SKETCH:     
		ret = sensor_write_array(sd, sensor_colorfx_sketch_regs, ARRAY_SIZE(sensor_colorfx_sketch_regs));
		break;
	case V4L2_COLORFX_SKY_BLUE:
		ret = sensor_write_array(sd, sensor_colorfx_sky_blue_regs, ARRAY_SIZE(sensor_colorfx_sky_blue_regs));
		break;
	case V4L2_COLORFX_GRASS_GREEN:
		ret = sensor_write_array(sd, sensor_colorfx_grass_green_regs, ARRAY_SIZE(sensor_colorfx_grass_green_regs));
		break;
	case V4L2_COLORFX_SKIN_WHITEN:
		ret = sensor_write_array(sd, sensor_colorfx_skin_whiten_regs, ARRAY_SIZE(sensor_colorfx_skin_whiten_regs));
		break;
	case V4L2_COLORFX_VIVID:
		ret = sensor_write_array(sd, sensor_colorfx_vivid_regs, ARRAY_SIZE(sensor_colorfx_vivid_regs));
		break;
	default:
		return -EINVAL;
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_s_colorfx error, return %x!\n",ret);
		return ret;
	}
//	mdelay(10);
	info->clrfx = value;
	
	return 0;
}

static int sensor_g_flash_mode(struct v4l2_subdev *sd,
    __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_flash_mode *flash_mode = (enum v4l2_flash_mode*)value;
	
	*flash_mode = info->flash_mode;
	return 0;
}

static int sensor_s_flash_mode(struct v4l2_subdev *sd,
    enum v4l2_flash_mode value)
{
	struct sensor_info *info = to_state(sd);
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	int flash_on,flash_off;
	
	flash_on = (dev->flash_pol!=0)?1:0;
	flash_off = (flash_on==1)?0:1;
	
	switch (value) {
	case V4L2_FLASH_MODE_OFF:
		csi_gpio_write(sd,&dev->flash_io,flash_off);
		break;
	case V4L2_FLASH_MODE_AUTO:
		return -EINVAL;
		break;  
	case V4L2_FLASH_MODE_ON:
		csi_gpio_write(sd,&dev->flash_io,flash_on);
		break;   
	case V4L2_FLASH_MODE_TORCH:
		return -EINVAL;
		break;
	case V4L2_FLASH_MODE_RED_EYE:   
		return -EINVAL;
		break;
	default:
		return -EINVAL;
	}
	
	info->flash_mode = value;
	return 0;
}

static int sensor_g_autofocus_mode(struct v4l2_subdev *sd,
    __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->af_mode;
	
	return 0;
}

static int sensor_s_autofocus_mode(struct v4l2_subdev *sd,
    enum v4l2_autofocus_mode value)
{
	struct sensor_info *info = to_state(sd);
	int ret=0;
	
	csi_dev_dbg("sensor_s_autofocus_mode = %d\n",value);
	
	switch(value) {
		case V4L2_AF_FIXED:
			break;
		case V4L2_AF_INFINITY:
			ret = sensor_s_infinity_af(sd);
			if (ret < 0)
			{
				csi_dev_err("sensor_af_infinity_regs err when sensor_s_autofocus_mode !\n");
				return ret;
			}
			break;
		case V4L2_AF_MACRO:
		  ret = sensor_s_macro_af(sd);
			if (ret < 0)
			{
				csi_dev_err("sensor_af_macro_regs err when sensor_s_autofocus_mode !\n");
				return ret;
			}
		  break;
		case V4L2_AF_AUTO:
		case V4L2_AF_TOUCH:
		case V4L2_AF_FACE:
//			ret = sensor_s_continueous_af(sd);
//			if (ret < 0)
//			{
//				csi_dev_err("sensor_s_continueous_af err when sensor_s_autofocus_mode!\n");
//				return ret;
//			}
			break;
					
		default:
			return 0;
	}
	
	info->af_mode = value;
	
	return 0;
}

static int sensor_g_autofocus_ctrl(struct v4l2_subdev *sd,
		struct v4l2_control *ctrl)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_autofocus_ctrl af_ctrl = ctrl->value;

	switch(af_ctrl) {
		case V4L2_AF_INIT:
			return sensor_g_single_af(sd);
			break;
		case V4L2_AF_RELEASE:
			break;
		case V4L2_AF_TRIG_SINGLE:
			return sensor_g_single_af(sd);
		case V4L2_AF_TRIG_CONTINUEOUS:
			break;
		case V4L2_AF_LOCK:
			break;
		case V4L2_AF_WIN_XY:
			break;
		case V4L2_AF_WIN_NUM:
			break;
					
		default:
			return 0;
	}
	
	ctrl->value = info->af_ctrl;
	return 0;	
}

static int sensor_s_autofocus_ctrl(struct v4l2_subdev *sd,
		struct v4l2_control *ctrl)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_autofocus_ctrl af_ctrl = ctrl->value;
	struct v4l2_pix_size *pix;
	
	csi_dev_dbg("sensor_s_autofocus_ctrl = %d\n",ctrl->value);
	
	switch(af_ctrl) {
		case V4L2_AF_INIT:
			return sensor_download_af_fw(sd);
		case V4L2_AF_RELEASE:
			return sensor_s_release_af(sd);
		case V4L2_AF_TRIG_SINGLE:
			return sensor_s_single_af(sd);
		case V4L2_AF_TRIG_CONTINUEOUS:
			return sensor_s_continueous_af(sd);
		case V4L2_AF_LOCK:
			return sensor_s_pause_af(sd);
		case V4L2_AF_WIN_XY:
			pix = (struct v4l2_pix_size*)ctrl->user_pt;
			return sensor_s_af_zone(sd,pix->width,pix->height);
			break;
		case V4L2_AF_WIN_NUM:
			break;
					
	  default:
			 return 0;
	}
	
	info->af_ctrl = ctrl->value;
	return 0;
}

static int sensor_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return sensor_g_brightness(sd, &ctrl->value);
	case V4L2_CID_CONTRAST:
		return sensor_g_contrast(sd, &ctrl->value);
	case V4L2_CID_SATURATION:
		return sensor_g_saturation(sd, &ctrl->value);
	case V4L2_CID_HUE:
		return sensor_g_hue(sd, &ctrl->value);	
	case V4L2_CID_SHARPNESS:
		return sensor_g_sharpness(sd, &ctrl->value);
	case V4L2_CID_VFLIP:
		return sensor_g_vflip(sd, &ctrl->value);
	case V4L2_CID_HFLIP:
		return sensor_g_hflip(sd, &ctrl->value);
	case V4L2_CID_GAIN:
		return sensor_g_gain(sd, &ctrl->value);
	case V4L2_CID_AUTOGAIN:
		return sensor_g_autogain(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE:
		return sensor_g_exp(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE_AUTO:
		return sensor_g_autoexp(sd, &ctrl->value);
	case V4L2_CID_DO_WHITE_BALANCE:
		return sensor_g_wb(sd, &ctrl->value);
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return sensor_g_autowb(sd, &ctrl->value);
	case V4L2_CID_COLORFX:
		return sensor_g_colorfx(sd,	&ctrl->value);
	case V4L2_CID_CAMERA_FLASH_MODE:
		return sensor_g_flash_mode(sd, &ctrl->value);
	case V4L2_CID_POWER_LINE_FREQUENCY:
		return sensor_g_band_filter(sd, &ctrl->value);
	case V4L2_CID_CAMERA_AF_MODE:
		return sensor_g_autofocus_mode(sd, &ctrl->value);
	case V4L2_CID_CAMERA_AF_CTRL:
		return sensor_g_autofocus_ctrl(sd, ctrl);
				
	default:
			return 0;
	}
	return -EINVAL;
}

static int sensor_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return sensor_s_brightness(sd, ctrl->value);
	case V4L2_CID_CONTRAST:
		return sensor_s_contrast(sd, ctrl->value);
	case V4L2_CID_SATURATION:
		return sensor_s_saturation(sd, ctrl->value);
	case V4L2_CID_HUE:
		return sensor_s_hue(sd, ctrl->value);		
	case V4L2_CID_SHARPNESS:
		return sensor_s_sharpness(sd, ctrl->value);
	case V4L2_CID_VFLIP:
		return sensor_s_vflip(sd, ctrl->value);
	case V4L2_CID_HFLIP:
		return sensor_s_hflip(sd, ctrl->value);
	case V4L2_CID_GAIN:
		return sensor_s_gain(sd, ctrl->value);
	case V4L2_CID_AUTOGAIN:
		return sensor_s_autogain(sd, ctrl->value);
	case V4L2_CID_EXPOSURE:
		return sensor_s_exp(sd, ctrl->value);
	case V4L2_CID_EXPOSURE_AUTO:
		return sensor_s_autoexp(sd,
				(enum v4l2_exposure_auto_type) ctrl->value);
	case V4L2_CID_DO_WHITE_BALANCE:
		return sensor_s_wb(sd,
				(enum v4l2_whiteblance) ctrl->value);	
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return sensor_s_autowb(sd, ctrl->value);
	case V4L2_CID_COLORFX:
		return sensor_s_colorfx(sd,
				(enum v4l2_colorfx) ctrl->value);
	case V4L2_CID_CAMERA_FLASH_MODE:
	  return sensor_s_flash_mode(sd,
	      (enum v4l2_flash_mode) ctrl->value);
	case V4L2_CID_POWER_LINE_FREQUENCY:
		return sensor_s_band_filter(sd,
	      (enum v4l2_power_line_frequency) ctrl->value);
	case V4L2_CID_CAMERA_AF_MODE:
		return sensor_s_autofocus_mode(sd,
	      (enum v4l2_autofocus_mode) ctrl->value);
	case V4L2_CID_CAMERA_AF_CTRL:
		return sensor_s_autofocus_ctrl(sd, ctrl);
		
	default:
			return 0;
			
	}
	return -EINVAL;
}


static int sensor_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_SENSOR, 0);
}


/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.g_chip_ident = sensor_g_chip_ident,
	.g_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
	.queryctrl = sensor_queryctrl,
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.ioctl = sensor_ioctl,
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.enum_mbus_fmt = sensor_enum_fmt,//linux-3.0
	.try_mbus_fmt = sensor_try_fmt,//linux-3.0
	.s_mbus_fmt = sensor_s_fmt,//linux-3.0
	.s_parm = sensor_s_parm,//linux-3.0
	.g_parm = sensor_g_parm,//linux-3.0
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
};

/* ----------------------------------------------------------------------- */

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;
//	int ret=0;

	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;
	v4l2_i2c_subdev_init(sd, client, &sensor_ops);

	info->fmt = &sensor_formats[0];
	info->ccm_info = &ccm_info_con;
	
	return 0;
}


static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ "nt99142", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

//linux-3.0
static struct i2c_driver sensor_driver = {
	.driver = {
		.owner = THIS_MODULE,
	.name = "nt99142",
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};
static __init int init_sensor(void)
{
	return i2c_add_driver(&sensor_driver);
}

static __exit void exit_sensor(void)
{
  i2c_del_driver(&sensor_driver);
}

module_init(init_sensor);
module_exit(exit_sensor);


