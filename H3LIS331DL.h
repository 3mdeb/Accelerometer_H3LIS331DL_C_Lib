/*
 * H3LIS331DL.h
 * A library for 3-Axis Digital Accelerometer(H3LIS331DL_t* device, ±400g)
 *  
 * Original work Copyright (H3LIS331DL_t* device, c) 2014 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : lawliet zou
 * Create Time: April 2014
 * Change Log :
 *
 * Modified work Copyright 2016 Andrzej Pawlowicz at 3mdeb - Embedded Systems Consulting
 * Website    : 3mdeb.com
 * Author     : Andrzej Pawlowicz (andrzej.pawlowicz1@gmail.com)
 * Create Time: March 2016
 * Change Log : 
 *		1) Port from C++ to C
 *		2) Adding missing funcitons for reading configuration registers
 *		3) Adding support for single axis reads
 *		4) Optimization of accelerations data reading by utilization of i2c multi byte reads
 *
 * The MIT License (H3LIS331DL_t* device, MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (H3LIS331DL_t* device, the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
 
#ifndef H3LIS331DL_H
#define H3LIS331DL_H

#include <stdint.h>


#define H3LIS331DL_WHO_AM_I_VALUE 		0x32  // H3LIS331DL device identification register value

//
// Missing byte type
//

typedef uint8_t byte;

//
// Macros
//
#define ValBit(VAR,Place)  (VAR & (1<<Place))

#define BIT(x) ( (x) )

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MEMS_SET                        0x01
#define MEMS_RESET                      0x00

#define H3LIS331DL_MEMS_I2C_ADDRESS     0x18//0x32

//Register and define
#define H3LIS331DL_WHO_AM_I             0x0F   // device identification register

// CONTROL REGISTER 1 
#define H3LIS331DL_CTRL_REG1            0x20
#define H3LIS331DL_PM                   BIT(5) //PowerMode selection: 000 - power down / 001 - normal mode / other - low power
#define H3LIS331DL_DR                   BIT(3) //output data rate: 00 - 50hz / 01 - 100hz / 10 - 400hz / 11 - 1000hz
#define H3LIS331DL_ZEN                  BIT(2) //Z-axis enable: 0 - disable / 1 - enable
#define H3LIS331DL_YEN                  BIT(1) //Y-axis enable: 0 - disable / 1 - enable
#define H3LIS331DL_XEN                  BIT(0) //Y-axis enable: 0 - disable / 1 - enable

//CONTROL REGISTER 2 
#define H3LIS331DL_CTRL_REG2            0x21
#define H3LIS331DL_BOOT                 BIT(7) //reboot memory content, default is 0
#define H3LIS331DL_HPM                  BIT(5) //High-pass-filter mode selection, default is 00
#define H3LIS331DL_FDS                  BIT(4) //Filter data selection, default is 0
#define H3LIS331DL_HPEN2                BIT(3) //High-pass filter enabled for interrupt 2 source, default is 0
#define H3LIS331DL_HPEN1                BIT(2) //High-pass filter enabled for interrupt 1 source, default is 0
#define H3LIS331DL_HPCF                 BIT(0) //High-pass filter cutoff frequency configuration, default is 00

//CONTROL REGISTER 3 
#define H3LIS331DL_CTRL_REG3            0x22
#define H3LIS331DL_IHL                  BIT(7) //Interrupt active high,low. default is 0
#define H3LIS331DL_PP_OD                BIT(6) //Push-pull/open drain selection on interrupt pad. default is 0
#define H3LIS331DL_LIR2                 BIT(5) //Latch interrupt request on INT2_SRC register, with INT2_SRC register cleared by read INT2_SRC itself. default is 0
#define H3LIS331DL_I2_CFG               BIT(3) //Data signal on INT2 pad control bits, default is 00
#define H3LIS331DL_LIR1                 BIT(2) //Latch interrupt request on the INT1_SRC register, with the INT1_SRC register cleared by reading the INT1_SRC register.
#define H3LIS331DL_I1_CFG               BIT(0) //Data signal on INT1 pad control bits, default is 00

//CONTROL REGISTER 4
#define H3LIS331DL_CTRL_REG4            0x23
#define H3LIS331DL_BDU                  BIT(7) //Block data update, default is 0
#define H3LIS331DL_BLE                  BIT(6) //Big/little endian data selection, default is 0
#define H3LIS331DL_FS                   BIT(4) //Full scale selection, default is 00(00:100g;01:200g;11:400g)
#define H3LIS331DL_ST_SIGN              BIT(3) //
#define H3LIS331DL_ST                   BIT(1) //
#define H3LIS331DL_SIM                  BIT(0) // SPI serial interface mode selection, default is 0

//CONTROL REGISTER 5
#define H3LIS331DL_CTRL_REG5            0x24 
#define H3LIS331DL_TURN_ON              BIT(0) // Turn-on mode selection selection for sleep to wake function. default is 00

#define H3LIS331DL_HP_FILTER_RESET      0x25   // 

//REFERENCE/DATA_CAPTURE
#define H3LIS331DL_REFERENCE_REG        0x26   //
#define H3LIS331DL_REF                  BIT(0) //

//STATUS_REG_AXIES 
#define H3LIS331DL_STATUS_REG           0x27   //

//OUTPUT REGISTER
#define H3LIS331DL_OUT_X_L              0x28   //x-axis acceleration data
#define H3LIS331DL_OUT_X_H              0x29   
#define H3LIS331DL_OUT_Y_L              0x2A   //y-axis acceleration data
#define H3LIS331DL_OUT_Y_H              0x2B
#define H3LIS331DL_OUT_Z_L              0x2C   //z-axis acceleration data
#define H3LIS331DL_OUT_Z_H              0x2D


//INTERRUPT 1 CONFIGURATION 
#define H3LIS331DL_INT1_CFG             0x30

//INTERRUPT 2 CONFIGURATION 
#define H3LIS331DL_INT2_CFG             0x34
#define H3LIS331DL_ANDOR                BIT(7)
#define H3LIS331DL_INT_6D               BIT(6)

//INT REGISTERS 
#define H3LIS331DL_INT1_THS             0x32
#define H3LIS331DL_INT1_DURATION        0x33
#define H3LIS331DL_INT2_THS             0x36
#define H3LIS331DL_INT2_DURATION        0x37

//INTERRUPT 1 SOURCE REGISTER 
#define H3LIS331DL_INT1_SRC             0x31
#define H3LIS331DL_INT2_SRC             0x35

//INT_CFG  bit mask
#define H3LIS331DL_INT_AND              0x80
#define H3LIS331DL_INT_OR               0x00
#define H3LIS331DL_INT_ZHIE_ENABLE      0x20
#define H3LIS331DL_INT_ZHIE_DISABLE     0x00
#define H3LIS331DL_INT_ZLIE_ENABLE      0x10
#define H3LIS331DL_INT_ZLIE_DISABLE     0x00
#define H3LIS331DL_INT_YHIE_ENABLE      0x08
#define H3LIS331DL_INT_YHIE_DISABLE     0x00
#define H3LIS331DL_INT_YLIE_ENABLE      0x04
#define H3LIS331DL_INT_YLIE_DISABLE     0x00
#define H3LIS331DL_INT_XHIE_ENABLE      0x02
#define H3LIS331DL_INT_XHIE_DISABLE     0x00
#define H3LIS331DL_INT_XLIE_ENABLE      0x01
#define H3LIS331DL_INT_XLIE_DISABLE     0x00

//INT_SRC  bit mask
#define H3LIS331DL_INT_SRC_IA           0x40
#define H3LIS331DL_INT_SRC_ZH           0x20
#define H3LIS331DL_INT_SRC_ZL           0x10
#define H3LIS331DL_INT_SRC_YH           0x08
#define H3LIS331DL_INT_SRC_YL           0x04
#define H3LIS331DL_INT_SRC_XH           0x02
#define H3LIS331DL_INT_SRC_XL           0x01

//STATUS REGISTER bit mask
#define H3LIS331DL_STATUS_REG_ZYXOR     0x80    // 1:new data set has over written the previous one
                                                // 0:no overrun has occurred (default)
#define H3LIS331DL_STATUS_REG_ZOR       0x40    // 0:no overrun has occurred (default)
                                                // 1:new Z-axis data has over written the previous one
#define H3LIS331DL_STATUS_REG_YOR       0x20    // 0:no overrun has occurred (default)
                                                // 1:new Y-axis data has over written the previous one
#define H3LIS331DL_STATUS_REG_XOR       0x10    // 0:no overrun has occurred (default)
                                                // 1:new X-axis data has over written the previous one
#define H3LIS331DL_STATUS_REG_ZYXDA     0x08    // 0:a new set of data is not yet avvious one
                                                // 1:a new set of data is available 
#define H3LIS331DL_STATUS_REG_ZDA       0x04    // 0:a new data for the Z-Axis is not availvious one
                                                // 1:a new data for the Z-Axis is available
#define H3LIS331DL_STATUS_REG_YDA       0x02    // 0:a new data for the Y-Axis is not available
                                                // 1:a new data for the Y-Axis is available
#define H3LIS331DL_STATUS_REG_XDA       0x01    // 0:a new data for the X-Axis is not available
                                                // 1:a new data for the X-Axis is available
#define H3LIS331DL_DATAREADY_BIT        H3LIS331DL_STATUS_REG_ZYXDA

// Forwar declaration
typedef struct H3LIS331DL H3LIS331DL_t;

typedef uint8_t H3LIS331DL_Axis_t;
typedef uint8_t H3LIS331DL_IntConf_t;

//define structure
typedef enum {
    MEMS_SUCCESS  = 0x01,
    MEMS_ERROR  = 0x00  
} H3LIS331DL_Status_t;

typedef enum {
    MEMS_ENABLE   = 0x01,
    MEMS_DISABLE    = 0x00  
} H3LIS331DL_State_t;

typedef enum {
	X_AXIS,
	Y_AXIS,
	Z_AXIS
} Axis_t;

typedef struct {
    int16_t AXIS_X;
    int16_t AXIS_Y;
    int16_t AXIS_Z;
} AxesRaw_t;

typedef enum {  
    H3LIS331DL_ODR_50Hz    = 0x00,
    H3LIS331DL_ODR_100Hz   = 0x01,  
    H3LIS331DL_ODR_400Hz   = 0x02,
    H3LIS331DL_ODR_1000Hz  = 0x03
} H3LIS331DL_ODR_t;

typedef enum {
    H3LIS331DL_CONTINUOUS_MODE = 0x00,
    H3LIS331DL_SINGLE_MODE     = 0x01,
    H3LIS331DL_SLEEP_MODE      = 0x02
} H3LIS331DL_Mode_M_t;

typedef enum {
    H3LIS331DL_POWER_DOWN   = 0x00,
    H3LIS331DL_NORMAL       = 0x01,
    H3LIS331DL_LOW_POWER_05 = 0x02,
    H3LIS331DL_LOW_POWER_1  = 0x03,
    H3LIS331DL_LOW_POWER_2  = 0x04,
    H3LIS331DL_LOW_POWER_5  = 0x05,
    H3LIS331DL_LOW_POWER_10 = 0x06,
} H3LIS331DL_Mode_t;

typedef enum {
    H3LIS331DL_HPM_NORMAL_MODE_RES = 0x00,
    H3LIS331DL_HPM_REF_SIGNAL      = 0x01,
    H3LIS331DL_HPM_NORMAL_MODE     = 0x02,
} H3LIS331DL_HPFMode_t;

typedef enum {
    H3LIS331DL_HPFCF_0 = 0x00,
    H3LIS331DL_HPFCF_1 = 0x01,
    H3LIS331DL_HPFCF_2 = 0x02,
    H3LIS331DL_HPFCF_3 = 0x03,
} H3LIS331DL_HPFCutOffFreq_t;

typedef enum {
    H3LIS331DL_INT_SOURCE      = 0x00,
    H3LIS331DL_INT_1OR2_SOURCE = 0x01,
    H3LIS331DL_DATA_READY      = 0x02,
    H3LIS331DL_BOOT_RUNNING    = 0x03
} H3LIS331DL_INT_Conf_t;

typedef enum {
    H3LIS331DL_SLEEP_TO_WAKE_DIS    = 0x00,
    H3LIS331DL_SLEEP_TO_WAKE_ENA    = 0x03,
} H3LIS331DL_Sleep_To_Wake_Conf_t;

typedef enum {
    H3LIS331DL_FULLSCALE_2    = 0x00,
    H3LIS331DL_FULLSCALE_4    = 0x01,
    H3LIS331DL_FULLSCALE_8    = 0x03,
} H3LIS331DL_Fullscale_t;

typedef enum {
    H3LIS331DL_BLE_LSB        = 0x00,
    H3LIS331DL_BLE_MSB        = 0x01
} H3LIS331DL_Endianess_t;

typedef enum {
    H3LIS331DL_SPI_4_WIRE = 0x00,
    H3LIS331DL_SPI_3_WIRE = 0x01
} H3LIS331DL_SPIMode_t;

typedef enum {
    H3LIS331DL_X_ENABLE     = 0x01,
    H3LIS331DL_X_DISABLE    = 0x00,
    H3LIS331DL_Y_ENABLE     = 0x02,
    H3LIS331DL_Y_DISABLE    = 0x00,
    H3LIS331DL_Z_ENABLE     = 0x04,
    H3LIS331DL_Z_DISABLE    = 0x00    
} H3LIS331DL_AXISenable_t;

typedef enum {
    H3LIS331DL_UP_SX  = 0x44,
    H3LIS331DL_UP_DX  = 0x42,
    H3LIS331DL_DW_SX  = 0x41,
    H3LIS331DL_DW_DX  = 0x48,
    H3LIS331DL_TOP    = 0x60,
    H3LIS331DL_BOTTOM = 0x50
} H3LIS331DL_POSITION_6D_t;

typedef enum {
    H3LIS331DL_INT_MODE_OR            = 0x00,
    H3LIS331DL_INT_MODE_6D_MOVEMENT   = 0x01,
    H3LIS331DL_INT_MODE_AND           = 0x02,
    H3LIS331DL_INT_MODE_6D_POSITION   = 0x03  
} H3LIS331DL_IntMode_t;

struct H3LIS331DL
{
    int16_t _adjVal[3];
    double _gains;
};

/*******************************************************************************
* Function Name  : H3LIS331DL_setParams
* Description    : Helper method initializing for H3LIS331DL sensor in a single call
* Input          : buffer to empty by AccAxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_init(H3LIS331DL_t* device,
		H3LIS331DL_ODR_t  odr,
		H3LIS331DL_Mode_t mode,
		H3LIS331DL_Fullscale_t fullScale);

/*******************************************************************************
* Function Name  : H3LIS331DL_setParams
* Description    : Sets parameters used as accelaration adjustment and gain
* Input          : buffer to empty by AccAxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
void H3LIS331DL_setParams(H3LIS331DL_t* device, int16_t val_x, int16_t val_y, int16_t val_z, double gains);

//Sensor Configuration Functions
/*******************************************************************************
* Function Name  : H3LIS331DL_getWHO_AM_I
* Description    : Read identification code from H3LIS331DL_WHO_AM_I register
* Input          : char to be filled with the Device identification Value
* Output         : None
* Return         : Status [value of FSS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_getWHO_AM_I(byte* val);

/*******************************************************************************
* Function Name  : H3LIS331DL_getCtrlReg1
* Description    : Gets H3LIS331DL CTRL_REG1 register content
* Input          : char to be filled with the CTRL_REG1 value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_getCtrlReg1(byte* val);

/*******************************************************************************
* Function Name  : H3LIS331DL_getCtrlReg3
* Description    : Gets H3LIS331DL CTRL_REG3 register content
* Input          : char to be filled with the CTRL_REG3 value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_getCtrlReg3(byte* val);

/*******************************************************************************
* Function Name  : H3LIS331DL_setODR
* Description    : Sets H3LIS331DL Accelerometer Output Data Rate
* Input          : Output Data Rate
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setODR(H3LIS331DL_ODR_t dr);

/*******************************************************************************
* Function Name  : H3LIS331DL_setMode
* Description    : Sets H3LIS331DLH Accelerometer Operating Mode
* Input          : Modality (H3LIS331DL_LOW_POWER, H3LIS331DL_NORMAL, H3LIS331DL_POWER_DOWN...)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setMode(H3LIS331DL_Mode_t pm);

/*******************************************************************************
* Function Name  : H3LIS331DL_setAxis
* Description    : Enable/Disable LIS331DLH Axis
* Input          : H3LIS331DL_X_ENABLE/H3LIS331DL_X_DISABLE | H3LIS331DL_Y_ENABLE/H3LIS331DL_Y_DISABLE
                   | H3LIS331DL_Z_ENABLE/H3LIS331DL_Z_DISABLE
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setAxis(H3LIS331DL_Axis_t axis);

/*******************************************************************************
* Function Name  : H3LIS331DL_setFullScale
* Description    : Sets the LIS331DLH FullScale
* Input          : H3LIS331DL_FULLSCALE_2/H3LIS331DL_FULLSCALE_4/H3LIS331DL_FULLSCALE_8
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setFullScale(H3LIS331DL_Fullscale_t fs);

/*******************************************************************************
* Function Name  : H3LIS331DL_setBDU
* Description    : Enable/Disable Block Data Update Functionality
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setBDU(H3LIS331DL_State_t bdu);

/*******************************************************************************
* Function Name  : H3LIS331DL_setBLE
* Description    : Set Endianess (MSB/LSB)
* Input          : H3LIS331DL_BLE_LSB / H3LIS331DL_BLE_MSB
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setBLE(H3LIS331DL_Endianess_t ble);

/*******************************************************************************
* Function Name  : H3LIS331DL_setSelfTest
* Description    : Set Self Test Modality
* Input          : MEMS_DISABLE/MEMS_ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setSelfTest(H3LIS331DL_State_t st);

/*******************************************************************************
* Function Name  : H3LIS331DL_setSelfTestSign
* Description    : Set Self Test Sign (Disable = st_plus, Enable = st_minus)
* Input          : MEMS_DISABLE/MEMS_ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setSelfTestSign(H3LIS331DL_State_t st_sign);

/*******************************************************************************
* Function Name  : H3LIS331DL_turnONEnable
* Description    : TurnON Mode selection for sleep to wake function
* Input          : H3LIS331DL_SLEEP_TO_WAKE_DIS/H3LIS331DL_SLEEP_TO_WAKE_ENA
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_turnONEnable(H3LIS331DL_Sleep_To_Wake_Conf_t stw);

/*******************************************************************************
* Function Name  : H3LIS331DL_setBOOT
* Description    : Rebot memory content
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setBOOT(H3LIS331DL_State_t boot);

/*******************************************************************************
* Function Name  : H3LIS331DL_setFDS
* Description    : Set Filter Data Selection
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setFDS(H3LIS331DL_State_t fds);

/*******************************************************************************
* Function Name  : H3LIS331DL_setSPI34Wire
* Description    : Set SPI mode
* Input          : Modality by H3LIS331DL_SPIMode_t Typedef (H3LIS331DL_SPI_4_WIRE, H3LIS331DL_SPI_3_WIRE)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setSPI34Wire(H3LIS331DL_SPIMode_t sim);

//Filtering Functions

/*******************************************************************************
* Function Name  : H3LIS331DL_setHPFMode
* Description    : Set High Pass Filter Modality
* Input          : H3LIS331DL_HPM_NORMAL_MODE_RES/H3LIS331DL_HPM_REF_SIGNAL/H3LIS331DL_HPM_NORMAL_MODE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setHPFMode(H3LIS331DL_HPFMode_t hpm);

/*******************************************************************************
* Function Name  : H3LIS331DL_setHPFCutOFF
* Description    : Set High Pass CUT OFF Freq
* Input          : H3LIS331DL_HPFCF [0,3]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setHPFCutOFF(H3LIS331DL_HPFCutOffFreq_t hpf);

H3LIS331DL_Status_t H3LIS331DL_setFilterDataSel(H3LIS331DL_State_t state);

/*******************************************************************************
* Function Name  : H3LIS331DL_setReference
* Description    : Sets Reference register acceleration value as a reference for HP filter
* Input          : Value of reference acceleration value (0-255)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setReference(int8_t ref);

//Interrupt Functions
/*******************************************************************************
* Function Name  : H3LIS331DL_setIntHighLow
* Description    : Set Interrupt active state (Disable = active high, Enable = active low)
* Input          : MEMS_DISABLE/MEMS_ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setIntHighLow(H3LIS331DL_State_t hil);

/*******************************************************************************
* Function Name  : H3LIS331DL_setIntPPOD
* Description    : Set Interrupt Push-Pull/OpenDrain Pad (Disable = Push-Pull, Enable = OpenDrain)
* Input          : MEMS_DISABLE/MEMS_ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setIntPPOD(H3LIS331DL_State_t pp_od);

/*******************************************************************************
* Function Name  : H3LIS331DL_setInt1DataSign
* Description    : Set Data signal Interrupt 1 pad
* Input          : Modality by H3LIS331DL_INT_Conf_t Typedef
                  (H3LIS331DL_INT_SOURCE, H3LIS331DL_INT_1OR2_SOURCE, H3LIS331DL_DATA_READY, H3LIS331DL_BOOT_RUNNING)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setInt1DataSign(H3LIS331DL_INT_Conf_t i_cfg);

/*******************************************************************************
* Function Name  : H3LIS331DL_setInt2DataSign
* Description    : Set Data signal Interrupt 2 pad
* Input          : Modality by H3LIS331DL_INT_Conf_t Typedef
                  (H3LIS331DL_INT_SOURCE, H3LIS331DL_INT_1OR2_SOURCE, H3LIS331DL_DATA_READY, H3LIS331DL_BOOT_RUNNING)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setInt2DataSign(H3LIS331DL_INT_Conf_t i_cfg);

/*******************************************************************************
* Function Name  : H3LIS331DL_setInt1HPEnable
* Description    : Set Interrupt1 hp filter enable/disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* example        : H3LIS331DL_SetInt1HPEnable(MEMS_ENABLE)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setInt1HPEnable(H3LIS331DL_State_t stat);

/*******************************************************************************
* Function Name  : H3LIS331DL_setInt2HPEnable
* Description    : Set Interrupt2 hp filter enable/disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* example        : H3LIS331DL_SetInt2HPEnable(MEMS_ENABLE)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setInt2HPEnable(H3LIS331DL_State_t stat);

/*******************************************************************************
* Function Name  : H3LIS331DL_int1LatchEnable
* Description    : Enable Interrupt 1 Latching function
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_int1LatchEnable(H3LIS331DL_State_t latch);

/*******************************************************************************
* Function Name  : H3LIS331DL_int2LatchEnable
* Description    : Enable Interrupt 2 Latching function
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_int2LatchEnable(H3LIS331DL_State_t latch);

/*******************************************************************************
* Function Name  : H3LIS331DL_resetInt1Latch
* Description    : Reset Interrupt 1 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_resetInt1Latch();

/*******************************************************************************
* Function Name  : H3LIS331DL_resetInt2Latch
* Description    : Reset Interrupt 2 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_resetInt2Latch();

/*******************************************************************************
* Function Name  : H3LIS331DL_setInt1Configuration
* Description    : Interrupt 1 Configuration (without 6D_INT)
* Input          : H3LIS331DL_INT_AND/OR | H3LIS331DL_INT_ZHIE_ENABLE/DISABLE | H3LIS331DL_INT_ZLIE_ENABLE/DISABLE...
* Output         : None
* Note           : You MUST use ALL input variable in the argument, as in example above
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setInt1Configuration(H3LIS331DL_IntConf_t ic);

/*******************************************************************************
* Function Name  : H3LIS331DL_setInt2Configuration
* Description    : Interrupt 2 Configuration (without 6D_INT)
* Input          : H3LIS331DL_INT_AND/OR | H3LIS331DL_INT_ZHIE_ENABLE/DISABLE | H3LIS331DL_INT_ZLIE_ENABLE/DISABLE...
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setInt2Configuration(H3LIS331DL_IntConf_t ic);

/*******************************************************************************
* Function Name  : H3LIS331DL_setInt1Threshold
* Description    : Sets Interrupt 1 Threshold
* Input          : Threshold = [0,127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setInt1Threshold(byte ths);

/*******************************************************************************
* Function Name  : H3LIS331DL_setInt2Threshold
* Description    : Sets Interrupt 2 Threshold
* Input          : Threshold = [0,127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setInt2Threshold(byte ths);

/*******************************************************************************
* Function Name  : H3LIS331DL_setInt1Duration
* Description    : Sets Interrupt 1 Duration
* Input          : Duration = [0,127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setInt1Duration(byte id);

/*******************************************************************************
* Function Name  : H3LIS331DL_setInt2Duration
* Description    : Sets Interrupt 2 Duration
* Input          : Duration = [0,127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setInt2Duration(byte id);

/*******************************************************************************
* Function Name  : H3LIS331DL_setInt1Mode
* Description    : Interrupt 1 Configuration mode (OR, 6D Movement, AND, 6D Position)
* Input          : H3LIS331DL_INT_MODE_OR, H3LIS331DL_INT_MODE_6D_MOVEMENT, H3LIS331DL_INT_MODE_AND, H3LIS331DL_INT_MODE_6D_POSITION
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setInt1Mode(H3LIS331DL_IntMode_t int_mode);

/*******************************************************************************
* Function Name  : H3LIS331DL_setInt2Mode
* Description    : Interrupt 2 Configuration mode (OR, 6D Movement, AND, 6D Position)
* Input          : H3LIS331DL_INT_MODE_OR, H3LIS331DL_INT_MODE_6D_MOVEMENT, H3LIS331DL_INT_MODE_AND, H3LIS331DL_INT_MODE_6D_POSITION
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_setInt2Mode(H3LIS331DL_IntMode_t int_mode);

/*******************************************************************************
* Function Name  : H3LIS331DL_getInt1Src
* Description    : Reset Interrupt 1 Latching function
* Input          : buffer to empty by Int1 Source Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_getInt1Src(byte* val);

/*******************************************************************************
* Function Name  : H3LIS331DL_getInt2Src
* Description    : Reset Interrupt 2 Latching function
* Input          : buffer to empty by Int2 Source Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_getInt2Src(byte* val);

/*******************************************************************************
* Function Name  : H3LIS331DL_getInt1SrcBit
* Description    : Reset Interrupt 1 Latching function
* Input          : H3LIS331DL_INT1_SRC_IA, H3LIS331DL_INT1_SRC_ZH, H3LIS331DL_INT1_SRC_ZL .....
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_getInt1SrcBit(byte statusBIT, byte* val);

/*******************************************************************************
* Function Name  : H3LIS331DL_getInt2SrcBit
* Description    : Reset Interrupt 2 Latching function
* Input          : H3LIS331DL_INT_SRC_IA, H3LIS331DL_INT_SRC_ZH, H3LIS331DL_INT_SRC_ZL .....
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_getInt2SrcBit(byte statusBIT, byte* val);

//Other Reading Functions
/*******************************************************************************
* Function Name  : getStatusReg
* Description    : Read the status register
* Input          : char to empty by Status Reg Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_getStatusReg(byte* val);

/*******************************************************************************
* Function Name  : getStatusBIT
* Description    : Read the status register BIT
* Input          : H3LIS331DL_STATUS_REG_ZYXOR, H3LIS331DL_STATUS_REG_ZOR, H3LIS331DL_STATUS_REG_YOR, H3LIS331DL_STATUS_REG_XOR,
                   H3LIS331DL_STATUS_REG_ZYXDA, H3LIS331DL_STATUS_REG_ZDA, H3LIS331DL_STATUS_REG_YDA, H3LIS331DL_STATUS_REG_XDA,
                   H3LIS331DL_DATAREADY_BIT
* Output         : status register BIT
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_getStatusBit(byte statusBIT, byte* val);

/*******************************************************************************
* Function Name  : H3LIS331DL_getAccAxesRawLegacy
* Description    : Read the Acceleration Values Output Registers
* 				   Function uses single byte I2C reads, hence should not be used
* 				   if multibyte reads are available. H3LIS331DL_getAccAxesRaw which
* 				   uses multi byte i2c reads should be used instead
* Input          : buffer to empty by AccAxesRaw_t Typedef
* Output         : raw acceleration values
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_getAccAxesRawLegacy(AxesRaw_t* buff);

/*******************************************************************************
* Function Name  : H3LIS331DL_getAccAxesRaw
* Description    : Read the Acceleration Values Output Registers using multi byte
* 				   i2c reads
* Input          : buffer to empty by AccAxesRaw_t Typedef
* Output         : raw acceleration values
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_getAccAxesRaw(AxesRaw_t* buff);

/*******************************************************************************
* Function Name  : H3LIS331DL_getAccRaw
* Description    : Read the Acceleration Values Output Registers for single axis
* Input          : axis for which data should be read
* Input          : buffer for acc value
* Output         : raw acceleration value
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_getAccRaw(Axis_t axis,int16_t* x);

/*******************************************************************************
* Function Name  : H3LIS331DL_readXYZ
* Description    : Read the adjusted acceleration values, using previously set adj values
* Input          : buffer to empty by AccAxesRaw_t Typedef
* Output         : Adjusted acceleration values
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_readXYZ(H3LIS331DL_t* device, int16_t* x, int16_t* y, int16_t* z);

/*******************************************************************************
* Function Name  : H3LIS331DL_readXYZ
* Description    : Returns adjusted and scaled acceleration values
* Input          : buffer to empty by AccAxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_getAcceleration(H3LIS331DL_t* device, double* xyz);
/*******************************************************************************
* Function Name  : get6DPositionInt1
* Description    : 6D Interrupt 1 Position Detect
* Input          : Byte to be filled with H3LIS331DL_POSITION_6D_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_get6DPositionInt1(byte* val);

/*******************************************************************************
* Function Name  : get6DPositionInt2
* Description    : 6D Interrupt 2 Position Detect
* Input          : Byte to be filled with H3LIS331DL_POSITION_6D_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_get6DPositionInt2(byte* val);



#endif /*__H3LIS331DL_H */
