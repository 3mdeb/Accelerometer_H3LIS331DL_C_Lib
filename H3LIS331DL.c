/*
 * H3LIS331DL.c
 * A library for 3-Axis Digital Accelerometer(Â±400g)
 *  
 * Copyright (c) 2014 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : lawliet zou
 * Create Time: April 2014
 * Change Log :
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
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
 
#include "H3LIS331DL.h"

//
// I2C single byte read function used by library
//
static uint8_t I2CReadReg(byte deviceAddr, byte regAddr, byte* val)
{
	//
	// Implement i2c single byte read here
	//
	return MEMS_ERROR;
}

//
// I2C multi byte read function used by library
//
static uint8_t I2CMultiReadReg(byte deviceAddr, byte regAddr, byte* bBuff, unsigned char ucLen)
{
	//
	// Implement i2c multi byte byte read here
	//
	return MEMS_ERROR;
}

//
// I2C single byte write function used by library
//
static uint8_t I2CWriteReg(byte deviceAddr, byte regAddr, byte val)
{
	//
	// Implement i2c single byte write function here
	//
    return MEMS_ERROR;
}

H3LIS331DL_Status_t H3LIS331DL_init(H3LIS331DL_t* device,
		H3LIS331DL_ODR_t  odr,
		H3LIS331DL_Mode_t mode,
		H3LIS331DL_Fullscale_t fullScale){

	H3LIS331DL_Status_t ret;

    device->_adjVal[0] = 0;
    device->_adjVal[1] = 0;
    device->_adjVal[2] = 0;

    //set output data rate
    ret = H3LIS331DL_setODR(odr);
    if (ret == MEMS_ERROR)
    {
    	return MEMS_ERROR;
    }
    //set PowerMode 
    H3LIS331DL_setMode(mode);
    if (ret == MEMS_ERROR)
    {
    	return MEMS_ERROR;
    }

    //set Fullscale
    H3LIS331DL_setFullScale(fullScale);
    if (ret == MEMS_ERROR)
    {
    	return MEMS_ERROR;
    }

    //set axis Enable
    H3LIS331DL_setAxis(H3LIS331DL_X_ENABLE | H3LIS331DL_Y_ENABLE |  H3LIS331DL_Z_ENABLE);
    if (ret == MEMS_ERROR)
    {
    	return MEMS_ERROR;
    }

    return MEMS_SUCCESS;
}

void H3LIS331DL_setParams(H3LIS331DL_t* device, int16_t val_x, int16_t val_y, int16_t val_z, double gains)
{
	device->_adjVal[0] = val_x;
	device->_adjVal[1] = val_y;
	device->_adjVal[2] = val_z;
	device->_gains = gains;
}

H3LIS331DL_Status_t H3LIS331DL_readXYZ(H3LIS331DL_t* device, int16_t* x, int16_t* y, int16_t* z){
    //get Acceleration Raw data  
    AxesRaw_t data;
    H3LIS331DL_Status_t ret = H3LIS331DL_getAccAxesRaw(&data);

    if(MEMS_SUCCESS == ret){
        *x = (data.AXIS_X - device->_adjVal[0]);
        *y = (data.AXIS_Y - device->_adjVal[1]);
        *z = (data.AXIS_Z - device->_adjVal[2]);
        return MEMS_SUCCESS;
    }
    return MEMS_ERROR;
}

H3LIS331DL_Status_t H3LIS331DL_getAcceleration(H3LIS331DL_t* device, double* xyz){
	H3LIS331DL_Status_t ret;
    AxesRaw_t data;
    ret = H3LIS331DL_getAccAxesRaw(&data);
    if (ret == MEMS_ERROR)
    {
    	return MEMS_ERROR;
    }
    
    xyz[0] = (data.AXIS_X - device->_adjVal[0]) * device->_gains;
    xyz[1] = (data.AXIS_Y - device->_adjVal[1]) * device->_gains;
    xyz[2] = (data.AXIS_Z - device->_adjVal[2]) * device->_gains;
    return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_getWHO_AM_I(byte* val){
  
    if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_WHO_AM_I, val) )
        return MEMS_ERROR;
  
    return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_getCtrlReg1(byte* val){
    if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG1, val) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_getCtrlReg3(byte* val){
    if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, val) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setODR(H3LIS331DL_ODR_t dr){
    byte value;
  
    if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG1, &value) )
        return MEMS_ERROR;
  
    value &= 0xE7;
    value |= dr<<H3LIS331DL_DR;
  
    if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG1, value) )
        return MEMS_ERROR;
  
    return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setMode(H3LIS331DL_Mode_t pm) {
    byte value;
  
    if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG1, &value) )
        return MEMS_ERROR;
  
    value &= 0x1F;
    value |= (pm<<H3LIS331DL_PM);   
  
    if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG1, value) )
        return MEMS_ERROR;
  
    return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setAxis(H3LIS331DL_Axis_t axis) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG1, &value) )
    return MEMS_ERROR;
  
  value &= 0xF8;
  value |= (0x07 & axis);
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG1, value) )
    return MEMS_ERROR;   
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setFullScale(H3LIS331DL_Fullscale_t fs) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, &value) )
    return MEMS_ERROR;
  
  value &= 0xCF;    
  value |= (fs<<H3LIS331DL_FS);
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setBDU(H3LIS331DL_State_t bdu) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, &value) )
    return MEMS_ERROR;
  
  value &= 0x7F;
  value |= (bdu<<H3LIS331DL_BDU);
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setBLE(H3LIS331DL_Endianess_t ble) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, &value) )
    return MEMS_ERROR;
  
  value &= 0xBF;    
  value |= (ble<<H3LIS331DL_BLE);
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setFDS(H3LIS331DL_State_t fds) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, &value) )
    return MEMS_ERROR;
  
  value &= 0xEF;    
  value |= (fds<<H3LIS331DL_FDS);
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setBOOT(H3LIS331DL_State_t boot) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, &value) )
    return MEMS_ERROR;
  
  value &= 0x7F;    
  value |= (boot<<H3LIS331DL_BOOT);
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setSelfTest(H3LIS331DL_State_t st) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, &value) )
    return MEMS_ERROR;
  
  value &= 0xFD;
  value |= (st<<H3LIS331DL_ST);
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setSelfTestSign(H3LIS331DL_State_t st_sign) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, &value) )
    return MEMS_ERROR;
  
  value &= 0xF7;
  value |= (st_sign<<H3LIS331DL_ST_SIGN);
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setIntHighLow(H3LIS331DL_State_t ihl) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, &value) )
    return MEMS_ERROR;
  
  value &= 0x7F;
  value |= (ihl<<H3LIS331DL_IHL);
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setIntPPOD(H3LIS331DL_State_t pp_od) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, &value) )
    return MEMS_ERROR;
  
  value &= 0xBF;
  value |= (pp_od<<H3LIS331DL_PP_OD);
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setInt1DataSign(H3LIS331DL_INT_Conf_t i_cfg) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, &value) )
    return MEMS_ERROR;
  
  value &= 0xFC;
  value |= (i_cfg<<H3LIS331DL_I1_CFG);
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setInt2DataSign(H3LIS331DL_INT_Conf_t i_cfg) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, &value) )
    return MEMS_ERROR;
  
  value &= 0xE7;
  value |= (i_cfg<<H3LIS331DL_I2_CFG);
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setSPI34Wire(H3LIS331DL_SPIMode_t sim) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, &value) )
    return MEMS_ERROR;
  
  value &= 0xFE;
  value |= (sim<<H3LIS331DL_SIM);
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_turnONEnable(H3LIS331DL_Sleep_To_Wake_Conf_t stw) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG5, &value) )
    return MEMS_ERROR;
  
  value &= 0x00;
  value |= (stw<<H3LIS331DL_TURN_ON);
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG5, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

#if 0
/*******************************************************************************
* Function Name  : HPFilterReset
* Description    : Reading register for reset the content of internal HP filter
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
H3LIS331DL_Status_t H3LIS331DL_HPFilterReset(void) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_HP_FILTER_RESET, &value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}
#endif


H3LIS331DL_Status_t H3LIS331DL_setReference(int8_t ref) {
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_REFERENCE_REG, ref) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setHPFMode(H3LIS331DL_HPFMode_t hpm) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, &value) )
    return MEMS_ERROR;
  
  value &= 0x9F;
  value |= (hpm<<H3LIS331DL_HPM);
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


H3LIS331DL_Status_t H3LIS331DL_setHPFCutOFF(H3LIS331DL_HPFCutOffFreq_t hpf) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, &value) )
    return MEMS_ERROR;
  
  value &= 0xFC;
  value |= (hpf<<H3LIS331DL_HPCF);
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
  
}

H3LIS331DL_Status_t H3LIS331DL_setInt2HPEnable(H3LIS331DL_State_t stat) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, &value) )
    return MEMS_ERROR;
  
  value &= 0xF7;
  value |= stat<<H3LIS331DL_HPEN2 ;
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}     

H3LIS331DL_Status_t H3LIS331DL_setInt1HPEnable(H3LIS331DL_State_t stat) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, &value) )
    return MEMS_ERROR;
  
  value &= 0xFB;
  value |= stat<<H3LIS331DL_HPEN1 ;
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}  

H3LIS331DL_Status_t H3LIS331DL_int1LatchEnable(H3LIS331DL_State_t latch) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, &value) )
    return MEMS_ERROR;
  
  value &= 0xFB;
  value |= latch<<H3LIS331DL_LIR1;
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_int2LatchEnable(H3LIS331DL_State_t latch) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, &value) )
    return MEMS_ERROR;
  
  value &= 0xDF;
  value |= latch<<H3LIS331DL_LIR2;
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_resetInt1Latch() {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_SRC, &value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_resetInt2Latch() {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_SRC, &value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setInt1Configuration(H3LIS331DL_IntConf_t ic) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_CFG, &value) )
    return MEMS_ERROR;
  
  value &= 0x40; 
  value |= ic;
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_CFG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setInt2Configuration(H3LIS331DL_IntConf_t ic) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_CFG, &value) )
    return MEMS_ERROR;
  
  value &= 0x40; 
  value |= ic;
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_CFG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setInt1Mode(H3LIS331DL_IntMode_t int_mode) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_CFG, &value) )
    return MEMS_ERROR;
  
  value &= 0x3F; 
  value |= (int_mode<<H3LIS331DL_INT_6D);
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_CFG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setInt2Mode(H3LIS331DL_IntMode_t int_mode) {
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_CFG, &value) )
    return MEMS_ERROR;
  
  value &= 0x3F; 
  value |= (int_mode<<H3LIS331DL_INT_6D);
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_CFG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_get6DPositionInt1(byte* val){
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_SRC, &value) )
    return MEMS_ERROR;
  
  value &= 0x7F;
  
  switch (value){
  case H3LIS331DL_UP_SX:   
    *val = H3LIS331DL_UP_SX;    
    break;
  case H3LIS331DL_UP_DX:   
    *val = H3LIS331DL_UP_DX;    
    break;
  case H3LIS331DL_DW_SX:   
    *val = H3LIS331DL_DW_SX;    
    break;
  case H3LIS331DL_DW_DX:   
    *val = H3LIS331DL_DW_DX;    
    break;
  case H3LIS331DL_TOP:     
    *val = H3LIS331DL_TOP;      
    break;
  case H3LIS331DL_BOTTOM:  
    *val = H3LIS331DL_BOTTOM;  
    break;
  }
  
  return MEMS_SUCCESS;  
}

H3LIS331DL_Status_t H3LIS331DL_get6DPositionInt2(byte* val){
  byte value;
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_SRC, &value) )
    return MEMS_ERROR;
  
  value &= 0x7F;
  
  switch (value){
  case H3LIS331DL_UP_SX:   
    *val = H3LIS331DL_UP_SX;    
    break;
  case H3LIS331DL_UP_DX:   
    *val = H3LIS331DL_UP_DX;    
    break;
  case H3LIS331DL_DW_SX:   
    *val = H3LIS331DL_DW_SX;    
    break;
  case H3LIS331DL_DW_DX:   
    *val = H3LIS331DL_DW_DX;    
    break;
  case H3LIS331DL_TOP:     
    *val = H3LIS331DL_TOP;      
    break;
  case H3LIS331DL_BOTTOM:  
    *val = H3LIS331DL_BOTTOM;   
    break;
  }
  
  return MEMS_SUCCESS;  
}

H3LIS331DL_Status_t H3LIS331DL_setInt1Threshold(byte ths) {
  if (ths > 127)
    return MEMS_ERROR;
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_THS, ths) )
    return MEMS_ERROR;    
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setInt1Duration(byte id) {
  if (id > 127)
    return MEMS_ERROR;
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_DURATION, id) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setInt2Threshold(byte ths) {
  if (ths > 127)
    return MEMS_ERROR;
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_THS, ths) )
    return MEMS_ERROR;    
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_setInt2Duration(byte id) {
  if (id > 127)
    return MEMS_ERROR;
  
  if( !I2CWriteReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_DURATION, id) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


H3LIS331DL_Status_t H3LIS331DL_getStatusReg(byte* val) {
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_STATUS_REG, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}

H3LIS331DL_Status_t H3LIS331DL_getStatusBit(byte statusBIT, byte *val) {
  byte value;  
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_STATUS_REG, &value) )
    return MEMS_ERROR;
  
  switch (statusBIT){
  case H3LIS331DL_STATUS_REG_ZYXOR:     
    if(value &= H3LIS331DL_STATUS_REG_ZYXOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  case H3LIS331DL_STATUS_REG_ZOR:       
    if(value &= H3LIS331DL_STATUS_REG_ZOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  case H3LIS331DL_STATUS_REG_YOR:       
    if(value &= H3LIS331DL_STATUS_REG_YOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }                                 
  case H3LIS331DL_STATUS_REG_XOR:       
    if(value &= H3LIS331DL_STATUS_REG_XOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  case H3LIS331DL_STATUS_REG_ZYXDA:     
    if(value &= H3LIS331DL_STATUS_REG_ZYXDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  case H3LIS331DL_STATUS_REG_ZDA:       
    if(value &= H3LIS331DL_STATUS_REG_ZDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  case H3LIS331DL_STATUS_REG_YDA:       
    if(value &= H3LIS331DL_STATUS_REG_YDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  case H3LIS331DL_STATUS_REG_XDA:       
    if(value &= H3LIS331DL_STATUS_REG_XDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }                                      
  }
  
  return MEMS_ERROR;
}

H3LIS331DL_Status_t H3LIS331DL_getAccAxesRaw(AxesRaw_t* sample) {
	byte bBuff[6];
	uint8_t res;
	byte regAddr = H3LIS331DL_OUT_X_L;

	//
	// Set most significant bit for auto-increment
	// H3LIS331DL spec:
	// If the MSB of the SUB field is ‘1’, the SUB (register address) is automatically increased to
	// allow multiple data read/write
	//
	regAddr |= 0x80;

	res = I2CMultiReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, regAddr, bBuff, 6);
    if (res == MEMS_ERROR)
    {
    	return MEMS_ERROR;
    }

    sample->AXIS_X = (bBuff[1]<<8)|bBuff[0];

    sample->AXIS_Y = (bBuff[3]<<8)|bBuff[2];

    sample->AXIS_Z = (bBuff[5]<<8)|bBuff[4];

    return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_getAccRaw(Axis_t axis, int16_t* val) {
	byte bBuff[2];
	uint8_t res;
	byte regAddr;

	switch (axis)
	{
	case X_AXIS: regAddr = H3LIS331DL_OUT_X_L; break;
	case Y_AXIS: regAddr = H3LIS331DL_OUT_Y_L; break;
	case Z_AXIS: regAddr = H3LIS331DL_OUT_Z_L; break;
	default: return MEMS_ERROR;
	}

	//
	// Set most significant bit for auto-increment
	// H3LIS331DL spec:
	// If the MSB of the SUB field is ‘1’, the SUB (register address) is automatically increased to
	// allow multiple data read/write
	//
	regAddr |= 0x80;

	res = I2CMultiReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, regAddr, bBuff, 2);
    if (res == MEMS_ERROR)
    {
    	return MEMS_ERROR;
    }

    val = (bBuff[1]<<8)|bBuff[0];

    return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_getAccAxesRawLegacy(AxesRaw_t* buff) {
    byte valueL = 0,valueH = 0;

    I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_OUT_X_L, &valueL);
    I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_OUT_X_H, &valueH);

    buff->AXIS_X = (valueH<<8)|valueL;

    I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_OUT_Y_L, &valueL);
    I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_OUT_Y_H, &valueH);

    buff->AXIS_Y = (valueH<<8)|valueL;

    I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_OUT_Z_L, &valueL);
    I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_OUT_Z_H, &valueH);

    buff->AXIS_Z = (valueH<<8)|valueL;

    return MEMS_SUCCESS;  
}

H3LIS331DL_Status_t H3LIS331DL_getInt1Src(byte* val) {
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_SRC, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

H3LIS331DL_Status_t H3LIS331DL_getInt2Src(byte* val) {
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_SRC, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


H3LIS331DL_Status_t H3LIS331DL_getInt1SrcBit(byte statusBIT, byte *val) {
  byte value;  
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_SRC, &value) )
    return MEMS_ERROR;
  
  if(statusBIT == H3LIS331DL_INT_SRC_IA){
    if(value &= H3LIS331DL_INT_SRC_IA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }    
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_ZH){
    if(value &= H3LIS331DL_INT_SRC_ZH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_ZL){
    if(value &= H3LIS331DL_INT_SRC_ZL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_YH){
    if(value &= H3LIS331DL_INT_SRC_YH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_YL){
    if(value &= H3LIS331DL_INT_SRC_YL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_XH){
    if(value &= H3LIS331DL_INT_SRC_XH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_XL){
    if(value &= H3LIS331DL_INT_SRC_XL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }    
  } 
  return MEMS_ERROR;
}

H3LIS331DL_Status_t H3LIS331DL_getInt2SrcBit(byte statusBIT, byte *val) {
  byte value;  
  
  if( !I2CReadReg(H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_SRC, &value) )
    return MEMS_ERROR;
  
  if(statusBIT == H3LIS331DL_INT_SRC_IA){
    if(value &= H3LIS331DL_INT_SRC_IA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_ZH){
    if(value &= H3LIS331DL_INT_SRC_ZH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_ZL){
    if(value &= H3LIS331DL_INT_SRC_ZL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_YH){
    if(value &= H3LIS331DL_INT_SRC_YH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }    
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_YL){
    if(value &= H3LIS331DL_INT_SRC_YL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_XH){
    if(value &= H3LIS331DL_INT_SRC_XH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_XL){
    if(value &= H3LIS331DL_INT_SRC_XL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }    
  } 
  return MEMS_ERROR;
}


