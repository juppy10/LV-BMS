//
// Created by bbark on 7/04/2021.
//

#include "main.h"

void writeRegister(uint8_t address, uint8_t data){
    uint8_t buf[3];        //is it meant to be uint or char??
    HAL_StatusTypeDef I2CStat;
    buf[0] = address; buf[1] = data;

    I2CStat = HAL_I2C_Master_Transmit(&hi2c1,BQ_I2CADDRESS,buf,2,HAL_MAX_DELAY);
    if (I2CStat!=HAL_OK){
        //Error?
        //Try again?
    }
    //Add CRC later
}

int readRegister(uint8_t address){
    uint8_t buf[1];
    HAL_StatusTypeDef I2CStat;

    buf[0] = address;
    HAL_I2C_Master_Transmit(&hi2c1,BQ_I2CADDRESS,buf,1,HAL_MAX_DELAY);
    I2CStat = HAL_I2C_Master_Receive(&hi2c1,BQ_I2CADDRESS,buf,1,HAL_MAX_DELAY);
    if (I2CStat!=HAL_OK){
        //Error?
        //Try again?
    }
    return buf[0];
}

int readRegister_2(uint8_t address){    //needs to be checked, for I2C auto increment
    uint8_t buf[2];
    int doubleData;
    HAL_StatusTypeDef I2CStat;

    buf[0] = address;
    HAL_I2C_Master_Transmit(&hi2c1,BQ_I2CADDRESS,buf,1,HAL_MAX_DELAY);
    I2CStat = HAL_I2C_Master_Receive(&hi2c1,BQ_I2CADDRESS,buf,2,HAL_MAX_DELAY);
    if (I2CStat!=HAL_OK){
        //Error?
        //Try again?
    }
    doubleData = (buf[0]<<8)|(buf[1]);  //concatenate the 2 bytes of data

    return doubleData;
}
