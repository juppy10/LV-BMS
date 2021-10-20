//
// Created by bbark on 7/04/2021.
//

#include "main.c"
#include "bq76930_func.h"
#include "registers.h"

void writeRegister(uint8_t address, uint8_t data){
    uint8_t buf[3];        
    HAL_StatusTypeDef I2CStat;
    buf[0] = address; buf[1] = data;

    I2CStat = HAL_I2C_Mem_Write(&hi2c1, BQ_I2CADDRESS, buf[0], 1, buf, 1, HAL_MAX_DELAY);
    if (I2CStat!=HAL_OK){
        //Error?
        //Try again?
    }
    //Add CRC later
}

uint8_t readRegister(uint8_t address){
    uint8_t buf[1];
    HAL_StatusTypeDef I2CStat;

    buf[0] = address;
    I2CStat = HAL_I2C_Mem_Read(&hi2c1,BQ_I2CADDRESS,buf[0],1,buf,1,HAL_MAX_DELAY);
    if (I2CStat!=HAL_OK){
        //Error?
        //Try again?
    }
    return buf[0];
}


//Takes address of first register 
//Returns 16-bit int where 4 MSB are first register and 4LSB are second register
uint16_t readRegister_2(uint8_t address){    //needs to be checked, for I2C auto increment
    uint8_t buf[2];
    HAL_StatusTypeDef I2CStat;

    buf[0] = address;
    I2CStat = HAL_I2C_Mem_Read(&hi2c1,BQ_I2CADDRESS,buf[0],1,buf,2,HAL_MAX_DELAY);
    if (I2CStat!=HAL_OK){
        //Error?
        //Try again?
    }
    return (buf[0]<<8) | (buf[1] & 0xff);
}
