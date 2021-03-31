//
// Created by Will on 29/03/2021.
//

#ifndef CODE_FUNCTIONS_H
#define CODE_FUNCTIONS_H

#define shunt_mOhm 4
#define max_temp_degC 60
#define OC_C_limit_A 10 //charge limit
#define OC_D_limit_A 30 //discharge limit - double check
#define UV_limit_mV 3000
#define OV_limit_mV 4200
#define BALANCE_THRES_mV 20 //20mV

//battery protection limit settings
void setTemperatureLimits(int max_degC);
long setShortCircuitProtection(long current_mA);
long setOvercurrentDischargeProtection(long current_mA);
int setCellUndervoltageProtection(int voltage_mV);
int setCellOvervoltageProtection(int voltage_mV);

//battery status
long  getBatteryCurrent(void);
uint16_t  getBatteryVoltage(void);
int  getCellVoltage(int idCell);
int  getMinCellVoltage(void);
int  getMaxCellVoltage(void);
float getTemperatureDegC(void);

//communication
void writeRegister(byte address, int data);
uint8_t readRegister(byte address);

#endif //CODE_FUNCTIONS_H
