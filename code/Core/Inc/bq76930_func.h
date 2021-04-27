//
// Created by yaboi on 8/04/2021.
//

#ifndef CODE_BQ76930_FUNC_H
#define CODE_BQ76930_FUNC_H

#define shunt_mOhm 4
#define deltaT_ms 250  //time difference between CC calculations in ms
#define max_temp_degC 60
#define OC_C_limit_A 10 //charge limit
#define OC_D_limit_A 30 //discharge limit - double check
#define UV_limit_mV 3000
#define OV_limit_mV 4200
#define BALANCE_THRES_mV 20 //20mV
#define NTCBeta 3970    //thermistor beta value

//battery protection limit settings
void setTemperatureLimits(int max_degC);
long setShortCircuitProtection(long current_mA);
long setOvercurrentDischargeProtection(long current_mA);
int setCellUndervoltageProtection(int voltage_mV, int delay_s);
int setCellOvervoltageProtection(int voltage_mV, int delay_s);

uint8_t getGain();

//battery status
long  getBatteryCurrent(void);
int  getBatteryVoltagemV(void);
int  updateCellVoltages();
int  getCellVoltage_mV(int idCell);
int  getMinCellVoltage(void);
int  getMaxCellVoltage(void);
float getTemperatureDegC(int isThermistor);
void updateCC(void);

#endif //CODE_BQ76930_FUNC_H
