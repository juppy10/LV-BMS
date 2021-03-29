//
// Created by Will on 29/03/2021.
//

#ifndef CODE_FUNCTIONS_H
#define CODE_FUNCTIONS_H

//hardware settings
void setShuntResistorValue(int res_mOhm);
void setThermistorBetaValue(int beta_K);

//battery protection limit settings
void setTemperatureLimits(int max_degC);
long setShortCircuitProtection(long current_mA);
long setOvercurrentChargeProtection(long current_mA);
long setOvercurrentDischargeProtection(long current_mA);
int setCellUndervoltageProtection(int voltage_mV);
int setCellOvervoltageProtection(int voltage_mV);

//balance settings
void setBalancingThresholds(need to puts things in here);

//battery status
long  getBatteryCurrent(void);
long  getBatteryVoltage(void);
int  getCellVoltage(int idCell);
int  getMinCellVoltage(void);
int  getMaxCellVoltage(void);
float getTemperatureDegC(void);

#endif //CODE_FUNCTIONS_H
