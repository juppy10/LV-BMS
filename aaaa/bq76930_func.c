//
// Created by yaboi on 8/04/2021.
//


#include "main.h"
#include "math.h"
#include "bq76930_func.h"
#include "registers.h"
#include <stdlib.h>

struct LiPo_battery{
	float voltage_mV;
	float cell_volt_mV[NUMBER_OF_CELLS];
	int packEnergy_mJ;
	int SOC;
	int packMaxEnergy_mJ;
} LVBatLiPo;

/*long setShortCircuitProtection(long current_mA){
    //need to check notebook
    uint8_t PROTECT_1;
    PROTECT_1 = (uint8_t) readRegister(PROTECT1);
    PROTECT_1 |= (SHORTCIRCUIT_THRESHOLD << SCD_THRESH);
    PROTECT_1 |= (SHORTCIRCUIT_DELAY << SCD_DELAY);
    writeRegister(PROTECT1, PROTECT_1);
}

long setOvercurrentDischargeProtection(long current_mA){
    //need to check notebook
    uint8_t PROTECT_2;
    PROTECT_2 = (uint8_t) readRegister(PROTECT2);
    PROTECT_2 |= (OVERCURRENT_THRESHOLD << OCD_THRESH);
    PROTECT_2 |= (OVERCURRENT_DELAY << OCD_DELAY);
    writeRegister(PROTECT2, PROTECT_2);
}*/

void setCellUndervoltageProtection(int voltage_mV, int delay_s) {
    uint8_t uv_trip;
    uint8_t protecc3Reg, delay_s_code, protecc3New;

    uv_trip = (((voltage_mV - (int8_t) ADCOFFSET) * 1000/ADCGAIN_S) >> 4) & 0x00FF; //need to check
    uv_trip += 1;

    writeRegister(UV_TRIP, uv_trip);

    protecc3Reg = readRegister(PROTECT3) & 0x3F;    //preserve first 6 LSB, set UV bits to 0

    if(delay_s > 16){
        delay_s = 16;
    }//if the delay is greater than 16, change it to 16
    delay_s_code = delay_s / 4;     //quotient with 4
    delay_s_code = (delay_s_code << 6);     //left shift by 6 (to make the 2LSB bits 6 and 7)
    protecc3New = delay_s_code | protecc3Reg;   //or with existing register

    writeRegister(PROTECT3,protecc3New);    //write to register
}

void setCellOvervoltageProtection(int voltage_mV, int delay_s){
    uint8_t uv_trip;
    uint8_t protecc3Reg, delay_s_code, protecc3New;

    uv_trip = (((voltage_mV - (int8_t) ADCOFFSET) * 1000/ADCGAIN_S) >> 4) & 0x00FF; //need to check
    uv_trip += 1;

    writeRegister(UV_TRIP, uv_trip);

    protecc3Reg = readRegister(PROTECT3) & 0xCF;    //preserve bits 4 and 5, set OV bits to 0

    if(delay_s > 16){
        delay_s = 16;
    }//if the delay is greater than 16, change it to 16
    delay_s_code = delay_s / 4;     //quotient with 4
    delay_s_code = (delay_s_code << 4);     //left shift by 6 (to make the 2LSB bits 7 and 8)
    protecc3New = delay_s_code | protecc3Reg;   //or with existing register

    writeRegister(PROTECT3,protecc3New);    //write to register
}

int getBatteryVoltagemV(){
    uint16_t BAT_Volt = readRegister_2(BAT_HI_BYTE);
    float VbatmV = 4*ADCGAIN_S*BAT_Volt*pow(10,-3)+(NUMBER_OF_CELLS*ADCOFFSET);  //determine battery voltage in mV
    LVBatLiPo.voltage_mV = VbatmV;  //update struct
    return VbatmV;
}

//long  getBatteryCurrent(void){}   //might not need it
/*uint8_t getGain() {
    uint8_t ADCGAIN_1, ADCGAIN_2, ADCGAIN_S;
    ADCGAIN_1 = readRegister(ADCGAIN1); //Read adcgain registers
    ADCGAIN_2 = readRegister(ADCGAIN2);
    ADCGAIN_S = ((ADCGAIN_1 << 1u)|(ADCGAIN_2 >> 5u)) & 0x1F; //sets bits 7,6,5 -> 0 preserves all other bits
    return ADCGAIN_S;

}*/

// TAKES ARRAY OF LENGTH "NUMBER_OF_CELLS" AND UPDATES THE VALUES IN THE ARRAY
void updateCellVoltages() {
    // V(cell) = GAIN x ADC(cell) + OFFSET
    for (uint8_t i = 0; i < NUMBER_OF_CELLS; i++) {
    	LVBatLiPo.cell_volt_mV[i] = getCellVoltage_mV(i + 1) * ADCGAIN_S*pow(10,-3) + ADCOFFSET; // CELL ID STARTS AT 1
    }
}

int getCellVoltage_mV(int idCell) {
    uint8_t v_hi, v_lo;
    v_hi = (uint8_t) readRegister(CELL_ADDRESS(idCell));
    v_lo = (uint8_t) readRegister(CELL_ADDRESS(idCell) + 1);
    int adc = (v_hi & 0b00111111u) << 8 | v_lo;
    return adc;
}

int getMinCellVoltage(void){
    int minCellVoltage_cell=1;
    updateCellVoltages();

    for(int i=1; i>NUMBER_OF_CELLS; i++){
        if(LVBatLiPo.cell_volt_mV[i] < LVBatLiPo.cell_volt_mV[i-1]){
            minCellVoltage_cell=i;
        }
    }
    return minCellVoltage_cell;
}

int getMaxCellVoltage(void){
    int maxCellVoltage_cell=1;
    updateCellVoltages();

    for(int i=1; i>NUMBER_OF_CELLS; i++){
        if(LVBatLiPo.cell_volt_mV[i] > LVBatLiPo.cell_volt_mV[i-1]){
            maxCellVoltage_cell=i;
        }
    }
    return maxCellVoltage_cell;
}

/*float getTemperatureDegC(int isThermistor){
    int Rts, Temp , ADCTemp = readRegister_2(TS1_HI_BYTE);
    ADCTemp *= 382/1000000;

    if(isThermistor){
        Rts = (10000*ADCTemp)/(3.3-ADCTemp);
        Temp = NTCBeta/log(Rts/(10000*pow(M_E,-NTCBeta/25))); //convert resistance to temp using formula for beta - NEED TO CHECK
    }else{
        Temp = 25-((ADCTemp-1.2)/0.0042);       //if measuring the die temp, this is the formula for the temperature
    }
    return Temp;
}*/

void updateCC(void){
    //assume that the CC_Ready bit has been detected high
    int CCmA, EnergyCCmJ, currBatVoltagemV = getBatteryVoltagemV();   //battery voltage
    int16_t currCCReg = readRegister_2(CC_HI_BYTE);   //current
    CCmA=currCCReg*8.44/shunt_mOhm;    //value in mA (8.44 from datasheet)
    if(abs(CCmA)<10){
        CCmA=0;
    }else{
        EnergyCCmJ=CCmA*currBatVoltagemV/1000*deltaT_ms/1000;  //energy since last CC reading       CHECK THIS AGAIN
        LVBatLiPo.packEnergy_mJ += EnergyCCmJ;
        LVBatLiPo.SOC = (LVBatLiPo.packEnergy_mJ/LVBatLiPo.packMaxEnergy_mJ)*100;
    }
}

/*void balance_charge(void)
{
    int max_cell = getMaxCellVoltage();
    int min_cell = getMinCellVoltage();

    while(LVBatLiPo.cell_volt_mV[max_cell] < 4.2) {
        while ((max_cell - min_cell) > CELL_IMBALANCE_THRESHOLD &&
               battery->cellVoltage[max_cell] > 4) {
            // START CELL BALANCE ON MAX CELL
            writeRegister(CELLBAL1, (1 << max_cell_voltage));
            HAL_Delay(wait_duration * 1000);

        }
        // START CELL BALANCE ON MAX CELL
        HAL_Delay(wait_duration * 1000);
    }
}*/
