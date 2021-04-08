//
// Created by yaboi on 8/04/2021.
//


long setShortCircuitProtection(long current_mA){
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
}

int setCellUndervoltageProtection(int voltage_mV, int delay_s) {
    uint8_t ADCGAIN_1, ADCGAIN_2, ADCGAIN_S, uv_trip;
    int8_t adc_offset;
    uint8_t protecc3Reg, delay_s_code, protecc3New;

    ADCGAIN_1 = readRegister(ADCGAIN1); //Read adcgain registers
    ADCGAIN_2 = readRegister(ADCGAIN2);
    ADCGAIN_S = (uint8_t) ((ADCGAIN_1 << 1)|(ADCGAIN_2 >> 5)) & 0x1F; //sets bits 7,6,5 -> 0 preserves all other bits

    adc_offset = (int8_t) readRegister(ADCOFFSET);

    uv_trip = (((voltage_mV - adc_offset) * 1000/ADCGAIN_S) >> 4) & 0x00FF; //need to check
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

int setCellOvervoltageProtection(int voltage_mV, int delay_s){
    uint8_t ADCGAIN_1, ADCGAIN_2, ADCGAIN_S, ov_trip;
    int8_t adc_offset;
    uint8_t protecc3Reg, delay_s_code, protecc3New;

    ADCGAIN_1 = readRegister(ADCGAIN1); //Read adcgain registers
    ADCGAIN_2 = readRegister(ADCGAIN2);
    ADCGAIN_S = ((ADCGAIN_1 << 1)|(ADCGAIN_2 >> 5)) & 0x1F; //sets bits 7,6,5 -> 0 preserves all other bits

    adc_offset = (int8_t) readRegister(ADCOFFSET);

    ov_trip = (((voltage_mV - adc_offset) * 1000/ADCGAIN_S) >> 4) & 0x00FF; //need to check

    writeRegister(OV_TRIP, ov_trip);

    protecc3Reg = readRegister(PROTECT3) & 0xCF;    //preserve bits 4 and 5, set OV bits to 0

    if(delay_s > 16){
        delay_s = 16;
    }//if the delay is greater than 16, change it to 16
    delay_s_code = delay_s / 4;     //quotient with 4
    delay_s_code = (delay_s_code << 4);     //left shift by 6 (to make the 2LSB bits 7 and 8)
    protecc3New = delay_s_code | protecc3Reg;   //or with existing register

    writeRegister(PROTECT3,protecc3New);    //write to register
}

uint16_t getBatteryVoltage(void){
    uint8_t HI_BYTE, LO_BYTE;
    uint16_t BAT_Volt;

    HI_BYTE = readRegister(BAT_HI_BYTE);
    LO_BYTE = readRegister(BAT_LO_BYTE);

    BAT_Volt = HI_BYTE << 8 | LO_BYTE;
    return BAT_Volt;
}

//long  getBatteryCurrent(void){}   //might not need it
uint8_t getGain() {
    uint8_t ADCGAIN_1, ADCGAIN_2, ADCGAIN_S;
    ADCGAIN_1 = readRegister(ADCGAIN1); //Read adcgain registers
    ADCGAIN_2 = readRegister(ADCGAIN2);
    ADCGAIN_S = ((ADCGAIN_1 << 1u)|(ADCGAIN_2 >> 5u)) & 0x1F; //sets bits 7,6,5 -> 0 preserves all other bits
    return ADCGAIN_S;

}

// TAKES ARRAY OF LENGTH "NUMBER_OF_CELLS" AND UPDATES THE VALUES IN THE ARRAY
int updateCellVoltages(int *voltages) {
    for (uint8_t i = 0; i < NUMBER_OF_CELLS; i++) {
        voltages[i] = getCellVoltage(i + 1); // CELL ID STARTS AT 1
    }
    return 0;
}

int  getCellVoltage(int idCell) {
    // V(cell) = GAIN x ADC(cell) + OFFSET
    uint8_t v_hi, v_lo;
    v_hi = (uint8_t) readRegister(CELL_ADDRESS(idCell));
    v_lo = (uint8_t) readRegister(CELL_ADDRESS(idCell) + 1);
    long adc = (v_hi & 0b00111111u) << 8 | v_lo;
    int8_t adc_offset = readRegister(ADCOFFSET);
    uint8_t gain = getGain();
    return adc * gain / 1000 + adc_offset;
}

int  getMinCellVoltage(void){

}

int  getMaxCellVoltage(void){

}

float getTemperatureDegC(void){

}
