#include "main.h"

bool check_system()
{
    // UPDATE CELL VOLTAGES
    updateCellVoltages();

    // UPDATE PACK TEMP

    battery->packTemp = getTemperatureDegC(0);

    int cell_max_voltage = getMaxCellVoltage();
    int cell_min_voltage = getMinCellVoltage();n
    if (placeholder_temp < PACK_OVER_TEMP &&
        cell_max_voltage < MAX_CELL_VOLTAGE &&
        cell_min_voltage > MIN_CELL_VOLTAGE) {
        return true;
    }

    return false;
}

void check_fault(uint8_t sys_stat, uint8_t bit1, uint8_t bit2, uint8_t wait_duration)
{
    // CHECK BITS FOR FAULT
    if (CHECK_BIT(sys_stat, bit1) && CHECK_BIT(sys_stat, bit2)) {

        // SLEEP FOR DURATION AND CHECK SYSTEM
        do {
            HAL_Delay(wait_duration * 1000);
        }
        while(check_system());

        // CLEAR FAULTS
        sys_stat = readRegister(SYS_STAT);
        sys_stat &= (1 << bit1); // CLEAR BIT1
        sys_stat &= (1 << bit2); // CLEAR BIT2
        writeRegister(SYS_STAT, sys_stat);

        // CLOSE CHG AND DSG FETS
        uint8_t sys_ctrl = readRegister(SYS_CTRL2);
        sys_ctrl &= (1 << 0); // SET BIT 0 to 0
        sys_ctrl &= (1 << 1); // SET BIT 1 to 0
        writeRegister(SYS_CTRL2, sys_ctrl);

    }
}

void alert_handler(uint8_t sys_stat)
{

    // KEEP FETS OPEN
    uint8_t sys_ctrl = readRegister(SYS_CTRL2);
    sys_ctrl &= ~(1 << 0); // SET BIT 0 to 0
    sys_ctrl &= ~(1 << 1); // SET BIT 1 to 0
    writeRegister(SYS_CTRL2, sys_ctrl);

    // STOP BALANCING CELL
    uint8_t cellbal1 = readRegister(CELLBAL1);
    cellbal1 &= ~(1 << 0); // SET BIT 0 to 0
    cellbal1 &= ~(1 << 1); // SET BIT 1 to 0
    cellbal1 &= ~(1 << 2); // SET BIT 2 to 0
    cellbal1 &= ~(1 << 3); // SET BIT 3 to 0
    writeRegister(CELLBAL1, cellbal1);

    // CHECK SCD/OCD FAULT BITS 0 AND 1 xxxx xx11
    check_fault(sys_stat, 0, 1, 20); // WAIT 20s

    // CHECK OV/UV FAULT BITS 2 AND 3 xxxx 11xx
    check_fault(sys_stat, 2, 3, 5); // WAIT 5s

    // CHECK XREADY FAULT BIT 5 xx1x xxxx
    check_fault(sys_stat, 5, 5, 5); // WAIT 5s


}
