//
// Created by yaboi on 29/03/2021.
//

#ifndef CODE_REGISTERS_H
#define CODE_REGISTERS_H

#define BQ_I2CADDRESS   0x08 << 1
#define SYS_STAT        0x00
#define CELLBAL1        0x01
#define SYS_CTRL1       0x04
#define SYS_CTRL2       0x05
#define PROTECT1        0x06
#define PROTECT2        0x07
#define PROTECT3        0x08
#define OV_TRIP         0x09
#define UV_TRIP         0x0A
#define CC_CFG          0x0B

#define VC1_HI_BYTE     0x0C
#define VC1_LO_BYTE     0x0D
#define VC2_HI_BYTE     0x0E
#define VC2_LO_BYTE     0x0F
#define VC3_HI_BYTE     0x10
#define VC3_LO_BYTE     0x11
#define VC4_HI_BYTE     0x12
#define VC4_LO_BYTE     0x13

// MAPS 1,2,3,4 TO THEIR RESPECTIVE HIGH BYTES REGISTERS
// LOW BYTE REGISTER IS THE ADDRESS + 1
#define CELL_ADDRESS(x) (x - 1)*2 + 0x0C


#define BAT_HI_BYTE     0x2A
#define BAT_LO_BYTE     0x2B

#define TS1_HI_BYTE     0x2C
#define TS1_LO_BYTE     0x2D
#define CC_HI_BYTE      0x32
#define CC_LO_BYTE      0x33

#define ADCGAIN1        0x50    //0b0101 0000
#define ADCGAIN2        0x59    //0b0101 1001
#define ADCGAIN_S       0x69    //Set this before prog.
#define ADCOFFSET       0x51    //0b0101 0001

// for bit clear operations of the SYS_STAT register
#define STAT_CC_READY           (0x80)
#define STAT_DEVICE_XREADY      (0x20)
#define STAT_OVRD_ALERT         (0x10)
#define STAT_UV                 (0x08)
#define STAT_OV                 (0x04)
#define STAT_SCD                (0x02)
#define STAT_OCD                (0x01)
#define STAT_FLAGS              (0x3F)

// REGISTER OFFSETS
#define OCD_DELAY 4u
#define OCD_THRESH 0u
#define SCD_DELAY 3u
#define SCD_THRESH 0u

// FUNCTION DECLARATION


uint8_t readRegister(uint8_t address);
void writeRegister(uint8_t address, uint8_t data);
uint16_t readRegister_2(uint8_t address);

#endif //CODE_REGISTERS_H

