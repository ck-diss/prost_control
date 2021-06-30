#ifndef ADS_H
#define ADS_H

#include <bcm2835.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>

#define   ADS_PWDN_PIN  RPI_V2_GPIO_P1_15
#define   ADS_CS_PIN    RPI_V2_GPIO_P1_24
#define   ADS_RESET_PIN RPI_V2_GPIO_P1_13
#define   ADS_START_PIN RPI_V2_GPIO_P1_11
#define   ADS_DRDY_PIN  RPI_V2_GPIO_P1_12

// fclk = 2.048 Mhz, TCLK = 488.2815 nanoseconds
#define TCLK 0.4882815 // microseconds
// ADS COMMANDS
#define   ADS_WAKEUP        0x02
#define   ADS_STANDBY       0x04
#define   ADS_RESET         0x06
#define   ADS_START         0x08
#define   ADS_STOP          0x0A
#define   ADS_RDATAC        0x10
#define   ADS_RDATA         0x12
#define   ADS_SDATAC        0x11
#define   ADS_RREG          0x20
#define   ADS_WREG          0x40

// ADS REGISTER MAPs
#define   ADS_ID            0x00
#define   ADS_CONFIG1       0x01
#define   ADS_CONFIG2       0x02
#define   ADS_CONFIG3       0x03
#define   ADS_LOFF          0x04
#define   ADS_CH1SET        0x05
#define   ADS_CH2SET        0x06
#define   ADS_CH3SET        0x07
#define   ADS_CH4SET        0x08
#define   ADS_CH5SET        0x09
#define   ADS_CH6SET        0x0A
#define   ADS_CH7SET        0x0B
#define   ADS_CH8SET        0x0C
#define   ADS_RLD_SENSP     0x0D
#define   ADS_RLD_SENSN     0x0E
#define   ADS_LOFF_SENSP    0x0F
#define   ADS_LOFF_SENSN    0x10
#define   ADS_LOFF_FLIP     0x11
#define   ADS_LOFF_STATP    0x12
#define   ADS_LOFF_STATN    0x13
#define   ADS_GPIO          0x14
#define   ADS_MISC1         0x15
#define   ADS_MISC2         0x16
#define   ADS_CONFIG4       0x17

#define   ADS_gain          0x30
//#define   gain              0x30


/* Prototypes for the functions */
int init_ADS(void);
void start_ADS(void);
void stop_ADS(void);
void readREG(char, char);
void writeREG(char, char);
extern int samplingRate;


class ads
{
public:
    ads();
};



#endif // ADS_H
