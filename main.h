#ifndef MAIN_H
#define MAIN_H

#include <QTextStream>
#include "hdf5_custom.h"
#include <bcm2835.h>


#define   LED_POWER     RPI_V2_GPIO_P1_29
#define   LED_STATUS    RPI_V2_GPIO_P1_32
//static const int TASTER_PIN = 8

#define     FALSE           0
#define     TRUE            1
//
#define   LED_ON            FALSE
#define   LED_OFF           TRUE

// SPI definitions
#define SPI_SPEED_SLOW  BCM2835_SPI_CLOCK_DIVIDER_512    // 32= 8mhz; 512 = 490kHz
#define SPI_SPEED_FAST  BCM2835_SPI_CLOCK_DIVIDER_32    // 32= 8mhz; 512 = 490kHz

extern unsigned char buffer[3];
extern int ADS_STARTED;
extern int STOP_PROG;
extern int blockSize;
extern unsigned long int counter;
extern int maxSampleCount;
extern double calibrationFactor;
extern int windowSize;
extern std::vector<double> arr;



int main (int,char* []);


#endif // MAIN_H
