#include <QApplication>
/*
gcc -Wall -o main main.c ads.c -l bcm2835
gcc -Wall -o main main.c ads.c -l bcm2835 -std=c11
h5cc -Wall -o main main.c ads.c hdf5.c -lbcm2835 -std=c11
*/

//#include <errno.h>
#include <bcm2835.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>


#include "main.h"
#include "ads.h"
#include "hdf5_custom.h"
#include "controller.h"
#include "serial.h"
#include "i2c.h"


//using namControllerespace std;
int ADS_STARTED;
int STOP_PROG;
unsigned long int counter;
int maxSampleCount;
int blockSize;


int main(int argc, char* argv[]) {
    QApplication app(argc,argv);
    Controller control;
    HDF5Class hdf5handler;
    i2c i2chandler;

    HDF5Class::hdf5Struct hdf5data;

    // SET VARIABLES
    ADS_STARTED = FALSE;
    STOP_PROG = FALSE;
    counter = 0;
    // edit: maxSampleCount = 18710;
    blockSize = 10000;

    //
    double **RESULTARRAY = new double*[blockSize];
    for (int i=0;i<blockSize;i++) {
        RESULTARRAY[i] = new double[12];
    }

    if (signal(SIGINT, Controller::sig_handler) == SIG_ERR) {
        printf("\ncan't catch SIGINT\n");
    }
    if (!bcm2835_init())
    {
      printf("bcm2835_init failed. Are you running as root??\n");
      return 1;
    }

    // GPIO init
    control.init_GPIO();

    if (!bcm2835_spi_begin())
    {
      printf("bcm2835_spi_begin failed. Are you running as root??\n");
      delay(2000);
      return 1;
    }

    control.init_SPI();

    printf("Initializing\n");

    //delay in milliseconds
    delay(500);

    if(!init_ADS()) {
        printf("ADS init failed!\n");
        delay(2000);
        return 1;
    }
    delay(500);

    i2chandler.setupLED();



    delay(3000);

    hdf5handler.createHDF5File(&hdf5data);


    control.init_Filter();

    control.init_Serial();

    control.init_IMU();

    control.init_Ultrasound();

    control.init_ADC();

    control.init_Motor();


    start_ADS();



    while(1)
    {
        if (bcm2835_gpio_eds(ADS_DRDY_PIN))
        {
            if (ADS_STARTED) {
                control.DRDY_interrupt(&hdf5data,RESULTARRAY);
                // Now clear the eds flag by setting it to 1
                bcm2835_gpio_set_eds(ADS_DRDY_PIN);

            }
        }

        //bcm2835_gpio_write (LED_STATUS, LED_ON) ; delay (500) ;
        //bcm2835_gpio_write (LED_STATUS,  LED_OFF) ; delay (500) ;
    }
    return app.exec();
}
