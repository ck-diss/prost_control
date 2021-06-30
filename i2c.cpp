 #include "i2c.h"
#include <stdio.h>
#include <QDebug>
#include <qtconcurrentrun.h>

//#include <wiringPi.h>
#include <wiringPiI2C.h>
//#define delayMicroseconds(x) bcm2835_delayMicroseconds(x)
#include "main.h"
#include <bcm2835.h>

int handler;
i2c::i2c()
{
    char _error = false;

}
void i2c::setupLED() {
    handler = wiringPiI2CSetup(MULTI_ADDRESS);
    int error;
    if (handler == -1) {
        printf("I2C connetion to MULTIPLEXER failed\n");
    }

    else {  
        error = wiringPiI2CWrite(handler,0x04 | 0x03); // write multiplexer
        if (error < 0) {
            qDebug() << "ERROR occured writing to MULTIPLEXER\n";
            delay(1000);
        }
        else {
            printf("MULTIPLEXER SET!\n");

            handler = wiringPiI2CSetup(LED_DRV_ADDRESS);

            if (handler == -1) {
                printf("I2C connetion failed on LED DRIVER\n");
            }


            error = wiringPiI2CWriteReg8(handler,LED_DRV_MODE1,0x00);
            if (error < 0) {
                qDebug() << "ERROR occured setting MODE1";
                delay(1000);
            }
            error = wiringPiI2CWriteReg8(handler,LED_DRV_MODE2,0x01);
            if (error < 0) {
                qDebug() << "ERROR occured setting MODE2";
                delay(1000);
            }
            wiringPiI2CWriteReg8(handler,LED_DRV_GRPPWM,LED_DRV_LEDON);

            wiringPiI2CWriteReg8(handler,LED_DRV_LEDOUT0,LED_DRV_ALLGRPPWM);
            wiringPiI2CWriteReg8(handler,LED_DRV_LEDOUT1,LED_DRV_ALLGRPPWM);

            printf("done\n");


            // ERROR Case (stop by setting _error=false
//            _error = true;
//            QtConcurrent::run(this, &i2c::errorCase);
//            _error = false; // turn off error Thread


            //errorCase(true);


            //erg = wiringPiI2CWrite(handler,LED_DRV_LEDOUT1);
            int erg = wiringPiI2CReadReg8(handler,LED_DRV_MODE2 | 0x01);
            qDebug() << "read REG: " << erg;
        }


    }
}

void i2c::write(int handler) {

    wiringPiI2CWriteReg8(handler,LED_DRV_GRPPWM,LED_DRV_LEDOFF);

}

void i2c::turnLED(int led_id, int on_off) {
    handler = wiringPiI2CSetup(LED_DRV_ADDRESS);

    if (handler == -1) {
        printf("I2C connetion failed on LED DRIVER\n");
    }
    if (on_off == LED_ON) { //on
        wiringPiI2CWriteReg8(handler,LED_DRV_PWM0+led_id,LED_DRV_LEDON);
    } else {
        wiringPiI2CWriteReg8(handler,LED_DRV_PWM0+led_id,LED_DRV_LEDOFF);
    }
    handler = wiringPiI2CSetup(LED_DRV_ADDRESS);
}


void i2c::changeIMU(int addr) {
    handler = wiringPiI2CSetup(MULTI_ADDRESS);
    if (handler == -1) {
        printf("I2C connetion to MULTIPLEXER failed\n");
    }

    else {
        int error = wiringPiI2CWrite(handler,addr | 0x03); // write multiplexer
        if (error < 0) {
            qDebug() << "ERROR occured writing to MULTIPLEXER\n";
            delay(1000);
        }
    }
}


void i2c::errorCase() {
    while (1) {
        if (_error==false) break;
        wiringPiI2CWriteReg8(handler,LED_DRV_GRPPWM,LED_DRV_LEDOFF);
        bcm2835_gpio_write (LED_STATUS,  LED_OFF);
        bcm2835_gpio_write (LED_POWER,  LED_OFF);
        delay(500);
        wiringPiI2CWriteReg8(handler,LED_DRV_GRPPWM,LED_DRV_LEDON);
        bcm2835_gpio_write (LED_STATUS, LED_ON);
        bcm2835_gpio_write (LED_POWER, LED_ON);

        delay(500);
    }

}
