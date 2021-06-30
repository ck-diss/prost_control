#include "adc.h"
#include <wiringPi.h>
#include <pcf8591.h>
#include <QDebug>

double force;
double fG;

ADC::ADC()
{
}
void ADC::setup() {
    if (wiringPiSetup() == -1) {
        qDebug() << "ERROR wiringPi Setup";
    } else {
        pcf8591Setup(80,0x48); //80 pinBase (random)
        pinMode (80, INPUT);
    }
}

double ADC::getPressureSensorVoltage() {
    //return (analogRead(80));
    return (analogRead(80)*3.3/255); //voltage
}

double ADC::getPressureSensorResistance() {
    return (3300 * (3.3 / getPressureSensorVoltage() -1.0)); //restistance
}

double ADC::getPressureSensorForce() {
    fG = 1.0 / getPressureSensorResistance();
    //Break parabolic curve down into two linear slopes
    if (fG <= 600) {
        force = (fG - 0.00075) / 0.00000032639;
    } else {
        force = fG / 0.000000642857;
    }
    return (force); //force
}
