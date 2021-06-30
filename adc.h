#ifndef ADC_H
#define ADC_H

class ADC
{
public:
    ADC();
    void setup();
    double getPressureSensorVoltage();
    double getPressureSensorResistance();
    double getPressureSensorForce();
};

#endif // ADC_H
