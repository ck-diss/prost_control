#ifndef I2C_H
#define I2C_H

#define     MULTI_ADDRESS           0x70 // MULTIPLEXER ADRESS
#define     SEL_PLEX_LED            0x01
#define     SEL_PLEX_EEPROM         0x02
#define     SEL_PLEX_CH2            0x04
#define     SEL_PLEX_CH3            0x08

#define     IMU_HARDWARE_ADDR       0x69 // IMU ADRESS hardware

#define     IMU1_ADDR_MULTI         0x04 // IMU 1 ADRESS from multiplexer
#define     IMU2_ADDR_MULTI         0x08 // IMU 2 ADRESS from multiplexer


#define     LED_DRV_ADDRESS         0x14 // LED DRIVER ADRESS
#define     LED_DRV_READ            0x01
#define     LED_DRV_WRITE           0x00

#define     LED_DRV_MODE1           0x00
#define     LED_DRV_MODE2           0x01
#define     LED_DRV_PWM0            0x02
#define     LED_DRV_PWM1            0x03
#define     LED_DRV_PWM2            0x04
#define     LED_DRV_PWM3            0x05
#define     LED_DRV_PWM4            0x06
#define     LED_DRV_PWM5            0x07
#define     LED_DRV_PWM6            0x08
#define     LED_DRV_PWM7            0x09
#define     LED_DRV_GRPPWM          0x0A
#define     LED_DRV_GRPFRQ          0x0B
#define     LED_DRV_LEDOUT0         0x0C
#define     LED_DRV_LEDOUT1         0x0D
#define     LED_DRV_SUBADR1         0x0E
#define     LED_DRV_SUBADR2         0x0F
#define     LED_DRV_SUBADR3         0x10
#define     LED_DRV_ALLCALL         0x11

#define     LED_DRV_ALLON           0x55
#define     LED_DRV_ALLOFF          0x00
#define     LED_DRV_ALLPWM          0xAA
#define     LED_DRV_ALLGRPPWM       0xFF

#define     LED_DRV_LEDON           0x20 // 50%
//#define     LED_DRV_LEDON           0xFF // 100%
#define     LED_DRV_LEDOFF          0x00


class i2c
{
public:
    i2c();
    void setupLED();
    void setupIMU();
    void write(int handler);
    void changeIMU(int addr);
    void turnLED(int led_id, int on_off);
    void errorCase();
private:
    char _error;
};

#endif // I2C_H
