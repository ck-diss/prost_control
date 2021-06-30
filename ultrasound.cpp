#include <QDebug>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>

#include "ultrasound.h"

#define US_TRIGGER      4 //RPI_V2_GPIO_P1_16
#define US_ECHO         5 //RPI_V2_GPIO_P1_18



using get_time = std::chrono::high_resolution_clock;

std::chrono::high_resolution_clock::time_point start_ultra;
std::chrono::high_resolution_clock::time_point end_ultra;

double us_distance;

Ultrasound::Ultrasound()
{
}
int Ultrasound::setup() {
    if (wiringPiSetup () == -1)
    {
      qDebug() << "Unable to start wiringPi";
      delay(2000);
      return 1 ;
    }
    pinMode(US_TRIGGER, OUTPUT);
    pinMode(US_ECHO, INPUT);
    //TRIGGER PIN MUST START LOW
    digitalWrite(US_TRIGGER,LOW);
    delay(30); //30ms
    us_distance = -1;
    return 0;

}
double Ultrasound::getDistance() {
    //TRIGGER PULSE
    digitalWrite(US_TRIGGER,HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIGGER,LOW);

    //Wait for ECHO start
    while (digitalRead(US_ECHO) == LOW);
    //Wait for ECHO end
    double startTime = micros();
    while (digitalRead(US_ECHO) == HIGH);
    double travelTime = micros() - startTime;


    //GET DiSTANCE
    // 340m/s or 29 microseconds per centimeter
    // * 2(hin und zurück) = 58
    //return(travelTime / 58);
    us_distance = travelTime / 58;

//    qDebug() << "Distanz cm: " << travelTime / 58;
    //qDebug() << "Distanz2 cm: " << travelTime * 34300 / 2 / 1000000;

    return us_distance;
}
