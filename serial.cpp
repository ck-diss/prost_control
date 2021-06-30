#include <QDebug>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#include "serial.h"

int SerialProstFD;
int SerialIMUFD;
int count;
unsigned int nextTime;

Serial::Serial()
{
}
int Serial::initSerialProsthesis() {
    printf("Init Serial Comm... ");
    if ((SerialProstFD = serialOpen ("/dev/ttyACM0", 115200)) < 0) //ProstControl
    {
        fprintf (stderr, "Unable to open serial port to prosthesis control: %s\n", strerror (errno)) ;
        delay(2000);
        return 1 ;
    }
    if (wiringPiSetup () == -1)
    {
      fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
      delay(2000);
      return 1 ;
    }
    printf("done\n");
}
int Serial::initSerialIMU() {
    printf("Init Serial Comm... ");
    if ((SerialIMUFD = serialOpen ("/dev/ttyS0", 115200 )) < 0)
    {
        fprintf (stderr, "Unable to open serial port to IMU: %s\n", strerror (errno)) ;
        delay(2000);
        return 1 ;
    }
    if (wiringPiSetup () == -1)
    {
      fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
      delay(2000);
      return 1 ;
    }
    printf("done\n");
}

int Serial::startSerial() {
      printf("Start Serial\n");

//      nextTime = millis () + 300 ;

//      const char turnLED[] = {'i', 'o'};

//      for (count = 0 ; count < 2 ; )
//      {
//        if (millis () > nextTime)
//        {
//          printf ("\nOut: %3d: ", turnLED[count%2]) ;
//          fflush (stdout) ;
//          serialPutchar (SerialProstFD, turnLED[count%2]) ;
//          nextTime += 1500 ;
//          ++count ;
//        }

//        delay (3) ;

//        if (serialDataAvail (SerialProstFD))
//        {
//          printf (" -> %3d", serialGetchar (SerialProstFD)) ;
//          fflush (stdout) ;
//        }
//      }

      printf ("\n") ;
      return 0 ;
}

void Serial::sendStringProsthesis(char *data) {
    if (SerialProstFD != -1) {
        fflush (stdout) ;
        serialPuts (SerialProstFD, data);
        //qDebug() << "send: " << data;
        checkDataProsthesis();
    } else {
        qDebug() << "Serial connection to prosthesis error";
    }
}

void Serial::sendBytesProsthesis(char *data, int len) {
    if (SerialProstFD != -1) {

        fflush (stdout) ;
        write(SerialProstFD,data,len);
        //qDebug() << "send: " << data;
        checkDataProsthesis();
    } else {
        qDebug() << "Serial connection to prosthesis error";
    }
}
// Necessary to remove Data on Line
void Serial::checkDataProsthesis() {
    if  (serialDataAvail (SerialProstFD))
    {
      // e = 101
      // f = 102
      // s = 115
      //printf ("returned: %c\n", serialGetchar (SerialProstFD)) ;
      serialGetchar (SerialProstFD);
      fflush (stdout) ;
    }
}

void Serial::sendStringIMU(char *data) {
    if (SerialIMUFD != -1) {
        fflush (stdout) ;
        serialPuts (SerialIMUFD, data);
        //qDebug() << "send: " << data;
        checkDataIMU();
    }else {
        qDebug() << "Serial connection to IMU error";
    }
}

void Serial::sendBytesIMU(char *data, int len) {
    if (SerialIMUFD != -1) {
        fflush (stdout) ;
        write(SerialIMUFD,data,len);
        checkDataIMU();
    } else {
        qDebug() << "Serial connection to IMU error";
    }
}
// Necessary to remove Data on Line
void Serial::checkDataIMU() {
    if  (serialDataAvail (SerialIMUFD))
    {
      // e = 101
      // f = 102
      // s = 115
      //printf ("returned: %c\n", serialGetchar (SerialProstFD)) ;
      serialGetchar (SerialIMUFD);
      fflush (stdout) ;
    }
}

void Serial::closeSerial() {
    serialClose(SerialProstFD);
    serialClose(SerialIMUFD);
    qDebug() << "Serial Connection closed";
}
