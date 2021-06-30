#include "controller.h"
#include "ads.h"
#include "hdf5_custom.h"
//#include "filterbutterworth.h"
#include <qtconcurrentrun.h>
#include <qfuturewatcher.h>
#include "rms.h"
#include "EMGFilters.h"
//#include <time.h>
#include <chrono>
#include <math.h>

#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include <iostream>
#include <bitset>

#include "main.h"
#include "serial.h"
#include "i2c.h"
#include "ultrasound.h"
#include "MotionSensor/sensor.h"
#include "MotionSensor.h"
#include "adc.h"
#include <bcm2835.h>
#include "MotionSensor/helper_3dmath.h"
#include "StepperMotor28BYJ48.h"


#define delay_ms(a) usleep(a*1000)


char TXARRAY[27];
char RXARRAY[27];
double calibrationFactor;
int arrayOffset;
//filter value
double filteredCH1;
double filteredCH2;
double filteredCH3;
double filteredCH4;
//rms value
double rms1;
double rms2;
double rms3;
double rms4;
//max rms
double maxRMS1;
double maxRMS2;
double maxRMS3;
double maxRMS4;
//min rms
double minRMS1;
double minRMS2;
double minRMS3;
double minRMS4;
//normalized rms
double normRMS1;
double normRMS2;
double normRMS3;
double normRMS4;
//controlSignal
double controlSigCh1;
double controlSigCh2;
double controlSigCh3;
double controlSigCh4;
// buffer to send via serial
char string_buf[3];
char charSent;

int channelPos[8];
int channelNeg[8];

int err;

unsigned int countSentPackages;

using get_time = std::chrono::high_resolution_clock;
using ns = std::chrono::nanoseconds;

std::chrono::high_resolution_clock::time_point time_start;
std::chrono::high_resolution_clock::time_point time_now;

std::chrono::high_resolution_clock::time_point time1;
std::chrono::high_resolution_clock::time_point time2;


std::chrono::high_resolution_clock::time_point time_start2;
std::chrono::high_resolution_clock::time_point time_now2;

std::chrono::high_resolution_clock::time_point time_imu_start;
std::chrono::high_resolution_clock::time_point time_imu_end;

Serial serialhandler;
i2c i2chandler;
i2c i2chandler2;
Ultrasound ultrasoundhandler;
Sensor imuhandler;
ADC adchandler;
StepperMotor28BYJ48 motorhandler;

Quaternion quar;

EMGFilters myFilter;
const int FILTER_CH1 = 1;
const int FILTER_CH2 = 2;
const int FILTER_CH3 = 3;
const int FILTER_CH4 = 4;

//typedef union {
//    float floatingPoint[4];
//    char binary[16];
//} binaryFloat;

typedef union {
    float floatingPoint[6];
    char binary[24];
} binaryFloat;

binaryFloat bf;

QFuture<double> t_getPressureSensorResistance;
QFuture<double> t_getDistance;
QFuture<std::vector<float>> t_imu1;
QFuture<std::vector<float>> t_imu2;
std::vector<float> vec;
extern double us_distance;

QFutureWatcher<std::vector<float>> watcher;

bool imu1_finished = false;
bool imu2_finished = false;



//std::chrono::duration<double, std::chrono::nanoseconds> time_difference;

Controller::Controller(){
    QObject::connect(&watcher, SIGNAL (finished()), this, SLOT(threadCompleted()));

    //CALIBRATION
    calibrationFactor = 9 / pow(2.0,24.0);
    qDebug() << "calFactor" << calibrationFactor;
    filteredCH1 = rms1 = maxRMS1 = normRMS1 = 0;
    filteredCH2 = rms2 = maxRMS2 = normRMS2 = 0;
    filteredCH3 = rms3 = maxRMS3 = normRMS3 = 0;
    filteredCH4 = rms4 = maxRMS4 = normRMS4 = 0;
    minRMS1 = minRMS2 = minRMS3 = minRMS4 = 1;
    countSentPackages=0;

}
Controller::~Controller(){
   /* workerThread.quit();
    workerThread.wait();*/
}

void Controller::sig_handler(int signo)
{
  if (signo == SIGINT) {
    printf("received SIGINT\n");
    STOP_PROG = TRUE;
  }
}

void Controller::CS_SELECT (void) {
    bcm2835_gpio_write(ADS_CS_PIN,LOW);
}

void Controller::CS_DESELECT (void) {
    bcm2835_gpio_write(ADS_CS_PIN,HIGH);
}

void Controller::init_GPIO () {
    printf("initGPIO\n");

    //bcm2835_set_debug(0);#include <cout>

    // Set the pin to be an output- start_time
    bcm2835_gpio_fsel (LED_POWER, BCM2835_GPIO_FSEL_OUTP) ;
    bcm2835_gpio_fsel (LED_STATUS, BCM2835_GPIO_FSEL_OUTP) ;
    bcm2835_gpio_fsel (ADS_PWDN_PIN, BCM2835_GPIO_FSEL_OUTP) ;
    bcm2835_gpio_fsel (ADS_RESET_PIN, BCM2835_GPIO_FSEL_OUTP) ;
    bcm2835_gpio_fsel (ADS_START_PIN, BCM2835_GPIO_FSEL_OUTP) ;
    bcm2835_gpio_fsel (ADS_CS_PIN, BCM2835_GPIO_FSEL_OUTP) ;
    bcm2835_gpio_fsel (ADS_DRDY_PIN, BCM2835_GPIO_FSEL_INPT) ;
    //  with a pullupprev_time_difference
    //bcm2835_gpio_set_pud(ADS_DRDY_PIN, BCM28- start_time35_GPIO_PUD_DOWN);

    // turn off leds
    bcm2835_gpio_write(LED_POWER, LED_OFF);
    bcm2835_gpio_write(LED_STATUS, LED_OFF);
}

void Controller::init_SPI () {
    printf("initSPI\n");

    // Set the pin to be an output
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                   // Mode1
    //set spi clk speed slower for sending commands
    bcm2835_spi_setClockDivider(SPI_SPEED_SLOW);    // 32= 8mhz; 512 = 490kHz
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                  // Control CS manually
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default
}

void Controller::init_IMU () {
    printf("initIMU\n");
    i2chandler.changeIMU(IMU1_ADDR_MULTI);
    imuhandler.ms_open();
    i2chandler.changeIMU(IMU2_ADDR_MULTI);
    imuhandler.ms_open();
}

void Controller::init_Ultrasound () {
    printf("init Ultrasound\n");
    ultrasoundhandler.setup();
}

void Controller::init_ADC () {
    printf("init ADC\n");
    adchandler.setup();
}

void Controller::init_Filter() {
    /* (frequency, sampleRate, resonanz  */
   // filter.SetupFilterButterworth(15,1000,0.8);
    //filterobj1.SetupFilterButterworth(15,samplingRate,0.8);
    //filterobj2.SetupFilterButterworth(15,samplingRate,0.8);
    //filterobj3.SetupFilterButterworth(15,samplingRate,0.8);
    //filterobj4.SetupFilterButterworth(15,samplingRate,0.8);
    SAMPLE_FREQUENCY filterSampleRate = SAMPLE_FREQ_1000HZ;
    NOTCH_FREQUENCY humFrequ = NOTCH_FREQ_50HZ;


//    AHF_CH1.init(filterSampleRate, humFrequ);
//    AHF_CH2.init(filterSampleRate, humFrequ);

//    LPF_CH1.init(FILTER_TYPE_LOWPASS, filterSampleRate);
//    LPF_CH2.init(FILTER_TYPE_LOWPASS, filterSampleRate);

//    HPF_CH1.init(FILTER_TYPE_HIGHPASS, filterSampleRate);
//    HPF_CH2.init(FILTER_TYPE_HIGHPASS, filterSampleRate);

    // no notchfilter, no lowpass, active highpass
    myFilter.init(filterSampleRate,humFrequ,true,true,true);
    qDebug() << "setup Filter Rate: " << samplingRate;
}

void Controller::init_Serial() {

    int ss = serialhandler.initSerialProsthesis();
    int dd = serialhandler.initSerialIMU();
    if (ss != 1 && dd != 1) //no error
        ss = serialhandler.startSerial();
}

void Controller::init_Motor () {
    printf("init Motor\n");
    motorhandler.init(RPI_V2_GPIO_P1_37,RPI_V2_GPIO_P1_35,RPI_V2_GPIO_P1_33,RPI_V2_GPIO_P1_31,4);

    motorhandler.setSteppingMethod(motorhandler.WAVE_DRIVE);

}


void Controller::DRDY_interrupt (HDF5Class::hdf5Struct* hdf5data, double **RESULTARRAY) {
    HDF5Class hdf5obj;
    Serial serialhandler;

    //printf("DRDY: %i\n", 6counter);RESULTARRAY
    if (counter == 0) {
        arrayOffset = blockSize/2;


        t_imu2 = QtConcurrent::run(imuhandler, &Sensor::ms_update, IMU2_ADDR_MULTI);
        t_imu2.end();
        t_imu1 = QtConcurrent::run(imuhandler, &Sensor::ms_update, IMU1_ADDR_MULTI);

        t_getDistance = QtConcurrent::run(ultrasoundhandler, &Ultrasound::getDistance);
        t_getPressureSensorResistance = QtConcurrent::run(adchandler, &ADC::getPressureSensorResistance);


    }
    if (STOP_PROG) {
        qDebug() << "stopping program";
        serialhandler.sendStringProsthesis("h");
        serialhandler.sendStringProsthesis("s");

        // disable event detect
        bcm2835_gpio_clr_afen(ADS_DRDY_PIN);
        arrayOffset = blockSize/2 - arrayOffset;
        // write rest of data to file
        hdf5obj.writeToHDF5(hdf5data, RESULTARRAY, counter%(blockSize/2), counter, arrayOffset);
        //QFuture<void> future = QtConcurrent::run(&HDF5Class::writeToHDF5,hdf5data, RESULTARRAY, counter%blockSize, counter);

        hdf5obj.closeHDF5File(hdf5data);
        serialhandler.closeSerial();
        stop_ADS();

    } else {
        // write DATA to HDF5 via Thread
        if (counter >= (blockSize/2) && counter % (blockSize/2) == 0){
            arrayOffset = blockSize/2 - arrayOffset;
            //qDebug() << arrayOffset;
            QtConcurrent::run(hdf5obj, &HDF5Class::writeToHDF5, hdf5data, RESULTARRAY, blockSize/2, counter, arrayOffset);
        }

        bcm2835_spi_transfernb(TXARRAY, RXARRAY, sizeof(TXARRAY));

        for(unsigned int i=3;i<sizeof(RXARRAY);i+=3) {
            RESULTARRAY[counter%blockSize][i/3] = (RXARRAY[i] << 24 | RXARRAY[i + 1] << 16 | RXARRAY[i + 2] << 8) / 256 * calibrationFactor;
        }
        // add Timestamp

        if (counter==0) {
            time_start = get_time::now();
        }

        // every Xms
        if (counter%(samplingRate/100)==0) { // 400 -> 2.5ms // 200 -> 5ms
            //time1 = get_time::now();
            //time2 = get_time::now();
            //qDebug() <<"timediff: " << std::chrono::duration_cast<std::chrono::microseconds>(time2 - time1).count();;


            /****************************************************/
            /* ADC
            /****************************************************/
//                qDebug() << "ADC Voltage: " << adc.getPressureSensorVoltage() << " V";
//                qDebug() << "ADC Resistance: " << adc.getPressureSensorResistance() << " ohm";
            //qDebug() << "ADC Force: " << adc.getPressureSensorForce() << " g";
            if (t_getPressureSensorResistance.isFinished()) {
                if (t_getPressureSensorResistance.result() < 200000) {
                    qDebug() << "Pressure resistance: " << t_getPressureSensorResistance.result();
                    //motorhandler.quarterRotation(1);
                    //if (motorhandler.isStarted()==false) {
                      motorhandler.startClockwise();
                      //countSteps+=5;
//                    t_motor = QtConcurrent::run(motorhandler, &StepperMotor28BYJ48::start, 5);
                   // }
                }
                else {
                    //qDebug() << "handler steps: " << motorhandler.getCountSteps();
                    if (motorhandler.getCountSteps() > 0) {
                        motorhandler.startCounterClockwise();

                    } else {
                       if (motorhandler.isStarted()==true) {
                            motorhandler.stop();
                       }
                    }
                }
                t_getPressureSensorResistance = QtConcurrent::run(adchandler, &ADC::getPressureSensorResistance);
            }

            if (counter%500==0) {
                /****************************************************/
                /* Ultrasound
                /****************************************************/
                //distance = ultrasoundhandler.getDistance();
                if (t_getDistance.isFinished()) {
                    if (t_getDistance.result()) {
                        //qDebug() << "Ultrasound distance: " << t_getDistance.result() << " cm";
                    }
                    t_getDistance = QtConcurrent::run(ultrasoundhandler, &Ultrasound::getDistance);
                }

            }


            // LEAD-OFF Detection
            // every 1000ms
            if (counter%(samplingRate/1)==0) {


                channelPos[0] = (RXARRAY[1] & 0x10);
                channelPos[1] = (RXARRAY[1] & 0x20);
                channelPos[2] = (RXARRAY[1] & 0x40);
                channelPos[3] = (RXARRAY[1] & 0x80);
                channelPos[4] = (RXARRAY[0] & 0x01);
                channelPos[5] = (RXARRAY[0] & 0x02);
                channelPos[6] = (RXARRAY[0] & 0x04);
                channelPos[7] = (RXARRAY[0] & 0x08);
                channelNeg[0] = (RXARRAY[2] & 0x10);
                channelNeg[1] = (RXARRAY[2] & 0x20);
                channelNeg[2] = (RXARRAY[2] & 0x40);
                channelNeg[3] = (RXARRAY[2] & 0x80);
                channelNeg[4] = (RXARRAY[1] & 0x01);
                channelNeg[5] = (RXARRAY[1] & 0x02);
                channelNeg[6] = (RXARRAY[1] & 0x04);
                channelNeg[7] = (RXARRAY[1] & 0x08);


                for (int i=0;i<8;i++) {
                    if (channelPos[i] + channelNeg[i] == 0) { //beide connected
                        QtConcurrent::run(i2chandler, &i2c::turnLED, i, LED_ON);

                        //i2chandler.turnLED(i,LED_ON);
                    } else {
                        QtConcurrent::run(i2chandler, &i2c::turnLED, i, LED_OFF);

                        //i2chandler.turnLED(i,LED_OFF);
                    }
                }




            } // end every sec std::chrono::duration_cast<std::chrono::microseconds>(
        } //end ever 10ms
        time_now = get_time::now();
        //auto time_difference = std::chrono::duration_cast<std::chrono::microseconds>(time_now - time_start);
        RESULTARRAY[counter%blockSize][0] = std::chrono::duration_cast<std::chrono::microseconds>(time_now - time_start).count();


        /****************************************************/
        /* Filter
        /****************************************************/

        //filter 1. channel
        filteredCH1 = myFilter.update(RESULTARRAY[counter%blockSize][1],FILTER_CH1);
        //filteredCH1 = filterobj1.UpdateFilterButterworth(RESULTARRAY[counter%blockSize][1]);
        //filter 2. channel
        filteredCH2 = myFilter.update(RESULTARRAY[counter%blockSize][2],FILTER_CH2);
//        //filter 3. channel
        filteredCH3 = myFilter.update(RESULTARRAY[counter%blockSize][3],FILTER_CH3);
//        //filter 4. channel
        filteredCH4 = myFilter.update(RESULTARRAY[counter%blockSize][4],FILTER_CH4);


        /* Calculations - Time Consuming? */

        //RMS of 1. channel
        //qDebug() << filteredCH1;
        rms1 = rmsobj1.updateRMS(filteredCH1);
        //RMS of 2. channel
        rms2 = rmsobj2.updateRMS(filteredCH2);
        //RMS of 3. channel
        rms3 = rmsobj3.updateRMS(filteredCH3);
        //RMS of 4. channel
        rms4 = rmsobj4.updateRMS(filteredCH4);

        //debug
        RESULTARRAY[counter%blockSize][5] = filteredCH1;
        RESULTARRAY[counter%blockSize][7] = filteredCH2;

        // GYRO
        //RESULTARRAY[counter%blockSize][8] = temp;
        RESULTARRAY[counter%blockSize][9] = 0;//ypr[YAW];
        RESULTARRAY[counter%blockSize][10] = 0;//ypr[PITCH];
        RESULTARRAY[counter%blockSize][11] = 0;//ypr[ROLL];





//        /***************************************************/
//        /*
//         * IMU HANDLING
//         *
//        /***************************************************/
//        // 200 -> 5ms
//        // 100 -> 10ms
//        // 50  -> 20ms -> passt
//        // 10  -> 100ms
        if (counter%(samplingRate/50) == 0) { //50->20ms
//        if (counter%(samplingRate*2) == 0) { //50->20ms
            /**** IMU send data to serial ****/


            time_imu_start = get_time::now();
            //err = ms_update();

            //qDebug() << "Distance cm: " << future.result();
            //err = ms_update(); //get IMU1 sensor information


            /***/
            /*
             * ORDER OF FUNC CALLS IS IMPORTANT!!
             *
             * ****/
//            qDebug() << "imu1 running: " <<t_imu1.isRunning();

//            qDebug() << "imu2 running: " <<t_imu2.isRunning();


//            watcher.setFuture(t_imu1);

//            qDebug() << "imu1 thread running: " << t_imu1.isRunning();
//            qDebug() << "imu2 thread running: " << t_imu2.isRunning();
            if (t_imu1.isFinished() && imu1_finished==false) {
                imu1_finished = true;
//                qDebug() << "in thread 1";cancel
                t_imu1.end();
                if (t_imu1.result()[0] != 555) { //falls daten passen
                    vec = t_imu1.result();

                    bf.floatingPoint[0] = vec[YAW];
                    bf.floatingPoint[1] = vec[PITCH];
                    bf.floatingPoint[2] = vec[ROLL];

//                    printf("yaw1 = %2.1f\tpitch1 = %2.1f\troll1 = %2.1f\tokk\t ",
//                    vec[YAW], vec[PITCH],vec[ROLL]);


                }
                else {
//                    printf("yaw1 = %2.1f\tpitch1 = %2.1f\troll1 = %2.1f\ter1\t",
//                           bf.floatingPoint[0],bf.floatingPoint[1],bf.floatingPoint[2]);
                }
                //i2chandler.changeIMU(IMU1_ADDR_MULTI);
                t_imu2 = QtConcurrent::run(imuhandler, &Sensor::ms_update, IMU2_ADDR_MULTI);
                imu2_finished = false;
            }
            else {
//                printf("yaw1 = %2.1f\tpitch1 = %2.1f\troll1 = %2.1f\ter2\t",
//                       bf.floatingPoint[0],bf.floatingPoint[1],bf.floatingPoint[2]);
            }

            if (t_imu2.isFinished() && imu2_finished==false) {
                imu2_finished = true;
//                qDebug() << "in thread 2";

                t_imu2.end();
                if (t_imu2.result()[0]!= 555) { //falls daten passen
                    vec = t_imu2.result();
                    bf.floatingPoint[3] = vec[YAW];
                    bf.floatingPoint[4] = vec[PITCH];
                    bf.floatingPoint[5] = vec[ROLL];

//                    printf("yaw2 = %2.1f\tpitch2 = %2.1f\troll2 = %2.1f\tokk\n",
//                    vec[YAW], vec[PITCH],vec[ROLL]);

                }
                else {
//                    printf("yaw2 = %2.1f\tpitch2 = %2.1f\troll2 = %2.1f\ter1\n",
//                           bf.floatingPoint[3],bf.floatingPoint[4],bf.floatingPoint[5]);
                }
                t_imu1 = QtConcurrent::run(imuhandler, &Sensor::ms_update, IMU1_ADDR_MULTI);
                imu1_finished = false;

            }
            else {
//                printf("yaw2 = %2.1f\tpitch2 = %2.1f\troll2 = %2.1f\ter2\n",
//                       bf.floatingPoint[3],bf.floatingPoint[4],bf.floatingPoint[5]);
            }

//            printf("yaw1 = %2.1f\tpitch1 = %2.1f\troll1 = %2.1f\ter1\t",
//                                       bf.floatingPoint[0],bf.floatingPoint[1],bf.floatingPoint[2]);
//            printf("yaw2 = %2.1f\tpitch2 = %2.1f\troll2 = %2.1f\ter1\n",
//                                       bf.floatingPoint[3],bf.floatingPoint[4],bf.floatingPoint[5]);
            serialhandler.sendBytesIMU(bf.binary,24);
//            time_imu_end = get_time::now();
//            qDebug() << "imu delay: " << std::chrono::duration_cast<std::chrono::microseconds>(time_imu_end - time_imu_start).count();

        }

        /***************************************************/
        /*
         *  calibration period of 5 seconds (50000000.1us)
         *  get max rms value
         *  send control signal to prostesis
         *
        /***************************************************/

        if (RESULTARRAY[counter%blockSize][0] < 5000000) {
            // wait highpass peak 1sec
            if (RESULTARRAY[counter%blockSize][0] > 1000000) {



                if (maxRMS1 < rms1) {
                    maxRMS1 = rms1;
                 }
                if (minRMS1 > rms1) {
                    minRMS1 = rms1;
                }
                if (maxRMS2 < rms2) {
                    maxRMS2 = rms2;
                }
                if (minRMS2 > rms2) {
                    minRMS2 = rms2;
                }
                if (maxRMS3 < rms3) {
                    maxRMS3 = rms3;
                }
                if (minRMS3 > rms3) {
                    minRMS3 = rms3;
                }
                if (maxRMS4 < rms4) {
                    maxRMS4 = rms4;
                }
                if (minRMS4 > rms4) {
                    minRMS4 = rms4;
                }
            }
        }
        else
        {
            // normRMS = rms/maxRMS;
            normRMS1 = (rms1-minRMS1) / (maxRMS1-minRMS1);
            normRMS2 = (rms2-minRMS2) / (maxRMS2-minRMS2);
            normRMS3 = (rms3-minRMS3) / (maxRMS3-minRMS3);
            normRMS4 = (rms4-minRMS4) / (maxRMS4-minRMS4);

            controlSigCh1 = rmsobj1.crossCorr(normRMS1,normRMS2)-1; // -1 um künstliches offset aus fkt herauszunehmen
            controlSigCh2 = rmsobj2.crossCorr(normRMS2,normRMS1)-1; // -1 um künstliches offset aus fkt herauszunehmen
            controlSigCh3 = rmsobj3.crossCorr(normRMS3,normRMS4)-1; // -1 um künstliches offset aus fkt herauszunehmen
            controlSigCh4 = rmsobj4.crossCorr(normRMS4,normRMS3)-1; // -1 um künstliches offset aus fkt herauszunehmen

            if(controlSigCh1<0) controlSigCh1=0; // negative werte nach crossCorr entfernen
            if(controlSigCh2<0) controlSigCh2=0;
            if(controlSigCh3<0) controlSigCh3=0;
            if(controlSigCh4<0) controlSigCh4=0;

            //qDebug() << rms1 << " : " << minRMS1 <<" : " << maxRMS1 << " : " << normRMS1;
            RESULTARRAY[counter%blockSize][6] = controlSigCh1; // -1 um künstliches offset aus fkt herauszunehmen

            RESULTARRAY[counter%blockSize][8] = controlSigCh2;

            RESULTARRAY[counter%blockSize][9] = controlSigCh3;

            RESULTARRAY[counter%blockSize][10] = controlSigCh4;


            /* Propagation Delay Measurement */

//            if (counter%8000 == 0) {
//                // test timing
//                bcm2835_gpio_write(LED_STATUS, LED_ON);
//                sprintf(string_buf,"e50");
//                serialhandler.sendStringProsthesis(string_buf);
//                time_now2 = get_time::now();
//                qDebug() << "da auch";
//                qDebug() << std::chrono::duration_cast<std::chrono::microseconds>(time_now2 - time_start2).count();

//            } else if (counter%4000 == 0) {
//                bcm2835_gpio_write(LED_STATUS, LED_OFF);
//                sprintf(string_buf,"f50");
//                serialhandler.sendStringProsthesis(string_buf);
//                time_start2 = get_time::now();
//                qDebug() << "da";
//            }
            //auto time_difference = std::chrono::duration_cast<std::chrono::microseconds>(time_now - time_start);


            // send control command every 5 ms
            if (counter%(samplingRate/200) == 0) {

                /*qDebug() << rms1;
                if (rms1 > 0.0001) {
                    bcm2835_gpio_write(LED_STATUS, LED_OFF);
                    qDebug() << "led off";

                } else {
                    bcmypr2835_gpio_write(LED_STATUS, LED_ON);
                    qDebug() << "led on";
                }*/

                /**/


                if (counter%(samplingRate/50) == 0) {

                //qDebug() << normRMS3 << "###" << normRMS4;
                }
                   //                sprintf(string_buf,"f%d",(int)(normRMS1*0.1100));
                //                serialhandler.sendStringProsthesis(string_buf);


                //                Serial.print(_q & 0xFF, BYTE);
                //                Serial.print((_q >> 8) & 0xFF, BYTE);" : " << normRMS2;
                /***************************************************/
                /*
                 *  generate control signal based on normalized RMS
                 *
                 *
                /***************************************************/
                //BIZEPS


                 // MONTAG: controlSigCh1 verwenden
                if (controlSigCh1 >= 0.05) {
                    bcm2835_gpio_write(LED_STATUS, LED_ON);
                    if (controlSigCh1 > 0.99)
                        controlSigCh1 = 0.99;
                    sprintf(string_buf,"f%d",(int)(controlSigCh1*100));
                    serialhandler.sendStringProsthesis(string_buf);
                    charSent = 'f';
                    countSentPackages++;
                } else {
                    // TRIZEPS contraction
                    if (controlSigCh2 >= 0.05) {
                        bcm2835_gpio_write(LED_POWER, LED_ON);
                        if (controlSigCh2 > 0.99)
                            controlSigCh2 = 0.99;
                        sprintf(string_buf,"e%d",(int)(controlSigCh2*100));
                        serialhandler.sendStringProsthesis(string_buf);
                        charSent = 'e';
                        countSentPackages++;

                    } else {
                        if (charSent != 's') {
                            bcm2835_gpio_write(LED_STATUS, LED_OFF);
                            bcm2835_gpio_write(LED_POWER, LED_OFF);
                            serialhandler.sendStringProsthesis("s");
                            charSent = 's';
                            countSentPackages++;
                        }
                    }
                }
                //flexor radialis
                if (controlSigCh3 >= 0.05) {
                    //bcm2835_gpio_write(LED_STATUS, LED_ON);
                    if (controlSigCh3 > 0.99)
                        controlSigCh3 = 0.99;
                    sprintf(string_buf,"p%d",(int)(controlSigCh3*100));
                    serialhandler.sendStringProsthesis(string_buf);
                    charSent = 'p';
                    countSentPackages++;
                } else {
                    // flexor superficialis contraction
                    if (controlSigCh4 >= 0.1) {
                        //bcm2835_gpio_write(LED_POWER, LED_ON);
                        if (controlSigCh4 > 0.99)
                            controlSigCh4 = 0.99;
                        sprintf(string_buf,"u%d",(int)(controlSigCh4*100));
                        serialhandler.sendStringProsthesis(string_buf);
                        charSent = 'u';
                        countSentPackages++;

                    } else {
                        if (charSent != 'h') {
                            //bcm2835_gpio_write(LED_STATUS, LED_OFF);
                            //bcm2835_gpio_write(LED_POWER, LED_OFF);
                            serialhandler.sendStringProsthesis("h");
                            charSent = 'h';
                            countSentPackages++;
                        }
                    }
                }
                //qDebug() << counter << " : " << countSentPackages;
            }/**/
      }
    }
    counter++;
    //printf("\n");
}
void Controller::threadCompleted() {
    qDebug() << "in thread completed";
    t_imu1.end();
    if (t_imu1.result()[0] != 555) { //falls daten passen
        vec = t_imu1.result();

        bf.floatingPoint[0] = vec[YAW];
        bf.floatingPoint[1] = vec[PITCH];
        bf.floatingPoint[2] = vec[ROLL];

        printf("yaw1 = %2.1f\tpitch1 = %2.1f\troll1 = %2.1f\tokk\t",
        vec[YAW], vec[PITCH],vec[ROLL]);


    }
    else {
        printf("yaw1 = %2.1f\tpitch1 = %2.1f\troll1 = %2.1f\ter1\t",
               bf.floatingPoint[0],bf.floatingPoint[1],bf.floatingPoint[2]);
    }
    //i2chandler.changeIMU(IMU1_ADDR_MULTI);
    t_imu2 = QtConcurrent::run(imuhandler, &Sensor::ms_update, IMU2_ADDR_MULTI);

}
