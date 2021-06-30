#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QTextStream>
#include "hdf5_custom.h"
#include "main.h"
#include "filterbutterworth.h"
#include "rms.h"


class Controller : public QObject
{
    Q_OBJECT

public:
    Controller();
    virtual ~Controller();
    static void sig_handler(int);
    void CS_SELECT(void);
    void CS_DESELECT(void);
    /* Prototypes for the functions */
    //void sig_handler(int);
    void init_GPIO(void);
    void init_SPI(void);
    void init_IMU(void);
    void init_Ultrasound(void);
    void init_ADC(void);
    void init_Filter(void);
    void init_Serial(void);
    void init_Motor(void);
    void DRDY_interrupt(HDF5Class::hdf5Struct *hdf5data, double**);
public slots:
    void threadCompleted(void);

private:
    FilterButterworth filterobj1;
    FilterButterworth filterobj2;
    FilterButterworth filterobj3;
    FilterButterworth filterobj4;
    RMS rmsobj1;
    RMS rmsobj2;
    RMS rmsobj3;
    RMS rmsobj4;
};



#endif // CONTROLLER_H
