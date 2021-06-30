#ifndef SERIAL_H
#define SERIAL_H

class Serial
{
public:
    Serial();
    int initSerialProsthesis();
    int initSerialIMU();
    int startSerial();

    void sendStringProsthesis(char *data);
    void sendBytesProsthesis(char *data, int len);
    void checkDataProsthesis();

    void sendStringIMU(char *data);
    void sendBytesIMU(char *data, int len);
    void checkDataIMU();

    void closeSerial();
};

#endif // SERIAL_H
