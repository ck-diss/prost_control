#ifndef FILTERBUTTERWORTH_H
#define FILTERBUTTERWORTH_H

class FilterButterworth
{
public:
    FilterButterworth();
    void SetupFilterButterworth(double,int,double);
    double UpdateFilterButterworth(double);
private:
    double resonance;

    double frequency;
    int sampleRate;

    double q, c, a1, a2, a3, b1, b2;
    double inputHistory[2];
    double outputHistory[3];
};

#endif // FILTERBUTTERWORTH_H
