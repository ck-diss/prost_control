#include "filterbutterworth.h"
#include <math.h>



FilterButterworth::FilterButterworth(){
    resonance = 0;

    frequency = 0;
    sampleRate = 0;

    c = a1 = a2 = a3 = b1 = b2 = 0;
    inputHistory[2] = {0};
    outputHistory[3] = {0};
};

//void FilterButterworth::SetupFilterButterworth(double frequency, int sampleRate, double resonance)
//{
//    c = (double)tan(M_PI * frequency / sampleRate);
//    a1 = 1.0f / (1.0f + resonance * c + c * c);
//    a2 = -2.0f * a1;
//    a3 = a1;
//    b1 = 2.0f * (c * c - 1.0f) * a1;
//    b2 = (1.0f - resonance * c + c * c) * a1;
//}

void FilterButterworth::SetupFilterButterworth(double frequency, int sampleRate, double resonance)
{
    c = 1.0f / (double)tan(M_PI * 0.0075);
    q  = sqrt(2.0);
    a1 = 1.0f / (1.0f + q*c + c * c);
    a2 = 2.0f * a1;
    a3 = a1;
    b1 = 2.0f * (c * c - 1.0f) * a1;
    b2 = -(1.0f - q * c + c * c) * a1;
}


double FilterButterworth::UpdateFilterButterworth(double newInput)
{
    double newOutput = a1 * newInput + a2 * inputHistory[0] + a3 * inputHistory[1] - b1 * outputHistory[0] - b2 * outputHistory[1];

    inputHistory[1] = inputHistory[0];
    inputHistory[0] = newInput;

    outputHistory[2] = outputHistory[1];
    outputHistory[1] = outputHistory[0];
    outputHistory[0] = newOutput;

    return (newOutput);
}

