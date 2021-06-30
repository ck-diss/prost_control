#include <QDebug>
#include "main.h"
#include "rms.h"
#include "ads.h"
#include <math.h>


RMS::RMS()
{

}

double RMS::updateRMS(double val)
{
    double newRMS = 0;
    // Windowsize 100ms (samplingRate/10)
//    qDebug() << "val: " << val;
    if(!arr.empty() && arr.size()>=windowSize) {
        arr.erase(arr.begin());
    }
    arr.push_back(val);
    if (arr.size()>=windowSize) {
        newRMS = getRMS(arr);
    }
    return(newRMS);
}


double RMS::getRMS(std::vector<double>& arr)
{
    int i;
    double sumsq;
    double calcRMS;
    sumsq = 0;
    for (i=0; i<arr.size(); i++) {
        sumsq += arr[i]*arr[i];
    }
    calcRMS = sqrt((static_cast<double>(1)/windowSize)*(sumsq));
    return (calcRMS);
}

// SABOLZSC IDEE

double RMS::getNormVal(double val)
{
    double newVal = 0;

    if(!arr.empty() && arr.size()>=windowSize) {
        arr.erase(arr.begin());
    }
    arr.push_back(val);
    if (arr.size()>=windowSize) {
        newVal = getMeanAbsDeviation(arr);
    }
    return(newVal);
}

double RMS::getMeanAbs(std::vector<double>& arr) {
    double sum = 0;
    for (int i=0; i<arr.size(); i++) {
        sum += std::abs(arr[i]);
    }
    //qDebug() << "sum: " << sum << " siuze: " << arr.size();

    return (sum/arr.size());
}

double RMS::getMeanAbsDeviation(std::vector<double>& arr) {
    double dev = 0;
    double meanAbs = getMeanAbs(arr);
    for (int i=0; i<arr.size(); i++) {
        dev += arr[i]-meanAbs;
    }
    //qDebug() << "mean: " << meanAbs << " dev: " << std::abs(dev);
    return (meanAbs/(std::abs(dev)));
}

double RMS::crossCorr(double val1, double val2) {
    return ((1+val1)/(1+val2));
}
