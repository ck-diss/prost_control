#ifndef RMS_H
#define RMS_H

#include <vector>

class RMS
{
public:
    RMS();
    double updateRMS(double);
    double getRMS(std::vector<double>&);

    double getNormVal(double);
    double getMeanAbs(std::vector<double>&);
    double getMeanAbsDeviation(std::vector<double>&);
    double crossCorr(double, double);
private:
    std::vector<double> arr;
};
#endif // RMS_H
