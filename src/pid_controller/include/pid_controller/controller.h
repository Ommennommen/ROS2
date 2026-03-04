#pragma once
#include <iostream>

struct Spring{
    double mass;
    double position;
    double velocity;
    double k;
    double c;
};

struct PID{
    double Kp;
    double Ki;
    double Kd;
    double totErr;
    double prevErr;
};
double PID_out(PID& pid, double err, double dt);
void saveCSV(const std::string& filename, double* val, int size);