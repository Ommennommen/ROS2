#include "pid_controller/controller.h"
#include <fstream>
#include <iostream>

double PID_out(PID& pid, double err, double dt){

    pid.totErr += err * dt;
    double derivative = (err-pid.prevErr)/dt;
    pid.prevErr = err;
    return  pid.Kp*err + pid.Ki*pid.totErr + pid.Kd*derivative;

}

void saveCSV(const std::string& filename, double* val, int size) {
    std::ofstream file(filename);
    for (int i =0; i < size; ++i){
        if (i > 0) file << ",";
        file << val[i];
    }
    file.close();
}