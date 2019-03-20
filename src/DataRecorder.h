#pragma once
#include <sstream>
#include <fstream>
#include <iostream>

class DataRecorder{
    int state;                     //recorder state
    const char* filename;
    unsigned int numbe_of_column;  //dimension of the data
    double delta_T;
    double recordingtime;          // recordingtime
    ofstream file;                 //file object that store the
public:
    void StartRecording();
    void RecorderUpdate(double* data);
    void StopRecording();
    void Initialize(const char* name,unsigned int ud, double controlrate);
};
