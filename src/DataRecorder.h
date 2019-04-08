#pragma once
#include <sstream>
#include <fstream>
#include <iostream>
using namespace std;

class DataRecorder{
    int state;                     //recorder state
    const char* filename;
    unsigned int numbe_of_column;  //dimension of the data
    double delta_T;
    double recordingtime;          // recordingtime
    ofstream file;                 //file object that store the
public:
    DataRecorder(const char* name,unsigned int ud, double controlrate);
    ~DataRecorder();
    void StartRecording();
    void RecorderUpdate(double* data);
    void StopRecording();
};
