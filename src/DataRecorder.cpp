#include "DataRecorder.h"
DataRecorder::DataRecorder(const char* name,unsigned int ud, double controlrate)
{
  filename = name;
  numbe_of_column = ud;
  state = 0;
  delta_T = 1/controlrate;
  recordingtime=0;
  //name a file
  file.open(filename, std::ofstream::out);
  file.close();
}

void DataRecorder::StartRecording()
{
    state = 1;
    file.open(filename, std::ofstream::out);
}
void DataRecorder::RecorderUpdate(double* data)
{
    if(state ==1)
    {
        file <<"t--"<<recordingtime<<"--data";
        for(int i=0;i<numbe_of_column;i++)
        {
        file <<"--"<<data[i];
        }
        file <<endl;
        recordingtime+=delta_T;
    }
}
void DataRecorder::StopRecording()
{
    state = 0;
    file.close();
}

DataRecorder::~DataRecorder()
{
    file.close();
}
