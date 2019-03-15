#include "SignalGenerator.h"

void SignalGenerator::SignalGenerator()
{

}

void SignalGenerator::Initialize(double SquareDuration,double SquareWaveAmplitude,double SquareWaveFrequency, double Rosrate)
{
    signalflag = 0;
    signalrunTime =0;
    signalduration = SquareDuration;
    signalfrequency = SquareWaveFrequency;
    signalamplitude = SquareWaveAmplitude;
    timeincrement = 1/Rosrate;
    angularrate = 2*M_PI*signalfrequency;

}
void SignalGenerator::Start()
{
    signalflag = 1;
    signalrunTime =0;
}


void SignalGenerator::Stop()
{
       signalflag = 0;
       signalflag = 0;
}

double SignalGenerator::RunTimeUpdate()
{
    if(signalflag == 1)//if the generator is running
    {
        if (signalrunTime<signalduration)
        {
            double signal = sin(angularrate*signalrunTime);//use sine wave to generate square wave
            if (signal>0)//normal
            {
                signalvalue = signalamplitude;
            }else
            {
                signalvalue = -signalamplitude;
            }
                signalrunTime+=timeincrement;
        }else// signal end, stop the output
        {
            signalflag = 0;
        }

    }else{
        signalvalue = 0;// if the generator stops
    }
    return signalvalue;
}
int SignalGenerator::GetSignalStatus()
{
    switch(signalflag)
    {
    case 1:
        //ROS_INFO("Signal generator is running");
        break;
    case 0:
        //ROS_INFO("Signal generator has stopped");
        break;
    }
    return signalflag;
}
