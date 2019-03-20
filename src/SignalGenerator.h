#pragma once
#ifndef M_PI
# define M_PI       3.14159265358979323846  /* pi */
#endif
class SignalGenerator{
    int    signalflag;            //signal generator running flag
    double signalrunTime;      //signal running time
    double signalfrequency;    //signal frequency
    double signalamplitude;    //signal amplitude
    double signalduration;     //signal duration
    double timeincrement;      //time increment based on the rosrate
    double angularrate;        //angular velocity of the signal
    double signalvalue;        //signal output value
public:
    void Initialize(double SquareDuration,double SquareWaveAmplitude,double SquareWaveFrequency, double Rosrate);
    void Stop();
    void Start();
    int GetSignalStatus();
    double RunTimeUpdate();

};
