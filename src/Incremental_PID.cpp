#include "Incremental_PID.h"
void Incremental_PID::Incremental_PID()
{

}

void Incremental_PID::Initialize(double controlrate, double kp_, double kd_, double ki_, double upperlimit_, double lowerlimit_)
{
    delta_T = 1/controlrate;
    // digital gains
    kp = kp_;
    ki = ki_*delta_T;
    kd = kd_/delta_T;
    upperlimit = upperlimit_;
    lowerlimit = lowerlimit_;
    //
    u_k = 0;
    e_k = 0;
    e_k_1 = 0;
    e_k_2 = 0;
}
void Incremental_PID::Start()
{
    pid_flag = 1;
}
void Incremental_PID::Stop()
{
    pid_flag = 0;
}
void Incremental_PID::Reset()
{
    pid_flag = 0;
    u_k = 0;
    e_k = 0;
    e_k_1 = 0;
    e_k_2 = 0;
}
void Incremental_PID::PID_update(double e)
{
    // update errors
    e_k_2 = e_k_1;
    e_k_1 = e_k;
    e_k = e;
    // calculate current output
    double delta_u;// calculate output increment
    delta_u = kp*(e_k-e_k_1)+ki*e_k+kd*(e_k-2*e_k_1+e_k_2);
    u_k += delta_u;
}
void Incremental_PID::RosWhileLoopRun(double e)
{

    switch(pid_flag)
    {
    case 0:
        break; // if stopped, then do nothing
    case 1: {
        // if all normal run update
                PID_update(e);
    }
        break;
    case 2: {
        PID_update(e);// still do update
    }
        break;

    }
}
unsigned int Incremental_PID::GetPIDState()
{
    return pid_flag;// return flag as state
}
double Incremental_PID::LinearError(double reference, double state)
{
    return reference-state;
}
double Incremental_PID::AngularError(double reference, double state)
{
    // only accepts rad
    double cr,sr;
    double cs,ss;
    cr = cos(reference);
    sr = sin(reference);
    cs = cos(state);
    ss = sin(state);
    // const
    double xr[2];
    xr[0] = cr;
    xr[1] = sr;
    double x[2];
    x[0] = cs;
    x[1] = ss;
    double y[2];
    y[0] = -ss;
    y[1] = cs;
    double xrs[2];// calculate the reference vector in the rotated coordinate
    xrs[0] = xr[0]*x[0]+xr[1]*x[1];
    xrs[1] = xr[0]*y[0]+xr[1]*y[1];
    return atan2(xrs[1],xrs[0]);
}
double Incremental_PID::Rad2Deg(double rad)
{
    return 180/M_PI*rad;
}
double Incremental_PID::Deg2Rad(double deg)
{
    return deg/180*M_PI;
}
double Incremental_PID::GetPIDOutPut()
{

    double u= u_k;
    // determine whether the PID has reached a preset bound
    if(u_k-upperlimit>0)
    {
        u = upperlimit;// limit the magnitude
        pid_flag = 2;
    }
    if(lowerlimit-u_k>0)
    {
        u = lowerlimit;// limit the magnitude
        pid_flag = 2;
    }

    return u;
}
