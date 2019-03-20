#pragma once

////////////////////////////// Incremental PID /////////////////////////////
class Incremental_PID{
    /* single channel digital pid controller
     * in the initialiation function, put kp ki and kd as for a continous PID, the initializer will automatically convert them into
     * corresponding digital PID gains
    */
    double delta_T;
    double kp;
    double ki;
    double kd;
    double upperlimit;
    double lowerlimit;
    double u_k;// output u(k)
    double e_k;//e(k)
    double e_k_1;//e(k-1)
    double e_k_2;//e(k-2)
    void PID_update(double e);// control output update
    unsigned int pid_flag;//  flag 0: stop  1: running 2; fault
public:
    void Initialize(double controlrate, double kp, double kd, double ki, double upperlimit, double lowerlimit);//
    void Start();//start the controller
    void Stop(); //stop the controller
    void Reset(); //reset the contoller
    unsigned int GetPIDState();
    void RosWhileLoopRun(double e);// in ros while loop update the controller e is the PID input and u is the output
    double GetPIDOutPut();
    // some utility tools to help calculate errors
    double LinearError(double reference, double state);
    double AngularError(double reference, double state);// only accepts rad
    double Rad2Deg(double rad);
    double Deg2Rad(double deg);

};
