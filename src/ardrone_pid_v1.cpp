#include "ros/ros.h"
#include "ARDroneCommand.h"
#include "SignalGenerator.h"
#include "OptiTrackFeedback.h"
#include "DataRecorder.h"
#include "Incremental_PID.h"
#include "KeyboardEvent.h"
using namespace std;

int main(int argc, char **argv)
{
    SignalGenerator sig_1;
    SignalGenerator vertial_signal;
    double Control_Rate = 40;// Hz the rate
    double SquareWaveTime = 40;// Time for the signal generator
    double SquareWaveAmplitude = 0.3;//m/s amplitude for square waves
    double SquareWaveFrequency = 0.25;//Frequency of the square wave
    double SignalOutput = 0;
    sig_1.Initialize(SquareWaveTime,SquareWaveAmplitude,SquareWaveFrequency,Control_Rate);
    vertial_signal.Initialize(SquareWaveTime,SquareWaveAmplitude,SquareWaveFrequency,Control_Rate);
    // Initialize ros node
    ros::init(argc, argv, "ardrone_pid_v1");
    ros::NodeHandle n;
    // Initialize OptiTrack System
    OptiTrackFeedback OpTiFeedback;
    OpTiFeedback.Initialize(n);
    // ARDrone class
    ARDroneCommand comchannel_1;
    comchannel_1.Initialize(n,Control_Rate);
    // PID control
    Incremental_PID yaw_rate_pid;
    Incremental_PID vertical_speed_pid;
    Incremental_PID horizontal_x_pid;
    Incremental_PID horizontal_z_pid;
    //yaw_rate_pid.Initialize(Control_Rate,3.9/100,0,26/100,0.9,-0.9);
    //yaw_rate_pid.Initialize(Control_Rate,1,0,1,1,-1);
    yaw_rate_pid.Initialize(Control_Rate,5,0,0,1,-1);// kp = 5 1
    vertical_speed_pid.Initialize(Control_Rate,2,0,0,1,-1);
    horizontal_x_pid.Initialize(Control_Rate,0.25,0,0,1,-1);
    horizontal_z_pid.Initialize(Control_Rate,0.25,0,0,1,-1);
    // Initialize Data Recorder
    DataRecorder dronestate_recorder("ARdroneState.txt",16,Control_Rate);
    double dronestate[16];
    /*  0 reference signal
        1 - 3 drone positition from OpTiFeedback (hx,hy,Altitude)
        4 - 6 drone filtered velocity from OpTiFeedback
        7 - 9 drone PID position control inputs
        10 - 12 drone raw velocity from OpTiFeedback
        13 drone yaw angles for OpTiFeedback
        14 drone yaw pif control inputs
        15 error signal
     */
    double yaw_angle_cmd = 0;
    double yaw_error;
    double vz_error;
    double command_altitue = 1.2;
    double command_x = 0;
    double command_z = 0;
    double z_error = 0;
    double euler_optitrack[3];
    double ke = 1;
    double k = 1;
    double s_x_w = 0;
    double s_y_w = 0;
    double s_x_b = 0;
    double s_y_b = 0;
    KeyboardEvent keyboardcontrol;
    ros::Rate loop_rate(Control_Rate);
  while (ros::ok())
  {
      keyboardcontrol.RosWhileLoopRun();
      switch (keyboardcontrol.GetPressedKey())
      {
        case U_KEY_T:
        {
          comchannel_1.TakeOFF();
          break;
        }
        case U_KEY_W:
        {
            dronestate_recorder.StartRecording();
            sig_1.Start();
            horizontal_x_pid.Start();
            horizontal_z_pid.Start();
            vertical_speed_pid.Start();
            yaw_rate_pid.Start();
            break;
        }
        case U_KEY_S:
        {
          sig_1.Stop();
          dronestate_recorder.StopRecording();
          horizontal_x_pid.Stop();
          horizontal_x_pid.Reset();
          horizontal_z_pid.Stop();
          horizontal_z_pid.Reset();
          vertical_speed_pid.Stop();
          vertical_speed_pid.Reset();
          yaw_rate_pid.Stop();
          yaw_rate_pid.Reset();
          break;
        }
        case U_KEY_Q:
        {
           comchannel_1.GetDroneState();
           OpTiFeedback.GetOptiTrackState();
           break;
        }
        case U_KEY_SPACE:
        {
          comchannel_1.Land();
          sig_1.Stop();
          dronestate_recorder.StopRecording();
          break;
        }
        case U_KEY_R:
        {
          comchannel_1.Reset();
          break;
        }
        case U_KEY_NONE:
        {
          break;
        }
      }
      // update sginal generator
      SignalOutput = sig_1.RunTimeUpdate();

      // send commands to ARDrone
      comchannel_1.VelocityCommandUpdate(
      horizontal_x_pid.GetPIDOutPut(),
      -horizontal_z_pid.GetPIDOutPut(),
      vertical_speed_pid.GetPIDOutPut(),
      yaw_rate_pid.GetPIDOutPut());

      comchannel_1.RosWhileLoopRun();
      ros::spinOnce();// do the loop once
      OpTiFeedback.RosWhileLoopRun();
      //yaw_error = SignalOutput*100 - comchannel_1.GetYawRate();// rate control
      OpTiFeedback.GetEulerAngleFromQuaterion_OptiTrackYUpConvention(euler_optitrack);// get body angles
      yaw_error = comchannel_1.Rad2Deg(comchannel_1.AngularError(comchannel_1.Deg2Rad(yaw_angle_cmd),euler_optitrack[2]));// angle from optitrack
      yaw_rate_pid.RosWhileLoopRun(yaw_error/100);// push error into pid
      // Horizontal position errors
      s_x_w = ke*(OpTiFeedback.GetPose().x -command_x)+OpTiFeedback.GetVelocity().vx;
      s_y_w = ke*(OpTiFeedback.GetPose().z -command_z)+OpTiFeedback.GetVelocity().vz;
      // Transfer velocity and poisiton
      s_x_b = s_x_w*cos(-euler_optitrack[2])+s_y_w*sin(-euler_optitrack[2]);
      s_y_b = -s_x_w*sin(-euler_optitrack[2])+s_y_w*cos(-euler_optitrack[2]);
      horizontal_x_pid.RosWhileLoopRun(-s_x_b);
      horizontal_z_pid.RosWhileLoopRun(-s_y_b);
      // Altitude Control Loop
      // vz_error = SignalOutput - OpTiFeedback.GetVelocity().vy;
      // vertical_speed_pid.RosWhileLoopRun(vz_error);

      z_error =  OpTiFeedback.GetPose().y - command_altitue;
      vz_error = -0.8*z_error-OpTiFeedback.GetVelocity().vy;
      vertical_speed_pid.RosWhileLoopRun(vz_error);// use s

      //save data
      if(sig_1.GetSignalStatus()==0)
      {
          dronestate_recorder.StopRecording();// if the signal stops, then stop recording

      }else{
        dronestate[0] = SignalOutput;
        dronestate[1] = OpTiFeedback.GetPose().x;
        dronestate[2] = OpTiFeedback.GetPose().y;
        dronestate[3] = OpTiFeedback.GetPose().z;
        dronestate[4] = OpTiFeedback.GetVelocity().vx;
        dronestate[5] = OpTiFeedback.GetVelocity().vy;
        dronestate[6] = OpTiFeedback.GetVelocity().vz;
        dronestate[7] = horizontal_x_pid.GetPIDOutPut();
        dronestate[8] = -horizontal_z_pid.GetPIDOutPut();
        dronestate[9] = vertical_speed_pid.GetPIDOutPut();
        dronestate[10] = OpTiFeedback.GetRaWVelocity().vx;
        dronestate[11] = OpTiFeedback.GetRaWVelocity().vy;
        dronestate[12] = OpTiFeedback.GetRaWVelocity().vz;
        dronestate[13] = euler_optitrack[2];
        dronestate[14] = yaw_rate_pid.GetPIDOutPut();
        dronestate[15] = z_error;// change this according to control task
        dronestate_recorder.RecorderUpdate(dronestate);
      }
      loop_rate.sleep();

  }
  return 0;
}
