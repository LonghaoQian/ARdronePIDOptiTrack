#include "ros/ros.h"

#include <termios.h>
#include "ARDroneCommand.h"
#include "SignalGenerator.h"
#include "OptiTrackFeedback.h"
#include "DataRecorder.h"
#include "Incremental_PID.h"
#include "KeyboardEvent.h"
using namespace std;

int keyboard_input;

///////////////////////////// Keyboard Command ///////////////////////////////
//Get Keyboard Input
char getch()
{
        keyboard_input = -1; //if not key is pressed, then return -1 for the keyboard_input. reset this flag every time
        fd_set set;
        struct timeval timeout;
        int rv;
        char buff = 0;
        int len = 1;
        int filedesc = 0;
        FD_ZERO(&set);
        FD_SET(filedesc, &set);

        timeout.tv_sec = 0;
        //timeout.tv_usec = 1000;//
        timeout.tv_usec = 1000;
        rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

        struct termios old = {0};
        if (tcgetattr(filedesc, &old) < 0)
                ROS_ERROR("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(filedesc, TCSANOW, &old) < 0)
                ROS_ERROR("tcsetattr ICANON");

        if(rv == -1){
                ROS_ERROR("select");
        }
        else if(rv == 0)
        {
                //{ROS_INFO("-----");
        }
        else
                {read(filedesc, &buff, len );
        keyboard_input = 1;}

        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
                ROS_ERROR ("tcsetattr ~ICANON");
        return (buff);
}
// Initialize all the stuff
void Initialization()
{
    keyboard_input = -1;
}





void KeybordEvent(char buff,ARDroneCommand& comchannel,
                  SignalGenerator& signal,
                  DataRecorder& recorder,
                  OptiTrackFeedback& optitrack,
                  Incremental_PID& yaw_rate_pid,
                  Incremental_PID& vertical_speed_pid,
                  Incremental_PID& x_speed_pid,
                  Incremental_PID& z_speed_pid)
{
    if(keyboard_input>-1)//if keyboard is
    {
        switch(buff)
               {/*t*/case 116: {
                    comchannel.TakeOFF();
                        ROS_INFO("TakeOFF_command_sent");}
                     break;
            /*Space*/case 32:
                    {
                        signal.Stop();
                        recorder.StopRecording();
                        comchannel.Land();
                        yaw_rate_pid.Stop();
                        vertical_speed_pid.Stop();
                        x_speed_pid.Stop();
                        x_speed_pid.Reset();
                        z_speed_pid.Stop();
                        z_speed_pid.Reset();
                        ROS_INFO("Land");
                    }
                     break;
                /*r*/case 114: {
                        comchannel.Reset();
                        ROS_INFO("Reset_command_sent");
                        comchannel.GetDroneState();
                     }
                     break;
                /*w*/case 119:
                     {
                        signal.Start();
                        recorder.StartRecording();
                        yaw_rate_pid.Start();
                        vertical_speed_pid.Start();
                         x_speed_pid.Start();
                          z_speed_pid.Start();
                        ROS_INFO("Start Test");
                     }
                     break;
                /*s*/case 115:
                     {
                        signal.Stop();
                        recorder.StopRecording();
                        yaw_rate_pid.Stop();
                        yaw_rate_pid.Reset();
                        vertical_speed_pid.Stop();
                        vertical_speed_pid.Reset();
                        x_speed_pid.Stop();
                        x_speed_pid.Reset();
                        z_speed_pid.Stop();
                        z_speed_pid.Reset();
                        ROS_INFO("End Test");
                     }
                     break;
                /*q*/case 113:{
                               comchannel.GetDroneState();
                               signal.GetSignalStatus();
                               if(optitrack.GetOptiTrackState()==1)
                               {
                                   ROS_INFO("OptiTrack Normal");
                               }else{

                                   ROS_INFO("No OptiTrack Data Feed");
                               }

                                }
                     break;
                /*e*/case 101: comchannel.Stop();
                     break;


              }

    }else
    { // do nothing if keyboard is not pressed
    }


}



int main(int argc, char **argv)
{
  //Initialization custom function
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
    DataRecorder psi_recorder;
    DataRecorder velocity_recorder;
    psi_recorder.Initialize("psi_data.txt",6,Control_Rate);
    velocity_recorder.Initialize("vz_data.txt",4,Control_Rate);
    double data[6];
    double dataz[4];
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
    // x and y poistion error
    KeyboardEvent keyboardcontrol;
    keyboardcontrol.PushCallBack(116,comchannel_1.TakeOFF());  /*t*/
    keyboardcontrol.PushCallBack(32, signal.Stop());  /*Space*/
    keyboardcontrol.PushCallBack(32, recorder.StopRecording());  /*Space*/
    keyboardcontrol.PushCallBack(32, comchannel.Land();  /*Space*/


    yaw_rate_pid.Stop();
    vertical_speed_pid.Stop();
    x_speed_pid.Stop();
    x_speed_pid.Reset();
    z_speed_pid.Stop();
    z_speed_pid.Reset();


    keyboardcontrol.PushCallBack(114,comchannel_1.TakeOFF());  /*r*/
    keyboardcontrol.PushCallBack(119,comchannel_1.TakeOFF());  /*w*/
    keyboardcontrol.PushCallBack(115,comchannel_1.TakeOFF());  /*s*/
    keyboardcontrol.PushCallBack(113,comchannel_1.TakeOFF());  /*q*/
    keyboardcontrol.PushCallBack(101,comchannel_1.TakeOFF());  /*e*/
    keyboardcontrol.BendingComplete();
// Set Ros Excution Rate
    ros::Rate loop_rate(Control_Rate);
    velocity_recorder.StartRecording();
    //double yaw_ref = 70;
  while (ros::ok())
  {

      //////////////////////////////// Fix this part with QT library later /////////////////////
      SignalOutput = sig_1.RunTimeUpdate();
      // send commands
      comchannel_1.VelocityCommandUpdate(horizontal_x_pid.GetPIDOutPut(),-horizontal_z_pid.GetPIDOutPut(),vertical_speed_pid.GetPIDOutPut(),yaw_rate_pid.GetPIDOutPut());
      data[1] = yaw_rate_pid.GetPIDOutPut();
      comchannel_1.RosWhileLoopRun();// flush the commands and calculations

      //ROS_INFO("Signal is [%f]", SignalOutput);
      ros::spinOnce();// do the loop once
      OpTiFeedback.RosWhileLoopRun();
      // Yaw control loop:
      //yaw_error = SignalOutput*100 - comchannel_1.GetYawRate();// rate control
      //yaw_error = comchannel_1.Rad2Deg(comchannel_1.AngularError(comchannel_1.Deg2Rad(yaw_angle_cmd),comchannel_1.Deg2Rad(comchannel_1.GetDroneYaw())));// angle control
      yaw_error = comchannel_1.Rad2Deg(comchannel_1.AngularError(comchannel_1.Deg2Rad(yaw_angle_cmd),euler_optitrack[2]));// angle from optitrack
      yaw_rate_pid.RosWhileLoopRun(yaw_error/100);// push error into pid
      // Altitude Control Loop
//      vz_error = SignalOutput - OpTiFeedback.GetVelocity().vy;
//      vertical_speed_pid.RosWhileLoopRun(vz_error);

      z_error =  OpTiFeedback.GetPose().y - command_altitue;
      vz_error = -0.8*z_error-OpTiFeedback.GetVelocity().vy;
      vertical_speed_pid.RosWhileLoopRun(vz_error);// use s.

      //save data
      if(sig_1.GetSignalStatus()==0)
      {
          psi_recorder.StopRecording();
      }
      data[0] = SignalOutput;

      data[2] = comchannel_1.GetYawRate();
      data[3] = comchannel_1.GetRawYawRate();
      data[4] = comchannel_1.GetDroneYaw();
      data[5] = yaw_error;
      psi_recorder.RecorderUpdate(data);

      // z loop:
      if(sig_1.GetSignalStatus()==0)
      {
          velocity_recorder.StopRecording();
      }
      dataz[0] = SignalOutput;
      dataz[1] = OpTiFeedback.GetPose().y;
      dataz[2] = OpTiFeedback.GetRaWVelocity().vy;
      dataz[3] = OpTiFeedback.GetVelocity().vy;
      velocity_recorder.RecorderUpdate(dataz);
      // Get body angles
      OpTiFeedback.GetEulerAngleFromQuaterion_OptiTrackYUpConvention(euler_optitrack);
      // Horizontal r
      s_x_w = ke*(OpTiFeedback.GetPose().x -command_x)+OpTiFeedback.GetVelocity().vx;
      s_y_w = ke*(OpTiFeedback.GetPose().z -command_z)+OpTiFeedback.GetVelocity().vz;
      // Transfer velocity and poisiton
      s_x_b = s_x_w*cos(-euler_optitrack[2])+s_y_w*sin(-euler_optitrack[2]);
      s_y_b = -s_x_w*sin(-euler_optitrack[2])+s_y_w*cos(-euler_optitrack[2]);
      horizontal_x_pid.RosWhileLoopRun(-s_x_b);
      horizontal_z_pid.RosWhileLoopRun(-s_y_b);

      //ROS_INFO("yaw angle is [%f] x position error is [%f] z position error is [%f]",euler_optitrack[2]*57.3,s_x_b,s_y_b);
      // loop wait
      loop_rate.sleep();

  }
  velocity_recorder.StopRecording();
  //pubLand.publish(landing_);// after the
  return 0;
}

