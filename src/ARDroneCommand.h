#pragma once
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
#ifndef windowsize_ARdrone
#define windowsize_ARdrone 5
#endif
class ARDroneCommand{
    int State; // command state
    // publishers
    ros::Publisher pubTakeoff;  //Takeoff command
    ros::Publisher pubLand;     // Landing command
    ros::Publisher pubReset;    // Reset command
    ros::Publisher pubMove;     // Move command
    // subscribers
    ros::Subscriber subNavdata; // Drone Nav Data
    // Command Messages
    std_msgs::Empty takeoff_;
    std_msgs::Empty landing_;
    std_msgs::Empty reset_;
    geometry_msgs::Twist command_;
    // Nav Messages
    static ardrone_autonomy::Navdata navdata;
    static void ReceiveNavdata(const ardrone_autonomy::Navdata& Navmsg);
    // Variables and Functions for yaw rate calculation
    double delta_T;
    double yaw_angle[2];
    double yaw_rate_raw[windowsize_ARdrone];
    double yaw_rate_filtered;
    void CalculateYawRateFromYawAngle();// calculate velocity info from pose update measurements
    void MovingWindowAveraging();// a filter using moving window
    void PushRawYawRate(double& newyawrate);// push newly measured velocity into raw velocity buffer
    void PushYawAngle();//push newly measured pose into dronepose buffer
public:
    void Initialize(ros::NodeHandle& n,double Controlrate);
    // Basic Command Functions
    void TakeOFF();
    void Land();
    void Reset();
    void VelocityCommandUpdate(double vx,double vy,double vz,double az);
    void Stop();
    void GetDroneState();//Get drone flying state
    double GetDroneYaw();
    double GetYawRate();
    double GetRawYawRate();
    void RosWhileLoopRun();// based on external command
    double AngularError(double reference, double state);
    double Rad2Deg(double rad);
    double Deg2Rad(double rad);
    /*
     * com = -1 0 1 2 3 4
    */

};
