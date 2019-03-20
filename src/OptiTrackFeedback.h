#pragma once
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#ifndef windowsize
#define windowsize 3
#endif
struct opitrack_velocity{
    double vx;
    double vy;
    double vz;
    double omega_x;
    double omega_y;
    double omega_z;
    double time_stamp;
};

struct opitrack_pose{
    double x;
    double y;
    double z;
    double q0;
    double q1;
    double q2;
    double q3;
    double t;
};

class OptiTrackFeedback{
    ros::Subscriber subOptiTrack;// OptiTrack Data
    static geometry_msgs::PoseStamped OptiTrackdata;
    static unsigned int OptiTrackFlag; // OptiTrackState 0: no data feed,: 1 data feed present
    unsigned int FeedbackState;// 0 no feedback, 1 has feedback
    static void OptiTrackCallback(const geometry_msgs::PoseStamped& msg);
    opitrack_velocity dronevelocity_raw[windowsize];// raw velocity buffer from numerical differentiation
    opitrack_velocity dronevelocity_filtered;// filtered velocity
    opitrack_pose     dronepose[2];// pose info from optitrack dronepose[1] should be the newly mesaured value, dronepose[0] is value of the last measurment (in world frame by default, if other frames
    // are used , please changle the frame selectioin in the launch file
    void CalculateVelocityFromPose();// calculate velocity info from pose update measurements
    void MovingWindowAveraging();// a filter using moving window
    void PushRawVelocity(opitrack_velocity& newvelocity);// push newly measured velocity into raw velocity buffer
    void PushPose();//push newly measured pose into dronepose buffer
    void SetZeroVelocity();
public:
    void Initialize(ros::NodeHandle& n);
    int GetOptiTrackState();
    opitrack_velocity GetVelocity();
    opitrack_velocity GetRaWVelocity();
    opitrack_pose GetPose();
    void RosWhileLoopRun();// This function should be put into ros while loop
    void GetEulerAngleFromQuaterion_NormalConvention(double (&eulerangle)[3]);
    void GetEulerAngleFromQuaterion_OptiTrackYUpConvention(double (&eulerangle)[3]);
};
