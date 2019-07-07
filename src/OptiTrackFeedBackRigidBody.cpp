#include "OptiTrackFeedBackRigidBody.h"

OptiTrackFeedBackRigidBody::OptiTrackFeedBackRigidBody(const char* name,ros::NodeHandle& n)
{
    subOptiTrack = n.subscribe(name, 1, &OptiTrackFeedBackRigidBody::OptiTrackCallback,this);
    //Initialize all velocity
    for(int i =0;i<windowsize;i++)
    {
        velocity_raw[i].omega_x = 0;
        velocity_raw[i].omega_y = 0;
        velocity_raw[i].omega_z = 0;
        velocity_raw[i].time_stamp = 0;
        velocity_raw[i].vx=0;
        velocity_raw[i].vy=0;
        velocity_raw[i].vz=0;
    }
    velocity_filtered.omega_x=0;
    velocity_filtered.omega_y=0;
    velocity_filtered.omega_z=0;
    velocity_filtered.time_stamp=0;
    velocity_filtered.vx=0;
    velocity_filtered.vy=0;
    velocity_filtered.vz=0;
    //Initialize all pose
    for(int i = 0;i<2;i++)
    {
        pose[i].q0 = 1;
        pose[i].q1 = 0;
        pose[i].q2 = 0;
        pose[i].q3 = 0;
        pose[i].t = 0;
        pose[i].x = 0;
        pose[i].y = 0;
        pose[i].z = 0;

    }
    // Initialize flag
    OptiTrackFlag = 0;
    FeedbackState = 0;
}

void OptiTrackFeedBackRigidBody::CalculateVelocityFromPose()
{

    /* Logic:
     * 1) push the current pose into buffer
     * 2) determine whether the buffer has a valid time value  (dronepose[0].t >0); if so calculate velocity
     * 3) if not just set the velocity_onestep as zero
     * 4) push current time, and velocity_onestep into the velocity buffer
     * 5) calculate filtered velocity
    */
    // perform the Logic:
    // step (1): push the current pose into buffer
    PushPose();
    // step (2): determine whether the buffer has a valid time value  (dronepose[0].t >0); if so calculate velocity
    double dt = 0.0;
    opitrack_velocity velocity_onestep;
  if (pose[0].t >0)// calculate only when last time stamp has been recorded.
  {
      // step (2)
      dt = pose[1].t - pose[0].t;// time step
      // calculate x direction velocity
      velocity_onestep.vx = (pose[1].x - pose[0].x)/dt;
      velocity_onestep.vy = (pose[1].y - pose[0].y)/dt;
      velocity_onestep.vz = (pose[1].z - pose[0].z)/dt;
      // will add rotation speed later
      velocity_onestep.omega_x = 0;
      velocity_onestep.omega_y = 0;
      velocity_onestep.omega_z = 0;
  }else// step (3): if not set velocity to zero and only record the time
  {
      velocity_onestep.vx = 0.0;
      velocity_onestep.vy = 0.0;
      velocity_onestep.vz = 0.0;
      velocity_onestep.omega_x = 0;
      velocity_onestep.omega_y = 0;
      velocity_onestep.omega_z = 0;
      // will add rotation speed later
  }
  velocity_onestep.time_stamp = pose[1].t;
  // step (4): push current time, and velocity_onestep into the velocity buffer
  PushRawVelocity(velocity_onestep);
  // step (5): calculate filtered velocity
  MovingWindowAveraging();
}
void OptiTrackFeedBackRigidBody::PushPose()
{
    pose[0] = pose[1];// straightforward push the pose into buffer
    // update the latest pose
    double t_current = (double)OptiTrackdata.header.stamp.sec + (double)OptiTrackdata.header.stamp.nsec*0.000000001;
    pose[1].t = t_current;
    // take a special note at the order of the quaterion
    pose[1].q0 = OptiTrackdata.pose.orientation.w;
    pose[1].q1 = OptiTrackdata.pose.orientation.x;
    pose[1].q2 = OptiTrackdata.pose.orientation.y;
    pose[1].q3 = OptiTrackdata.pose.orientation.z;
    // pose is straight forward
    pose[1].x =  OptiTrackdata.pose.position.x;
    pose[1].y =  OptiTrackdata.pose.position.y;
    pose[1].z =  OptiTrackdata.pose.position.z;
}

void OptiTrackFeedBackRigidBody::PushRawVelocity(opitrack_velocity& newvelocity)
{
    /* Logic:
     * a(i-1) = a(i), i = 2...windowsize
     * should fristly start from  i = 2. a(1) = a(2); a(2) = a(3);....; a(N-1) = a(N)
     * secondly a(N) = a_new
    */
    for(int i = 1;i<windowsize;i++)//first step
    {
        velocity_raw[i-1] = velocity_raw[i];
    }
    velocity_raw[windowsize-1] = newvelocity;// second step update the last variable in the velocity buffer
}

void OptiTrackFeedBackRigidBody::MovingWindowAveraging()
{

    /* Logic: Average the raw velocity measurement in the
    */
    double weight = (double)1/windowsize;// the weight on each velocity to be summed up.
    // create a temporary variable to store the summed velocity and initialize it witht the 1st buffer value
    opitrack_velocity velocitytemp;
    velocitytemp.omega_x= weight*velocity_raw[0].omega_x;
    velocitytemp.omega_y= weight*velocity_raw[0].omega_y;
    velocitytemp.omega_z= weight*velocity_raw[0].omega_z;
    velocitytemp.vx = weight*velocity_raw[0].vx;
    velocitytemp.vy = weight*velocity_raw[0].vy;
    velocitytemp.vz = weight*velocity_raw[0].vz;

    for(int i = 1;i<windowsize;i++)// sum starts from the second buffer value
    {
        velocitytemp.omega_x+= weight*velocity_raw[i].omega_x;
        velocitytemp.omega_y+= weight*velocity_raw[i].omega_y;
        velocitytemp.omega_z+= weight*velocity_raw[i].omega_z;
        velocitytemp.vx += weight*velocity_raw[i].vx;
        velocitytemp.vy += weight*velocity_raw[i].vy;
        velocitytemp.vz += weight*velocity_raw[i].vz;
    }
    velocitytemp.time_stamp = velocity_raw[windowsize-1].time_stamp;
    // the filtered vlocity is just the weighted summed result
    velocity_filtered = velocitytemp;
}

opitrack_pose OptiTrackFeedBackRigidBody::GetPose()
{
    return pose[1];//return the latest pose
}

opitrack_velocity OptiTrackFeedBackRigidBody::GetVelocity()
{
    return velocity_filtered;// return the filtered velocity
}
opitrack_velocity OptiTrackFeedBackRigidBody::GetRaWVelocity()
{
    return velocity_raw[windowsize-1];// return the filtered velocity
}
void  OptiTrackFeedBackRigidBody::SetZeroVelocity()
{
    for(int i =0;i<windowsize;i++)
    {
        velocity_raw[i].omega_x = 0;
        velocity_raw[i].omega_y = 0;
        velocity_raw[i].omega_z = 0;
        velocity_raw[i].vx=0;
        velocity_raw[i].vy=0;
        velocity_raw[i].vz=0;
    }
    velocity_filtered.omega_x=0;
    velocity_filtered.omega_y=0;
    velocity_filtered.omega_z=0;
    velocity_filtered.vx=0;
    velocity_filtered.vy=0;
    velocity_filtered.vz=0;
}

void OptiTrackFeedBackRigidBody::RosWhileLoopRun()
{
    if(OptiTrackFlag==1)
    {// update the velocity only when there is OptiTrack feedback
        CalculateVelocityFromPose();
        FeedbackState=1;
    }else{
        // if the optitrack measurements no longer feedback, when the pose update will stop and we only return 0 velocity
        SetZeroVelocity();
        FeedbackState=0;
    }

    OptiTrackFlag = 0;// reset the feedback flag to 0
}
int OptiTrackFeedBackRigidBody::GetOptiTrackState()
{
    if (FeedbackState==1) {
      ROS_INFO("OptiTrack:Normal");
    }else{
      ROS_INFO("OptiTrack:No FeedBack");
    }
    return FeedbackState;

}
void OptiTrackFeedBackRigidBody::GetEulerAngleFromQuaterion_NormalConvention(double (&eulerangle)[3])
{


    /* Normal means the following https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    */
//    eulerangle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
//    eulerangle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
//    eulerangle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));


    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (pose[1].q0 * pose[1].q1 + pose[1].q2 * pose[1].q3);
    double cosr_cosp = +1.0 - 2.0 * (pose[1].q1 * pose[1].q1 +pose[1].q2 * pose[1].q2);
    double roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (pose[1].q0 * pose[1].q2 - pose[1].q3 * pose[1].q1);
    double pitch;
    if (fabs(sinp) >= 1)
           pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
           pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (pose[1].q0 * pose[1].q3 + pose[1].q1 * pose[1].q2);
    double cosy_cosp = +1.0 - 2.0 * (pose[1].q2 * pose[1].q2 + pose[1].q3 * pose[1].q3);
    double yaw = atan2(siny_cosp, cosy_cosp);
    //double yaw  = atan2(2.0 * (dronepose[1].q3 * dronepose[1].q0 + dronepose[1].q1 * dronepose[1].q2), -1.0 + 2.0 * (dronepose[1].q0 * dronepose[1].q0 + dronepose[1].q1 * dronepose[1].q1));
    eulerangle[0] = roll;
    eulerangle[1] = pitch;
    eulerangle[2] = yaw;

}

void OptiTrackFeedBackRigidBody::GetEulerAngleFromQuaterion_OptiTrackYUpConvention(double (&eulerangle)[3])
{

    // OptiTrack gives a quaternion with q2 and q3 flipped.
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (pose[1].q0 * pose[1].q1 + pose[1].q3 * pose[1].q2);
    double cosr_cosp = +1.0 - 2.0 * (pose[1].q1 * pose[1].q1 +pose[1].q3 * pose[1].q3);
    double roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (pose[1].q0 * pose[1].q3 - pose[1].q2 * pose[1].q1);
    double pitch;
    if (fabs(sinp) >= 1)
           pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
           pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (pose[1].q0 * pose[1].q2 + pose[1].q1 * pose[1].q3);
    double cosy_cosp = +1.0 - 2.0 * (pose[1].q2 * pose[1].q2 + pose[1].q3 * pose[1].q3);
    double yaw = atan2(siny_cosp, cosy_cosp);
    eulerangle[0] = roll;
    eulerangle[1] = pitch;
    eulerangle[2] = yaw;

}

void OptiTrackFeedBackRigidBody::OptiTrackCallback(const geometry_msgs::PoseStamped& msg)
{
        // must use head information to distiguish the correct 
        OptiTrackdata = msg; // update optitrack data
        OptiTrackFlag = 1;// signal a new measurement feed has been revcieved.
}
// declare static member variables
//geometry_msgs::PoseStamped OptiTrackFeedBackRigidBody::OptiTrackdata;
//unsigned int OptiTrackFeedBackRigidBody::OptiTrackFlag;