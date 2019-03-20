#include "ARDroneCommand.h"

void ARDroneCommand::ARDroneCommand()
{

}

void ARDroneCommand::Initialize(ros::NodeHandle& n,double Controlrate)
{
    //Advertise Publisher
    pubTakeoff   = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
    pubLand      = n.advertise<std_msgs::Empty>("ardrone/land", 1);
    pubReset     = n.advertise<std_msgs::Empty>("ardrone/reset", 1);
    pubMove      = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    subNavdata   = n.subscribe("ardrone/navdata", 1, this->ReceiveNavdata);
    State = 0;
    for(int i=0;i<windowsize_ARdrone;i++)
    {
        yaw_rate_raw[i] = 0;
    }
    yaw_angle[0] = 0;
    yaw_angle[1] = 0;
    yaw_rate_filtered = 0;
    delta_T = 1/Controlrate;
}
void ARDroneCommand::Land()
{
    pubLand.publish(landing_);
}
void ARDroneCommand::TakeOFF()
{
    //This function will only perform take_off when the navdata.state ==2 (Landed)
    if(navdata.state==2)
    {pubTakeoff.publish(takeoff_);}
}
void ARDroneCommand::Reset()
{pubReset.publish(reset_);}
void  ARDroneCommand::VelocityCommandUpdate(double vx,double vy,double vz,double az)
{
    command_.linear.x = vx;
    command_.linear.y = vy;
    command_.linear.z = vz;
    command_.angular.z = az;
}
void ARDroneCommand::Stop()
{
    command_.linear.x = 0;
    command_.linear.y = 0;
    command_.linear.z = 0;
    command_.angular.z = 0;
}
void ARDroneCommand::GetDroneState()
{
    /* 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test
       6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping
    Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)*/
    switch(navdata.state)
    {
    case 0:
        ROS_INFO("Unknown");
        break;
    case 1:
        ROS_INFO("Init");
        break;
    case 2:
        ROS_INFO("Landed");
        break;
    case 3:
        ROS_INFO("Flying");
        break;
    case 4:
        ROS_INFO("Hovering");
        break;
    case 5:
        ROS_INFO("Test");
        break;
    case 6:
        ROS_INFO("Test");
        break;
    case 7:
        ROS_INFO("Goto Fix Point");
        break;
    case 8:
        ROS_INFO("Landing");
        break;
    case 9:
        ROS_INFO("Looping");
        break;
    }
    ROS_INFO("Battery precent is: [%f]",navdata.batteryPercent);
}
void ARDroneCommand::RosWhileLoopRun()
{
    //Do the velocity calculation
    CalculateYawRateFromYawAngle();
    // push moving command to drone
    pubMove.publish(command_);

}

double ARDroneCommand::GetDroneYaw()
{
    return navdata.rotZ; // return yaw angle in magnetometer (mag change it to yaw_angle[1])
}


void ARDroneCommand::PushYawAngle()
{
    yaw_angle[0] = yaw_angle[1];
    yaw_angle[1] = navdata.rotZ;
}
void ARDroneCommand::PushRawYawRate(double &newyawrate)
{
    /* Logic:
     * a(i-1) = a(i), i = 2...windowsize
     * should fristly start from  i = 2. a(1) = a(2); a(2) = a(3);....; a(N-1) = a(N)
     * secondly a(N) = a_new
    */
    for(int i = 1;i<windowsize_ARdrone;i++)//first step
    {
        yaw_rate_raw[i-1] = yaw_rate_raw[i];
    }
    yaw_rate_raw[windowsize_ARdrone-1] = newyawrate;// second step update the last variable in the velocity buffer
}

void ARDroneCommand::MovingWindowAveraging()
{
    /* Logic: Average the raw velocity measurement in the
    */
    double weight = (double)1/windowsize_ARdrone;// the weight on each velocity to be summed up.
    // create a temporary variable to store the summed velocity and initialize it witht the 1st buffer value
    double velocitytemp;
    velocitytemp = weight*yaw_rate_raw[0];

    for(int i = 1;i<windowsize_ARdrone;i++)// sum starts from the second buffer value
    {
        velocitytemp += weight*yaw_rate_raw[i];
    }
    // the filtered vlocity is just the weighted summed result
    yaw_rate_filtered = velocitytemp;
}

void ARDroneCommand::CalculateYawRateFromYawAngle()
{
    PushYawAngle();
    double yaw_rate_onestep;



    yaw_rate_onestep = Rad2Deg(AngularError(Deg2Rad(yaw_angle[1]), Deg2Rad(yaw_angle[0])))/delta_T;
    PushRawYawRate(yaw_rate_onestep);
    MovingWindowAveraging();
}
double  ARDroneCommand::GetYawRate()
{
    return yaw_rate_filtered;
}
double  ARDroneCommand::GetRawYawRate()
{
    return yaw_rate_raw[windowsize_ARdrone-1];
}

double ARDroneCommand::AngularError(double reference, double state)
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
double  ARDroneCommand::Rad2Deg(double rad)
{
    return 180/M_PI*rad;
}
double  ARDroneCommand::Deg2Rad(double deg)
{
    return deg/180*M_PI;
}
void ARDroneCommand::ReceiveNavdata(const ardrone_autonomy::Navdata& Navmsg)
{
        navdata = Navmsg; // update navdata
}
//ardrone_autonomy::Navdata ARDroneCommand::navdata; //declare the static variable
