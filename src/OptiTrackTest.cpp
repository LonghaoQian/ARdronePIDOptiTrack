#include "ros/ros.h"
#include "OptiTrackFeedBackRigidBody.h"
#include "KeyboardEvent.h"

using namespace std;

int main(int argc, char **argv)
{
    double Control_Rate = 40;// Hz the rate

    // Initialize ros node
    ros::init(argc, argv, "OptiTrackTest");
    ros::NodeHandle n;
    // Initialize OptiTrack System
    OptiTrackFeedBackRigidBody Opti_RigidBody1("/vrpn_client_node/RigidBody1/pose",n,3,3);
    OptiTrackFeedBackRigidBody Opti_RigidBody2("/vrpn_client_node/RigidBody2/pose",n,3,3);
    KeyboardEvent keyboardcontrol;
    ros::Rate loop_rate(Control_Rate);
  while (ros::ok())
  {
      Opti_RigidBody1.RosWhileLoopRun();
      Opti_RigidBody2.RosWhileLoopRun();
      keyboardcontrol.RosWhileLoopRun();
      switch (keyboardcontrol.GetPressedKey())
      {
        
        case U_KEY_Q:
        {
           Opti_RigidBody1.GetOptiTrackState();
           Opti_RigidBody2.GetOptiTrackState();
           ///ROS_INFO("Rigidbody 1 Position [%f]",Opti_RigidBody1.GetPose().x);
           //ROS_INFO("Rigidbody 1 Position [%f]",Opti_RigidBody2.GetPose().x);
           break;
        }
        case U_KEY_NONE:
        {
          break;
        }
      }
      ros::spinOnce();// do the loop once
      loop_rate.sleep();

  }
  return 0;
}
