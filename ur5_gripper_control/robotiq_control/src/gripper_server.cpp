#include "ros/ros.h"
// #include "robotiq_control/GripperControl.h"
#include "ropi_msgs/GripperControl.h"
#include "robotiq_control/gripper_ur_control.h"

class GripperServiceClass
{
public:
    ros::NodeHandle n;
    std::shared_ptr<GripperUR> g;
    ros::ServiceServer service;
    GripperServiceClass()
    {
        service = n.advertiseService("gripper_control", &GripperServiceClass::gripper_callback, this);
	    g.reset(new GripperUR());
        g->init();
        ros::spin();
    }

    bool gripper_callback(ropi_msgs::GripperControl::Request &req,
                      ropi_msgs::GripperControl::Response &res)
    {
        g->moveto(req.position);
        res.success = true;
        return true;
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_test"); // init ROS nodes
    GripperServiceClass gsc;
    return 0;
}