#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/GetPlan.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "d_star_node");

    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    ros::Publisher pub = nh.advertise<nav_msgs::Path>("/move_base/TrajectoryPlannerROS/global_plan", 5);

    // Create start and goal poses
    geometry_msgs::PoseStamped start;
    geometry_msgs::PoseStamped goal;

    // Initialize start
    start.header.frame_id = "map";
    start.pose.position.x = 0;
    start.pose.position.y = 0;

    // Initialize goal
    goal.header.frame_id = "map";
    goal.pose.position.x = 7;
    goal.pose.position.y = 4;

    nav_msgs::GetPlan srv_plan;

    // Populate service req
    srv_plan.request.start = start;
    srv_plan.request.goal = goal;
    srv_plan.request.tolerance = 0.0;

    ros::Rate loop_rate(1);

    // Give it a second to boot up
    loop_rate.sleep();

    // Make srv call to make plan
    if (client.call(srv_plan)) {
        ROS_INFO("Plan was made");
        srv_plan.response.plan.header.frame_id = "map";
        pub.publish(srv_plan.response.plan);
    }
    else {
        ROS_ERROR("Failed to make plan");
        return 1;
    }


    return 0;
}
