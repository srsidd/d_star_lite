#include <pluginlib/class_list_macros.h>
#include "d_star_lite.hpp"

//Default Constructor
namespace d_star_planner {

GlobalPlanner::GlobalPlanner (){

}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
 initialize(name, costmap_ros);
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
    if (!initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        costmap_ = costmap;
        frame_id_ = frame_id;
        ROS_INFO("Frame id of costmap is %s", frame_id_.c_str());

        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();
        ROS_INFO("Costmap for cx %d and cy is %d", cx, cy);
        p_calc_ = new global_planner::PotentialCalculator(cx, cy);

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

        // private_nh.param("allow_unknown", allow_unknown_, true);
        // private_nh.param("planner_window_x", planner_window_x_, 0.0);
        // private_nh.param("planner_window_y", planner_window_y_, 0.0);
        // private_nh.param("default_tolerance", default_tolerance_, 0.0);
        // private_nh.param("publish_scale", publish_scale_, 100);

        make_plan_srv_ = private_nh.advertiseService("make_plan", &GlobalPlanner::makePlanService, this);

        initialized_ = true;
        ROS_INFO("Initialized d star lite planner");
    }
    else {
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
    }
}

bool GlobalPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    ROS_INFO("Received srv call to make d star lite plan");
    geometry_msgs::PoseStamped start = req.start;
    geometry_msgs::PoseStamped goal = req.goal;
    return true;
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    ROS_INFO("Got req to make plan");
    plan.push_back(start);

    for (int i=0; i<20; i++) {
        geometry_msgs::PoseStamped new_goal = goal;
        tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

        new_goal.pose.position.x = -2.5+(0.05*i);
        new_goal.pose.position.y = -3.5+(0.05*i);

        new_goal.pose.orientation.x = goal_quat.x();
        new_goal.pose.orientation.y = goal_quat.y();
        new_goal.pose.orientation.z = goal_quat.z();
        new_goal.pose.orientation.w = goal_quat.w();

        plan.push_back(new_goal);
    }
    plan.push_back(goal);
    return true;
}
};

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(d_star_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)
