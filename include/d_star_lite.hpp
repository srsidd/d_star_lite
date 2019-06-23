/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/GetPlanRequest.h>
#include <nav_msgs/GetPlanResponse.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <global_planner/potential_calculator.h>
#include <global_planner/expander.h>
#include <global_planner/grid_path.h>
#include <global_planner/gradient_path.h>
#include <global_planner/traceback.h>
#include <global_planner/orientation_filter.h>

#ifndef D_STAR_PLANNER_CPP
#define D_STAR_PLANNER_CPP

namespace d_star_planner {

class GlobalPlanner : public nav_core::BaseGlobalPlanner {
public:

    GlobalPlanner();
    GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan
                 );

private:
    double step_size_ = 1;
    double min_dist_from_robot_ = 0.01;
    double planner_window_x_ = 0.0;
    double planner_window_y_ = 0.0;
    double default_tolerance_ = 0.0;

    int publish_scale_;

    bool initialized_ = false;
    bool allow_unknown_ = true;

    std::string frame_id_;

    ros::Publisher plan_pub_;
    ros::Publisher potential_pub_;
    ros::ServiceServer make_plan_srv_;

    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;

    base_local_planner::WorldModel* world_model_; 

    global_planner::PotentialCalculator* p_calc_;
    global_planner::Expander* planner_;
    global_planner::Traceback* path_maker_;
    global_planner::OrientationFilter* orientation_filter_;

    double footprintCost(double x_i, double y_i, double theta_i);

    bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);
};
}

#endif
