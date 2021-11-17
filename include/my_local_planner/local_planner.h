#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf2_ros/buffer.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/map_grid.h>
#include <base_local_planner/local_planner_util.h>
#include <nav_msgs/Path.h>

namespace local_planner{

  class LocalPlanner : public nav_core::BaseLocalPlanner{

    public:

      //Interface functions

      LocalPlanner();

      LocalPlanner(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      ~LocalPlanner();

      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      bool isGoalReached();


      //Extra Functions

      void publish_transformed_local_plan(const std::vector<geometry_msgs::PoseStamped> &plan, const geometry_msgs::PoseStamped &goal_);

    private:

      bool initialized_;
      costmap_2d::Costmap2D* costmap_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      //tf::TransformListener* tf_;
      tf2_ros::Buffer *tf_;
      
      ros::Publisher local_costmap_publisher;
      __uint32_t size_x_, size_y_;

      base_local_planner::MapGrid *map_grid_;
      base_local_planner::LocalPlannerUtil* local_planner_util_;
      
      ros::Publisher transformed_local_plan_pub;

  };
};

#endif