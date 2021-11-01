#include <pluginlib/class_list_macros.h>
#include <my_local_planner/local_planner.h>

PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

namespace local_planner{

        LocalPlanner::LocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}


        LocalPlanner::LocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
         : costmap_ros_(NULL), tf_(NULL), initialized_(false)
         {
                initialize(name, tf, costmap_ros);
         }

       

        void LocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
        {

                if(!initialized_)
                {

                        ROS_INFO("Please initialize the local planner.\n");
                        return;
                 }

                return;
        }       

        bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
        {

                if(!initialized_)
                {
                ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                return false;
                }

                return true;
        }

        bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
        {

                if(!initialized_)
                {
                    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                    return false;
                }

                return true;

        }

        bool LocalPlanner::isGoalReached()
        {
                if(!initialized_)
                {
                        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                        return false;
                }

                return true;
        }

        
};