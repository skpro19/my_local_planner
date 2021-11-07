#include <pluginlib/class_list_macros.h>
#include <my_local_planner/local_planner.h>

PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

namespace local_planner{

        LocalPlanner::LocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}


        LocalPlanner::LocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
         : costmap_ros_(NULL), tf_(NULL), initialized_(false){      
                
                ROS_INFO("Inside the local_planner constructor!\n");
                initialize(name, tf, costmap_ros);
         
        }

        LocalPlanner::~LocalPlanner(){

        }

        void LocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
                
                ROS_INFO("Inside the initialize function!\n");
                if(!initialized_){

                        initialized_ = true;
                        ROS_INFO("local_planner has been initialized!\n");
                        //computeVelocityCommands();
                        //return;
                }

                return;
        }       

        bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
        {
                
                ROS_INFO("Inside the setPlan function!\n");
                if(!initialized_){
                
                        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                        return false;
                
                }

                return true;
        }

        bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
        {
                
                ROS_INFO("Inside the computeVelocityCommands function!\n");
                
                if(!initialized_){

                    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                    return false;
                
                }
                
                /*ros::Time t1 = ros::Time::now();
                double t1_secs = t1.toSec();

                while(true){
                
                        cmd_vel.angular.z = 1;
                        ros::Time t2 = ros::Time::now(); 
                        double t2_secs = t2.toSec();

                        ros::Duration d(5.0);
                        double secs = d.toSec();

                        if(t2_secs - t1_secs > 5) {
                                
                                ROS_INFO("5 seconds have elapsed\n -- breaking!");
                                break;
                        }

                        ROS_INFO("Still inside the while loop\n");

                }*/

                        
                ROS_INFO("Inside the cmd_vel.linear.x=1 loop!\n");
                cmd_vel.linear.x = 1;
                return true;
                
                //return true;

        }

        bool LocalPlanner::isGoalReached(){

                ROS_INFO("Inside the isGoalReached function!\n");
                if(!initialized_){
                        
                        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                        return false;
                
                }

                return false;
        
        }

        
};