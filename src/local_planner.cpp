#include <pluginlib/class_list_macros.h>
#include <my_local_planner/local_planner.h>



PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

using namespace std;

namespace local_planner{


        LocalPlanner::LocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {

                ROS_INFO("Inside the first constructor!\n");

        }

       
        LocalPlanner::~LocalPlanner(){


        }
        

        void LocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
                
                ROS_INFO("Inside the initialize function!\n");
                
                ros::NodeHandle nh_("my_local_planner");

                transformed_local_plan_pub = nh_.advertise<nav_msgs::Path>("transformed_local_plan", 1000, true); //enabled latching        

                //Publishers and Subscibers
                
                if(!initialized_){

                        initialized_ = true;
                        ROS_INFO("local_planner has been initialized!\n");
                        
                }
                
                costmap_ros_ = costmap_ros;
                costmap_ = costmap_ros_->getCostmap();                
                tf_ = tf;
                
                ROS_WARN("global_frame_: %s\n", costmap_ros_->getGlobalFrameID().c_str());

                //size_x_ = costmap_->getSizeInCellsX();
                //size_y_ = costmap_->getSizeInCellsY(); 


                //ROS_INFO("size_x_: %d\n" , size_x_);
                //ROS_INFO("size_y_: %d\n", size_y_);

                local_planner_util_ = new base_local_planner::LocalPlannerUtil();
                local_planner_util_->initialize(tf_, costmap_, costmap_ros_->getGlobalFrameID());

                return;
        }       

        void LocalPlanner::publish_transformed_local_plan(const vector<geometry_msgs::PoseStamped> &plan, const geometry_msgs::PoseStamped &goal_){

                nav_msgs::Path path_ = {};

                path_.header.frame_id = goal_.header.frame_id;
                path_.header.stamp = ros::Time::now();

                path_.poses = plan;

                transformed_local_plan_pub.publish(path_);

        }

        bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
        {
                
                ROS_WARN("Inside the setPlan function!\n");

                //return false;

                if(!initialized_){
                
                        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                        return false;
                
                }

                local_planner_util_->setPlan(orig_global_plan);
                
                geometry_msgs::PoseStamped global_pose_;
                bool global_pose_flag_ = costmap_ros_->getRobotPose(global_pose_);

                if (!global_pose_flag_) {
                        
                        ROS_ERROR("Unable to get the global_pose_ --- Inside the setPlan function!\n");
                        return false;

                }

                vector<geometry_msgs::PoseStamped> transformed_plan_;
                bool local_plan_flag_ = local_planner_util_->getLocalPlan(global_pose_, transformed_plan_);
                
                if(!local_plan_flag_) {

                        ROS_ERROR("Unable to get the local plan --- Inside the setPlan function!\n");
                        return false;
                }

                geometry_msgs::PoseStamped goal_; 
                local_planner_util_->getGoal(goal_);

                publish_transformed_local_plan(transformed_plan_, goal_);
                
                ROS_WARN("Sleeping for 10 seconds!\n");
                ros::Duration(10.0).sleep();
                
                return true;
        }



        bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
        {
                
                ROS_INFO("Inside the computeVelocityCommands function!\n");
                
                if(!initialized_){

                    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                    return false;
                
                }
     

                return true;

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