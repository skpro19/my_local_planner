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
                
                if(!initialized_){

                        initialized_ = true;
                        ROS_INFO("local_planner has been initialized!\n");
                        
                }
                
                my_costmap_ros = costmap_ros;
                costmap_ros_ = my_costmap_ros->getCostmap();
                
                tf_ = tf;
                global_frame_ = my_costmap_ros->getGlobalFrameID();        

                ROS_WARN("global_frame_: %s\n", global_frame_.c_str());

                size_x_ = costmap_ros_->getSizeInCellsX();
                size_y_ = costmap_ros_->getSizeInCellsY(); 


                ROS_INFO("size_x_: %d\n" , size_x_);
                ROS_INFO("size_y_: %d\n", size_y_);

                local_planner_util_ = new base_local_planner::LocalPlannerUtil();
                local_planner_util_->initialize(tf_, costmap_ros_, global_frame_);
                
                return;
        }       

        bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
        {
                
                ROS_WARN("Inside the setPlan function!\n");

                return false;

                if(!initialized_){
                
                        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                        return false;
                
                }

                global_plan_ = orig_global_plan;

                ROS_WARN("global_plan_.size(): %d\n", (int)global_plan_.size());                

                local_planner_util_->setPlan(orig_global_plan);                
                
                geometry_msgs::PoseStamped global_pose_;
                bool global_pose_flag_ = my_costmap_ros->getRobotPose(global_pose_);

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