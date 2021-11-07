#include <my_local_planner/local_planner.h>

using namespace std;

int main(int argc, char **argv){

    ros::init(argc, argv, "my_local_planner");

    local_planner::LocalPlanner *my_local_planner = new local_planner::LocalPlanner()

}