/*
 * @Author: Chu, Chen-You 
 * @Date: 2018-09-19 13:56:46 
 * @Last Modified by: Chu, Chen-You
 * @Last Modified time: 2018-09-26 18:07:19
 */

#include "ros/ros.h"
#include "strategy_nodeHandle.h"
#include "FIRA_behavior.h"
#include "FIRA_pathplan.h"

int main(int argc, char** argv){

    StrategyNodeHandle node(argc, argv);
    StrategyBehavior behavior;
    StrategyPathplan pathplan;
    ros::Rate loop_rate(20);

    while(ros::ok()){
        behavior.setEnv(node.getEnv());
        behavior.readRole(node.getRole());
        pathplan.setEnv(node.getEnv());
        pathplan.setAction(behavior.getAction());       
        pathplan.personalStrategy();
        node.pubMotionCmd();
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
    
    std::cout << "==========Finish==========" << std::endl;
    return 0;
}