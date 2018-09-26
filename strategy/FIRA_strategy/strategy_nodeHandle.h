/*
 * @Author: Chu, Chen-You 
 * @Date: 2018-09-19 13:58:50 
 * @Last Modified by: Chu, Chen-You
 * @Last Modified time: 2018-09-26 18:35:11
 */
#ifndef STRATEGY_NODEHARD_H
#define STRATEGY_NODEHARD_H

#include "ros/ros.h"
#include <ros/package.h>
#include "../common/BaseNode.h"
#include "../common/Env.h"

#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include "vision/Object.h"

#define NODENAME "FIRA_strategy"
#define PACKAGENAME "strategy"
#define VISION_TOPIC "vision/object"
#define SAVEPARAM_TOPIC "FIRA/strategy/saveparam"
#define ROLE_TOPIC "FIRA/strategy/role"
#define TEAMCOLOR_TOPIC "FIRA/strategy/teamcolor"
#define MOTOR_CMD_TOPIC "FIRA/cmd_vel"

class StrategyNodeHandle:public BaseNode{

public:
    StrategyNodeHandle(int argc, char** argv);
    virtual ~StrategyNodeHandle();
    int* getRole();
    Environment* getEnv();
    void pubMotionCmd();
protected:
    void rosCommsInit();

private:
    ros::NodeHandle* node_ptr;
    Environment* env;
    int* role;
    void initAttr();
    void subVision(const vision::Object::ConstPtr&); 
    void subSaveParam(const std_msgs::Int32::ConstPtr&);
    void subRole(const std_msgs::Int32MultiArray::ConstPtr&);
    void subTeamcolor(const std_msgs::String::ConstPtr&);
    ros::Subscriber vision;
    ros::Subscriber save_param;
    ros::Subscriber robot_role;
    ros::Subscriber team_color;
    ros::Publisher motor_cmd;
    void loadParam(ros::NodeHandle*);
    void dumpParam();
};

#endif