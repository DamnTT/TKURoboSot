/*
 * @Author: Chu, Chen-You 
 * @Date: 2018-09-25 15:01:09 
 * @Last Modified by: Chu, Chen-You
 * @Last Modified time: 2018-09-26 18:39:12
 */
#include "strategy_nodeHandle.h"

StrategyNodeHandle::StrategyNodeHandle(int argc, char** argv):
    BaseNode(argc, argv, NODENAME){
    initAttr();
    onInit();
    // initParam();
    loadParam(node_ptr);
}

StrategyNodeHandle::~StrategyNodeHandle(){
    delete[] env;
    delete[] role;
}
void StrategyNodeHandle::initAttr(){
    role = new int[3]();
    env = new Environment;
}
void StrategyNodeHandle::rosCommsInit(){
    node_ptr = new ros::NodeHandle;
    vision = node_ptr->subscribe<vision::Object>(VISION_TOPIC, 1000, &StrategyNodeHandle::subVision, this);
    save_param = node_ptr->subscribe<std_msgs::Int32>(SAVEPARAM_TOPIC, 1000, &StrategyNodeHandle::subSaveParam, this);
    robot_role = node_ptr->subscribe<std_msgs::Int32MultiArray>(ROLE_TOPIC, 1000, &StrategyNodeHandle::subRole, this);
    team_color = node_ptr->subscribe<std_msgs::String>(TEAMCOLOR_TOPIC, 1000, &StrategyNodeHandle::subTeamcolor, this);
    motor_cmd = node_ptr->advertise<geometry_msgs::Twist>(MOTOR_CMD_TOPIC, 1000);
}
void StrategyNodeHandle::subVision(const vision::Object::ConstPtr& msg){
    double ball_distance(msg->ball_dis);
    double yellow_distance(msg->yellow_dis);
    double blue_distance(msg->blue_dis);
    int robot_num = env->param.team_strategy.general.robot_num;

    env->home[robot_num].ball.distance = ball_distance/100;
    env->home[robot_num].ball.angle= msg->ball_ang;

    if(env->topic.team_color == "Blue" || env->topic.team_color == "blue"){
        env->home[robot_num].op_goal.distance = blue_distance/100;
        env->home[robot_num].op_goal.angle = msg->blue_ang;
        env->home[robot_num].goal.distance = yellow_distance/100;
        env->home[robot_num].goal.angle = msg->yellow_ang;
        env->home[robot_num].goal_large_area.angle = msg->yellow_fix_ang;
        env->home[robot_num].goal_large_area.distance = msg->yellow_fix_dis;
        env->home[robot_num].op_goal_large_area.angle = msg->blue_fix_ang;
        env->home[robot_num].op_goal_large_area.distance = msg->blue_fix_dis;
    }else{
        env->home[robot_num].op_goal.distance = yellow_distance/100;
        env->home[robot_num].op_goal.angle = msg->yellow_ang;
        env->home[robot_num].goal.distance = blue_distance/100;
        env->home[robot_num].goal.angle = msg->blue_ang;
        env->home[robot_num].goal_large_area.angle = msg->blue_fix_ang;
        env->home[robot_num].goal_large_area.distance = msg->blue_fix_dis;
        env->home[robot_num].op_goal_large_area.angle = msg->yellow_fix_ang;
        env->home[robot_num].op_goal_large_area.distance = msg->yellow_fix_dis;
    }
}
void StrategyNodeHandle::subSaveParam(const std_msgs::Int32::ConstPtr& msg){
    loadParam(node_ptr);
}
void StrategyNodeHandle::subRole(const std_msgs::Int32MultiArray::ConstPtr &msg){
    for(int i=0;i<3;i++){
        role[i] = msg->data[i];
    }
}
void StrategyNodeHandle::loadParam(ros::NodeHandle *node_ptr){
    dumpParam();
    if(!node_ptr->getParam("/FIRA/strategy/general/robot_num", env->param.team_strategy.general.robot_num))
        env->param.team_strategy.general.robot_num = 0;
    for(int i=0;i<PLAYERS_PER_SIDE;i++){
        std::string param_name = "/FIRA/strategy/pathplan/R" + std::to_string(i+1);
        if(!node_ptr->getParam(param_name, env->param.personal_strategy.path_plan[i].velocity_s_planning))
            env->param.personal_strategy.path_plan[i].velocity_s_planning.push_back(2.2);
            env->param.personal_strategy.path_plan[i].velocity_s_planning.push_back(0.3);
            env->param.personal_strategy.path_plan[i].velocity_s_planning.push_back(50);
            env->param.personal_strategy.path_plan[i].velocity_s_planning.push_back(30);
            env->param.personal_strategy.path_plan[i].velocity_s_planning.push_back(20);
            env->param.personal_strategy.path_plan[i].velocity_s_planning.push_back(3);
            env->param.personal_strategy.path_plan[i].velocity_s_planning.push_back(144);
            env->param.personal_strategy.path_plan[i].velocity_s_planning.push_back(5);
    }
}
void StrategyNodeHandle::dumpParam(){
    printf("0%%, please wait......\n");
    std::string package_path = ros::package::getPath(PACKAGENAME);
    std::string cmd = "rosparam dump " + package_path + "/config/param.yaml" + " /FIRA";
    const char* path_dump = cmd.c_str();
    printf("75%%, please wait......\n");
    printf("dump path: %s\n", (package_path + "/config").c_str());
    system(path_dump);
    printf("100%% done!\n");
}
int* StrategyNodeHandle::getRole(){
    return role;
}
Environment* StrategyNodeHandle::getEnv(){
    return env;
}
void StrategyNodeHandle::pubMotionCmd(){
    geometry_msgs::Twist motion_command;
    int robot_num = env->param.team_strategy.general.robot_num;
    motion_command.linear.x = env->home[robot_num].v_x;
    motion_command.linear.y = env->home[robot_num].v_y;
    motion_command.angular.z = env->home[robot_num].v_yaw;
    motor_cmd.publish(motion_command);
}
void StrategyNodeHandle::subTeamcolor(const std_msgs::String::ConstPtr& msg){
    env->topic.team_color = msg->data;
}