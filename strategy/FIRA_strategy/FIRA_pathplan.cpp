/*
 * @Author: Chu, Chen-You 
 * @Date: 2018-09-25 15:01:20 
 * @Last Modified by: Chu, Chen-You
 * @Last Modified time: 2018-09-26 17:01:23
 */

#include "FIRA_pathplan.h"

StrategyPathplan::StrategyPathplan(){
    // do nothing;

}

StrategyPathplan::~StrategyPathplan(){
    // do nothing;
}

void StrategyPathplan::personalStrategy(){
    int robot_num = env->param.team_strategy.general.robot_num;
    // printf("robot_num:%d\n", robot_num);
    // printf("action:%d\n", action[robot_num]);
    switch(action[robot_num]){
        case action_Halt:
            strategyHalt();
            break;
        case action_Goalkeeper:
            strategyGoalkeeper();
            break;
        case action_Attack:
            strategyAttack();
            break;
        default:
            printf("undefine action!\n");
            exit(1);
    }
    velocityPlanning();
}

void StrategyPathplan::initAttr(){
    action = nullptr;
    env = nullptr;
}

void StrategyPathplan::strategyGoalkeeper(){
    //
}

void StrategyPathplan::strategyAttack(){
    //
}

void StrategyPathplan::strategyHalt(){
    int robot_num = env->param.team_strategy.general.robot_num;
    env->home[robot_num].v_x = 0;
    env->home[robot_num].v_y = 0;
    env->home[robot_num].v_yaw = 0;
}

void StrategyPathplan::setEnv(Environment* env){
    this->env = env;
}

void StrategyPathplan::setAction(int* action){
    this->action = action;
}

void StrategyPathplan::velocityPlanning(){
    int robot_num = env->param.team_strategy.general.robot_num;
    double v_x = env->home[robot_num].v_x;
    double v_y = env->home[robot_num].v_y;
    double velocity = hypot(v_x, v_y);
    double alpha = atan2(v_y, v_x) * RAD2DEG;
    double angle = env->home[robot_num].v_yaw;
    double angle_out = angle;

    double dis_max = env->param.personal_strategy.path_plan[robot_num].velocity_s_planning[0];
    double dis_min = env->param.personal_strategy.path_plan[robot_num].velocity_s_planning[1];
    double velocity_max = env->param.personal_strategy.path_plan[robot_num].velocity_s_planning[2];
    double velocity_min = env->param.personal_strategy.path_plan[robot_num].velocity_s_planning[3];
    double angular_velocity_max = env->param.personal_strategy.path_plan[robot_num].velocity_s_planning[4];
    double angular_velocity_min = env->param.personal_strategy.path_plan[robot_num].velocity_s_planning[5];
    double angle_max = env->param.personal_strategy.path_plan[robot_num].velocity_s_planning[6];
    double angle_min = env->param.personal_strategy.path_plan[robot_num].velocity_s_planning[7];

    if(velocity == 0)
        velocity = 0;
    else if(velocity > dis_max)
        velocity = velocity_max;
    else if(velocity < dis_min)
        velocity = velocity_min;
    else
        velocity =  (velocity_max - velocity_min) * (cos((((velocity - dis_min) / (dis_max - dis_min) - 1) * M_PI)) + 1 ) / 2 + velocity_min;

    if(angle == 0)
        angle_out = 0;
    else if(fabs(angle) > angle_max)
        angle_out = angular_velocity_max;
    else if(fabs(angle) < angle_min)
        angle_out = angular_velocity_min;
    else
        angle_out = (angular_velocity_max - angular_velocity_min) * (cos((((angle - angle_min) / (angle_max - angle_min) - 1) * M_PI)) + 1) / 2 + angular_velocity_min;
    if(angle < 0)
        angle_out = -angle_out;

    env->home[robot_num].v_x = velocity * cos(alpha*DEG2RAD);
    env->home[robot_num].v_y = velocity * sin(alpha*DEG2RAD);
    env->home[robot_num].v_yaw = angle_out;
}