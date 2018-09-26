/*
 * @Author: Chu, Chen-You 
 * @Date: 2018-09-19 16:34:37 
 * @Last Modified by: Chu, Chen-You
 * @Last Modified time: 2018-09-26 18:37:37
 */

#include "FIRA_behavior.h"

StrategyBehavior::StrategyBehavior(){
    initAttr();
};

StrategyBehavior::~StrategyBehavior(){
    delete[] action;
};

void StrategyBehavior::initAttr(){
    env = nullptr;
    action = new int[3];
}
void StrategyBehavior::readRole(int* role){
    int robot_num = env->param.team_strategy.general.robot_num;
    switch(role[robot_num]){
            case Role_Goalkeeper:
                goalkeeper();
                break;
            case Role_Attack:
                attack();
                break;
            case Role_Support:
                support();
                break;
            case Role_Halt:
                halt();
                break;
            case Role_PenaltyKick:
                penaltyKick();
                break;
            case Role_ThrowIn:
                throwIn();
                break;
            case Role_CornerKick:
                cornerKick();
                break;
            case Role_FreeKick:
                freeKick();
                break;
            default:
                printf("Null behavior\n");
                exit(1);
    }
};

void StrategyBehavior::setEnv(Environment* env){
    this->env = env;
};

int* StrategyBehavior::getAction(){
    return action;
}

/*
    behavior function
*/
void StrategyBehavior::goalkeeper(){
    printf("behavior goalkeeper\n");
}

void StrategyBehavior::attack(){
    //
}

void StrategyBehavior::support(){
    //
}

void StrategyBehavior::halt(){
    action[env->param.team_strategy.general.robot_num] = action_Halt;
}

void StrategyBehavior::penaltyKick(){
    //
}

void StrategyBehavior::throwIn(){
    // 
}

void StrategyBehavior::cornerKick(){
    //
}

void StrategyBehavior::freeKick(){
    //
}