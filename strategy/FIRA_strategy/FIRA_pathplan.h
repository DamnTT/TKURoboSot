/*
 * @Author: Chu, Chen-You 
 * @Date: 2018-09-25 15:00:58 
 * @Last Modified by: Chu, Chen-You
 * @Last Modified time: 2018-09-25 21:04:54
 */

#ifndef FIRA_PATHPLAN_H
#define FIRA_PATHPLAN_H

#include "../common/Env.h"

class StrategyPathplan{
public:
    StrategyPathplan();
    ~StrategyPathplan();
    void personalStrategy();
    void setEnv(Environment*);
    void setAction(int*);
private:
    Environment* env;
    int* action;
    void initAttr();
    void strategyGoalkeeper();
    void strategyAttack();
    void strategyHalt();
    void velocityPlanning();

};

#endif