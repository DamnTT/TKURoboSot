/*
 * @Author: Chu, Chen-You 
 * @Date: 2018-09-19 16:23:44 
 * @Last Modified by: Chu, Chen-You
 * @Last Modified time: 2018-09-20 13:54:44
 */
#ifndef FIRA_BEHAVIOR_H
#define FIRA_BEHAVIOR_H

#include "../common/Env.h"

class StrategyBehavior{
public:
    StrategyBehavior();
    ~StrategyBehavior();
    void readRole(int*);
    void setEnv(Environment*);
    int* getAction();
private:
    Environment* env;
    int* action;
    void initAttr();
    void goalkeeper();
    void attack();
    void support();
    void halt();
    void penaltyKick();
    void throwIn();
    void cornerKick();
    void kickOff();
    void freeKick();
};

#endif