/**
 * @file BaseNode.h
 *
 * @brief Ros communication central!
 *
 * @date February 2014
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef BASENODE_HPP_
#define BASENODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/


#include <ros/ros.h>
#include <string>


/*****************************************************************************
** Class
*****************************************************************************/

class BaseNode  {
public:
    BaseNode(int argc, char** argv, const std::string &name );
    virtual ~BaseNode();

    bool onInit();
    bool onInit(const std::string &master_url, const std::string &host_url);
    void shutdown();


protected:
    virtual void rosCommsInit() = 0;
    int init_argc;
    char** init_argv;
    const std::string node_name;
};

#endif /* NODE_HPP_ */
