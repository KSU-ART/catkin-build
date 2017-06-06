#ifndef _IARC_STATES_H_
#define _IARC_STATES_H_
// defines all the decision_node states

#include "decision_node.h"

class find_robot_node: public decision_node
{
public:
    decision_node* run_node()
    {
        // check condition

        // run action
        // return the child node
        return this;
    }
};

class start_node: public decision_node
{
public:
    decision_node* run_node()
    {
        // check condition
        // run action
        // return the child node
        return this;
    }
};



#endif


