#ifndef _DECISION_NODE_CLASS_H_
#define _DECISION_NODE_CLASS_H_
// Data structure for holding the states of the craft
// it handles the other possible states each state can reach
// and what actions are available in each state
// For a list of actions and conditionals, see actions.cpp

#include <vector>

class decision_node
{
public:
    virtual decision_node* run_node()
    {
        return this;
    }
};

#endif
