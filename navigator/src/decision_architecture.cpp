// Designs the decision tree for IARC
// KSU's main stratage for IARC

#include "states.h"
#include "ai_nav.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigator");
	ai_navigator n1;
	n1.init();
}

