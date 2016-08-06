#include "ai_nav.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigator");
	ai_navigator n1;
	n1.init();
}
