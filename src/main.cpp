#include <iostream>
#include "VineyardColorFilter.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vineyard_color_filter");
	ros::NodeHandle nodeHandle("~");

	VineyardColorFilter myColorFilter(nodeHandle);

	myColorFilter.filter(argc, argv);

}
