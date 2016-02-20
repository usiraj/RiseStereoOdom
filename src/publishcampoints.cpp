/*
 * publishcampoints.cpp
 *
 *  Created on: Aug 29, 2015
 *      Author: usama
 *  Publishes campoints from stereo camera images topics
 */

#include <RiseStereoOdom/ROSCamPoints.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "publishcampoints");
	ROSCamPoints campub;
	ros::spin();
	return 0;
}


