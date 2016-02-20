/*
 * publishcampoints.cpp
 *
 *  Created on: Aug 29, 2015
 *      Author: usama
 *  Publishes campoints from stereo camera images topics
 */

#include <RiseStereoOdom/ROSRiseOdom.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "RiseStereoOdom");
	RosRiseOdom riseodom;
	ros::spin();
	return 0;
}


