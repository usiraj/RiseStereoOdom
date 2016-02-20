/*
 * ROSCamPoints.h
 *
 *  Created on: Aug 29, 2015
 *      Author: usama
 */

#ifndef ROSCAMPOINTS_H_
#define ROSCAMPOINTS_H_

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <riseodom/stereopreprocess.h>

#include <RiseStereoOdom/camstereopoints.h>

class ROSCamPoints{
public:
	ROSCamPoints();
	virtual ~ROSCamPoints();
	// callbacks
	void stereo_info(const sensor_msgs::CameraInfo::ConstPtr &info);	//callback for handling right camera info
	void image_callback(const sensor_msgs::ImageConstPtr &imagel, const sensor_msgs::ImageConstPtr &imager);	// callback for image
protected:
	ros::NodeHandle n;
	ros::Publisher campublish;
	ros::Subscriber infosub;
	message_filters::Subscriber<sensor_msgs::Image> *sublimage;
	message_filters::Subscriber<sensor_msgs::Image> *subrimage;
	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> *msgsync;
	RiseOdom::StereoPreProcess imp;
private:
	int cutoff_disparity;
	double baseline;
	double focus;
	cv::Matx33f cam;
	cv::Size imgsize;
	// settings for key points
	int bucket_window;
	int keypts_max_points;
	double keypts_fast_threshold;
	// settings for matching
	int yerr,xmax;
	double lowe;
	int freak_threshold;
	std::string stereo_name;
	std::string pubname;
	bool caminit;
	// helping members
	void init_all();
	void process_data(const cv::Mat &imgl, const cv::Mat &imgr);
	void publish_data(const std_msgs::Header &hdr);
};

#endif /* ROSCAMPOINTS_H_ */
