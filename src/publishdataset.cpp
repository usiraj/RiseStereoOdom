/*
 * publishdataset.cpp
 *
 *  Created on: Aug 26, 2015
 *      Author: usama
 *  Publishes images from data sets, Left and right image properly timed
 */
#include <riseodom/image_database.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

sensor_msgs::CameraInfo getCamInfo(const ImageDatabase *dbase, bool left);

int main(int argc, char **argv){
	ros::init(argc, argv, "publishdataset");
	ros::NodeHandle n("~");
	double _approxdelay = 0.1;
	double _scale = 1.;
	int _seqno = 1;
	bool _kittidataset = false;
	std::string _pathtodataset("/mnt/work/Research/DataBases/Vision/labdatabase/");
	std::string _camframe("/camframe");
	ImageDatabase *db;
	// get parameters
	n.param<std::string>("path", _pathtodataset, _pathtodataset);
	n.param<std::string>("camframe", _camframe, _camframe);
	n.param<bool>("kitti", _kittidataset, _kittidataset);
	n.param<int>("sequence", _seqno, _seqno);
	n.param<double>("scale", _scale, _scale);
	n.param<double>("approx_delay", _approxdelay, _approxdelay);
	if ( _kittidataset ){
		ROS_INFO("Using Kitti Dataset");
	} else {
		ROS_INFO("Using Custom Dataset");
	}
	ROS_INFO("Dataset base directory is : %s", _pathtodataset.c_str());
	ROS_INFO("Publishing Sequence no. is : %d", _seqno);
	ROS_INFO("Scaling is %lf", _scale);
	ROS_INFO("camframe is : %s", _camframe.c_str());
	ROS_INFO("approximate delay for publishing is set to %lf seconds", _approxdelay);
	// remap topics
	ROS_INFO("Namespace is: %s", n.getNamespace().c_str());
	std::string stereo_name = ros::names::remap("stereo");
	std::string topic_left(stereo_name);
	std::string topic_right(stereo_name);
	std::string topic_camleft(stereo_name);
	std::string topic_camright(stereo_name);
	topic_left.append("/left/image_rect");
	topic_right.append("/right/image_rect");
	topic_camleft.append("/left/camera_info");
	topic_camright.append("/right/camera_info");
	ROS_INFO("Publishing rectified left at : %s", topic_left.c_str());
	ROS_INFO("Publishing rectified right at : %s", topic_right.c_str());
	ROS_INFO("Publishing left camera info at : %s", topic_camleft.c_str());
	ROS_INFO("Publishing right camera info at : %s", topic_camright.c_str());
	// publishers
	image_transport::ImageTransport it(n);
	image_transport::Publisher publ = it.advertise(topic_left.c_str(), 2);
	image_transport::Publisher pubr = it.advertise(topic_right.c_str(), 2);
	ros::Publisher publinfo = n.advertise<sensor_msgs::CameraInfo>(topic_camleft.c_str(), 2);
	ros::Publisher pubrinfo = n.advertise<sensor_msgs::CameraInfo>(topic_camright.c_str(), 2);
	// create database retrieveal object
	if ( _kittidataset ){
		db = new KittiDatabase(_seqno, _pathtodataset.c_str());
	} else {
		db = new ImageDatabase(_seqno, _pathtodataset.c_str());
	}
	// apply scale
	db->set_scale(_scale);
	cv::Mat _imagel, _imager;
	double _time;
	sensor_msgs::CameraInfo caminfoleft, caminforight;
	caminfoleft = getCamInfo(db, true);
	caminforight = getCamInfo(db, false);
	ROS_INFO("Created Image Database Object, Starting Publishing");
	std_msgs::Header hdr;
	hdr.frame_id = _camframe;
	ros::Time sttime = ros::Time::now();
	for ( int _no = 0; ros::ok() ; ++_no){
		if ( !(db->load_grayimage(_no, &_imagel, &_imager, &_time))) break;
		ros::Duration _deltat(_time);
		// header
		hdr.seq = _no;
		hdr.stamp = sttime + _deltat;
		// publish all
		caminfoleft.header = hdr;
		caminforight.header = hdr;
		sensor_msgs::ImagePtr msgl = cv_bridge::CvImage(hdr, "mono8", _imagel).toImageMsg();
		sensor_msgs::ImagePtr msgr = cv_bridge::CvImage(hdr, "mono8", _imager).toImageMsg();
		// publish
		publ.publish(msgl);
		pubr.publish(msgr);
		publinfo.publish(caminfoleft);
		pubrinfo.publish(caminforight);
		ROS_INFO("Published image no . %d", _no);
		ros::Duration(_approxdelay).sleep();
	}
	ROS_INFO("End of Image Sequence, Shutting Down");
	delete db;
	return 0;
}


sensor_msgs::CameraInfo getCamInfo(const ImageDatabase *dbase, bool left){
	sensor_msgs::CameraInfo ret;
	ret.distortion_model = std::string("plumb_bob");
	// set ds to zeros
	std::vector<double> distortion;
	for (int _i = 0; _i < 5; ++_i) distortion.push_back(0.);
	// set r to identity
	boost::array<double, 9ul> rect;
	for ( int _i = 0; _i < 9; ++_i){
		rect[_i] = 0.;
	}
	rect[0] = 1.;	rect[4] = 1.;	rect[8] = 1.;
	// set intrinsic
	cv::Matx33d _k;
	boost::array<double, 9ul> intr;
	if ( left ) _k = dbase->getIntrinsicLeft();
	else _k = dbase->getIntrinsicRight();
	for ( int _r = 0, j = 0; _r < 3; ++_r){
		for ( int _c = 0; _c < 3; ++_c, ++j){
			intr[j] = _k(_r,_c);
		}
	}
	// set extrinsic
	cv::Matx34d _p;
	boost::array<double, 12ul> extr;
	if ( left ) _p = dbase->getExtrinsicLeft();
	else _p = dbase->getExtrinsicRight();
	for ( int _r = 0, j = 0; _r < 3; ++_r){
		for ( int _c = 0; _c < 4; ++_c, ++j){
			extr[j] = _p(_r, _c);
		}
	}
	// image size
	cv::Size imsize = dbase->image_size();
	ret.width = imsize.width;
	ret.height = imsize.height;
	ret.D = distortion;
	ret.K = intr;
	ret.P = extr;
	ret.R = rect;
	return ret;
}


