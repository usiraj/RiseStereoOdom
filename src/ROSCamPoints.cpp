/*
 * ROSCamPoints.cpp
 *
 *  Created on: Aug 29, 2015
 *      Author: usama
 */

#include <RiseStereoOdom/ROSCamPoints.h>
#include <cv_bridge/cv_bridge.h>

/*
 * ROSCAMPOINTS
 */
ROSCamPoints::ROSCamPoints(){
	// default init
	this->bucket_window = 60;
	this->keypts_max_points = 1000;
	this->keypts_fast_threshold = 8;
	this->baseline = 0.06;
	this->focus = 1.;
	this->cutoff_disparity = 3;
	this->caminit = false;
	this->yerr = 3;	this->xmax = 64; this->lowe = 0.8;	this->freak_threshold = 64;
	// loading parameters from parameter servers
	this->n.param<int>("keypts_bucket_window", this->bucket_window, this->bucket_window);
	this->n.param<int>("keypts_maxpts", this->keypts_max_points, this->keypts_max_points);
	this->n.param<double>("keypts_fast_threshold", this->keypts_fast_threshold, this->keypts_fast_threshold);
	this->n.param<int>("match_yerr", this->yerr, this->yerr);
	this->n.param<int>("match_xmax", this->xmax, this->xmax);
	this->n.param<double>("match_lowe", this->lowe, this->lowe);
	this->n.param<int>("freak_threshold", this->freak_threshold, this->freak_threshold);
	this->n.param<int>("cutoff_disparity", this->cutoff_disparity, this->cutoff_disparity);
	if ( this->cutoff_disparity < 0) this->cutoff_disparity = 0;
	ROS_INFO("match y : %d, x max : %d, lowe ratio : %lf, freak threshold : %d", this->yerr, this->xmax, this->lowe, this->freak_threshold);
	ROS_INFO("keypoints window : %d, maximum : %d, fast threshold : %lf", this->bucket_window,
			this->keypts_max_points, this->keypts_fast_threshold);
	ROS_INFO("cutoff dispariy : %d", this->cutoff_disparity);
	this->imp.setMatching(this->yerr, this->xmax, this->lowe, this->freak_threshold);
	this->stereo_name = ros::names::remap("stereo");
	this->pubname = ros::names::remap("campoints");
	ROS_INFO("stereo is remapped to : %s", this->stereo_name.c_str());
	// Publishers
	this->campublish = this->n.advertise<RiseStereoOdom::camstereopoints>(this->pubname, 5);
	ROS_INFO("Will publish cam points on %s", this->pubname.c_str());
	// subscribers
	std::string _leftname(this->stereo_name);	_leftname.append("/left/image_rect");
	std::string _rightname(this->stereo_name);	_rightname.append("/right/image_rect");
	// message filters
	this->sublimage = new message_filters::Subscriber<sensor_msgs::Image>(this->n, _leftname, 5);
	this->subrimage = new message_filters::Subscriber<sensor_msgs::Image>(this->n, _rightname, 5);
	ROS_INFO("Subscribing to %s", _leftname.c_str());
	ROS_INFO("Subscribing to %s", _rightname.c_str());
	this->msgsync = new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>(*(this->sublimage), *(this->subrimage), 2);
	// subscribing to right camera info
	std::string _rightcaminfo(this->stereo_name);
	_rightcaminfo.append("/right/camera_info");
	this->infosub = this->n.subscribe<sensor_msgs::CameraInfo>(_rightcaminfo,1, &ROSCamPoints::stereo_info, this );
	ROS_INFO("Subscribed to %s for right camera info", _rightcaminfo.c_str());
}
ROSCamPoints::~ROSCamPoints(){
	if ( this->msgsync != NULL){
		delete this->msgsync;
	}
	if ( this->sublimage != NULL){
		delete this->sublimage;
	}
	if (this->subrimage != NULL){
		delete this->subrimage;
	}
}
// callbacks
void ROSCamPoints::stereo_info(const sensor_msgs::CameraInfo::ConstPtr& info){
	// callback for handling stereo camera info
	if ( !(this->caminit) ){
		cv::Matx34d _Pf;
		// initialize camera
		for ( int _r = 0, j=0; _r < 3; ++_r){
			for ( int _c = 0; _c < 4; ++_c, ++j){
				_Pf(_r, _c) = info->P[j];
			}
		}
		if ( _Pf(0,0) <= 1.0 ){
			for ( int _r = 0, j = 0; _r < 3; ++_r ){
				for ( int _c = 0; _c < 3; ++_c, ++j){
					this->cam(_r, _c) = info->K[j];
				}
			}
			this->focus = this->cam(0,0);
			this->baseline = info->P[3];
		} else {
			this->focus = _Pf(0, 0);
			for ( int _r = 0; _r < 3; ++_r ){
				for ( int _c = 0; _c < 3; ++_c){
					this->cam(_r, _c) = _Pf(_r, _c);
				}
			}
			this->baseline = _Pf(0, 3) / this->focus;
		}
		if ( this->baseline < 0) this->baseline = -(this->baseline);
		// image size
		this->imgsize.width = info->width;
		this->imgsize.height = info->height;
		ROS_INFO("Baseline is : %lf", this->baseline);
		ROS_INFO("Focus is : %lf", this->focus);
		ROS_INFO("Cam is : %.4f, %.4f, %.4f\n%.4f, %.4f, %.4f\n %.4f, %.4f, %.4f",
				this->cam(0,0), this->cam(0,1), this->cam(0,2),
				this->cam(1,0), this->cam(1,1), this->cam(1,2),
				this->cam(2,0), this->cam(2,1), this->cam(2,2));
		ROS_INFO("Image size is : %d, %d", this->imgsize.width, this->imgsize.height);
		this->init_all();
		ROS_INFO("Initialized");
		this->caminit = true;
		// start subscriber for camera images
		ROS_INFO("Registering Callback for images");
		this->msgsync->registerCallback(boost::bind(&ROSCamPoints::image_callback, this, _1, _2)) ;
	}
}
void ROSCamPoints::image_callback(const sensor_msgs::ImageConstPtr &imagel, const sensor_msgs::ImageConstPtr &imager){
	cv::Mat _imgl, _imgr;
	std_msgs::Header hdr = imagel->header;
	_imgr = cv_bridge::toCvCopy(imager, "mono8")->image;
	_imgl = cv_bridge::toCvCopy(imagel, "mono8")->image;
	while ( !(this->caminit) ){
		ROS_WARN("Camera not Initialized");
		sched_yield();
	}
	// do the processing
	double _t0 = cv::getTickCount();
	this->process_data(_imgl, _imgr);
	double _difft = 1000. * ( cv::getTickCount() - _t0) / cv::getTickFrequency();
	// publish
	this->publish_data(hdr);
	ROS_INFO("Processed :%d, took %f mill seconds", hdr.seq, _difft);
}
void ROSCamPoints::init_all(){
	this->imp.initialize(this->bucket_window, this->keypts_max_points, this->keypts_fast_threshold, this->imgsize);
	this->imp.setCam(this->cam, this->baseline, this->cutoff_disparity);
	ROS_INFO("Initialized Image PreProcess");
}
void ROSCamPoints::process_data(const cv::Mat &imgl, const cv::Mat &imgr){
	// process the data
	this->imp.ProcessImages(imgl, imgr);
}
void ROSCamPoints::publish_data(const std_msgs::Header &hdr){
	RiseStereoOdom::camstereopoints _msg;
	_msg.header = hdr;
	_msg.baseline = this->baseline;
	_msg.focus = this->focus;
	for ( int _r =0, _j = 0; _r < 3; ++_r){
		for ( int _c = 0; _c < 3; ++_c, ++_j){
			_msg.camintrinsic[_j] = this->cam(_r, _c);
		}
	}
	// copy 2d points
	RiseOdom::Frame_Data latest = this->imp.getFrameData();
	int all_size = latest.points.size();
	////////// resize to proper sizes /////////
	_msg.points.resize(all_size);
	_msg.descriptors.resize(all_size);
	for ( int i = 0; i < all_size; ++i ){
		_msg.points[i].invalid = latest.points[i].invalid;
		_msg.points[i].lX = latest.points[i].left.x ;
		_msg.points[i].lY = latest.points[i].left.y ;
		_msg.points[i].rX = latest.points[i].right.x;
		_msg.points[i].rY = latest.points[i].right.y;
		_msg.points[i].point.x = latest.points[i].leftcamPoint.x;
		_msg.points[i].point.y = latest.points[i].leftcamPoint.y;
		_msg.points[i].point.z = latest.points[i].leftcamPoint.z;
		///////// copy descriptor ///////
		for ( int j = 0; j < 16; ++j){
			_msg.descriptors[i].descriptor[j] = latest.descriptors.at<int>(i,j);
		}
	}
	// publish
	this->campublish.publish(_msg);
}

