/*
 * RosRiseOdom.cpp
 *
 *  Created on: Aug 25, 2015
 *      Author: usama
 */

#include <RiseStereoOdom/ROSRiseOdom.h>

#include <nav_msgs/Odometry.h>

RosRiseOdom::RosRiseOdom() : n("~"){
	this->vodom = 0;
	this->motionmodel = 0;
	this->caminit = false;
	this->focus = 0.;
	this->baseline = 0.;
	this->_trlist.clear();
	ROS_INFO("Starting Rise Stereo Vodom, Loading from parameter server");
	this->settings.LoadParameters(this->n);
	ROS_INFO("Loaded parameters from parameter server");
	// remap
	this->settings.campoints = ros::names::remap(this->settings.campoints);
	ROS_INFO("Subscribing to : %s for campoints", this->settings.campoints.c_str());
	this->settings.pubodom = ros::names::remap("odom");
	ROS_INFO("Publishing odometry to : %s", this->settings.pubodom.c_str());
	// publishers
	this->pubodom = this->n.advertise<nav_msgs::Odometry>(this->settings.pubodom, 2);
	////////// Initialize /////////
	this->initialize_structure();
}

RosRiseOdom::~RosRiseOdom() {
	if ( this->settings.saveatend ){
		if ( this->vodom != NULL){
			save_poses(this->settings.savefile_pose.c_str(), this->_trlist);
		}
	}
	if (this->vodom != NULL){
		delete this->vodom;
		this->vodom = 0;
	}
	if ( this->motionmodel != NULL){
		delete this->motionmodel;
		this->motionmodel = 0;
	}
}
// other functions
bool RosRiseOdom::initialize_structure(){
	// try to initialize from settings that are possible, if some error occurs return false
	if (this->vodom != NULL){
		delete this->vodom;
		this->vodom = 0;
	}
	if ( this->motionmodel != NULL){
		delete this->motionmodel;
		this->motionmodel = 0;
	}
	// test nframe is valid
	if ( (this->settings.nframes < 2) || (this->settings.nframes > 20) ){
		ROS_ERROR("nframes should be between 2 and 20, for best memory usage use 3.");
		return false;
	} else {
		this->vodom = new RiseOdom::StereoVodom(this->settings.nframes);
		ROS_INFO("Created StereoVodom object");
	}
	ROS_INFO("SPKF is : %d", this->settings.spkf_optim);
	this->vodom->EnableSPKF(this->settings.spkf_optim);
	// initialize sigma point kalman filter
	if ( (this->settings.spkf_alpha < 0) || (this->settings.spkf_alpha > 1.) ){
		ROS_ERROR("spkf_alpha should be between 0. and 1.");
		return false;
	}
	if ( this->settings.spkf_beta < 0) {
		ROS_ERROR("spkf_beta should be positive");
		return false;
	}
	if ( this->settings.spkf_k < 0 ){
		ROS_ERROR("spkf_k should be be positive");
		return false;
	}
	this->motionmodel = new ISPKF_2D(this->settings.spkf_alpha, this->settings.spkf_beta, this->settings.spkf_k);
	this->motionmodel->set_iteration_criteria(1E-6, 20);
	this->motionmodel->setstatecovs(this->settings.spkf_cov_Q_pos, this->settings.spkf_cov_Q_ang, this->settings.spkf_cov_Q_axis,
			this->settings.spkf_cov_Q_vellin, this->settings.spkf_cov_Q_velang);
	this->motionmodel->setrmatcovs(1E-10, 1E-10, 1E-10, this->settings.spkf_cov_R_vellin, this->settings.spkf_cov_R_velang);
	TransformationMatrix bTcam;
	bTcam.setTranslation(cv::Matx31d(this->settings.base2cam_x,this->settings.base2cam_y,this->settings.base2cam_z));
	bTcam.setRotationFixed(cv::Matx31d(this->settings.base2cam_angx,this->settings.base2cam_angy,this->settings.base2cam_angz));
	this->motionmodel->setBaseCameraTransform(bTcam);
	this->motionmodel->StartFilter();
	this->vodom->register_ispkf(this->motionmodel);
	ROS_INFO("Registered Motion Model");
	this->vodom->setdesiredreprojectionerror(this->settings.ransac_reproj);
	this->vodom->setitereations(this->settings.ransac_confidence, this->settings.ransac_outliers);
	this->vodom->setsearchcriteria(this->settings.match_lowe, this->settings.freak_threshold,
			this->settings.match_radius, this->settings.use_ransac);
	ROS_INFO("Using Ransac : %d", this->settings.use_ransac);
	ROS_INFO("Search Params Set");
	ROS_INFO("Initialize Structure Complete");
	// subscribing to campoints
	this->camsub = this->n.subscribe<RiseStereoOdom::camstereopoints>(this->settings.campoints, 2,
			&RosRiseOdom::data_recv, this);
	ROS_INFO("Subscribed to %s for stereo cam points", this->settings.campoints.c_str());
	return true;
}

void RosRiseOdom::data_recv(const RiseStereoOdom::camstereopoints::ConstPtr &pts){
	if ( !this->caminit){
		this->focus = pts->focus;
		this->baseline = pts->baseline;
		for ( int _r = 0, _j = 0; _r < 3; ++_r){
			for ( int _c = 0; _c < 3; ++_c, ++_j){
				this->cam(_r, _c) = pts->camintrinsic[_j];
			}
		}
		ROS_INFO("Baseline is : %lf, focus is :%lf", this->baseline, this->focus);
		ROS_INFO("CamIntrinsic : %.4f %.4f %.4f", this->cam(0,0), this->cam(0,1), this->cam(0,2));
		ROS_INFO("CamIntrinsic : %.4f %.4f %.4f", this->cam(1,0), this->cam(1,1), this->cam(1,2));
		ROS_INFO("CamIntrinsic : %.4f %.4f %.4f", this->cam(2,0), this->cam(2,1), this->cam(2,2));
		this->vodom->initialize(this->baseline, this->focus, this->cam);
		ROS_INFO("Camera Initialized");
		this->caminit = true;
	}
	double _dtime = pts->header.stamp.toSec();
	ROS_INFO("Processing image no: %d , 2d points : %lu", pts->header.seq, pts->descriptors.size());
	RiseOdom::Frame_Data _data = CamStereoPointstoFrameData(pts);
	// Process Data
	this->vodom->add_imagedata(_data, _dtime);
	this->vodom->estimate_latest_motion(0);
	this->vodom->local_optimization();
	// ready to publish results
	this->publishresults(pts->header.seq);
}

void RosRiseOdom::publishresults(const int _imgno){
	// publish results
	TransformationMatrix last_pose =  this->motionmodel->getBaseCameraTransform() * this->vodom->get_pose() * this->motionmodel->getBaseCameraTransform().getInverse();	// the latest pose
	//////////// save if enabled /////////
	if ( this->settings.saveatend ){
		this->_trlist.push_back(last_pose);
	}
	//////////////////////////////////////
	cv::Point2f _vel = this->vodom->get_vel();
	// publish base_link to world transform
	tf::Transform trans;
	cv::Matx31d _lpxyz = last_pose.getTranslation();
	Quaternion _lpqtr = last_pose.getQuaternion();
	trans.setOrigin(tf::Vector3(_lpxyz(0), _lpxyz(1), _lpxyz(2)));
	trans.setRotation(tf::Quaternion(_lpqtr.getX(), _lpqtr.getY(), _lpqtr.getZ(), _lpqtr.getW()));
	this->br.sendTransform(tf::StampedTransform(trans, ros::Time::now(), this->settings.worldframe, this->settings.baseframe));
	// publish odometry
	nav_msgs::Odometry _msg;
	_msg.header.frame_id = this->settings.worldframe;
	_msg.header.seq = _imgno;
	_msg.header.stamp = ros::Time::now();
	_msg.child_frame_id = this->settings.baseframe;
	_msg.pose.pose.orientation.w = _lpqtr.getW();
	_msg.pose.pose.orientation.x = _lpqtr.getX();
	_msg.pose.pose.orientation.y = _lpqtr.getY();
	_msg.pose.pose.orientation.z = _lpqtr.getZ();
	_msg.pose.pose.position.x = _lpxyz(0);
	_msg.pose.pose.position.y = _lpxyz(1);
	_msg.pose.pose.position.z = _lpxyz(2);
	_msg.twist.twist.linear.x = _vel.x;
	_msg.twist.twist.angular.z = _vel.y;
	this->pubodom.publish(_msg);
}

// SETTINGS

// Settings
void RiseOdom_Settings::LoadParameters(ros::NodeHandle &handle){
	// basic settings
	handle.param<int>("nframes", this->nframes, this->nframes);
	handle.param<bool>("spkf_optim", this->spkf_optim, this->spkf_optim);
	handle.param<bool>("save_enable", this->saveatend, this->saveatend);
	handle.param<bool>("ransac_enable", this->use_ransac, this->use_ransac);
	// default camear offsets if not published
	handle.param<double>("base2cam_x", this->base2cam_x, this->base2cam_x);
	handle.param<double>("base2cam_y", this->base2cam_y, this->base2cam_y);
	handle.param<double>("base2cam_z", this->base2cam_z, this->base2cam_z);
	handle.param<double>("base2cam_angx", this->base2cam_angx, this->base2cam_angx);
	handle.param<double>("base2cam_angy", this->base2cam_angy, this->base2cam_angy);
	handle.param<double>("base2cam_angz", this->base2cam_angz, this->base2cam_angz);
	handle.param<std::string>("camframe", this->camframe, this->camframe);
	handle.param<std::string>("baseframe", this->baseframe, this->baseframe);
	handle.param<std::string>("worldframe", this->worldframe, this->worldframe);
	// settings for matching
	handle.param<double>("match_lowe", this->match_lowe, this->match_lowe);
	handle.param<double>("match_radius", this->match_radius, this->match_radius);
	handle.param<int>("freak_threshold", this->freak_threshold, this->freak_threshold);

	// settings for ransac
	handle.param<double>("ransac_confidence", this->ransac_confidence, this->ransac_confidence);
	handle.param<double>("ransac_outliers", this->ransac_outliers, this->ransac_outliers);
	handle.param<double>("ransac_reprojerror", this->ransac_reproj, this->ransac_reproj);
	// settings for sigma point kalman filter
	handle.param<double>("spkf_alpha", this->spkf_alpha, this->spkf_alpha);
	handle.param<double>("spkf_beta", this->spkf_beta, this->spkf_beta);
	handle.param<double>("spkf_k", this->spkf_k, this->spkf_k);
	handle.param<double>("spkf_cov_Q_pos", this->spkf_cov_Q_pos, this->spkf_cov_Q_pos);
	handle.param<double>("spkf_cov_Q_ang", this->spkf_cov_Q_ang, this->spkf_cov_Q_ang);
	handle.param<double>("spkf_cov_Q_axis", this->spkf_cov_Q_axis, this->spkf_cov_Q_axis);
	handle.param<double>("spkf_cov_Q_vellin", this->spkf_cov_Q_vellin, this->spkf_cov_Q_vellin);
	handle.param<double>("spkf_cov_Q_velang", this->spkf_cov_Q_velang, this->spkf_cov_Q_velang);
	handle.param<double>("spkf_cov_R_vellin", this->spkf_cov_R_vellin, this->spkf_cov_R_vellin);
	handle.param<double>("spkf_cov_R_velang", this->spkf_cov_R_velang, this->spkf_cov_R_velang);
	// extra files //
	handle.param<std::string>("save_pose", this->savefile_pose, this->savefile_pose);
}

RiseOdom_Settings::RiseOdom_Settings(){
	this->nframes = 3;
	this->spkf_optim = true;
	this->saveatend = false;
	// default camera offset if not published
	this->camframe = "camframe";
	this->baseframe = "base_link";
	this->worldframe = "/world";
	this->base2cam_x = 0.;
	this->base2cam_y = 0.;
	this->base2cam_z = 0.;
	this->base2cam_angx = -M_PI_2;
	this->base2cam_angy = 0.;
	this->base2cam_angz = -M_PI_2;
	// settings for matcher
	this->match_lowe = 0.9;
	this->match_radius = 90;
	this->freak_threshold = 64;
	// settings for ransac //
	this->use_ransac = true;
	this->ransac_confidence = 0.995;
	this->ransac_outliers = 0.75;
	this->ransac_reproj = 4.;
	// settings for sigma point kalman filter
	this->spkf_alpha = 1E-4;
	this->spkf_beta = 2;
	this->spkf_k = 0;
	this->spkf_cov_Q_pos = 1E-7;
	this->spkf_cov_Q_ang = 1E-7;
	this->spkf_cov_Q_axis = 1E-7;
	this->spkf_cov_Q_vellin = 1E-7;
	this->spkf_cov_Q_velang = 1E-7;
	this->spkf_cov_R_vellin = 1E-10;
	this->spkf_cov_R_velang = 1E-10;
	// files to save to
	this->savefile_pose = std::string("riseodompose.txt");
	// topic names
	this->campoints = std::string("campoints");
	this->pubodom = std::string("odom");
}
RiseOdom_Settings& RiseOdom_Settings::operator =(const RiseOdom_Settings &rhs){
	this->nframes = rhs.nframes;
	this->spkf_optim = rhs.spkf_optim;
	this->saveatend = rhs.saveatend;
	// default camera offset if not published
	this->camframe = rhs.camframe;
	this->baseframe = rhs.baseframe;
	this->worldframe = rhs.worldframe;
	this->base2cam_x = rhs.base2cam_x;
	this->base2cam_y = rhs.base2cam_y;
	this->base2cam_z = rhs.base2cam_z;
	this->base2cam_angx = rhs.base2cam_angx;
	this->base2cam_angy = rhs.base2cam_angy;
	this->base2cam_angz = rhs.base2cam_angz;
	// settings for matcher
	this->match_lowe = rhs.match_lowe;
	this->match_radius = rhs.match_radius;
	this->freak_threshold = rhs.freak_threshold;
	// settings for ransac //
	this->use_ransac = rhs.use_ransac;
	this->ransac_confidence = rhs.ransac_confidence;
	this->ransac_outliers = rhs.ransac_confidence;
	this->ransac_reproj = rhs.ransac_reproj;
	// settings for sigma point kalman filter
	this->spkf_alpha = rhs.spkf_alpha;
	this->spkf_beta = rhs.spkf_beta;
	this->spkf_k = rhs.spkf_k;
	this->spkf_cov_Q_pos = rhs.spkf_cov_Q_pos;
	this->spkf_cov_Q_ang = rhs.spkf_cov_Q_ang;
	this->spkf_cov_Q_axis = rhs.spkf_cov_Q_axis;
	this->spkf_cov_Q_vellin = rhs.spkf_cov_Q_vellin;
	this->spkf_cov_Q_velang = rhs.spkf_cov_Q_velang;
	this->spkf_cov_R_vellin = rhs.spkf_cov_R_vellin;
	this->spkf_cov_R_velang = rhs.spkf_cov_R_velang;
	// files to save to
	this->savefile_pose = rhs.savefile_pose;
	// topic names
	this->campoints = rhs.campoints;
	this->pubodom = rhs.pubodom;

	return (*this);
}
RiseOdom_Settings::RiseOdom_Settings(const RiseOdom_Settings &rhs){
	this->nframes = rhs.nframes;
	this->spkf_optim = rhs.spkf_optim;
	this->saveatend = rhs.saveatend;
	// default camera offset if not published
	this->camframe = rhs.camframe;
	this->baseframe = rhs.baseframe;
	this->worldframe = rhs.worldframe;
	this->base2cam_x = rhs.base2cam_x;
	this->base2cam_y = rhs.base2cam_y;
	this->base2cam_z = rhs.base2cam_z;
	this->base2cam_angx = rhs.base2cam_angx;
	this->base2cam_angy = rhs.base2cam_angy;
	this->base2cam_angz = rhs.base2cam_angz;
	// settings for matcher
	this->match_lowe = rhs.match_lowe;
	this->match_radius = rhs.match_radius;
	this->freak_threshold = rhs.freak_threshold;
	// settings for ransac //
	this->use_ransac = rhs.use_ransac;
	this->ransac_confidence = rhs.ransac_confidence;
	this->ransac_outliers = rhs.ransac_confidence;
	this->ransac_reproj = rhs.ransac_reproj;
	// settings for sigma point kalman filter
	this->spkf_alpha = rhs.spkf_alpha;
	this->spkf_beta = rhs.spkf_beta;
	this->spkf_k = rhs.spkf_k;
	this->spkf_cov_Q_pos = rhs.spkf_cov_Q_pos;
	this->spkf_cov_Q_ang = rhs.spkf_cov_Q_ang;
	this->spkf_cov_Q_axis = rhs.spkf_cov_Q_axis;
	this->spkf_cov_Q_vellin = rhs.spkf_cov_Q_vellin;
	this->spkf_cov_Q_velang = rhs.spkf_cov_Q_velang;
	this->spkf_cov_R_vellin = rhs.spkf_cov_R_vellin;
	this->spkf_cov_R_velang = rhs.spkf_cov_R_velang;
	// files to save to
	this->savefile_pose = rhs.savefile_pose;
	// topic names
	this->campoints = rhs.campoints;
	this->pubodom = rhs.pubodom;
}

///////////// Conversions ///////////
RiseOdom::Frame_Data CamStereoPointstoFrameData(const RiseStereoOdom::camstereopoints::ConstPtr &pts){
	RiseOdom::Frame_Data res;
	int total_elems = pts->descriptors.size();
	res.descriptors = cv::Mat(total_elems, 64, CV_8UC1);
	res.points.resize(total_elems);
	for ( int _r = 0; _r < total_elems; ++_r){
		res.frame_id = pts->header.seq;
		res.points[_r].invalid = pts->points[_r].invalid;
		res.points[_r].left.x = pts->points[_r].lX;
		res.points[_r].left.y = pts->points[_r].lY;
		res.points[_r].right.x = pts->points[_r].rX;
		res.points[_r].right.y = pts->points[_r].rY;
		res.points[_r].leftcamPoint.x = pts->points[_r].point.x;
		res.points[_r].leftcamPoint.y = pts->points[_r].point.y;
		res.points[_r].leftcamPoint.z = pts->points[_r].point.z;
		for ( int _c = 0; _c < 16; ++_c){
			res.descriptors.at<int>(_r,_c) = pts->descriptors[_r].descriptor[_c] ;
		}
	}
	return res;
}
