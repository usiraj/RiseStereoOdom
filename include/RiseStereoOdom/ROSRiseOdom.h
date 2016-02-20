/*
 * RosSGSlam.h
 *
 *  Created on: Aug 25, 2015
 *      Author: usama
 */

#ifndef ROSSGSLAM_H_
#define ROSSGSLAM_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <riseodom/stereovodom.h>
#include <RiseStereoOdom/camstereopoints.h>

struct RiseOdom_Settings{
	// settings for sgslam
	int nframes;
	bool spkf_optim;
	bool saveatend;
	// default camera offset if not published
	std::string camframe;
	std::string baseframe;
	std::string worldframe;
	double base2cam_x;
	double base2cam_y;
	double base2cam_z;
	double base2cam_angx;
	double base2cam_angy;
	double base2cam_angz;
	// settings for matcher
	double match_lowe;
	double match_radius;
	int freak_threshold;
	// settings for ransac //
	bool use_ransac;
	double ransac_confidence;
	double ransac_outliers;
	double ransac_reproj;
	// settings for sigma point kalman filter
	double spkf_alpha;
	double spkf_beta;
	double spkf_k;
	double spkf_cov_Q_pos;
	double spkf_cov_Q_ang;
	double spkf_cov_Q_axis;
	double spkf_cov_Q_vellin;
	double spkf_cov_Q_velang;
	double spkf_cov_R_vellin;
	double spkf_cov_R_velang;
	// files to save to
	std::string savefile_pose;
	// topic names
	std::string campoints;
	std::string pubodom;
	/////////// constructor destructors ///////////
	RiseOdom_Settings();	// default constructor
	RiseOdom_Settings(const RiseOdom_Settings &other);			// copy constructor
	RiseOdom_Settings& operator=(const RiseOdom_Settings &rhs);	// assignment operator
	///////////// helpers ///////////
	void LoadParameters( ros::NodeHandle &handle);
};


class RosRiseOdom {
public:
	RosRiseOdom();
	virtual ~RosRiseOdom();
	bool initialize_structure();
	// callbacks
	void data_recv(const RiseStereoOdom::camstereopoints::ConstPtr &pts);
	void publishresults(const int _imgno);
protected:
	RiseOdom_Settings settings;
	ros::NodeHandle n;
	ISPKF_2D *motionmodel;
	RiseOdom::StereoVodom *vodom;
	// subscribers and publishers
	ros::Subscriber camsub;
	ros::Publisher pubodom;
	tf::TransformBroadcaster br;
private:
	double baseline;
	double focus;
	cv::Matx33d cam;
	bool caminit;
	std::vector<TransformationMatrix> _trlist;
};

RiseOdom::Frame_Data CamStereoPointstoFrameData(const RiseStereoOdom::camstereopoints::ConstPtr &pts);

#endif /* ROSSGSLAM_H_ */
