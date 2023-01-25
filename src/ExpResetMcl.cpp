//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later
//Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 

#include "emcl/ExpResetMcl.h"
#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include "nav_msgs/Odometry.h"

namespace emcl {

ExpResetMcl::ExpResetMcl(const Pose &p, int num, const Scan &scan,
				const std::shared_ptr<OdomModel> &odom_model,
				const std::shared_ptr<LikelihoodFieldMap> &map,
//                const yolov5_pytorch_ros::BoundingBoxes& bbox,
                const YAML::Node& landmark_config,
				double alpha_th, double open_space_th,
				double expansion_radius_position, double expansion_radius_orientation)
	: alpha_threshold_(alpha_th), open_space_threshold_(open_space_th),
	  expansion_radius_position_(expansion_radius_position),
	  expansion_radius_orientation_(expansion_radius_orientation), Mcl::Mcl(p, num, scan, odom_model, map)
{
}

ExpResetMcl::~ExpResetMcl()
{
}

void ExpResetMcl::sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv, YAML::Node& landmark_config,double cov_matrix[9],double gps_x,double gps_y,double gps_yaw)

{
	if(processed_seq_ == scan_.seq_)
		return;

	Scan scan;
	int seq = -1;
	while(seq != scan_.seq_){//trying to copy the latest scan before next 
		seq = scan_.seq_;
		scan = scan_;
	}

	scan.lidar_pose_x_ = lidar_x;
	scan.lidar_pose_y_ = lidar_y;
	scan.lidar_pose_yaw_ = lidar_t;

	int i = 0;
	if (!inv) {
		for(auto e : scan.ranges_)
			scan.directions_16bit_.push_back(
				Pose::get16bitRepresentation(scan.angle_min_ + (i++)*scan.angle_increment_)
			);
	} else {
		for(auto e : scan.ranges_)
			scan.directions_16bit_.push_back(
				Pose::get16bitRepresentation(scan.angle_max_ - (i++)*scan.angle_increment_)
			);
	}

	double valid_pct = 0.0;
	int valid_beams = scan.countValidBeams(&valid_pct);
	if(valid_beams == 0)
		return;

	for(auto &p : particles_) {
//        p.w_ *= p.likelihood(map_.get(), scan);
          p.w_ += p.gps_weight(cov_matrix, gps_x, gps_y, gps_yaw);
//        auto w_v = p.vision_weight(bbox, landmark_config);
//        if(bbox.bounding_boxes.size() > 0){
//            p.w_ *= w_v;
//        }
    }
	alpha_ = normalizeBelief()/valid_beams;
	//alpha_ = nonPenetrationRate( particles_.size() / 20, map_.get(), scan); //new version
	ROS_INFO("ALPHA: %f / %f", alpha_, alpha_threshold_);
    ROS_INFO("particles size : %zu",particles_.size());
	if(alpha_ < alpha_threshold_ and valid_pct > open_space_threshold_) {
        ROS_INFO("RESET");
//        vision_sensorReset(bbox, landmark_config, particles_);
        expansionReset();
        for (auto &p: particles_){
//            p.w_ *= p.likelihood(map_.get(), scan);
            p.w_ += p.gps_weight(cov_matrix, gps_x, gps_y, gps_yaw);
//            auto w_v = p.vision_weight(bbox, landmark_config);
//            if(bbox.bounding_boxes.size() > 0){
//                p.w_ *= w_v;
//            }
        }
	}
    else{
        if(particles_.size()>500){
            particles_.pop_back();
        }
    }

	if(normalizeBelief() > 0.000001)
		resampling();
	else
		resetWeight();

	processed_seq_ = scan_.seq_;
}

void ExpResetMcl::expansionReset(void)
{
	for(auto &p : particles_){
		double length = 2*((double)rand()/RAND_MAX - 0.5)*expansion_radius_position_;
		double direction = 2*((double)rand()/RAND_MAX - 0.5)*M_PI;

		p.p_.x_ += length*cos(direction);
		p.p_.y_ += length*sin(direction);
		p.p_.t_ += 2*((double)rand()/RAND_MAX - 0.5)*expansion_radius_orientation_;
		p.w_ = 1.0/particles_.size();
	}
}


//void ExpResetMcl::vision_sensorReset(yolov5_pytorch_ros::BoundingBoxes &bbox, YAML::Node &landmark_config,std::vector<Particle> &result) {
//    srand((unsigned)time(NULL));
//    if (bbox.bounding_boxes.size() != 0) {
//        for(auto observed_landmark : bbox.bounding_boxes){
//            for(YAML::const_iterator l_ = landmark_config["landmark"][observed_landmark.Class].begin(); l_!= landmark_config["landmark"][observed_landmark.Class].end(); ++l_){
//                for (int i = 0; i <= 1; i++) {
//                    Pose p_;
//                    p_.x_ = l_->second["pose"][0].as<double>() + (double) rand() / RAND_MAX * 10;
//                    p_.y_ = l_->second["pose"][1].as<double>() + (double) rand() / RAND_MAX * 10;
//                    p_.t_ = 2 * M_PI * rand() / RAND_MAX - M_PI;
//                    Particle P(p_.x_, p_.y_, p_.t_, 0);
//                    result.push_back(P);
//                    result.erase(result.begin());
//                }
//            }
//        }
//    }
//}
}
