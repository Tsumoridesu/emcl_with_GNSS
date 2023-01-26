//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later
//Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 

#include "emcl/Particle.h"
#include "emcl/Mcl.h"

#include <cmath>
#include <vector>
#include <rosconsole/macros_generated.h>

namespace emcl {



Particle::Particle(double x, double y, double t, double w) : p_(x, y, t)
{
	w_ = w;
}

double get_determinant(double *cov_matrix)
{
    double det = cov_matrix[0] * cov_matrix[4] * cov_matrix[8] +
                 cov_matrix[1] * cov_matrix[5] * cov_matrix[6] +
                 cov_matrix[3] * cov_matrix[7] * cov_matrix[2] -
                 cov_matrix[2] * cov_matrix[4] * cov_matrix[6] -
                 cov_matrix[1] * cov_matrix[3] * cov_matrix[8] -
                 cov_matrix[0] * cov_matrix[5] * cov_matrix[7];

    return det;
}
int get_inverse(double *cov_matrix, double *result)
{
    double det = get_determinant(cov_matrix);
    double inv[9] = {   1 * (cov_matrix[4] * cov_matrix[8] - cov_matrix[5] * cov_matrix[7]) / det,
                        -1 * (cov_matrix[3] * cov_matrix[8] - cov_matrix[5] * cov_matrix[6]) / det,
                        1 * (cov_matrix[3] * cov_matrix[7] - cov_matrix[4] * cov_matrix[6]) / det,
                        -1 * (cov_matrix[1] * cov_matrix[8] - cov_matrix[2] * cov_matrix[7]) / det,
                        1 * (cov_matrix[0] * cov_matrix[8] - cov_matrix[2] * cov_matrix[6]) / det,
                        -1 * (cov_matrix[0] * cov_matrix[7] - cov_matrix[1] * cov_matrix[6]) / det,
                        1 * (cov_matrix[1] * cov_matrix[5] - cov_matrix[2] * cov_matrix[4]) / det,
                        -1 * (cov_matrix[0] * cov_matrix[5] - cov_matrix[2] * cov_matrix[3]) / det,
                        1 * (cov_matrix[0] * cov_matrix[4] - cov_matrix[1] * cov_matrix[3]) / det};

    memcpy(result, inv, 9*sizeof(double));

    return 0;
}

int mult_1_3_x_3_3(double *mat_1_3, double *mat_3_3, double *result)
{
    double r[3] = { mat_1_3[0] * mat_3_3[0] + mat_1_3[1] * mat_3_3[3] + mat_1_3[2] * mat_3_3[6],
                    mat_1_3[0] * mat_3_3[1] + mat_1_3[1] * mat_3_3[4] + mat_1_3[2] * mat_3_3[7],
                    mat_1_3[0] * mat_3_3[2] + mat_1_3[1] * mat_3_3[5] + mat_1_3[2] * mat_3_3[8]};

    memcpy(result, r, 3*sizeof(double));

    return 0;
}

double mult_1_3_x_3_1(double *mat_1_3, double *mat_3_1)
{
    double result = mat_1_3[0] * mat_3_1[0] + mat_1_3[1] * mat_3_1[1] + mat_1_3[2] * mat_3_1[2];
    return result;
}


double Particle::gps_weight(double *cov_matrix, double gps_x, double gps_y, double gps_yaw)
{
    double mat[3] = {p_.x_ - gps_x, p_.y_ - gps_y, p_.t_ - gps_yaw};
    double result[3];
    double inverse[9];
    get_inverse(cov_matrix, inverse);
    mult_1_3_x_3_3(mat, inverse, result);
    double temp = mult_1_3_x_3_1(result, mat);
    double gps_weight_ = 1/(pow(2*M_PI, 1.5)* sqrt(get_determinant(cov_matrix)))*exp(-0.5*temp);
    return gps_weight_;
}

double Particle::likelihood(LikelihoodFieldMap *map, Scan &scan)
{
	uint16_t t = p_.get16bitRepresentation();
    double ans = 0.0;
    if(map->isOccupied(p_.x_,p_.y_)){
        ans = 0.0;
    }
    else{
        double lidar_x = p_.x_ + scan.lidar_pose_x_*Mcl::cos_[t]
                    - scan.lidar_pose_y_*Mcl::sin_[t];
        double lidar_y = p_.y_ + scan.lidar_pose_x_*Mcl::sin_[t]
                    + scan.lidar_pose_y_*Mcl::cos_[t];
        uint16_t lidar_yaw = Pose::get16bitRepresentation(scan.lidar_pose_yaw_);


        for(int i=0;i<scan.ranges_.size();i+=scan.scan_increment_){
            if(not scan.valid(scan.ranges_[i]))
                continue;
            uint16_t a = scan.directions_16bit_[i] + t + lidar_yaw;
            double lx = lidar_x + scan.ranges_[i] * Mcl::cos_[a];
            double ly = lidar_y + scan.ranges_[i] * Mcl::sin_[a];

            ans += map->likelihood(lx, ly);
        }
    }
	return ans;
}

bool Particle::wallConflict(LikelihoodFieldMap *map, Scan &scan, double threshold, bool replace)
{
	uint16_t t = p_.get16bitRepresentation();
	double lidar_x = p_.x_ + scan.lidar_pose_x_*Mcl::cos_[t]
				- scan.lidar_pose_y_*Mcl::sin_[t];
	double lidar_y = p_.y_ + scan.lidar_pose_x_*Mcl::sin_[t]
				+ scan.lidar_pose_y_*Mcl::cos_[t];
	uint16_t lidar_yaw = Pose::get16bitRepresentation(scan.lidar_pose_yaw_);

	std::vector<int> order;
	if(rand()%2){
		for(int i=0;i<scan.ranges_.size();i+=scan.scan_increment_)
			order.push_back(i);
	}else{
		for(int i=scan.ranges_.size()-1;i>=0;i-=scan.scan_increment_)
			order.push_back(i);
	}

	int hit_counter = 0;
	for(int i : order){
		if(not scan.valid(scan.ranges_[i]))
			continue;
	
		double range = scan.ranges_[i];
		uint16_t a = scan.directions_16bit_[i] + t + lidar_yaw;
	
		double hit_lx, hit_ly;
		double hit_lx1, hit_ly1, r1;
		uint16_t a1;
		if(isPenetrating(lidar_x, lidar_y, range, a, map, hit_lx, hit_ly)){
			if(hit_counter == 0){
				hit_lx1 = hit_lx;
				hit_ly1 = hit_ly;
				r1 = range;
				a1 = a;
			}

			hit_counter++;
		}else
			hit_counter = 0;

		if(hit_counter*scan.angle_increment_ >= threshold){
			if(replace)
				sensorReset(lidar_x, lidar_y,
						r1, a1, hit_lx1, hit_ly1,
						range, a, hit_lx, hit_ly);
			return true;
		}
	}
	return false;
}

bool Particle::isPenetrating(double ox, double oy, double range, uint16_t direction,
		LikelihoodFieldMap *map, double &hit_lx, double &hit_ly)
{
	bool hit = false;
	for(double d=map->resolution_;d<range;d+=map->resolution_){
		double lx = ox + d * Mcl::cos_[direction];
		double ly = oy + d * Mcl::sin_[direction];

		if((not hit) and map->likelihood(lx, ly) > 0.99){
			hit = true;
			hit_lx = lx;
			hit_ly = ly;
		}
		else if(hit and map->likelihood(lx, ly) == 0.0){ // openspace after hit
			return true; // penetration
		}
	}
	return false;
}

void Particle::sensorReset(double ox, double oy,
		double range1, uint16_t direction1, double hit_lx1, double hit_ly1,
		double range2, uint16_t direction2, double hit_lx2, double hit_ly2)
{
	double p1_x = ox + range1 * Mcl::cos_[direction1];
	double p1_y = oy + range1 * Mcl::sin_[direction1];
	double p2_x = ox + range2 * Mcl::cos_[direction2];
	double p2_y = oy + range2 * Mcl::sin_[direction2];

	double cx = (hit_lx1 + hit_lx2)/2;
	double cy = (hit_ly1 + hit_ly2)/2;

	p_.x_ -= (p1_x + p2_x)/2 - cx;
	p_.y_ -= (p1_y + p2_y)/2 - cy;

	double theta_delta = atan2(p2_y - p1_y, p2_x - p1_x) - atan2(hit_ly2 - hit_ly1, hit_lx2 - hit_lx1);
	/*
	double d = std::sqrt((p_.x_ - cx)*(p_.x_ - cx) + (p_.y_ - cy)*(p_.y_ - cy));

	double theta = atan2(p_.y_ - cy, p_.x_ - cx) - theta_delta;
	p_.x_ = cx + d * std::cos(theta);
	p_.y_ = cy + d * std::cos(theta);
*/
	p_.t_ -= theta_delta;
}

Particle Particle::operator =(const Particle &p)
{
	p_ = p.p_;
	w_ = p.w_;
	return *this;
}

}
