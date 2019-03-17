#include "cost.h"
#include <fstream>
#include <cmath>
#include <vector>
#include <iostream>

#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Cost::Cost() {}

double Cost::costKeepLane(vector<double> min_distance, vector<double> lane_speed) {//double front_speed, double target_speed) {
	double cost = 0.0;

	double front_dist = min_distance[0];
	double front_speed = lane_speed[0]*2.24;

	double delta_v = target_speed - front_speed;

	// add cost for front vehicle distance
	if (front_dist < target_dist && front_dist > 0.1){
		cost += front_cost / front_dist;
	}

	// add cost for front vehicle speed if slower than 5mph under the target speed (speed limit)
	if (delta_v > 5) {
		cost += speed_cost * fabs(delta_v) / target_speed;
	}

	return cost;
}

double Cost::costLaneChange(vector<double> min_distance, vector<double> lane_speed) {
	double cost = 0.0;

	double front_dist = min_distance[0];
	double rear_dist = min_distance[1];
	double front_speed = lane_speed[0]*2.24;  // convert to mph
	double rear_speed = lane_speed[1];

	double delta_v = target_speed - front_speed;

	// add cost of vehicle in front of ego vehicle in adjacent lane
	cost += front_cost / front_dist;

	// add cost of vehicle behind ego vehicle in adjacent lane
	if (rear_dist < target_dist) {
		cost += rear_cost / rear_dist;
	} 

	// add speed cost if the vehicle in adjacent lane is within certain distance 
	// and is at least 5 mph slower than the target speed (speed limit) 
	if (front_dist < target_dist){
		if (delta_v > 5) {
			cost += speed_cost * fabs(delta_v) / target_speed;
		}
	}

	// add cost for a lane change
	cost *= lane_change_cost;	

	return cost;
}

double Cost::JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT - jerk minimizing d-value 
    */
    
  MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T,5*T*T*T*T,
			    6*T, 12*T*T, 20*T*T*T;
		
	VectorXd B = VectorXd(3);	    
	B << end[0]-(start[0]+start[1]*T+0.5*start[2]*T*T),
			    end[1]-(start[1]+start[2]*T),
			    end[2]-start[2];
			    
	MatrixXd Ai = A.inverse();
	

	VectorXd C = VectorXd(3);
	C = Ai*B;

	VectorXd result = VectorXd(6);
	result << start[0], start[1], .5*start[2], C[0], C[1], C[2];

	VectorXd t = VectorXd(6);
	t << 1.0, T, T*T, T*T*T, T*T*T*T, T*T*T*T*T;

	double output = t.transpose() * result;
	cout << output << std::endl;
	return output;
}