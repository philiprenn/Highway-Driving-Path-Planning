#include "cost.h"
#include <fstream>
#include <cmath>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Cost::Cost() {}

double Cost::costKeepLane(vector<double> min_distance, vector<double> lane_speed) {//double front_speed, double target_speed) {
	double cost = 0.0;

	double front_dist = min_distance[0];
	double rear_dist = min_distance[1];
	double front_speed = lane_speed[0]*2.24;
	double rear_speed = lane_speed[1];

	double target_dist = 70.0;
	double target_speed = 50.0;
	double delta_v = target_speed - front_speed;


	if (front_dist < target_dist && front_dist > 0.1){
		cost += front_cost / front_dist;
	}
	if ((front_speed < target_speed) && (delta_v > 5)) {
		cost += speed_cost * fabs(delta_v) / target_speed; // target_speed / front_speed*2.24;
	}
	

	return cost;
}

double Cost::costLaneChange(vector<double> min_distance, vector<double> lane_speed) {
	double cost = 0.0;

	double front_dist = min_distance[0];
	double rear_dist = min_distance[1];
	double front_speed = lane_speed[0]*2.24;
	double rear_speed = lane_speed[1];

	double target_dist = 70.0;
	double target_speed = 50.0;
	double delta_v = target_speed - front_speed;

	cost += front_cost / front_dist;

	if (rear_dist < target_dist) {
		cost += rear_cost / rear_dist;
	} 

	// add to cost if the vehicle in adjacent lane is within certain distance and is at least 5 mph 
	// slower than the target speed 
	if (front_dist < target_dist){
		if ((front_speed < target_speed) && (delta_v > 5)) {
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

    OUTPUT - an vector of length 6, each value corresponding to a coefficent in the polynomial 
             s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
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
	
	//vector <double> result = {start[0], start[1], .5*start[2]};

	// for(int i = 0; i < C.size(); i++)
	// {
	//     result.push_back(C.data()[i]);
	// }

	VectorXd result = VectorXd(6);
	result << start[0],
				start[1],
				.5*start[2],
				C[0], 
				C[1], 
				C[2];
	VectorXd t = VectorXd(6);
	t << 1.0, T, T*T, T*T*T, T*T*T*T, T*T*T*T*T;

	double out = t.transpose() * result;
	
    return out;
}