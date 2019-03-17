#ifndef COST_H
#define COST_H

#include <fstream>
#include <cmath>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class Cost{
public:

	Cost();

	double costKeepLane(vector<double> min_distance, vector<double> lane_speed); //double front_speed, double target_speed);

	double costLaneChange(vector<double> min_distance, vector<double> lane_speed); //double front_dist, double rear_dist);

	double JMT(vector<double> start, vector <double> end, double T);

	double target_dist = 70.0;
	double target_speed = 50.0;

	double front_cost = 1.0;

	double rear_cost = 0.5;

	double lane_change_cost = 1.2;

	double speed_cost = 1.5;

};
#endif  // COST_H
