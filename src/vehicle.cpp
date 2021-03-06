#include "vehicle.h"
#include <algorithm>
#include <math.h>
#include <iterator>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "cost.h"
#include "constants.h"

using std::string;
using std::vector;

Vehicle::Vehicle(){}

Vehicle::~Vehicle() {}

void Vehicle::update_info(float d, double s, double v) {
    this->d = d;
    this->lane = get_lane(d);
    this->s = s;
    this->v = v;
}

int Vehicle::get_lane(float d) {
	int lane;
	if (d > 0.0 && d < 4.0) {  		// LEFT lane 
		lane = 0;  
	}
	else if (d > 4.0 && d < 8.0) {  // CENTER lane
		lane = 1;  
	}
	else if (d > 8.0 && d < 12.0) { // RIGHT lane
    	lane = 2;  
    }
    return lane;
}

// Adaptive Cruise Control: adjust the vehicle speed relative to front vehicle
double Vehicle::adjust_ACC(vector<Vehicle> forward_vehicles) {
	double adjust_vel = 0;
	double max_adjust_vel = 0;
	// loop through all sensed vehicles
	for (int i=0; i < forward_vehicles.size(); i++)
	{
		// check sensed vehicles in FRONT of ego vehicle in the SAME lane
		if (forward_vehicles[i].lane == this->lane && forward_vehicles[i].s > this->s)
		{	
			// calculate difference of other vehicles speed/position relative to ego vehicle
			float delta_v = this->v - (forward_vehicles[i].v * 2.24); // convert to mph
			float dist = forward_vehicles[i].s - this->s;

			// DEBUG - distance to front vehicle  
			// std::cout << dist << std::endl;

			// adjust ego speed proportionally to the forward vehicle's
			// relative position and speed
			if (dist < 10.0) {
				if (delta_v > 2.0) {
					adjust_vel = MAX_ACCEL;	// emergency braking
				}
				else{
					adjust_vel = 0.2;
				} 
			}
			else if (dist < 15.0) {
				if (delta_v > 2.0){
					adjust_vel = 0.3;
				}
				else{
					adjust_vel = 0.2;
				}
			}
			else if (dist < 20.0) {
				adjust_vel = 0.2;
			}
			else if (delta_v > 7.0) {
				adjust_vel = MAX_ACCEL;  // brake hard for cut-ins and very slow vehicles
			}
			else if (delta_v < 1.0) {
				adjust_vel = 0.0;  // maintain speed of forward vehicle
			}
			else {
				adjust_vel = 0.15;
			}

			// adjust velocity by the largest value 
			// (`adjust_vel` increases as sensed vehicles get closer)
			if (adjust_vel > max_adjust_vel) {
				max_adjust_vel = adjust_vel;
			}			
		}
	}

	return max_adjust_vel;
}

int Vehicle::choose_next_state(vector<Vehicle> other_vehicles) {
	Cost state_cost;
	vector<string> states = successor_states();
	float cost;
	float min_cost = 10000.0;  // large number
	int lane;
	int next_lane = this->lane;

	// loop through all possible states
	for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
		vector<double> min_distance;
		vector<double> lane_speeds;

		if (*it == "KL") {	// compute keep lane cost
			lane = this->lane;

			// get distance and lane speeds of closest vehicles to the front and rear
			tie(min_distance, lane_speeds) = check_lanes(lane, other_vehicles);
			// compute keep lane cost
			cost = state_cost.costKeepLane(min_distance, lane_speeds); //lane_speeds[0], 50.0);
		}
		else {	// compute lane change cost
			// left lane check
			if (*it == "LCL") {
				lane = this->lane - 1;
			}
			// right lane check
			else if (*it == "LCR") {
				lane = this->lane + 1;
			}

			// get distance and lane speeds of closest vehicles to the front and rear
			tie(min_distance, lane_speeds) = check_lanes(lane, other_vehicles);
			// compute cost of the lane change
			cost = state_cost.costLaneChange(min_distance, lane_speeds); //min_distance[0], min_distance[1]);
		}

		// DEBUG
		std::cout << cost << "   ";

		// choose state with the lowest cost
		if (cost < min_cost) {
			min_cost = cost;
			next_lane = lane;
		}
	}

	// DEBUG
	std::cout << std::endl;
	
	// return the lane with the lowest cost
	return next_lane;
}

vector<string> Vehicle::successor_states() {
	// find the possible next states for the ego vehicle depending on current lane
	vector<string> states;
	states.push_back("KL");
	if (this->lane == 1) {
		states.push_back("LCL");
		states.push_back("LCR");
	}
	else if (this->lane == 0) {
		states.push_back("LCR");
	}
	else if (this->lane == 2) {
		states.push_back("LCL");
	}

	return states;
}

std::tuple< vector<double>, vector<double> > Vehicle::check_lanes(int check_lane, vector<Vehicle> other_vehicles) {
	/* 
	check the specified lane for other vehicles.

	RETURN: distance and speed of other vehicles [front_dist, rear_dist], [front_vel, rear_vel]
	*/

	// initialize variables to store distance and speed values
	double front_min_dist = 100.0;
	double rear_min_dist = 100.0;
	double front_v = 100.0;
	double rear_v = 100.0;

	for (int i=0; i < other_vehicles.size(); i++) 
	{
		// check vehicles in the lane of interest
		if (other_vehicles[i].lane == check_lane) 
		{
			// store distance and speed info of closest vehicles relative to ego vehicle
			double delta_dist = this->s - other_vehicles[i].s;
			double delta_v = this->v - other_vehicles[i].v*2.24;

			if ((delta_dist < front_min_dist) && (other_vehicles[i].s > this->s)) {
				front_min_dist = fabs(delta_dist);
				front_v = other_vehicles[i].v;
			}
			if ((delta_dist < rear_min_dist) && (other_vehicles[i].s < this->s)) {
				rear_min_dist = fabs(delta_dist);
				rear_v = other_vehicles[i].v;
			}
			// check if any vehicles in "safe zone" next to ego vehicle
			if (fabs(delta_dist) < 10) {
				// if vehicles close too ego, return low (distance) values which results in high cost
				// this could be improved
				return {{0.0001, 0.0001}, {0.0001, 0.0001}};
			}
		}
	}
	
	return {{front_min_dist, rear_min_dist}, {front_v, rear_v}};
}