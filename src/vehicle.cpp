#include "vehicle.h"
#include <algorithm>
#include <math.h>
#include <iterator>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "cost.h"

using std::string;
using std::vector;
//using namespace std;

Vehicle::Vehicle(){}

Vehicle::Vehicle(int id) {
    this->id = id;
}

Vehicle::~Vehicle() {}

void Vehicle::update_info(float d, double s, double v, double a) {
    this->d = d;
    this->lane = get_lane(d);
    this->s = s;
    this->v = v;
    this->a = a;
    //this->state = state;

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
	// double front_min_dist = 50;
	double adjust_vel, max_adjust_vel, front_speed = 0;
	for (int i=0; i < forward_vehicles.size(); i++){
		// check for vehicle in front of ego vehicle in the same lane
		if (forward_vehicles[i].lane == this->lane && forward_vehicles[i].s > this->s){		
			float delta_v = this->v - (forward_vehicles[i].v * 2.24); // convert to mph
			float dist = forward_vehicles[i].s - this->s;

			// 	front_speed = forward_vehicles[i].v;
			std::cout << dist << std::endl;

			if (dist < 10.0 && delta_v > 2.0){
				adjust_vel = 0.4;
			}
			else if (dist < 15.0 && delta_v > 5.0){
				adjust_vel = 0.2;
			}
			else if (dist < 25.0 && delta_v > 5.0){ // && delta_v > 7){
				adjust_vel = 0.15;
			}
			// else if (dist < 25){
			// 	adjust_vel = 0.2;
			// }
			else if (delta_v < 1.0) {
				adjust_vel = 0.0;  // dont adjust speed
			}
			else {
				adjust_vel = 0.1;
			}

			// adjust velocity by the largest value (adjust_vel increases as other vehicles get closer)
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
    float min_cost = 10000.0;
    vector<float> costs;
    int lane;
    vector<vector<Vehicle>> final_trajectories;
	int next_state = this->lane;;

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
    	else{	// compute lane change cost
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
    	
    	costs.push_back(cost);
		// DEBUG
    	std::cout << cost << "   ";

		// choose state with the lowest cost
    	if (cost < min_cost) {
    		min_cost = cost;
    		next_state = lane;
    	}
    }

	// DEBUG
    std::cout << std::endl;
    
    return next_state;
}

vector<string> Vehicle::successor_states() {
	// find the possible next states for the ego vehicle depending on current lane
	vector<string> states;
	states.push_back("KL");
	if (this->lane == 1) {
		lanes_available = 2;
		states.push_back("LCL");
		states.push_back("LCR");
	}
	else if (this->lane == 0) {
		lanes_available -= 1;
		states.push_back("LCR");
	}
	else if (this->lane == 2) {
		lanes_available -= 1;
		states.push_back("LCL");
	}

	return states;
}

std::tuple< vector<double>, vector<double> > Vehicle::check_lanes(int check_lane, vector<Vehicle> other_vehicles) { //vector<double>
	/* 
	check the specified lane for other vehicles.

	RETURN: distance and speed of other vehicles [front_dist, rear_dist], [front_vel, rear_vel]
	*/
	double front_min_dist = 100.0;
	double rear_min_dist = 100.0;
	double front_v = 100.0;
	double rear_v = 100.0;
	

	for (int i=0; i < other_vehicles.size(); i++) {
		if (other_vehicles[i].lane == check_lane) {
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
			if (fabs(delta_dist) < 15) {
				// clear = this->lane;
				// return clear;
				return {{0.0001, 0.0001}, {0.0001, 0.0001}};
			}
		}
	}
	
	return {{front_min_dist, rear_min_dist}, {front_v, rear_v}};
}