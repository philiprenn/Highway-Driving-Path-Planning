
#include <string>
#include <vector>
#include <tuple>

using std::string;
using std::vector;

//using namespace std;

class Vehicle 
{
public:
	int id;

	Vehicle();
	Vehicle(int id);

	virtual ~Vehicle();

	void update_info(float d, double s, double v, double a);

	int get_lane(float d);

	double adjust_ACC(vector<Vehicle> forward_vehicles);

	int choose_next_state(vector<Vehicle> other_vehicles);

	vector<string> successor_states();

	std::tuple< vector<double>, vector<double> > check_lanes(int check_lane, vector<Vehicle> other_vehicles);

	int L = 1;

	int preferred_buffer = 6; // impacts "keep lane" behavior.

	int lane, s, goal_lane, goal_s, lanes_available;

	float d, v, target_speed, a;

	string state;
};