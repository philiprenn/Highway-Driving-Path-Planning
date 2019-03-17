
#include <string>
#include <vector>
#include <tuple>

using std::string;
using std::vector;

class Vehicle 
{
public:
	int id;

	Vehicle();

	virtual ~Vehicle();

	void update_info(float d, double s, double v);

	int get_lane(float d);

	double adjust_ACC(vector<Vehicle> forward_vehicles);

	int choose_next_state(vector<Vehicle> other_vehicles);

	vector<string> successor_states();

	std::tuple< vector<double>, vector<double> > check_lanes(int check_lane, vector<Vehicle> other_vehicles);

	int lane;

	float d, s, v, target_speed;

	string state;
};