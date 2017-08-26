#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_
#include <iostream>
#include "json.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

class PathPlanner {
public:

	// lane number of the road 0-2
	int lane;

	// reference velocity, max 50MPH
	double ref_vel;

	// path selection
	//enum PathPlans = {KEEP_LANE = 1, CHANGE_LANE_LEFT = 2}

	// trajectory



	/**
	* Constructor
	*/
	PathPlanner();

	/**
	* Destructor
	*/
	virtual ~PathPlanner();

	// behavior planner
	void UpdateState(json json_data);

private:
	// Lane change method
	void ChangeLane(char direction);

	// Control acceleration
	void ControlAcceleration(bool too_close);

};

#endif