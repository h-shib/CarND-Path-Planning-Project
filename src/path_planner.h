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

	enum Plans {KEEP_LANE, CHANGE_LANE_LEFT, CHANGE_LANE_RIGHT};

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
	void TakeAction(Plans plan, json json_data);

	// Lane change method
	void ChangeLane(char direction);

	// Control acceleration
	void ControlAcceleration(json json_data);

	void KeepLane(json json_data);

	void ChangeLaneLeft(json json_data);

	void ChangeLaneRight(json json_data);

};

#endif