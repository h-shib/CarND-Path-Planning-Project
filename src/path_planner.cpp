#include "path_planner.h"
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

PathPlanner::PathPlanner() {}
PathPlanner::~PathPlanner() {}

void PathPlanner::UpdateState(json json_data) {

	double car_x     = json_data["x"];
	double car_y     = json_data["y"];
	double car_s     = json_data["s"];
	double car_d     = json_data["d"];
	double car_yaw   = json_data["yaw"];
	double car_speed = json_data["speed"];

	// Previous path data given to the Planner
	auto previous_path_x = json_data["previous_path_x"];
	auto previous_path_y = json_data["previous_path_y"];
	// Previous path's end s and d values 
	double end_path_s = json_data["end_path_s"];
	double end_path_d = json_data["end_path_d"];

	// Sensor Fusion Data, a list of all other cars on the same side of the road.
	auto sensor_fusion = json_data["sensor_fusion"];

	int prev_size = previous_path_x.size();



	// create trajectory
	// calculate costs
	// choose best action


	// viable actions
	//enum plans {KEEP_LANE, CHANGE_LANE_LEFT};
	Plans p;
	p = KEEP_LANE;


	if (prev_size > 0) {
	  car_s = end_path_s;
	}

	bool too_close = false;


	for (int i = 0; i < sensor_fusion.size(); i++) {
    // car is in my lane
    float d = sensor_fusion[i][6];
    if (d < (4+lane*4) && d > (lane*4)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += (double)prev_size * .02 * check_speed;

      if ((check_car_s > car_s) && ((check_car_s - car_s) < 40)) {
      	if (lane > 0) {
      		p = CHANGE_LANE_LEFT;
      	} else {
      		p = CHANGE_LANE_RIGHT;
      	}
      }
    }
  }
  TakeAction(p, json_data);

}

void PathPlanner::TakeAction(Plans plan, json json_data) {
	switch(plan) {
		case KEEP_LANE: KeepLane(json_data); break;
		case CHANGE_LANE_LEFT: ChangeLaneLeft(json_data); break;
		case CHANGE_LANE_RIGHT: ChangeLaneRight(json_data); break;
	}
}

void PathPlanner::ChangeLane(char direction) {
	if (direction == 'L') {
		lane -= 1;
	} else if (direction == 'R') {
		lane += 1;
	}
}

void PathPlanner::ControlAcceleration(json json_data) {

	double car_s = json_data["s"];
	double end_path_s = json_data["end_path_s"];
	auto previous_path_x = json_data["previous_path_x"];
	auto sensor_fusion = json_data["sensor_fusion"];
	int prev_size = previous_path_x.size();

	if (prev_size > 0) {
    car_s = end_path_s;
  }

  bool too_close = false;

	// check distance to the forward car
	for (int i = 0; i < sensor_fusion.size(); i++) {
		float d = sensor_fusion[i][6];
		if (d > (lane*4) && d < (4+lane*4)) {
			double check_vx = sensor_fusion[i][3];
			double check_vy = sensor_fusion[i][4];
			double check_speed = sqrt(check_vx*check_vx + check_vy*check_vy);
			double check_car_s = sensor_fusion[i][5];

			check_car_s += (double)prev_size * .02 * check_speed;

			if ((check_car_s > car_s) && (check_car_s - car_s) < 40) {
				too_close = true;
			}
		}
	}

	if (too_close) {
    ref_vel -= .224;
  } else if (ref_vel < 49.5) {
    ref_vel += .224;
  }
}


void PathPlanner::KeepLane(json json_data) {
	ControlAcceleration(json_data);
}

void PathPlanner::ChangeLaneLeft(json json_data) {
	if (lane > 0) {
		lane -= 1;
	}
	ControlAcceleration(json_data);
}

void PathPlanner::ChangeLaneRight(json json_data) {
	lane += 1;
	ControlAcceleration(json_data);
}































