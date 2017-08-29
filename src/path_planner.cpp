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

	double car_s = json_data["s"];

	// Previous path data given to the Planner
	auto previous_path_x = json_data["previous_path_x"];
	double end_path_s = json_data["end_path_s"];
	int prev_size = previous_path_x.size();
	if (prev_size > 0) {
	  car_s = end_path_s;
	}

	// Sensor Fusion Data, a list of all other cars on the same side of the road.
	auto sensor_fusion = json_data["sensor_fusion"];

	// check if ego car is changing lane and accept only KEEP_LANE for next state
	if (lane_change_count > 0) {
		lane_change_count += 1;
		if (lane_change_count < 100) {
			cout << "changing lane" << endl;
			plan = KEEP_LANE;
			TakeAction(plan, json_data);
			return;
		}
		lane_change_count = 0;
	}

	// check the car in front of ego car
	for (int i = 0; i < sensor_fusion.size(); i++) {
		float d = sensor_fusion[i][6];
		if (d < (4+lane*4) && d > (lane*4)) {
			double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += (double)prev_size * .02 * check_speed;

      // if the front car is close to ego car, prepare for lane changing
      if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
      	plan = PREPARE_CHANGE_LANE;
      }
		}
	}

	// check the safety for changing lane
	if (plan == PREPARE_CHANGE_LANE) {
		bool safe_to_change_lane_left = true;
		bool safe_to_change_lane_right = true;
		
		for (int i = 0; i < sensor_fusion.size(); i++) {
			float d = sensor_fusion[i][6];
			double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += (double)prev_size * .02 * check_speed;

      if (d > (lane*4-4) && d < (lane*4) && (check_car_s - car_s) < 30 && (check_car_s - car_s) > -15) {
      	safe_to_change_lane_left = false;
      }
      if (d > (lane*4+4) && d < (lane*4+8) && (check_car_s - car_s) < 30 && (check_car_s - car_s) > -15) {
      	safe_to_change_lane_right = false;
      }
		}
		if (lane < 1 || ref_vel > 35) safe_to_change_lane_left = false;
		if (lane > 1 || ref_vel > 35) safe_to_change_lane_right = false;

		if (safe_to_change_lane_left) plan = CHANGE_LANE_LEFT;
		if (safe_to_change_lane_right) plan = CHANGE_LANE_RIGHT;
	}

	cout << "Action: " << plan << endl;
	TakeAction(plan, json_data);

}

void PathPlanner::TakeAction(Plans plan, json json_data) {
	switch(plan) {
		case KEEP_LANE: KeepLane(json_data); break;
		case PREPARE_CHANGE_LANE: PrepareChangeLane(json_data); break;
		case CHANGE_LANE_LEFT: ChangeLaneLeft(json_data); break;
		case CHANGE_LANE_RIGHT: ChangeLaneRight(json_data); break;
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
  double min_dist = 100;

	// check distance to the forward car
	for (int i = 0; i < sensor_fusion.size(); i++) {
		float d = sensor_fusion[i][6];
		if (d > (lane*4) && d < (4+lane*4)) {
			double check_vx = sensor_fusion[i][3];
			double check_vy = sensor_fusion[i][4];
			double check_speed = sqrt(check_vx*check_vx + check_vy*check_vy);
			double check_car_s = sensor_fusion[i][5];

			check_car_s += (double)prev_size * .02 * check_speed;

			if ((check_car_s > car_s) && (check_car_s - car_s) < 30) {
				too_close = true;
				if (min_dist > (check_car_s - car_s)) min_dist = (check_car_s - car_s);
			}
		}
	}

	if (too_close) {
    ref_vel -= .224 * (2 - min_dist/30.);
  } else if (ref_vel < 49.5) {
    ref_vel += .35;
  }
}


void PathPlanner::KeepLane(json json_data) {
	ControlAcceleration(json_data);
}

void PathPlanner::PrepareChangeLane(json json_data) {
	ControlAcceleration(json_data);
}

void PathPlanner::ChangeLaneLeft(json json_data) {
	if (lane > 0) {
		lane -= 1;
		lane_change_count = 1;
		ControlAcceleration(json_data);
	}
}

void PathPlanner::ChangeLaneRight(json json_data) {
	if (lane < 2) {
		lane += 1;
		lane_change_count = 1;
	}
	ControlAcceleration(json_data);
}
