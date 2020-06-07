#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <chrono>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

void setNextWayPoints(vector<double> &next_x_vals, vector<double> &next_y_vals,
                      const double ref_velocity, const int path_size, tk::spline spl,
                      const double &ref_yaw, const double &ref_x, const double &ref_y);

bool updateDrivingLane(bool &on_lane_switch_cooldown, int &lane, const bool &ready_for_lane_change, const bool &is_left_lane_available, const bool &is_right_lane_available);

void updateVelocity(double &ref_vel, const bool &is_too_close, const double &vehicleInFrontSpeed);

// todo - change sensor_fusion to const ref since we are not changing any date from it just accessing it
void checkVehicleInFront(bool &is_too_close, bool &prepare_for_lane_change, const nlohmann::json &sensor_fusion, const int &lane, const int &prev_size, const double &car_s, double &vehicleInFrontSpeed);

int main()
{

  auto start = std::chrono::steady_clock::now();
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Starting on lane 1
  int lane = 1;

  // reference initial velocity
  double ref_vel = 0.0;

  bool on_lane_switch_cooldown = false;

  int previous_lane = -1;

  const double switch_cooldown_timer = 5.0;

  // We edited the lamnbda to include our outside vals (lane and ref_val)
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &ref_vel, &lane, &on_lane_switch_cooldown, &start, &switch_cooldown_timer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                                                                uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      // ensuring incoming message is not empty
      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          double vehicleInFrontSpeed = 0;

          // setting current car current serret unit (how far along the road) to be the last end_paths serret unit
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }

          // setting flags for state machine transitions
          bool is_too_close = false;
          bool prepare_for_lane_change = false;
          bool ready_for_lane_change = false;
          bool is_left_lane_available = true;
          bool is_right_lane_available = true;
          bool has_switched_lane = false;

          // checking sensor fusion data to check if cars are too close to in front of us

          checkVehicleInFront(is_too_close, prepare_for_lane_change, sensor_fusion, lane, prev_size, car_s, vehicleInFrontSpeed);

          if (prepare_for_lane_change)
          {
            int num_vehicles_left = 0;
            int num_vehicles_right = 0;
            // checking if left and right are available
            for (size_t i = 0; i < sensor_fusion.size(); ++i)
            {
              Vehicle vehicle(sensor_fusion[i]);

              // checking left lane, we do not check if we are lane 0. since this is the lane to the very left already
              if (isInLane(vehicle.d, lane - 1))
              {
                // we only care about vehicles in front of us non the other lanes not before us
                if (vehicle.s > car_s)
                {
                  bool enough_space_in_front = vehicle.s > car_s + 30;
                  // std::cout << "STATUS: Left lane space availability is: " << enough_space_in_front << std::endl;
                  if (!enough_space_in_front)
                    is_left_lane_available = false;
                }

                ++num_vehicles_left;                                   // increasing count of vehicles to our left
                vehicle.s += (double)prev_size * 0.02 * vehicle.speed; // taking into account our previous vehicles's speed and adding it to our current serret measurements
                bool too_close_to_change = (vehicle.s > car_s - safety_margin / 2) && (vehicle.s < car_s + safety_margin / 2);

                if (too_close_to_change)
                {
                  // std::cout << "LEFT LANE IS NOT FREE DONT GET CLOSE!" << std::endl;
                  is_left_lane_available = false;
                }
              }
              //checking lane to the right
              else if (isInLane(vehicle.d, lane + 1))
              {
                ++num_vehicles_right; // increasing count of vehicles to our right
                // std::cout << "STATUS: Sensor: " << vehicle.s << " Our car: " << car_s << std::endl;
                if (vehicle.s > car_s)
                {
                  bool enough_space_in_front = vehicle.s > car_s + 30;
                  // std::cout << "STATUS: Right lane space availability is: " << enough_space_in_front << std::endl;
                  if (!enough_space_in_front)
                    is_right_lane_available = false;
                }

                vehicle.s += (double)prev_size * 0.02 * vehicle.speed;
                bool too_close_to_change = (vehicle.s > car_s - safety_margin / 2) && (vehicle.s < car_s + safety_margin / 2);

                // bool enough_space_in_front = vehicle.s > car_s + 20;
                // std::cout << "STATUS: Right lane space availability is: " << enough_space_in_front << std::endl;

                if (too_close_to_change)
                {
                  // std::cout << "RIGHT LANE IS NOT FREE DONT GET CLOSE!" << std::endl;
                  is_right_lane_available = false;
                }
              }

              if (is_left_lane_available || is_right_lane_available)
              {
                ready_for_lane_change = true;
              }
            }

            if (lane == 0)
              is_left_lane_available = false;

            if (lane == 2)
              is_right_lane_available = false;

            std::cout << "LEFT " << num_vehicles_left << " RIGHT " << num_vehicles_right << std::endl;
            std::cout << "LEFT: " << is_left_lane_available << "\t Right: " << is_right_lane_available << std::endl;
          }

          // performing lane change based on lane availability and being ready to change lanes
          has_switched_lane = updateDrivingLane(on_lane_switch_cooldown, lane, ready_for_lane_change, is_left_lane_available, is_right_lane_available);

          if (has_switched_lane)
          {
            start = std::chrono::steady_clock::now();
            on_lane_switch_cooldown = true;
          }

          // std::cout << "Current Lane Number is: " << lane << std::endl;

          // updating velocity based on restrictions
          updateVelocity(ref_vel, is_too_close, vehicleInFrontSpeed);

          // list of waypoints that will be used for spline interpolations
          vector<double> pts_x;
          vector<double> pts_y;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          // if previous size is almost empty, use the car as a starting reference
          if (prev_size < 2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            pts_x.push_back(prev_car_x);
            pts_x.push_back(car_x);

            pts_y.push_back(prev_car_y);
            pts_y.push_back(car_y);
          }
          else
          {
            // using the previous last 2 previous points as initial points
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            pts_x.push_back(ref_x_prev);
            pts_x.push_back(ref_x);

            pts_y.push_back(ref_y_prev);
            pts_y.push_back(ref_y);
          }

          // creating points at 30, 60 and 90 points further along in frenetert coordinates than our current position based on update lanes
          vector<double> next_wp0 = getCartesian(car_s + 30, (lane_width * lane + lane_width / 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getCartesian(car_s + 60, (lane_width * lane + lane_width / 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getCartesian(car_s + 90, (lane_width * lane + lane_width / 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          pts_x.push_back(next_wp0[0]);
          pts_x.push_back(next_wp1[0]);
          pts_x.push_back(next_wp2[0]);

          pts_y.push_back(next_wp0[1]);
          pts_y.push_back(next_wp1[1]);
          pts_y.push_back(next_wp2[1]);

          for (size_t i = 0; i < pts_x.size(); ++i)
          {
            double shift_x = pts_x[i] - ref_x;
            double shift_y = pts_y[i] - ref_y;

            pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // DEBUGGING START
          // std::cout << "x size: " << pts_x.size() << std::endl;
          // std::cout << "y size: " << pts_y.size() << std::endl;
          // std::cout << "yaw: " << ref_yaw << std::endl;
          // std::cout << "X values: " << pts_x[0] << " | " << pts_x[1] << " | " << pts_x[2] << " | " << pts_x[3] << " | " << pts_x[4] << std::endl;
          // DEBUGGING END

          // creating spline
          tk::spline spl;
          spl.set_points(pts_x, pts_y);

          // defining vectors that will hold the x y points for the planner to execute
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // starting with all previous points
          for (size_t i = 0; i < previous_path_x.size(); ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          setNextWayPoints(next_x_vals, next_y_vals, ref_vel, previous_path_x.size(), spl, ref_yaw, ref_x, ref_y);

          // Packaging waypoints to send to sim
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          auto end = std::chrono::steady_clock::now();
          std::chrono::duration<double> elapsed_seconds = end - start;

          if (elapsed_seconds.count() > switch_cooldown_timer)
          {
            on_lane_switch_cooldown = false;
          }

          // std::cout << "Elapsed time: " << elapsed_seconds.count() << std::endl;

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}

// Updating the vector values with the next waypoints that will be sent over to the simulator
void setNextWayPoints(vector<double> &next_x_vals,
                      vector<double> &next_y_vals,
                      const double ref_velocity,
                      const int path_size,
                      tk::spline spl,
                      const double &ref_yaw,
                      const double &ref_x,
                      const double &ref_y)
{
  double target_x = 30.0; // target value in the spline, this would essentially be the 30 serret units in the future value we calculated earlier
  double target_y = spl(target_y);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double x_add_on = 0.0;

  // This loop for the most part will only run a single time, however in the very beggining it will run 50 times since we don't have previous paths
  for (size_t i = 1; i <= 50 - path_size; ++i)
  {
    // N holds the value for the next measurement in the future. This can be noted by the 0.02 which is the tick rate for the simulator
    // and the 2.237 value used to convert meter per second to miles per hour when multiplied by velocity.
    double N = target_dist / (0.02 * ref_velocity / 2.237);
    double x_point = x_add_on + target_x / N; // This point holds the next 'future' value that we are interested that is in the spline.
    double y_point = spl(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back into previous coordinate system
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    // incrementing our current x y coordinates to the newly calculated points.
    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}

bool updateDrivingLane(bool &on_lane_switch_cooldown, int &lane, const bool &ready_for_lane_change, const bool &is_left_lane_available, const bool &is_right_lane_available)
{
  // we check for the following flags and ensuring that our current lane is not the left-most left lane

  // only change lanes if we are not on cooldown
  if (!on_lane_switch_cooldown)
  {
    if (ready_for_lane_change && is_left_lane_available && lane > 0)
    {
      std::cout << "STATUS: Switching to left lane: " << lane << std::endl;
      lane -= 1;
      std::cout << "New lane is: " << lane << std::endl;
      return true;
    }
    // we check for the following flags and ensuring that our current lane is not the right-most left lane
    else if (ready_for_lane_change && is_right_lane_available && lane < 2)
    {
      std::cout << "STATUS: Switching to right lane: " << lane << std::endl;
      lane += 1;
      std::cout << "New lane is: " << lane << std::endl;
      return true;
    }
    else if (ready_for_lane_change && (!is_right_lane_available || !is_left_lane_available))
    {
      // TODO - Add a new flag for this particular situation. Maybe adjust your speed based on the speed of the car in front of you(?)
      // std::cout << "STATUS: Neither right or left lane available. Maintaining current lane." << std::endl;
      // std::cout << "STATUS: Left: " << is_left_lane_available << "Right: " << is_right_lane_available << std::endl;
      return false;
    }
  }
  return false;
}

// adjusting current velocity by 1.prioritizing safety and then 2. efficiency, by accelerating as long as we are not over the speed limit
void updateVelocity(double &ref_vel, const bool &is_too_close, const double &vehicleInFrontSpeed)
{
  if (is_too_close)
  {
    double weighted_speed = ref_vel * 0.8 + vehicleInFrontSpeed * 0.2;
    double calculated_deceleration = ref_vel - weighted_speed;

    // this value is too high and will cause jerk. // defaulting to regular interval deceleration
    if (calculated_deceleration > deceleration || calculated_deceleration < 0)
    {
      ref_vel -= deceleration;
      std::cout << "Using regular deceleration: " << deceleration << std::endl;
      std::cout << "Speed: " << ref_vel << std::endl;
    }
    else
    {
      ref_vel -= calculated_deceleration;
      std::cout << "Using weighted deceleration: " << calculated_deceleration << std::endl;
      std::cout << "Speed: " << ref_vel << std::endl;
    }
  }
  else if (ref_vel < max_safe_speed)
  {
    ref_vel += acceleration; //acceleration speed
  }
  if (vehicleInFrontSpeed > 0)
  {
    std::cout << "Hey we have a vehicle in front of us, let's adjust our speed to theirs!" << std::endl;
  }
}

void checkVehicleInFront(bool &is_too_close, bool &prepare_for_lane_change, const nlohmann::json &sensor_fusion, const int &lane, const int &prev_size, const double &car_s, double &vehicleInFrontSpeed)
{
  for (size_t i = 0; i < sensor_fusion.size(); ++i)
  {
    // creating instance of vehicle of current sensor fusion vehice
    Vehicle vehicle(sensor_fusion[i]);

    if (isInLane(vehicle.d, lane))
    {
      vehicle.s += (double)prev_size * 0.02 * vehicle.speed;

      bool is_in_front_of_us = vehicle.s > car_s;                            // checking the serret units to see if it's above our
      bool is_closer_than_safety_margin = vehicle.s - car_s < safety_margin; // comparing the vehicle to car, and checking it agains the safety marging we have in place

      if (is_in_front_of_us && is_closer_than_safety_margin)
      {
        std::cout << "STATUS: Vehicle in front of us is going at: " << vehicle.speed * 2.237 << std::endl;
        vehicleInFrontSpeed = vehicle.speed * 2.237; //converting speed from meters per second to miles per hour
        is_too_close = true;
        prepare_for_lane_change = true;
      }
    }
  }
}
