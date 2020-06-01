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


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
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
  while (getline(in_map_, line)) {
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


  // We edited the lamnbda to include our outside vals (lane and ref_val)
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_vel, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
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
          std::cout << "previous path: " << previous_path_x << std::endl;
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          // setting current car current serret unit (how far along the road) to be the last end_paths serret unit
          if(prev_size > 0)
          {
              car_s = end_path_s;
          }

          // setting flags for state machine transitions
          bool is_too_close = false;
          bool prepared_for_lane_change = false;
          bool ready_for_lane_change = false;
          bool is_left_lane_free = true;
          bool is_right_lane_free = true;

          for(size_t i = 0; i < sensor_fusion.size(); ++i)
          {
              // packing information about the vehicles around us in a struct format
              Vehicle vehicle(sensor_fusion[i]);

              if(isInLane(vehicle.d, lane))
              {
                  vehicle.s += (double)prev_size * 0.02 * vehicle.speed;

                  bool is_in_front_of_us = vehicle.s > car_s; // checking the serret units to see if it's above our
                  bool is_closer_than_safety_margin = vehicle.s - car_s < safety_margin; // comparing the vehicle to car, and checking it agains the safety marging we have in place

                  if(is_in_front_of_us && is_closer_than_safety_margin)
                  {
                      is_too_close = true;
                      prepared_for_lane_change = true;
                  }
              }
          }

          if(prepared_for_lane_change)
          {
              int num_vehicles_left = 0;
              int num_vehicles_right = 0;
              // checking if left and right are available
              for(size_t i = 0; i <sensor_fusion.size(); ++i)
              {
                    Vehicle vehicle (sensor_fusion[i]);

                    // checking left and then right lane

                    if(isInLane(vehicle.d, lane - 1))
                    {
                        ++num_vehicles_left; // increasing count of vehicles to our left
                        vehicle.s += (double)prev_size * 0.02 * vehicle.speed; // taking into account our previous vehicles's speed and adding it to our current serret measurements
                        bool too_close_to_change = (vehicle.s > car_s - safety_margin / 2) && (vehicle.s < car_s + safety_margin / 2);
                        if(too_close_to_change)
                        {
                            is_left_lane_free = false;
                        }
                    }
                    else if(isInLane(vehicle.d, lane - 1))
                    {
                        ++num_vehicles_right; // increasing count of vehicles to our right
                        vehicle.s += (double)prev_size * 0.02 * vehicle.speed;
                        bool too_close_to_change = (vehicle.s < car_s - safety_margin / 2) && (vehicle.s < car_s + safety_margin / 2);
                        if(too_close_to_change)
                        {
                            is_right_lane_free = false;
                        }

                    }

                    if(is_left_lane_free || is_right_lane_free)
                    {
                        ready_for_lane_change = true;
                    }
              }



          }

          // performing lane change

          // we check for the following flags and ensuring that our current lane is not the left-most left lane
          if(ready_for_lane_change && is_left_lane_free && lane > 0)
          {
              lane -=1;
          }
          // we check for the following flags and ensuring that our current lane is not the right-most left lane
          else if (ready_for_lane_change && is_right_lane_free && lane < 2)
          {
              lane +=1;
          }

          // adjusting current vehicle speed whether we are too close to other vehicles or accelerating if we are below the maximum speed limit. T
          // this is because we aim to be as efficient and be as fast as wecan as long as we are within the speed limits and from a safe distance from another car
          if(is_too_close)
          {
              ref_vel -= 0.224; // decelration around 5m/s
          }
          else if(ref_vel < max_safe_speed)
          {
              ref_vel +=0.224; //TODO - parametize these values
          }

          // list of waypoints that will be used for spline interpolations
          vector<double> pts_x;
          vector<double> pts_y;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty, use the car as a starting reference
          if(prev_size < 2)
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
            				std::cout << "previous size is more than 2" << std::endl;

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
          vector<double> next_wp0 = getCartesian(car_s + 30, (lane_width * lane + lane_width /  2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getCartesian(car_s + 60, (lane_width * lane + lane_width /  2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getCartesian(car_s + 90, (lane_width * lane + lane_width /  2), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          pts_x.push_back(next_wp0[0]);
          pts_x.push_back(next_wp1[0]);
          pts_x.push_back(next_wp2[0]);

          pts_y.push_back(next_wp0[1]);
          pts_y.push_back(next_wp1[1]);
          pts_y.push_back(next_wp2[1]);

          for(size_t i = 0; i < pts_x.size(); ++i)
          {
              double shift_x = pts_x[i] - ref_x;
              double shift_y = pts_y[i] - ref_y;

              std::cout << "value in loop: " << pts_x[i] << std::endl;
              pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
			  			
          }
          // creating spline
          std::cout << "x size: " << pts_x.size() << std::endl;
          std::cout << "y size: " << pts_y.size() << std::endl;
          std::cout << "yaw: " << ref_yaw << std::endl;
          std::cout << "X values: " << pts_x[0] << " | " <<pts_x[1]<< " | " << pts_x[2]<< " | " << pts_x[3]<< " | " << pts_x[4] << std::endl;
          tk::spline spl;
          spl.set_points(pts_x, pts_y);
          std::cout << "We didn't crash this time"<< std::endl;

          //definited x y points for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // starting with all previous points
          for(size_t i = 0; i < previous_path_x.size(); ++i)
          {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }

          // calculating breaking up spline points to travel at reference velocity

          double target_x = 30.0;
          double target_y = spl(target_y);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0.0;

          for(size_t i = 1; i <= 50 - previous_path_x.size(); ++i)
          {
              double N = target_dist / (0.02 * ref_vel / 2.24);
              double x_point = x_add_on + target_x / N;
              double y_point = spl(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // rotate back into previous coordinate system
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
          }
          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}