#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
//#include "MPC.h"
#include "Car.h"
#include "UtilFunctions.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

/*
 * Move polynomial functions to "UtilFunctions.h",
 * so that other classes could use them as well
 */

int main() {
  uWS::Hub h;

  Car myCar;

  // MPC is initialized here!
//  MPC mpc;

  h.onMessage([&myCar](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    std::string sdata = std::string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      std::string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          std::vector<double> ptsx = j[1]["ptsx"]; // [m] +east
          std::vector<double> ptsy = j[1]["ptsy"]; // [m] +north
          double px = j[1]["x"]; // [m] +east
          double py = j[1]["y"]; // [m] +north
          double psi = j[1]["psi"]; // [rad] +CCW from east
          double v = j[1]["speed"]; // [mph]
//          double steer_fb = j[1]["steering_angle"]; // [rad]

          // Update the Car states
          Car::Coord loc;
            loc.x = px;
            loc.y = py;
          myCar.set_location(loc);
          myCar.set_psi(psi);
          myCar.set_v(v);

          // convert waypoints (WPT) from map to vehicle body coordinates
          Eigen::VectorXd wptx_body(ptsx.size()); // [m] +fwd
          Eigen::VectorXd wpty_body(ptsy.size()); // [m] +left
          for (size_t i=0; i<ptsx.size(); i++) {
            Car::Coord map_coord;
              map_coord.x = ptsx[i];
              map_coord.y = ptsy[i];
            Car::Coord body_coord = myCar.map2body(map_coord);
            wptx_body(i) = body_coord.x;
            wpty_body(i) = body_coord.y;
          }
          // Find coeffs of 2nd-order polynomial curve fit
          Eigen::VectorXd coeffs = polyfit(wptx_body, wpty_body, 2);
          myCar.setCoeffs(coeffs);

          myCar.calc_nav_errs();
          myCar.update();

          double steer_value = myCar.get_steerCmd();  // [rad], (+) right turn
          double throttle_value = myCar.get_accelCmd(); // (-1,1)
//          double steer_value = 0.0;  // [rad], (+) right turn
//          double throttle_value = 0.0; // (-1,1)


          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          std::vector<double> mpc_x_vals;
          std::vector<double> mpc_y_vals;

//          myCar.test_polyder(mpc_x_vals, mpc_y_vals); // for verification testing only. draws tangent line
          myCar.get_predTraj(mpc_x_vals, mpc_y_vals);

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //For display of the waypoints/reference line
          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          next_x_vals.assign(wptx_body.data(), wptx_body.data()+wptx_body.size());
          next_y_vals.assign(wpty_body.data(), wpty_body.data()+wpty_body.size());

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
