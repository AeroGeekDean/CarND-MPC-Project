#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Car.h"
#include "UtilFunctions.h" // <--- many global functions moved here!!!
#include "json.hpp"

// for convenience
using json = nlohmann::json;
using namespace std::chrono;

// For keeping track of time to compute dt between data messages
steady_clock::time_point time_past;
duration<double> delta_t;

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

  static bool sim_initialized = false;

  h.onMessage([&myCar](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    std::string sdata = std::string(data).substr(0, length);
//    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      std::string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {

          // This value assumes the model presented in the classroom is used.
          //
          // It was obtained by measuring the radius formed by running the vehicle in the
          // simulator around in a circle with a constant steering angle and velocity on a
          // flat terrain.
          //
          // Lf was tuned until the the radius formed by the simulating the model
          // presented in the classroom matched the previous radius.
          //
          // This is the length from front to CoG that has a similar radius.
          const double Lf = 2.67;

          std::chrono::milliseconds latency_ms(100); // <------ set controller latency here!!

          if (!sim_initialized) {
            time_past = steady_clock::now();
            sim_initialized = true;
          }

          // j[1] is the data JSON object
          std::vector<double> ptsx = j[1]["ptsx"]; // [m] +east
          std::vector<double> ptsy = j[1]["ptsy"]; // [m] +north
          double px = j[1]["x"]; // [m] +east
          double py = j[1]["y"]; // [m] +north
          double psi = j[1]["psi"]; // [rad] +CCW from east
          double v = j[1]["speed"]; // [mph]

          // Grabbing control inputs from prv frame. Turns out these have a
          // TWO frame delay from simulator!! Because asynchronous processes(?).    <---- !!!
          double steer_fb = j[1]["steering_angle"]; // [rad]
          double throttle_fb = j[1]["throttle"]; // [ND] (-1,1)

          // --- Time ---

          // Compute elapsed time since last frame
          steady_clock::time_point time_now = steady_clock::now();
          delta_t = duration_cast<duration<double>>( time_now - time_past ); // dt in [sec]
          double dt = delta_t.count();
          time_past = time_now; // update past value

          // --- Handle vehicle states ---

          v *= mph2mps(); // convert speed from [mph] to [m/s] right away

          // --- Deadreckon ahead, to compensate for latencies ---

          // Propagate the states forward by the actual MEASURED frame time, dt, PLUS additional 5%
          //
          // ASSUMPTION: dt of previous frame is good predictor of current frame dt.
          // Any artificially imposed latency (ie: sleep() ) will automatically be captured by dt measurement.
          std::vector<double> x1 = propagate_state_map_coord({px, py, psi, v, -steer_fb/Lf, throttle_fb},
                                                             dt*1.05); // <--- actual dt + 5% more

          // update states with propagated values
          px = x1[0];
          py = x1[1];
          psi= x1[2];
          v  = x1[3];

          // Update the myCar with states
          myCar.set_x(px);
          myCar.set_y(py);
          myCar.set_psi(psi);
          myCar.set_v(v);

          myCar.set_steerFb(steer_fb);  // <---- these are additional stuff beyond classroom material
          myCar.set_accelFb(throttle_fb);

          // --- Handle waypoints (WPT) ---

          Eigen::VectorXd wptx_body(ptsx.size()); // [m] +fwd
          Eigen::VectorXd wpty_body(ptsy.size()); // [m] +left

          // convert from map to vehicle body coordinates
          for (size_t i=0; i<ptsx.size(); i++) {
            myCar.map2body(ptsx[i], ptsy[i], wptx_body(i), wpty_body(i));
          }

          int order_n = 3;
          // Find coeffs of n-th polynomial curve fit
          Eigen::VectorXd coeffs = polyfit(wptx_body, wpty_body, order_n);
          myCar.setCoeffs(coeffs);

          // --- Run MPC ---

          myCar.calc_nav_errs(); // find cte & psi_err
          myCar.update(); // MPC is contained within myCar

          // -- handle Outputs ---

          double steer_value    = myCar.get_steerCmd()/Lf;  // [rad], (+) right turn
          double throttle_value = myCar.get_accelCmd(); // (-1,1)

          std::cout << std::fixed << std::setprecision(3);
          std::cout << "Hz " << std::setw(6) << 1./dt
                    << "  dt " << std::setw(5) << dt << "  |"
                    << "  steer = " << std::setw(7) << rad2deg(steer_value)
                    << "  steer_fb = " << std::setw(7) << rad2deg(steer_fb)
                    << "  throttle = " << std::setw(6) << throttle_value
                    << "  throttle_fb = " << std::setw(6) << throttle_fb
                    << std::endl;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          // --- Display MPC trajectory ---

          std::vector<double> mpc_x_vals;
          std::vector<double> mpc_y_vals;

          myCar.get_predTraj(mpc_x_vals, mpc_y_vals);

          // TESTING: verifying my different coordinate systems and their
          //          transformation are correct, by drawing a straight tangent
          //          line at some distance ahead of the curve-fitted Ref Trajectory
//          myCar.test_polyder(mpc_x_vals, mpc_y_vals);

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // --- Display Ref trajectory ---

          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;

          // TESTING: show the raw WPTS, without curve fitting
//          next_x_vals.assign(wptx_body.data(), wptx_body.data()+wptx_body.size());
//          next_y_vals.assign(wpty_body.data(), wpty_body.data()+wpty_body.size());

          // generate curve fitted Ref Traj
          double dx = 5; // segment length [m]
          int num_seg = 15;
          for (int i=0; i<num_seg; i++) {
            next_x_vals.push_back(dx*i);
            next_y_vals.push_back(polyeval(coeffs, dx*i));
          }

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
          std::this_thread::sleep_for(std::chrono::milliseconds(latency_ms));
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
    sim_initialized = false;
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
