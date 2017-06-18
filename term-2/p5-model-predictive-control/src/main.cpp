#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  
  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
           * Implementation start
           *
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          
          // Transform ptsx, ptsy to car coords
          // TODO: implement in separate function
          for (int i = 0; i < ptsx.size(); i++) {
            double dtx = ptsx[i] - px;
            double dty = ptsy[i] - py;
            
            ptsx[i] = dtx * cos(psi) + dty * sin(psi);
            ptsy[i] = dty * cos(psi) - dtx * sin(psi);
            
          }
          
          
          // Put ptsx and ptsy data into vectors
          Eigen::VectorXd ptsxvec = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
          Eigen::VectorXd ptsyvec = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());
          
          // Fit polynomial to x and y coordinates
          auto coeffs = polyfit(ptsxvec, ptsyvec, 3);

          // shifted coords so car at 0,0 and angle is 0, so set x to 0
          
          // Estimate cross-track error (horizontal works reasonably well unless there's a lot of warpage
          double cte = polyeval(coeffs, 0);
          // Calculate orientation error
          // TODO: check derivation is correct
          double epsi = -atan(coeffs[1]);
          
          Eigen::VectorXd state(6);
          
          /* Convert units */
          std::cout << "cte: " << cte << "m?" << endl;
          
          std::cout << "velocity: " << v << "mph" << endl;
          // the car weaves a lot if I convert from mph to m/s
          // v = v * 1600 / 3600;
          // std::cout << "velocity: " << v << "m/s" << endl;
          
          
          const double Lf = 2.67;
          
          
          // predict the state 100ms into the future before you send it to the solver in order to compensate for the latency.

          // Latency of 100ms, so predict 100ms (0.1s) ahead
          
          double dt = 0.1;
          // Previous steering angle and throttle
          double delta = j[1]["steering_angle"];
          double prev_a = mpc.prev_a;
          
          // Predict (x = y = psi = 0)
          double predicted_x = v * dt;
          double predicted_y = 0;
          double predicted_psi = - v * delta / Lf * dt;
          double predicted_v = v + prev_a * dt;
          double predicted_cte = cte + v * CppAD::sin(epsi) * dt;
          double predicted_epsi = epsi + predicted_psi;
          
          state << predicted_x, predicted_y, predicted_psi, predicted_v, predicted_cte, predicted_epsi;
          
          
          // Solve using MPC
          // coeffs to predict future cte and epsi
          auto result = mpc.Solve(state, coeffs);
          
          double steer_value = result[0]/ (deg2rad(25)*Lf);
          std::cout << "steer_value: " << steer_value << endl;
          double throttle_value = result[1];
          
          // Discarded. TODO: Delete attribute
          // mpc.prev_delta = steer_value;
          mpc.prev_a = throttle_value;
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          // std::cout << "result size: " << result.size() << endl;
          for (int i = 2; i < result.size(); i++) {
            if(i%2 == 0){
              mpc_x_vals.push_back(result[i]);
            } else {
              mpc_y_vals.push_back(result[i]);
            }
            // std::cout << result[i] << endl;
            // std::cout << "i: " << i << endl;
          }
          
          
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          next_x_vals.resize(ptsxvec.size());
          next_y_vals.resize(ptsyvec.size());
          

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          for (int i = 0; i < ptsxvec.size(); i++) {
            next_x_vals[i] = ptsxvec[i];
            next_y_vals[i] = ptsyvec[i];
          }
          

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          
          /*
           *
           * End implementation
           *
           */

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
