#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include <utility>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// Numer of points to send to the simulator
#define POINTS_TO_SEND 50
// 20 ms in s distance
#define DT 0.02
// Points to populate for prediction
#define POINTS_TO_KEEP 50
// Path planning distance in meter
#define STEP_SIZE 45.0

double feasibility = pow(10,7); // Feasibility = 10**7
double collision = pow(10,6); //COLLISION  = 10 ** 6
double danger = pow(10,5); //DANGER     = 10 ** 5
double reachGoal = pow(10,5); //REACH_GOAL = 10 ** 5
double comfort = pow(10,4); //COMFORT    = 10 ** 4
double efficiency = pow(10,2); //EFFICIENCY = 10 ** 2
double target_speed = 49.6;//mph
double sampleTime = 0.02;


double feasibilityCost(int lane){
	double cost = 0;
	if (lane < 0 || lane > 2){
		return feasibility;
	}
	return cost;
}

double changeLaneLeftCost(){
	double cost = 0;
	return cost;
}

double changeLaneRightCost(){
	double cost = 0;
	return cost;
}
double keepLaneCost(){
	double cost = 0;
	cost -= efficiency;
	return cost;
}
double speedLimitCost(double v){
	double cost = 0;
	if (v > target_speed){
		return danger;
	}
	else{
		double deltaSpeed = target_speed - v;
		double pct = deltaSpeed / target_speed;
		double weight = pow(pct,2);
		cost =  weight * reachGoal;
	}
	return cost;
}

double collisionCost(vector<vector<double>> sensorVec, int lane, int prevSize, double car_s){
	double cost=0;
	float d;
	for (int i=0;i<sensorVec.size();++i){
		d = sensorVec[i][6];
		//cout << "d: "<< d << endl;
		if(d < (2+4*lane+2) && d >(2+4*lane-2)){
			double vx = sensorVec[i][3];
			double vy = sensorVec[i][4];
			double other_car_speed = sqrt(vx*vx+vy*vy);
			double other_car_s = sensorVec[i][5];
			//cout << "other_car_s: " << other_car_s<< endl;
			//cout << "car_s: " << car_s<< endl;
			other_car_s += static_cast<double>(prevSize)*sampleTime*other_car_speed;
			if(other_car_s > car_s && (other_car_s-car_s) < 35){
				cost+=10*collision;
			}
		}
	}
	return cost;
}
double bufferCost(vector<vector<double>> sensorVec, int lane, int prevSize, double car_s){
	double cost = 0;
	float d;
	for (int i=0;i<sensorVec.size();++i){
		d = sensorVec[i][6];
		//cout << "d: "<< d << endl;
		if(d < (2+4*lane+2) && d >(2+4*lane-2)){
			double vx = sensorVec[i][3];
			double vy = sensorVec[i][4];
			double other_car_speed = sqrt(vx*vx+vy*vy);
			double other_car_s = sensorVec[i][5];
			//cout << "other_car_s: " << other_car_s<< endl;
			//cout << "car_s: " << car_s<< endl;
			other_car_s += static_cast<double>(prevSize)*sampleTime*other_car_speed;
			if(other_car_s > car_s && (other_car_s-car_s) < 55){
				cost+=10*danger;
			}
		}
	}
	return cost;
}

double calculateCost(vector<vector<double>> sensorVec, int lane, int prevSize, double car_s){
	double cost = 0;
	map<int,double> costMap;
	costMap[0] = 0;
	costMap[1] = 0;
	costMap[2] = 0;
	map<int,double>::iterator it = costMap.begin();
    while(it != costMap.end()){
    	//cout << "costMap: "<< it->first << endl;
    	//Left lane
    	if (it->first == 0){
    		costMap[0] += feasibilityCost(lane-1);
    		if (costMap[0] == 0){
    			costMap[0] += collisionCost(sensorVec, lane-1, prevSize, car_s);
    			costMap[0] += changeLaneLeftCost();
    			costMap[0] += bufferCost(sensorVec, lane-1, prevSize, car_s);
    		}
    	}
    	//Right
    	else if (it->first == 1){
    		costMap[1] += feasibilityCost(lane+1);
    		if (costMap[1] == 0){
    			costMap[1] += collisionCost(sensorVec, lane+1, prevSize, car_s);
    			costMap[1] += changeLaneRightCost();
    			costMap[1] += bufferCost(sensorVec, lane+1, prevSize, car_s);
    		}
    	}
    	//Keep Lane
    	if (it->first == 2){
    		costMap[2] += feasibilityCost(lane);
    		if (costMap[2] == 0){
    			costMap[2] += collisionCost(sensorVec, lane, prevSize, car_s);
    			costMap[2] += keepLaneCost();
    			costMap[2] += bufferCost(sensorVec, lane, prevSize, car_s);
    		}
    	}
    	++it;
    }
    cout << "Left Lane Cost: " << costMap[0] << endl;
    cout << "Right Lane Cost: " << costMap[1] << endl;
    cout << "Keep Lane Cost: " << costMap[2] << endl;
    auto x = std::min_element(costMap.begin(), costMap.end(),
        [](const pair<int, int>& p1, const pair<int, int>& p2) {
            return p1.second < p2.second; });
    //cout << "Minimum Val: " << x->first << endl;
    if (costMap[0] >= danger && costMap[1] >= danger && costMap[2] >= danger){
    	return 3;
    }
    else if((costMap[0] == costMap[2])){
    	return 2;
    }
    return x->first;
}

double laneSpeed(vector<vector<double>> sensorVec, int lane, double car_s){
	double speed = 100;
	double d;
	for (int i=0;i<sensorVec.size();++i){
		d = sensorVec[i][6];
		//cout << "d: "<< d << endl;
		if(d < (2+4*lane+2) && d >(2+4*lane-2)){
			double vx = sensorVec[i][3];
			double vy = sensorVec[i][4];
			double other_car_speed = sqrt(vx*vx+vy*vy);
			double other_car_s = sensorVec[i][5];
			if (other_car_speed < speed){
				speed = other_car_speed;
			}
			//cout << "laneSpeed: " << speed<< endl;
			//cout << "car_s: " << car_s<< endl;
		}
	}
	return speed;
}
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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];
	double heading = atan2( (map_y-y),(map_x-x) );
	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}
	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}
	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];
	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;
	double frenet_d = distance(x_x,x_y,proj_x,proj_y);
	//see if d value is positive or negative by comparing it to a center point
	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);
	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}
	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);
	return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;
	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}
	int wp2 = (prev_wp+1)%maps_x.size();
	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);
	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
	double perp_heading = heading-pi()/2;
	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);
	return {x,y};
}

int main() {
  uWS::Hub h;
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  double car_speed_p = 0; // car acceleration on the prev step
  double car_acc_p = 0; // car acceleration on the prev step
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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
  
  int laneNum = 1;
  double cur_speed = 0; // current speed
  double c_time = 0;
  bool init = true;
  bool slowDown = false;
  double currentSpeed=0;
  h.onMessage([&init,&slowDown,&currentSpeed,&laneNum,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,
  &car_speed_p, &car_acc_p, &cur_speed, &c_time](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];
          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
          	
          	int prev_size = previous_path_x.size();
          	
    		// Creating vector for sensor fusion
            //################################################################
			// 				Creating vector for sensor fusionGenerating XY points for prediction
			//################################################################
          	vector<vector<double>> sensorVec;
          	for (int i=0;i<sensor_fusion.size();++i){
          		vector<double> tmpVec;
          		for (int j=0;j<sensor_fusion[i].size();++j){
              		tmpVec.push_back(sensor_fusion[i][j]);
          		}
          		sensorVec.push_back(tmpVec);

          	}
            //################################################################
			// 				End of Creating vector for sensor fusionGenerating XY points for prediction
			//################################################################
            //################################################################
			// 				Cost function of different state machines
			//################################################################
         	vector<vector<double>> laneCost;
        	int action = calculateCost(sensorVec, laneNum, prev_size,car_s);
        	switch (action){
        		case 0:
        			cout << "Decision --> Left Lane Change" << endl;
        			laneNum += -1;
        			break;
        		case 1:
        			cout << "Decision --> Right Lane Change" << endl;
        			laneNum += +1;
        			break;
        		case 2:
        			cout << "Decision --> Keep Lane " << endl;
        			laneNum += 0;
        			break;
        		case 3:
        			cout << "Decision --> Slow down" << endl;
        			slowDown = true;
        			break;
        	};
            //################################################################
			// 				End of Cost function of different state machines
			//################################################################
            //################################################################
			// 				Speed control
			//################################################################
			currentSpeed = car_speed;
			if (init){
				if (currentSpeed > 45.5){
					init = false;
				}
				else{
					currentSpeed+=4.0;
				}
			}
			else{
				if(slowDown){
					double speed = 2.24*laneSpeed(sensorVec, laneNum, car_s);
					//cout << "speed: "<< speed << endl;
					if (currentSpeed <= speed ){
						slowDown = false;
					}
					else if (currentSpeed > 30){
						currentSpeed-=2.4;
						cout << "Slowing down"<< endl;
					}
				}
				else{
					if (currentSpeed <= 48.4){
						currentSpeed += 1;
					}
				}
			}
            //################################################################
			// 				End of Speed control
			//################################################################
            //################################################################
			// 				Generating XY points for prediction
			//################################################################
          	// Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
			// x and y points will be used for prediction of trajectory
			// Using previous x and y points
            vector<double> ptsx;
            vector<double> ptsy;
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            if (prev_size < 2){
				double prev_car_x = car_x - cos(car_yaw);
				double prev_car_y = car_y - sin(car_yaw);
				ptsx.push_back(prev_car_x);
				ptsx.push_back(car_x);
				ptsy.push_back(prev_car_y);
				ptsy.push_back(car_y);
			}
			else {
				ref_x = previous_path_x[prev_size-1];
				ref_y = previous_path_y[prev_size-1];
				double prev_ref_x = previous_path_x[prev_size-2];
				double prev_ref_y = previous_path_y[prev_size-2];
				ref_yaw = atan2(ref_y-prev_ref_y,ref_x-prev_ref_x);
				ptsx.push_back(prev_ref_x);
				ptsx.push_back(ref_x);
				ptsy.push_back(prev_ref_y);
				ptsy.push_back(ref_y);
			}
            // Converting Frenet cordinates to Polar coordinates for polynomial fitting
			const int s_step = 3;
			const double s_dist = STEP_SIZE;
			for (int i=0; i < s_step; i++){
				vector<double> wp = getXY(car_s+s_dist*(i+1), (2 + 4 * (laneNum)), map_waypoints_s, map_waypoints_x, map_waypoints_y);
				ptsx.push_back(wp[0]);
				ptsy.push_back(wp[1]);
			}
			// Translating local coordinates to global coordinates.
			double sin_yaw = sin(0 - ref_yaw);
			double cos_yaw = cos(0 - ref_yaw);
			for (int i = 0; i < ptsx.size(); i++){
				double shift_x = ptsx[i]-ref_x;
				double shift_y = ptsy[i]-ref_y;
				ptsx[i]=shift_x*cos_yaw-shift_y*sin_yaw;
				ptsy[i]=shift_y*cos_yaw+shift_x*sin_yaw;
			}
			tk::spline s;  // Create spline
			s.set_points(ptsx, ptsy); 
            //################################################################
			// 				End of Generating XY points for prediction
			//################################################################
            //################################################################
			// 				Populating set of XY points
			//################################################################
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
          	int point_transfer = min(prev_size, POINTS_TO_KEEP);
			for (int i = 0; i < point_transfer; i++){
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}
			double target_y = s(s_dist);
			double dist = sqrt(target_y*target_y+s_dist*s_dist);
			double x_add_on = 0;
			double N = dist / (DT * (currentSpeed/2.24));
			double cos_y = cos(ref_yaw);
			double sin_y = sin(ref_yaw);
			for (int i = 0; i < POINTS_TO_SEND-point_transfer; i++){
				double x_point = x_add_on+s_dist/N;
				double y_point = s(x_point);
				x_add_on = x_point;
				double x_ref = x_point;
				double y_ref = y_point;
				x_point = x_ref * cos_y - y_ref * sin_y + ref_x;
				y_point = x_ref * sin_y + y_ref * cos_y + ref_y;
				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
			}
            //################################################################
			// 				End Populating set of XY points
			//################################################################
			json msgJson;
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
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
















































































