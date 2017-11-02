
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#define REF_VELOCITY 45
#define REACH_GOAL 0
#define DESIRED_PREP 0
#define DESIRED_DIST_BUFFER 30
#define DESIRED_SPEED_BUFFER 20


#define EFFICIENCY pow(10,3)
#define COLLISION pow(10,5)
#define DANGER pow(10,2)
#define COMFORT pow(10,2)

using namespace std;
// for convenience
using json = nlohmann::json;
int lane_prep = 0;
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

//Cost Functions

double distance_from_goal(int distance,double vel,int lane, string state)
{
    double multiplier = 1;
    if(state.compare("KL") != 0 && distance > 3){
        multiplier = 1.2 ;
    }     
    double time_to_goal = distance/vel;    
    return multiplier*REACH_GOAL/time_to_goal;
};

double get_inefficiency_cost(double vel,int lane, string state)
{   

    if(state.compare("LCL") == 0 || state.compare("LCR")==0){
        return  0;
    }   
    double pct = (REF_VELOCITY - vel)/ REF_VELOCITY;   
    return pow(pct,2) * EFFICIENCY;
}


double get_collision_cost(double vel,int lane, double collide)
{
  return pow(collide, 2) * COLLISION;
}

double get_buffer_cost(double vel,int lane, double buffer)
{

  if (buffer == 0 || vel == 0 )
  {
    return 0;
  }

  double multiplier = pow(( DESIRED_DIST_BUFFER/buffer),2);
  return multiplier * DANGER;
}

double get_change_lane_cost(double vel,int lane, string state)
{

  if(state.compare("LCL") == 0||state.compare("LCR") == 0){
      return COMFORT;}
  return -COMFORT;
}


double check_state_cost(vector < vector<double> >  sensor_fusion, double vel, int lane, string state, double car_s,int path_size) { 

    double distance_cost,inefficiency_cost,collision_cost,buffer_cost,change_lane_cost =0.0;

    double buffer =  DESIRED_DIST_BUFFER;
    double collide = 0;

    for (int i = 0; i < sensor_fusion.size();i++){
    
      float  d = sensor_fusion[i][6];
      int lane_new;


      if(state.compare("KL")==0){
        lane_new = lane;
      }
      else if(state.compare("LCR")==0){
        lane_new = lane + 1;
      }
      else if(state.compare("LCL")==0){
        lane_new = lane - 1;
      }

      int d_new = 2+4*lane_new;

      if (d > (d_new - 2) && d < (d_new + 2) )  {  
          double vx = sensor_fusion[i][3];
          double vy = sensor_fusion[i][4];
          
          double check_speed = sqrt(vx *vx + vy*vy);  // next car's speed
          double check_car_s = sensor_fusion[i][5];

          check_car_s += (double)path_size*0.02*check_speed ;

          if(abs(check_car_s - car_s) <  DESIRED_DIST_BUFFER ){ 
            if(abs(check_car_s - car_s) <  buffer ){                
              buffer = abs(check_car_s - car_s); 
            }
            if(abs(check_car_s - car_s) <  (DESIRED_DIST_BUFFER - 5) ){   
//              cout << "Collision!!" << endl;
               collide = 1;
            }
          }


      } // end of loop for cars in same lane
    } // end of for loop for checking all cars




//    distance_cost = distance_from_goal(distance,vel,lane,state);
    inefficiency_cost= get_inefficiency_cost(vel,lane,state);
    change_lane_cost = get_change_lane_cost(vel,lane,state);
    collision_cost = get_collision_cost(vel,lane,collide);
    buffer_cost = get_buffer_cost(vel,lane,buffer);
    cout <<"State " <<state<<" Efficiency: "<<inefficiency_cost<<" Buffer: "<<buffer_cost<<" Change: "<<change_lane_cost<<" Collision: "<<collision_cost<<endl;
    return distance_cost+inefficiency_cost+buffer_cost+change_lane_cost+collision_cost;

   }


void execute_state(string  state, double& vel, int& lane) { 
 
    if(state.compare("CS") == 0)
    {

    }
    else if(state.compare("KL") == 0)
    {
      lane = lane ;  
    }
    else if(state.compare("LCL") == 0)
    {
      if(lane_prep++<DESIRED_PREP){execute_state("PLCL",vel,lane);return;}
      lane = lane - 1;
      lane_prep = 0;
    }
    else if(state.compare("LCR") == 0)
    {
      if(lane_prep++<DESIRED_PREP){execute_state("PLCR",vel,lane);return;}
      lane = lane + 1;
      lane_prep = 0;
    }
    else if(state.compare("PLCL") == 0 || state.compare("PLCR") == 0)
    {
      //vel += (REF_VELOCITY - vel)/REF_VELOCITY  ;
    }

}


vector<string> check_possible_states(int lane) { 

    vector<string> poss_lanes = {"KL"};
    if (lane!=0)
    {
        poss_lanes.push_back("LCL");
        //poss_lanes.push_back("PLCL");
        
    }


    if( lane != 2)
    {
        poss_lanes.push_back("LCR");
        //poss_lanes.push_back("PLCR");
        
    }
     
    return poss_lanes;
   }


void update_state(vector < vector<double> >   sensor_fusion, double& vel, int& lane, double car_s,int path_size) { 
    int idx ;
    double best_cost = 99999999999999;
    vector<string> possible_states;

    possible_states  = check_possible_states(lane);  


    for(int i=0;i<possible_states.size();i++) 
    {

        double ocost = check_state_cost(sensor_fusion,vel,lane,possible_states[i], car_s,path_size);

        if( ocost < best_cost)
        {
            best_cost = ocost;
            idx = i;
        }        
    }    

    cout <<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<," <<endl;
    cout <<"Final state " << possible_states[idx]<<endl;  
    execute_state(possible_states[idx],vel,lane);

}

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

  int lane_ref = 1;
  double vel_ref = 0;
  double old_vel_ref = 0;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

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


  h.onMessage([&lane_ref,&old_vel_ref,&vel_ref,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
            
            vector<double> x_points;
            vector<double> y_points;
            double x_ref = car_x;
            double y_ref = car_y;
            double yaw_ref = deg2rad(car_yaw);

            cout << " X " << car_x << endl ;  
            cout << " Y "  << car_y << endl ;  
            cout << " car_yaw "  << car_yaw << endl ;  
            cout << " Speed " << car_speed << endl ;  
            cout << " S "  << car_s << endl ;  
            cout << " D "  << car_d << endl ;  


            int path_size = previous_path_x.size();
            
            if(path_size > 0)
            {
              car_s = end_path_s;
              cout << "Got Path" << endl;  
            }
      
            bool too_close = false;
 
            for (int i = 0; i < sensor_fusion.size();i++){
            
              float  d = sensor_fusion[i][6];

              if (d > (2+4*lane_ref - 2) && d < (2+4*lane_ref +2)  ){   // if its in the same lane

                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                
                double check_speed = sqrt(vx *vx + vy*vy);  // next car's speed
                double check_car_s = sensor_fusion[i][5];


                check_car_s += (double)path_size*0.02*check_speed ;

                if( ( check_car_s > car_s) && ((check_car_s - car_s) <  30 )){ 
                  too_close =true;    
                  cout << "Update State" << endl;  

                  update_state(sensor_fusion,vel_ref,lane_ref,car_s,path_size);            
                }
              } // end of loop for cars in same lane
            } // end of for loop for checking all cars

            if(too_close)
            {
              vel_ref -= 0.4/pow(REF_VELOCITY - vel_ref,0.5) ;
              too_close = false;
              if(car_speed < 35 && car_speed/vel_ref < 1.025 ){vel_ref = car_speed*(1-0.025);}
              //if(car_speed > 35 ){vel_ref = car_speed - 1.5*0.224;}

              cout << "Braking.." << endl ; 

            }            
            else if(vel_ref < REF_VELOCITY)
            {
              vel_ref += (REF_VELOCITY - vel_ref)/REF_VELOCITY  ;   
              if(vel_ref > REF_VELOCITY){vel_ref = REF_VELOCITY;}
              if(car_speed > 35 && vel_ref / car_speed > 1.01 ){vel_ref = car_speed*1.01;}
              //if(car_speed < 35 ){vel_ref = car_speed + 1;}

               cout << "Accelerating.." << endl ; 

            }

            if(path_size < 2)
            {
                cout << "No points.." << endl ; 

                double prev_car_x = car_x - cos(car_yaw); 
                double prev_car_y = car_y - sin(car_yaw); 
                x_points.push_back(prev_car_x);
                y_points.push_back(prev_car_y);
                x_points.push_back(car_x);
                y_points.push_back(car_y);
             }
            else
            {
                x_ref = previous_path_x[path_size-1];
                y_ref = previous_path_y[path_size-1];

                double prev_x_ref = previous_path_x[path_size-2];
                double prev_y_ref  = previous_path_y[path_size-2];

                yaw_ref = atan2(y_ref - prev_y_ref,x_ref - prev_x_ref);
                x_points.push_back(prev_x_ref);
                y_points.push_back(prev_y_ref);
                x_points.push_back(x_ref);
                y_points.push_back(y_ref);
            }

            double l_points;
            std::vector<int> s_diff ;

            if(car_d < 4+4*lane_ref && car_d > 4*lane_ref)
            {
            l_points = 3;
            s_diff = {40,60,90,0,0,0};
 
            } 
            else
            {
            l_points = 12;
            s_diff = {10,15,20,25,30,35,40,50,60,70,80,90};
            }

            cout << "Target Ref " << lane_ref << endl ; 

            for(int i=1;i<=l_points;i++)
            {    
              vector<double> wayp = getXY(car_s + s_diff[i-1],2+4*lane_ref,map_waypoints_s,map_waypoints_x,map_waypoints_y);
            
              x_points.push_back(wayp[0]);
              y_points.push_back(wayp[1]);

            }

            for(int i = 0; i < x_points.size(); i++){
                double shift_x = x_points[i]-x_ref;
                double shift_y = y_points[i]-y_ref;

                x_points[i] = (shift_x*cos(-yaw_ref)-shift_y*sin(-yaw_ref));
                y_points[i] = (shift_x*sin(-yaw_ref)+shift_y*cos(-yaw_ref));

            }

            tk::spline s;
            s.set_points(x_points,y_points);
 
            for(int i = 0; i < path_size; i++)
            {   
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
            double x_add_on = 0;
            double y_ref_last = 0;
            for(int i = 0; i < 50-path_size; i++)
            {   
                double N = 2.24*50*(target_dist/(vel_ref));
                double x_pt = x_add_on + (target_x)/N;
                double y_pt = s(x_pt);
    
                x_add_on = x_pt;
 
                double x_r = x_pt;
                double y_r = y_pt;


                x_pt = (x_r*cos(yaw_ref)-y_r*sin(yaw_ref));
                y_pt = (x_r*sin(yaw_ref)+y_r*cos(yaw_ref));

                x_pt += x_ref;
                y_pt += y_ref;

                //if (y_ref_last > 0 && y_ref_last-y_ref > 0.1) y_ref = (y_ref+y_ref_last)/2;
                //y_ref_last = getFrenet(x_pt,y_pt,yaw_ref,);   
                //if( abs(car_d - mov[1]) > 0.4 ) {vel_ref-=0.25;cout << "Slow down " << endl;}
                //cout << "Values " << old_vel_ref<<"Old " <<car_d << endl;
                //if (abs((car_d - old_vel_ref)) > 1) {
                //vel_ref > car_speed ? vel_ref=car_speed+2:vel_ref= car_speed - 2;}

                next_x_vals.push_back(x_pt);
                next_y_vals.push_back(y_pt);

            }



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
















































































