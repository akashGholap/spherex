/** @file demo_flight_control.h
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef DEMO_FLIGHT_CONTROL_H
#define DEMO_FLIGHT_CONTROL_H

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
//#include <armadillo.h>
//#include <eigen/dense.h>
// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>


#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

/*!
 * @brief a bare bone state machine to track the stage of the mission
 */
class Mission
{
public:
  // The basic state transition flow is:
  // 0---> 1 ---> 2 ---> ... ---> N ---> 0
  // where state 0 means the mission is note started
  // and each state i is for the process of moving to a target point.
  int state;

  int inbound_counter;
  int outbound_counter;
  int break_counter;
  int vel_counter;
  int wait_counter;
  int up_counter;

  double xi,yi,zi; //initial position
  double xf,yf,zf; //final position
  double Rx,Ry,Rz;
  double d,theta,phi;
  double v;
  double optimum_time;

  //double Velx,Vely,Velz_c,Velz;

  float target_offset_x;
  float target_offset_y;
  float target_offset_z;
  float target_yaw;
  double x_vel, y_vel, z_vel,z_vel_current;
  bool start_flag;
  bool opt_flag;

  sensor_msgs::NavSatFix start_gps_location;
  geometry_msgs::Point start_local_position;

  bool finished;
  bool land;

  Mission() : state(0), inbound_counter(0), outbound_counter(0), break_counter(0),
              target_offset_x(0.0), target_offset_y(0.0), target_offset_z(0.0),
              finished(false),wait_counter(0),start_flag(false),x_vel(0.0),y_vel(0.0),z_vel(0.0),vel_counter(0),z_vel_current(0.0),
              xi(0.0),yi(0.0),zi(0.0),xf(0.0),yf(0.0),zf(0.0),Rx(0),Ry(0),Rz(0),optimum_time(0.11),land(false),d(0.0),theta(0.0),phi(0.0),v(0.0),opt_flag(0)
  {
  }

  //void step();

  void setTarget(float x, float y, float z, float yaw)
  {
    target_offset_x = x;
    target_offset_y = y;
    target_offset_z = z;
    target_yaw      = yaw;
  }
 void step();
  void reset()
  {
    inbound_counter = 0;
    outbound_counter = 0;
    break_counter = 0;
    finished = false;
    start_flag = true;
  }

 bool hopex(double, double, double, double);

 int hopex_to_pos(float x, float y, float z, float yaw);

 int hop_vel_pos(float x, float y, float z, float yaw);

 int create_position_matrix(std::vector<std::vector<float>> &pos_matrix, float x, float y, float z, float yaw);

 void Hop_step();

 bool hop_step(double xc,double yc, double zc, double t);

 void hop_fill_vel(double Vx, double Vy, double Vz, double yaw);

 void Hop_step_vel_pos();

 void set_mission(double xf_, double yf_, double zf_, double xi_, double yi_, double zi_ )
 {
   xf =xf_;
   yf = yf_;
   zf = zf_;
   xi = xi_;
   yi = yi_;
   zi_= zi_;
   Rx = xf - xi;
   Ry = yf - yi;
   Rz = zf - zi;
   x_vel = 0;
   y_vel = 0;
   z_vel = 0;
   d = xf_;
   theta = yf_;
   phi = zf_;
   v = 0;
   opt_flag = 0;
   wait_counter = 0;
   start_flag = true;
   land = false;
   finished =false;
 }

};

void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin);

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);



void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);

bool takeoff_land(int task);

void getVelocity_callback(const geometry_msgs::Vector3Stamped& vel_from_sdk);

bool obtain_control();

bool is_M100();

bool monitoredTakeoff();

bool M100monitoredTakeoff();

bool set_local_position();

double optimization_function(double x);

bool set_optimum_velocity();

bool landing_initiate(void);

bool arm_motors(void);
bool disarm_motors(void);

#endif // DEMO_FLIGHT_CONTROL_H
