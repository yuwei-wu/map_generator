#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>



#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Eigen>
#include <random>

// for semantic map visualization
#include <semantics_msgs/SemanticArray.h>
#include <semantics_msgs/Circle.h>

#include <visualization_msgs/MarkerArray.h>


using namespace std;

vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device rd;
default_random_engine eng(rd());
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;


ros::Publisher _all_map_cloud_pub, _all_map_semantics_pub, _all_map_semantics_pub_vis;

vector<double> _state;

int _obs_num;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;
std::string _frame_id;

bool _map_ok = false;
bool _has_odom = false;
bool _set_semantics = false;

int circle_num_;
double radius_l_, radius_h_, z_l_, z_h_;
double theta_;
uniform_real_distribution<double> rand_radius_;
uniform_real_distribution<double> rand_radius2_;
uniform_real_distribution<double> rand_theta_;
uniform_real_distribution<double> rand_z_;

sensor_msgs::PointCloud2 globalMap_pcd;

pcl::PointCloud<pcl::PointXYZ> cloudMap;
semantics_msgs::SemanticArray global_semantics_msg;

visualization_msgs::MarkerArray semantics_vis;
visualization_msgs::Marker semantics_mk;


void RandomMapGenerate() {
  pcl::PointXYZ pt_random;
  geometry_msgs::Pose pt;
  pt.orientation.w = 1.0;


  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);

  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);


  // only enable the clyinders in the forest case
  // generate polar obs
  for (int i = 0; i < _obs_num; i++) {
    double x, y, w, h;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);
    h = rand_h(eng);
    int heiNum = ceil(h / _resolution);

    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
      i--;
      continue;
    }

    if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
      i--;
      continue;
    }

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    pt.position.x= x;
    pt.position.y = y;
    pt.position.z = 0.5*h;
    semantics_mk.pose = pt;
    semantics_mk.scale.x = semantics_mk.scale.y = w; // less then 1
    semantics_mk.scale.z = h;
    semantics_vis.markers.push_back(semantics_mk);
    semantics_mk.id += 1;


    semantics_msgs::Circle tree_model;

    tree_model.id = i;
    tree_model.pos.x = x;
    tree_model.pos.y = y;
    tree_model.r = w;
    
    global_semantics_msg.circles.push_back(tree_model);

    int widNum = ceil( w / _resolution) +1;
    
    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {

        for (int t = -2.0; t < heiNum; t++) {
          pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
          pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
          pt_random.z = (t + 0.5) * _resolution + 1e-2;

          //@yuwei: to make it as semantics
          if (  (pt_random.x - x)*(pt_random.x - x) + (pt_random.y - y)*(pt_random.y - y)   >  w*w/ 4.0  ){
            continue;
          }
          cloudMap.points.push_back(pt_random);
        }
      }
    

  }

  // generate circle obs // even if we generate, we will not include them into our shared information
  for (int i = 0; i < circle_num_; ++i) {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
      i--;
      continue;
    }

    if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
      i--;
      continue;
    }

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0, 1;

    double radius1 = rand_radius_(eng);
    double radius2 = rand_radius2_(eng);

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz) {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
          }
    }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generate random map ");

  _map_ok = true;
}

int i = 0;
void pubSensedPoints() {
  // if (i < 10) {
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = _frame_id;
  _all_map_cloud_pub.publish(globalMap_pcd);
  // }

  global_semantics_msg.mav_id = -1; // -1 for global, 0 + for the mav_id
  global_semantics_msg.header.frame_id = _frame_id;
  _all_map_semantics_pub.publish(global_semantics_msg);


  _all_map_semantics_pub_vis.publish(semantics_vis);

  return;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "random_map_sensing");
  ros::NodeHandle n("~");

  _all_map_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);

  // semantics publishers
  _all_map_semantics_pub = n.advertise<semantics_msgs::SemanticArray>("global_semantics", 1);
  _all_map_semantics_pub_vis = n.advertise<visualization_msgs::MarkerArray>("global_semantics_vis", 1);


  n.param("init_state_x", _init_x, 0.0);
  n.param("init_state_y", _init_y, 0.0);

  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);

  // clearance for multi robots.
  _x_size -= 2.0;
  _y_size -= 2.0;


  n.param("map/obs_num", _obs_num, 30);
  n.param("map/resolution", _resolution, 0.1);
  n.param("map/circle_num", circle_num_, 30);
  n.param("map/frame_id", _frame_id, string("map"));


  n.param("ObstacleShape/lower_rad", _w_l, 0.3);
  n.param("ObstacleShape/upper_rad", _w_h, 0.8);
  n.param("ObstacleShape/lower_hei", _h_l, 3.0);
  n.param("ObstacleShape/upper_hei", _h_h, 7.0);
  n.param("ObstacleShape/set_semantics", _set_semantics, false);


  n.param("ObstacleShape/radius_l", radius_l_, 7.0);
  n.param("ObstacleShape/radius_h", radius_h_, 7.0);
  n.param("ObstacleShape/z_l", z_l_, 7.0);
  n.param("ObstacleShape/z_h", z_h_, 7.0);
  n.param("ObstacleShape/theta", theta_, 7.0);

  n.param("sensing/radius", _sensing_range, 10.0);
  n.param("sensing/radius", _sense_rate, 10.0);

  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  _z_limit = _z_size;

  semantics_mk.header.frame_id = _frame_id;
  semantics_mk.header.stamp = ros::Time::now();
  semantics_mk.type = visualization_msgs::Marker::CYLINDER;
  semantics_mk.action  = visualization_msgs::Marker::ADD;
  semantics_mk.id = 0;
  semantics_mk.color.r = 0.5;
  semantics_mk.color.g = 0.5;
  semantics_mk.color.b = 0.5;
  semantics_mk.color.a = 0.6;

  ros::Duration(0.5).sleep();

  RandomMapGenerate();

  ros::Rate loop_rate(_sense_rate);

  while (ros::ok()) {
    pubSensedPoints();
    ros::spinOnce();
    loop_rate.sleep();
  }
}