#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>

// for semantic map visualization
#include <semantic_msgs/SemanticArray.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device rd;
default_random_engine eng(1.0);
double seed_;
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;
uniform_real_distribution<double> rand_b_l;
uniform_real_distribution<double> rand_b_h;

ros::Publisher _all_map_cloud_pub, _all_map_semantics_pub,
    _all_map_semantics_pub_vis;

vector<double> _state;

uint map_number = 1;
int _obs_num, _box_num;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h, _b_l_l, _b_l_h, _b_h_l,
    _b_h_h;
double _z_limit, _resolution, _sense_rate, _init_x, _init_y;
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
semantic_msgs::SemanticArray global_semantics_msg;

void RandomMapGenerate() {
  global_semantics_msg.polyhedrons.clear();
  global_semantics_msg.cylinders.clear();
  cloudMap.clear();

  pcl::PointXYZ pt_random;
  geometry_msgs::Pose pt;
  pt.orientation.w = 1.0;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);

  rand_b_l = uniform_real_distribution<double>(_b_l_l, _b_l_h);
  rand_b_h = uniform_real_distribution<double>(_b_h_l, _b_h_h);

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

    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 1.5) {
      i--;
      continue;
    }

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    pt.position.x = x;
    pt.position.y = y;
    pt.position.z = 0.5 * h;
    // semantics_mk.pose = pt;
    // semantics_mk.scale.x = semantics_mk.scale.y = w; // less then 1
    // semantics_mk.scale.z = h;
    // //semantics_vis.markers.push_back(semantics_mk);
    // semantics_mk.id += 1;

    semantic_msgs::Cylinder tree_model;

    tree_model.id = i;
    tree_model.pos.x = x;
    tree_model.pos.y = y;
    tree_model.r = 0.5 * w;
    tree_model.h = h;

    global_semantics_msg.cylinders.push_back(tree_model);

    int widNum = ceil(w / _resolution) + 1;

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
        for (int t = -2.0; t < heiNum; t++) {
          pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
          pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
          pt_random.z = (t + 0.5) * _resolution + 1e-2;

          //@yuwei: to make it as semantics
          if ((pt_random.x - x) * (pt_random.x - x) +
                  (pt_random.y - y) * (pt_random.y - y) >
              w * w / 4.0) {
            continue;
          }
          cloudMap.points.push_back(pt_random);
        }
      }
  }

  // generate box
  for (int i = 0; i < _box_num; i++) {
    double x, y, w1, w2, h, theta;
    x = rand_x(eng);
    y = rand_y(eng);

    w1 = rand_b_l(eng);      // width in x  // should be total width
    w2 = 0.3 * rand_w(eng);  // width in y
    h = rand_b_h(eng);
    theta = rand_theta_(eng);

    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
      i--;
      continue;
    }

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum1 = ceil(0.5 * w1 / _resolution);
    int widNum2 = ceil(0.5 * w2 / _resolution);

    semantic_msgs::Polyhedron box_model;

    box_model.id = i;
    box_model.r = 0.5 * w1;
    box_model.points.resize(6);
    box_model.normals.resize(6);

    Eigen::MatrixXd b_points(3, 6);
    b_points << -0.5 * w1, 0.5 * w1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5 * w2,
        0.5 * w2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5 * h, 0.5 * h;
    Eigen::Vector3d pos;

    for (int i = 0; i < 6; i++) {
      pos = rotate * b_points.col(i);
      box_model.points[i].x = x + pos(0);
      box_model.points[i].y = y + pos(1);
      box_model.points[i].z = 0.5 * h + pos(2);

      pos.normalize();
      box_model.normals[i].x = pos(0);
      box_model.normals[i].y = pos(1);
      box_model.normals[i].z = pos(2);
    }

    int size = ceil(w1 / w2);
    box_model.rps.resize(size);
    // std::cout << "w2 is " << w2 <<  "w1 is " << w1 <<  std::endl;
    // std::cout << "x is " << x <<  "y is " << y <<  "theta is " << theta <<
    // std::endl; std::cout << "the size is " << size << std::endl;
    int idx = 0;
    for (double r = -w1 + w2; r < w1 + w2; r += 2.0 * w2) {
      box_model.rps[idx].x = x + cos(theta) * r;
      box_model.rps[idx].y = y + sin(theta) * r;
      // std::cout << " idx is " << idx << std::endl;
      // std::cout << " box_model.rps[idx] is " << box_model.rps[idx].x <<
      // box_model.rps[idx].y << std::endl;
      idx += 1;
    }

    global_semantics_msg.polyhedrons.push_back(box_model);

    int heiNum = ceil(h / _resolution);

    for (int r = -widNum1; r < widNum1; r++) {
      for (int s = -widNum2; s < widNum2; s++) {
        for (int t = -2.0; t < heiNum; t++) {
          Eigen::Vector3d box_scale =
              rotate * Eigen::Vector3d((r + 0.5) * _resolution,
                                       (s + 0.5) * _resolution,
                                       (t + 0.5) * _resolution);

          pt_random.x = x + box_scale(0) + 1e-2;
          pt_random.y = y + box_scale(1) + 1e-2;
          pt_random.z = box_scale(2) + 1e-2;
          if (abs(pt_random.x) >= 0.5 * _x_size ||
              abs(pt_random.y) >= 0.5 * _y_size ||
              abs(pt_random.z) >= _z_size) {
            continue;
          }

          cloudMap.points.push_back(pt_random);
        }
      }
    }
  }

  // generate circle obs // even if we generate, we will not include them into
  // our shared information
  for (int i = 0; i < circle_num_; ++i) {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 1.5) {
      i--;
      continue;
    }

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

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
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution,
                                           ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
          }
    }
  }

  // generate ground
  for (double r = _x_l; r < _x_h; r += _resolution) {
    for (double s = _y_l; s < _y_h; s += _resolution) {
      for (double t = 0.0; t < 0.2; t += 0.1) {
        pt_random.x = r + 1e-2;
        pt_random.y = s + 1e-2;
        pt_random.z = t + 1e-2;

        cloudMap.points.push_back(pt_random);
      }
    }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generate random map ");

  _map_ok = true;
}
bool changeMapCallback(std_srvs::Empty::Request& req,
                       std_srvs::Empty::Response& res) {
  eng.seed(++seed_);
  _map_ok = false;
  RandomMapGenerate();  // auto set map ok
  return true;
}
void pubSensedPoints() {
  // if (i < 10) {
  if (!_map_ok) {
    ROS_WARN("Map not ready yet");
    return;
  }
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = _frame_id;
  _all_map_cloud_pub.publish(globalMap_pcd);

  global_semantics_msg.mav_id = -1;  // -1 for global, 0 + for the mav_id
  global_semantics_msg.header.frame_id = _frame_id;
  _all_map_semantics_pub.publish(global_semantics_msg);

  return;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "random_map_sensing");
  ros::NodeHandle n("~");

  _all_map_cloud_pub =
      n.advertise<sensor_msgs::PointCloud2>("/global_cloud", 1);
  // semantics publishers
  _all_map_semantics_pub =
      n.advertise<semantic_msgs::SemanticArray>("/global_semantics", 1);

  // define service that can change map
  ros::ServiceServer change_map_service =
      n.advertiseService("/gen_new_map", changeMapCallback);

  //_all_map_semantics_pub_vis =
  // n.advertise<visualization_msgs::MarkerArray>("global_semantics_vis", 1);

  n.param("init_state_x", _init_x, 0.0);
  n.param("init_state_y", _init_y, 0.0);

  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);

  n.param("map/obs_num", _obs_num, 30);
  n.param("map/resolution", _resolution, 0.1);
  n.param("map/circle_num", circle_num_, 30);

  /******/
  n.param("map/box_num", _box_num, 30);
  n.param("ObstacleShape/lower_b_len", _b_l_l, 3.0);
  n.param("ObstacleShape/upper_b_len", _b_l_h, 7.0);
  n.param("ObstacleShape/lower_b_hei", _b_h_l, 0.1);
  n.param("ObstacleShape/upper_b_hei", _b_h_h, 4.0);
  n.param("ObstacleShape/seed", seed_, 1.0);

  eng.seed(seed_);

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

  n.param("sensing/rate", _sense_rate, 10.0);

  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  _z_limit = _z_size;

  ros::Duration(0.5).sleep();

  RandomMapGenerate();

  ros::Rate loop_rate(_sense_rate);

  while (ros::ok()) {
    pubSensedPoints();
    ros::spinOnce();
    loop_rate.sleep();
  }
}