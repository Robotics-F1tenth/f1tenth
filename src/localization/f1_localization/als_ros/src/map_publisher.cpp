#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// PGM (P5) binary reader
bool read_pgm_file(const std::string &file_path,
                   std::vector<uint8_t> &map_data,
                   int &width, int &height, int &maxval_out)
{
  std::ifstream file(file_path, std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "Failed to open PGM file: " << file_path << std::endl;
    return false;
  }

  std::string magic;
  file >> magic;
  if (magic != "P5") {
    std::cerr << "Invalid PGM format (expected P5): " << magic << std::endl;
    return false;
  }

  auto skip_comments = [&file]() {
    file >> std::ws;
    while (file.peek() == '#') { std::string dummy; std::getline(file, dummy); file >> std::ws; }
  };

  skip_comments(); file >> width;
  skip_comments(); file >> height;
  skip_comments(); int maxval = 255; file >> maxval; maxval_out = maxval;

  char whitespace; file.get(whitespace);
  if (!file.good() || width <= 0 || height <= 0 || maxval <= 0) {
    std::cerr << "Invalid PGM header values." << std::endl; return false;
  }

  const std::size_t sz = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  map_data.resize(sz);
  file.read(reinterpret_cast<char*>(map_data.data()), static_cast<std::streamsize>(sz));
  if (file.gcount() != static_cast<std::streamsize>(sz)) {
    std::cerr << "PGM payload size mismatch. Expected " << sz << " bytes, got "
              << file.gcount() << " bytes." << std::endl;
    return false;
  }
  return true;
}

// yaw(rad) -> quaternion about Z
inline void yaw_to_quat(double yaw, double &qx, double &qy, double &qz, double &qw)
{
  const double h = 0.5 * yaw;
  qx = 0.0; qy = 0.0; qz = std::sin(h); qw = std::cos(h);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("map_publisher");

  // Params
  node->declare_parameter<std::string>("pgm_file_path", "/home/a/f1_localization/src/als_ros/maps/map7.pgm");
  node->declare_parameter<std::string>("yaml_file_path", "/home/a/f1_localization/src/als_ros/maps/map7.yaml");
  node->declare_parameter<std::string>("frame_id", "map");
  node->declare_parameter<std::string>("topic_name", "map");

  std::string pgm_path, yaml_path, frame_id, topic_name;
  node->get_parameter("pgm_file_path", pgm_path);
  node->get_parameter("yaml_file_path", yaml_path);
  node->get_parameter("frame_id", frame_id);
  node->get_parameter("topic_name", topic_name);

  // Publisher (transient_local -> latched-like)
  auto map_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  // YAML
  YAML::Node y = YAML::LoadFile(yaml_path);
  const double resolution = y["resolution"].as<double>();
  const auto origin = y["origin"].as<std::vector<double>>();
  const double occ_th = y["occupied_thresh"].as<double>();
  const double free_th = y["free_thresh"].as<double>();
  const int unknown_value = y["unknown_value"] ? y["unknown_value"].as<int>() : 205;

  // PGM
  std::vector<uint8_t> pgm_data;
  int w = 0, h = 0, maxval = 255;
  if (!read_pgm_file(pgm_path, pgm_data, w, h, maxval)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load PGM: %s", pgm_path.c_str());
    return -1;
  }
  if (maxval != 255) {
    RCLCPP_WARN(node->get_logger(), "PGM maxval is %d (not 255). Scaling as if 255.", maxval);
  }

  // Build OccupancyGrid
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = frame_id;
  msg.info.resolution = resolution;
  msg.info.width  = static_cast<uint32_t>(w);
  msg.info.height = static_cast<uint32_t>(h);

  const double ox = origin.size() >= 1 ? origin[0] : 0.0;
  const double oy = origin.size() >= 2 ? origin[1] : 0.0;
  const double oyaw = origin.size() >= 3 ? origin[2] : 0.0;

  msg.info.origin.position.x = ox;
  msg.info.origin.position.y = oy;
  msg.info.origin.position.z = 0.0;

  double qx, qy, qz, qw;
  yaw_to_quat(oyaw, qx, qy, qz, qw);
  msg.info.origin.orientation.x = qx;
  msg.info.origin.orientation.y = qy;
  msg.info.origin.orientation.z = qz;
  msg.info.origin.orientation.w = qw;

  msg.info.map_load_time = node->get_clock()->now();      // fixed timestamp basis
  msg.header.stamp       = msg.info.map_load_time;        // fixed stamp for static map

  // Pixel -> occupancy
  msg.data.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
  for (int yrow = 0; yrow < h; ++yrow) {
    const int ry = h - 1 - yrow; // keep image Y flip
    for (int xcol = 0; xcol < w; ++xcol) {
      const std::size_t src = static_cast<std::size_t>(xcol) + static_cast<std::size_t>(ry) * static_cast<std::size_t>(w);
      const std::size_t dst = static_cast<std::size_t>(xcol) + static_cast<std::size_t>(yrow) * static_cast<std::size_t>(w);

      const uint8_t raw = pgm_data[src];
      if (raw == static_cast<uint8_t>(unknown_value)) { msg.data[dst] = -1; continue; }

      const double pixel = static_cast<double>(raw) * (255.0 / static_cast<double>(maxval));
      const double occ   = (255.0 - pixel) / 255.0;

      if (occ > occ_th)       msg.data[dst] = 100;
      else if (occ < free_th) msg.data[dst] = 0;
      else                    msg.data[dst] = -1;
    }
  }

  // One-shot publish (transient_local ensures late-joiners receive it)
  map_pub->publish(msg);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
