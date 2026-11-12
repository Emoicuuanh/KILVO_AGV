// Chuyển PointCloud2 (velodyne gazebo plugin) -> livox_ros_driver/CustomMsg để KILVO chạy đúng
// chế độ Livox Mid360 (lidar_type: 1) như tác giả đã kiểm chứng.
//
// offset_time: Gazebo chụp cả vòng quét TỨC THỜI (không có méo do chuyển động), nên mặc định
// offset_time = 0 cho mọi điểm -> KILVO không khử méo nhầm. (Bộ xử lý Velodyne của KILVO luôn tự
// suy thời gian từ góc phương vị và sẽ "khử méo" sai, vì vậy không dùng lidar_type Velodyne.)
// ~time_mode:=azimuth gán offset theo góc phương vị trong ~scan_period chỉ để thử nghiệm.
#include <cmath>
#include <livox_ros_driver/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class LidarToLivox
{
public:
  explicit LidarToLivox(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  {
    pnh.param<std::string>("input_topic", in_topic_, "/lidar/points");
    pnh.param<std::string>("output_topic", out_topic_, "/livox/lidar");
    pnh.param<std::string>("time_mode", time_mode_, "zero");
    pnh.param<double>("scan_period", scan_period_, 0.1);
    pnh.param<int>("n_lines", n_lines_, 4);
    pnh.param<double>("min_range", min_range_, 0.1);
    pub_ = nh.advertise<livox_ros_driver::CustomMsg>(out_topic_, 10);
    sub_ = nh.subscribe(in_topic_, 10, &LidarToLivox::cb, this, ros::TransportHints().tcpNoDelay());
    ROS_INFO("lidar_to_livox: %s -> %s (time_mode=%s)", in_topic_.c_str(), out_topic_.c_str(), time_mode_.c_str());
  }

private:
  void cb(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    bool has_ring = false, has_int = false;
    for (const auto &f : msg->fields)
    {
      if (f.name == "ring") has_ring = true;
      if (f.name == "intensity") has_int = true;
    }
    const size_t n = size_t(msg->width) * msg->height;
    livox_ros_driver::CustomMsg out;
    out.header = msg->header;
    out.timebase = msg->header.stamp.toNSec();
    out.lidar_id = 0;
    out.points.reserve(n);

    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z");
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> ii;
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint16_t>> ir;
    if (has_int) ii.reset(new sensor_msgs::PointCloud2ConstIterator<float>(*msg, "intensity"));
    if (has_ring) ir.reset(new sensor_msgs::PointCloud2ConstIterator<uint16_t>(*msg, "ring"));

    const double r2min = min_range_ * min_range_;
    for (size_t k = 0; k < n; ++k, ++ix, ++iy, ++iz)
    {
      const float x = *ix, y = *iy, z = *iz;
      float inten = ii ? **ii : 100.f;
      uint16_t ring = ir ? **ir : 0;
      if (ii) ++(*ii);
      if (ir) ++(*ir);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      if (x * x + y * y + z * z < r2min) continue;
      livox_ros_driver::CustomPoint p;
      p.x = x; p.y = y; p.z = z;
      p.reflectivity = static_cast<uint8_t>(std::min(255.f, std::max(0.f, inten)));
      p.tag = 0x10;  // "normal point" như Livox
      p.line = static_cast<uint8_t>(ring % n_lines_);
      if (time_mode_ == "azimuth")
      {
        const double az = std::atan2(y, x) + M_PI;  // [0, 2pi)
        p.offset_time = static_cast<uint32_t>(az / (2 * M_PI) * scan_period_ * 1e9);
      }
      else
        p.offset_time = 0;
      out.points.push_back(p);
    }
    if (time_mode_ == "azimuth")
      std::sort(out.points.begin(), out.points.end(),
                [](const livox_ros_driver::CustomPoint &a, const livox_ros_driver::CustomPoint &b) { return a.offset_time < b.offset_time; });
    out.point_num = out.points.size();
    pub_.publish(out);
  }

  std::string in_topic_, out_topic_, time_mode_;
  double scan_period_, min_range_;
  int n_lines_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_to_livox");
  ros::NodeHandle nh, pnh("~");
  LidarToLivox node(nh, pnh);
  ros::spin();
  return 0;
}
