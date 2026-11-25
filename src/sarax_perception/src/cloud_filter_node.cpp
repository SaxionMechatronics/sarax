// cloud_filter_node — camera-frame ROI filtering of the depth cloud (consumed by the vision-repo normal estimation).
//
// The ROI box is expressed in the camera "body" convention:
//   x = forward (depth), y = left, z = up
// Parameter camera_frame_type says how the incoming cloud is expressed:
//   "x_forward" : already in that convention (Gazebo rgbd_camera, camera_link)
//   "optical"   : ROS optical frame, x = right, y = down, z = forward
//                 (e.g. RealSense *_optical_frame); mapped before the ROI test.
// Points are published unchanged in the input frame.
//
// Keep a rectangular window around the blade so MLS/PCA do not process the
// whole scene. Tune with SARAX_CLOUD_* env vars in scripts/ros.sh.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <stdexcept>
#include <string>

class CloudFilter : public rclcpp::Node
{
public:
    CloudFilter() : Node("cloud_filter"), first_frame_(true)
    {
        declare_parameter("input_topic",  "/camera/points");
        declare_parameter("output_topic", "/camera/points_filtered");
        declare_parameter("max_range", 5.0);
        declare_parameter("min_x", 0.20);
        declare_parameter("max_x", 5.0);
        declare_parameter("max_abs_y", 1.5);
        declare_parameter("min_z", 0.0);
        declare_parameter("max_z", 1.5);
        declare_parameter("camera_frame_type", std::string("x_forward"));

        std::string in  = get_parameter("input_topic").as_string();
        std::string out = get_parameter("output_topic").as_string();
        max_range_ = get_parameter("max_range").as_double();
        min_x_     = get_parameter("min_x").as_double();
        max_x_     = get_parameter("max_x").as_double();
        max_abs_y_ = get_parameter("max_abs_y").as_double();
        min_z_     = get_parameter("min_z").as_double();
        max_z_     = get_parameter("max_z").as_double();
        const std::string frame_type = get_parameter("camera_frame_type").as_string();
        if (frame_type == "optical") {
            optical_ = true;
        } else if (frame_type != "x_forward") {
            throw std::invalid_argument(
                "camera_frame_type must be 'x_forward' or 'optical', got '" + frame_type + "'");
        }
        max_range2_  = static_cast<float>(max_range_ * max_range_);
        min_x_f_     = static_cast<float>(min_x_);
        max_x_f_     = static_cast<float>(max_x_);
        max_abs_y_f_ = static_cast<float>(max_abs_y_);
        min_z_f_     = static_cast<float>(min_z_);
        max_z_f_     = static_cast<float>(max_z_);

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            out, rclcpp::SystemDefaultsQoS());
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            in, rclcpp::SystemDefaultsQoS(),
            std::bind(&CloudFilter::callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "cloud_filter: %s → %s  frame=%s  range<=%.1f  x=[%.2f, %.2f]  |y|<=%.2f  z=[%.2f, %.2f]",
            in.c_str(), out.c_str(), optical_ ? "optical" : "x_forward",
            max_range_, min_x_, max_x_, max_abs_y_, min_z_, max_z_);
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (first_frame_ && !cloud->empty()) {
            first_frame_ = false;
            float zmin = 1e9f, zmax = -1e9f;
            float xmin = 1e9f, xmax = -1e9f;
            float ymin = 1e9f, ymax = -1e9f;
            size_t n_roi = 0;
            for (const auto &p : cloud->points) {
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
                xmin = std::min(xmin, p.x);
                xmax = std::max(xmax, p.x);
                ymin = std::min(ymin, p.y);
                ymax = std::max(ymax, p.y);
                zmin = std::min(zmin, p.z);
                zmax = std::max(zmax, p.z);
                if (inside_roi(p)) ++n_roi;
            }
            RCLCPP_INFO(get_logger(),
                "First frame: %zu pts  X=[%.3f, %.3f]  Y=[%.3f, %.3f]  Z=[%.3f, %.3f]  ROI pts=%zu",
                cloud->size(), xmin, xmax, ymin, ymax, zmin, zmax, n_roi);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        out_cloud->reserve(cloud->size());
        size_t rm_range = 0, rm_depth = 0, rm_width = 0, rm_height = 0;

        for (const auto &p : cloud->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            const pcl::PointXYZ b = to_body(p);
            if (b.x*b.x + b.y*b.y + b.z*b.z > max_range2_) { ++rm_range; continue; }
            if (b.x < min_x_f_ || b.x > max_x_f_)          { ++rm_depth; continue; }
            if (std::abs(b.y) > max_abs_y_f_)              { ++rm_width; continue; }
            if (b.z < min_z_f_ || b.z > max_z_f_)          { ++rm_height; continue; }
            out_cloud->points.push_back(p);
        }

        out_cloud->width    = out_cloud->points.size();
        out_cloud->height   = 1;
        out_cloud->is_dense = true;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "in=%zu  out=%zu  removed: range=%zu  depth=%zu  width=%zu  height=%zu",
            cloud->size(), out_cloud->size(), rm_range, rm_depth, rm_width, rm_height);

        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*out_cloud, out_msg);
        out_msg.header = msg->header;
        pub_->publish(out_msg);
    }

    // Map a point into the x-forward / y-left / z-up convention of the ROI box.
    pcl::PointXYZ to_body(const pcl::PointXYZ &p) const
    {
        if (!optical_) return p;
        return pcl::PointXYZ(p.z, -p.x, -p.y);
    }

    bool inside_roi(const pcl::PointXYZ &q) const
    {
        const pcl::PointXYZ p = to_body(q);
        return p.x*p.x + p.y*p.y + p.z*p.z <= max_range2_
            && p.x >= min_x_f_ && p.x <= max_x_f_
            && std::abs(p.y) <= max_abs_y_f_
            && p.z >= min_z_f_ && p.z <= max_z_f_;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
    double max_range_;
    double min_x_;
    double max_x_;
    double max_abs_y_;
    double min_z_;
    double max_z_;
    float max_range2_;
    float min_x_f_;
    float max_x_f_;
    float max_abs_y_f_;
    float min_z_f_;
    float max_z_f_;
    bool   first_frame_;
    bool   optical_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudFilter>());
    rclcpp::shutdown();
    return 0;
}
