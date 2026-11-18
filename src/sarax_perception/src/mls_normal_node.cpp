// ============================================================
// MLS Normal Estimation Node for ROS2 Jazzy - v2
// Now with h (Gaussian parameter) exposed!
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <fstream>
#include <cmath>

class MLSNormalEstimation : public rclcpp::Node
{
public:
    MLSNormalEstimation() : Node("mls_normal_estimation")
    {
        this->declare_parameter("search_radius", 0.05);
        this->declare_parameter("polynomial_order", 2);
        this->declare_parameter("sqr_gauss_param", -1.0);
        this->declare_parameter("input_topic",
                                "/camera/points_filtered");
        this->declare_parameter("output_topic", "/mls_normals");
        this->declare_parameter("arrow_scale", 0.2);
        this->declare_parameter("arrow_step", 50);
        this->declare_parameter("voxel_size", 0.01);
        this->declare_parameter("csv_output_path", "");
        this->declare_parameter("max_range", 5.0);

        search_radius_ = this->get_parameter("search_radius").as_double();
        polynomial_order_ = this->get_parameter("polynomial_order").as_int();
        sqr_gauss_param_ = this->get_parameter("sqr_gauss_param").as_double();
        arrow_scale_ = this->get_parameter("arrow_scale").as_double();
        arrow_step_ = this->get_parameter("arrow_step").as_int();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        std::string input_topic =
            this->get_parameter("input_topic").as_string();
        output_topic_ =
            this->get_parameter("output_topic").as_string();

        // If h not set, default to search_radius^2
        if (sqr_gauss_param_ < 0)
        {
            sqr_gauss_param_ = search_radius_ * search_radius_;
        }

        // Must match ros_gz_bridge which publishes with Reliable (SystemDefaultsQoS)
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&MLSNormalEstimation::cloud_callback,
                      this, std::placeholders::_1));

        cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/mls_normal_markers", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MLSNormalEstimation::republish, this));

        done_ = false;

        RCLCPP_INFO(this->get_logger(),
                    "MLS Normal node v2 started");
        RCLCPP_INFO(this->get_logger(),
                    "  search_radius=%.4f", search_radius_);
        RCLCPP_INFO(this->get_logger(),
                    "  sqr_gauss_param (h^2)=%.6f  =>  h=%.4f",
                    sqr_gauss_param_, std::sqrt(sqr_gauss_param_));
        RCLCPP_INFO(this->get_logger(),
                    "  poly_order=%d, voxel=%.4f",
                    polynomial_order_, voxel_size_);
        RCLCPP_INFO(this->get_logger(),
                    "  arrow_scale=%.3f, step=%d",
                    arrow_scale_, arrow_step_);
    }

private:
    void cloud_callback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (done_)
            return;
        done_ = true;

        pcl::PointCloud<pcl::PointXYZ>::Ptr raw(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *raw);
        if (raw->points.empty())
        {
            done_ = false;
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Got frame: %zu points", raw->points.size());

        // Defensive filter: the blade ROI is done upstream by cloud_filter_node.
        // Keep this range check so MLS can also be run directly on another cloud.
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
            new pcl::PointCloud<pcl::PointXYZ>);
        float max_r2 = static_cast<float>(max_range_ * max_range_);
        for (const auto &p : raw->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            if (p.x * p.x + p.y * p.y + p.z * p.z > max_r2) continue;
            filtered->points.push_back(p);
        }
        filtered->width = filtered->points.size();
        filtered->height = 1;
        filtered->is_dense = true;
        raw = filtered;
        RCLCPP_INFO(this->get_logger(),
                    "After range filter (%.1fm): %zu points",
                    max_range_, raw->points.size());

        // Downsample
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        if (voxel_size_ > 0.0005)
        {
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(raw);
            vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            vg.filter(*cloud);
        }
        else
        {
            cloud = raw;
        }

        RCLCPP_INFO(this->get_logger(),
                    "After downsample: %zu points", cloud->points.size());

        // Set up MLS
        pcl::PointCloud<pcl::PointNormal>::Ptr mls_output(
            new pcl::PointCloud<pcl::PointNormal>);

        pcl::MovingLeastSquares<pcl::PointXYZ,
                                pcl::PointNormal>
            mls;
        mls.setInputCloud(cloud);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZ>());
        mls.setSearchMethod(tree);

        // MLS parameters
        mls.setSearchRadius(search_radius_);
        mls.setComputeNormals(true);
        mls.setPolynomialOrder(polynomial_order_);
        mls.setSqrGaussParam(sqr_gauss_param_);

        RCLCPP_INFO(this->get_logger(),
                    "Computing MLS (radius=%.4f, h=%.4f, poly=%d)...",
                    search_radius_, std::sqrt(sqr_gauss_param_),
                    polynomial_order_);

        auto start = this->now();
        mls.process(*mls_output);
        auto elapsed = (this->now() - start).seconds();

        RCLCPP_INFO(this->get_logger(),
                    "Computed %zu normals in %.2f seconds",
                    mls_output->points.size(), elapsed);

        // Flip normals toward camera
        int flipped = 0;
        for (size_t i = 0; i < mls_output->points.size(); i++)
        {
            auto &pt = mls_output->points[i];
            if (std::isnan(pt.normal_x))
                continue;

            float dot = pt.normal_x * (-pt.x) +
                        pt.normal_y * (-pt.y) +
                        pt.normal_z * (-pt.z);
            if (dot < 0.0f)
            {
                pt.normal_x = -pt.normal_x;
                pt.normal_y = -pt.normal_y;
                pt.normal_z = -pt.normal_z;
                flipped++;
            }
        }
        RCLCPP_INFO(this->get_logger(),
                    "Flipped %d normals toward camera", flipped);

        // Curvature stats
        float min_c = 1e9, max_c = 0, sum_c = 0;
        int valid = 0;
        for (size_t i = 0; i < mls_output->points.size(); i++)
        {
            float c = mls_output->points[i].curvature;
            if (!std::isnan(c))
            {
                min_c = std::min(min_c, c);
                max_c = std::max(max_c, c);
                sum_c += c;
                valid++;
            }
        }
        RCLCPP_INFO(this->get_logger(),
                    "Curvature: min=%.5f max=%.5f mean=%.5f (%d valid)",
                    min_c, max_c, valid > 0 ? sum_c / valid : 0.0f, valid);

        // Store cloud message
        pcl::toROSMsg(*mls_output, stored_cloud_msg_);
        stored_cloud_msg_.header = msg->header;

        // Build markers
        stored_markers_.markers.clear();

        visualization_msgs::msg::Marker del;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        del.header = msg->header;
        stored_markers_.markers.push_back(del);

        // Camera sphere
        visualization_msgs::msg::Marker cam;
        cam.header = msg->header;
        cam.ns = "camera";
        cam.id = 0;
        cam.type = visualization_msgs::msg::Marker::SPHERE;
        cam.action = visualization_msgs::msg::Marker::ADD;
        cam.pose.position.x = 0.0;
        cam.pose.position.y = 0.0;
        cam.pose.position.z = 0.0;
        cam.pose.orientation.w = 1.0;
        cam.scale.x = 0.02;
        cam.scale.y = 0.02;
        cam.scale.z = 0.02;
        cam.color.r = 0.0;
        cam.color.g = 1.0;
        cam.color.b = 0.0;
        cam.color.a = 1.0;
        cam.lifetime = rclcpp::Duration(0, 0);
        stored_markers_.markers.push_back(cam);

        // Blue arrows
        int id = 1;
        for (size_t i = 0; i < mls_output->points.size();
             i += arrow_step_)
        {
            const auto &pt = mls_output->points[i];
            if (std::isnan(pt.normal_x) ||
                std::isnan(pt.normal_y) ||
                std::isnan(pt.normal_z))
                continue;

            visualization_msgs::msg::Marker arrow;
            arrow.header = msg->header;
            arrow.ns = "mls_normals";
            arrow.id = id++;
            arrow.type =
                visualization_msgs::msg::Marker::ARROW;
            arrow.action =
                visualization_msgs::msg::Marker::ADD;

            geometry_msgs::msg::Point start;
            start.x = pt.x;
            start.y = pt.y;
            start.z = pt.z;

            geometry_msgs::msg::Point end;
            end.x = pt.x + pt.normal_x * arrow_scale_;
            end.y = pt.y + pt.normal_y * arrow_scale_;
            end.z = pt.z + pt.normal_z * arrow_scale_;

            arrow.points.push_back(start);
            arrow.points.push_back(end);

            arrow.scale.x = 0.005;   // shaft diameter
            arrow.scale.y = 0.012;   // head diameter
            arrow.scale.z = 0.0;

            arrow.color.r = 0.1;
            arrow.color.g = 0.3;
            arrow.color.b = 1.0;
            arrow.color.a = 1.0;
            arrow.lifetime = rclcpp::Duration(0, 0);

            stored_markers_.markers.push_back(arrow);
        }

        RCLCPP_INFO(this->get_logger(),
                    "Built %d arrows + camera marker", id);

        /*====================================================*/
        // Save normals to CSV (only if csv_output_path parameter is set)
        std::string csv_path = this->get_parameter("csv_output_path").as_string();
        if (!csv_path.empty()) {
            std::ofstream csv(csv_path);
            csv << "x,y,z,nx,ny,nz" << std::endl;
            for (size_t i = 0; i < mls_output->points.size(); i++)
            {
                auto &pt = mls_output->points[i];
                if (std::isnan(pt.normal_x))
                    continue;
                csv << pt.x << "," << pt.y << "," << pt.z << ","
                    << pt.normal_x << "," << pt.normal_y << ","
                    << pt.normal_z << std::endl;
            }
            csv.close();
            RCLCPP_INFO(this->get_logger(), "Saved normals to %s", csv_path.c_str());
        }
        /*====================================================*/

        RCLCPP_INFO(this->get_logger(),
                    "DONE! Republishing at 1 Hz.");
    }

    void republish()
    {
        if (!done_)
            return;
        cloud_publisher_->publish(stored_cloud_msg_);
        marker_publisher_->publish(stored_markers_);
    }

    double search_radius_;
    int polynomial_order_;
    double sqr_gauss_param_;
    double arrow_scale_;
    int arrow_step_;
    double voxel_size_;
    double max_range_;
    std::string output_topic_;
    bool done_;

    sensor_msgs::msg::PointCloud2 stored_cloud_msg_;
    visualization_msgs::msg::MarkerArray stored_markers_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<MLSNormalEstimation>());
    rclcpp::shutdown();
    return 0;
}
