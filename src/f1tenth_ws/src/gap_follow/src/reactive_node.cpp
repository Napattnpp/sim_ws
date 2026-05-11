#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

using std::placeholders::_1;

class ReactiveFollowGap : public rclcpp::Node {
   public:
    ReactiveFollowGap() : Node("reactive_node_cpp") {
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(this->drive_topic, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(this->lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, _1));
        
        // --- VISUALIZATIONS --- //
        virtual_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/virtual_scan", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/steering_marker", 10);    
        gap_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/gap_boundaries", 10);
        
        this->declare_parameter("max_range", 6.0);
        this->declare_parameter("car_half_width", 0.20);
        this->declare_parameter("disparity_threshold", 0.3);
        this->declare_parameter("max_gap_threshold", 0.82);
        this->declare_parameter("depth_threshold_offset", 2.2);
    }

   private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    std::vector<double> processed_lidar;

    /// Create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    
    // --- VISUALIZATIONS --- //
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr virtual_scan_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gap_marker_pub_;

    // Helper 1: Preprocessor
    void preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, int start_i, int end_i) {
    	double max_range;
    	this->get_parameter("max_range", max_range);
    	
        if (this->processed_lidar.size() != scan_msg->ranges.size()) {
            this->processed_lidar.assign(scan_msg->ranges.size(), 0.0);
        }
        for (int i = start_i; i <= end_i; i++) {
            if (std::isnan(scan_msg->ranges[i]) || std::isinf(scan_msg->ranges[i]) || scan_msg->ranges[i] > max_range) {
                this->processed_lidar[i] = max_range;
            } else {
                this->processed_lidar[i] = scan_msg->ranges[i];
            }
        }
    }

    // Helper 2: The Disparity Extender
    void extend_disparities(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, int start_i, int end_i) {
    	std::vector<double> virtual_lidar = this->processed_lidar;
    	
        double car_half_width;
        double disparity_threshold;
        this->get_parameter("car_half_width", car_half_width);
        this->get_parameter("disparity_threshold", disparity_threshold);

        for (int i = start_i; i < end_i; i++) {
            double diff = this->processed_lidar[i] - this->processed_lidar[i+1];

            if (std::abs(diff) > disparity_threshold) {
                bool left_is_closer = diff < 0; 
                int closer_i = left_is_closer ? i : i + 1;
                double closer_dist = this->processed_lidar[closer_i];

                double angle_span = 0.0;
                if (closer_dist > car_half_width) {
                    angle_span = std::asin(car_half_width / closer_dist);
                } else {
                    angle_span = M_PI / 2.0; 
                }
                int num_indices = std::ceil(angle_span / scan_msg->angle_increment);

                if (left_is_closer) {
                    int extend_end = std::min(end_i, closer_i + num_indices);
                    for (int j = closer_i + 1; j <= extend_end; j++) {
                        virtual_lidar[j] = std::min(virtual_lidar[j], closer_dist);
                    }
                } else {
                    int extend_start = std::max(start_i, closer_i - num_indices);
                    for (int j = extend_start; j < closer_i; j++) {
                        virtual_lidar[j] = std::min(virtual_lidar[j], closer_dist);
                    }
                }
            }
        }
        this->processed_lidar = virtual_lidar;
    }

    // Helper 3: Find Max Gap
    std::pair<int, int> find_max_gap(int start_i, int end_i) {
    	double max_gap_threshold;
    	this->get_parameter("max_gap_threshold", max_gap_threshold);
    	
        int largest_starting_i = start_i;
        int longest_gap = 0;
        int curr_gap = 0;
        int curr_start = start_i;

        for (int i = start_i; i <= end_i; i++) {
            // Lowered threshold to prevent corner cutting panic
            if (this->processed_lidar[i] <= max_gap_threshold) {
                curr_gap = 0;
                curr_start = i + 1; 
            } else {
                curr_gap++;
                if (curr_gap > longest_gap) {
                    longest_gap = curr_gap;
                    largest_starting_i = curr_start;
                }
            }
        }
        return std::make_pair(largest_starting_i, longest_gap);
    }
    
    // Helper 4: Center of the Farthest Region
    int find_best_point(int start_i, int gap_distance) {
        double max_depth = 0;
        for (int i = start_i; i < start_i + gap_distance; i++) {
            if (this->processed_lidar[i] > max_depth) {
                max_depth = this->processed_lidar[i];
            }
        }

        if (max_depth == 0.0) return start_i + (gap_distance / 2);

        int current_start = -1;
        int current_length = 0;
        int best_start = -1;
        int max_length = 0;
        
        double depth_threshold_offset;
        this->get_parameter("depth_threshold_offset", depth_threshold_offset);
        
        double depth_threshold = max_depth - depth_threshold_offset;

        for (int i = start_i; i < start_i + gap_distance; i++) {
            if (this->processed_lidar[i] >= depth_threshold) {
                if (current_start == -1) current_start = i;
                current_length++;
            } else {
                if (current_length > max_length) {
                    max_length = current_length;
                    best_start = current_start;
                }
                current_start = -1;
                current_length = 0;
            }
        }
        
        if (current_length > max_length) {
            max_length = current_length;
            best_start = current_start;
        }

        return best_start + (max_length / 2);
    }
    
    // Helper 5: Publish Virtual LiDAR Scan
    void publish_virtual_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        auto virtual_scan = sensor_msgs::msg::LaserScan();
        virtual_scan.header = scan_msg->header;
        virtual_scan.angle_min = scan_msg->angle_min;
        virtual_scan.angle_max = scan_msg->angle_max;
        virtual_scan.angle_increment = scan_msg->angle_increment;
        virtual_scan.time_increment = scan_msg->time_increment;
        virtual_scan.scan_time = scan_msg->scan_time;
        virtual_scan.range_min = scan_msg->range_min;
        virtual_scan.range_max = scan_msg->range_max;
        
        std::vector<float> float_ranges(this->processed_lidar.begin(), this->processed_lidar.end());
        virtual_scan.ranges = float_ranges;
        this->virtual_scan_pub_->publish(virtual_scan);
    }

    // Helper 6: Publish Speed-Dynamic Steering Arrow
    void publish_steering_marker(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, double steering_angle, double speed) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = scan_msg->header.frame_id; 
        marker.header.stamp = this->now();
        marker.ns = "steering_target";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start_point, end_point;
        start_point.x = 0.0; start_point.y = 0.0; start_point.z = 0.0;

        double arrow_length = 2.0; 
        end_point.x = arrow_length * std::cos(steering_angle);
        end_point.y = arrow_length * std::sin(steering_angle);
        end_point.z = 0.0;

        marker.points.push_back(start_point);
        marker.points.push_back(end_point);

        marker.scale.x = 0.05; marker.scale.y = 0.1; marker.scale.z = 0.1;  
        marker.color.a = 1.0;

        // NEW: Dynamic Color based on speed
        if (speed >= 3.0) {
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; // Green (Fast)
        } else if (speed >= 1.5) {
            marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0; // Yellow (Medium)
        } else {
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; // Red (Slow)
        }

        this->marker_pub_->publish(marker);
    }

    // Helper 7: Publish Gap Goalposts
    void publish_gap_boundaries(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, int gap_start_i, int gap_length) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = scan_msg->header.frame_id;
        marker.header.stamp = this->now();
        marker.ns = "gap_boundaries";
        marker.id = 1; 
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST; // Allows rendering multiple spheres at once
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Calculate left goalpost (start of gap)
        double start_angle = scan_msg->angle_min + (gap_start_i * scan_msg->angle_increment);
        double start_dist = this->processed_lidar[gap_start_i];
        geometry_msgs::msg::Point p1;
        p1.x = start_dist * std::cos(start_angle);
        p1.y = start_dist * std::sin(start_angle);
        p1.z = 0.0;

        // Calculate right goalpost (end of gap)
        int gap_end_i = std::min((int)this->processed_lidar.size() - 1, gap_start_i + gap_length);
        double end_angle = scan_msg->angle_min + (gap_end_i * scan_msg->angle_increment);
        double end_dist = this->processed_lidar[gap_end_i];
        geometry_msgs::msg::Point p2;
        p2.x = end_dist * std::cos(end_angle);
        p2.y = end_dist * std::sin(end_angle);
        p2.z = 0.0;

        marker.points.push_back(p1);
        marker.points.push_back(p2);

        // Goalpost visual size and color (Magenta)
        marker.scale.x = 0.2; marker.scale.y = 0.2; marker.scale.z = 0.2;
        marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.8; marker.color.b = 0.3;

        this->gap_marker_pub_->publish(marker);
    }

    // Main Callback
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        double fov_angle = 85.0 * M_PI / 180.0;
        int start_i = std::max(0, (int)std::round((-fov_angle - scan_msg->angle_min) / scan_msg->angle_increment));
        int end_i = std::min((int)scan_msg->ranges.size() - 1, (int)std::round((fov_angle - scan_msg->angle_min) / scan_msg->angle_increment));

        preprocess_lidar(scan_msg, start_i, end_i);
        extend_disparities(scan_msg, start_i, end_i);
        std::pair<int, int> gap = find_max_gap(start_i, end_i);
        int best_angle_i = find_best_point(gap.first, gap.second);

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = scan_msg->angle_min + (best_angle_i * scan_msg->angle_increment);
        
        if (std::abs(drive_msg.drive.steering_angle) < (10.0 * M_PI / 180.0)) {
            drive_msg.drive.speed = 3.0;
        } else if (std::abs(drive_msg.drive.steering_angle) < (20.0 * M_PI / 180.0)) {
            drive_msg.drive.speed = 1.5;
        } else {
            drive_msg.drive.speed = 1.0;
        }
        
        this->publisher_->publish(drive_msg);
        
        // --- VISUALIZATIONS --- //
        publish_virtual_scan(scan_msg);
        publish_steering_marker(scan_msg, drive_msg.drive.steering_angle, drive_msg.drive.speed);
        publish_gap_boundaries(scan_msg, gap.first, gap.second);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
