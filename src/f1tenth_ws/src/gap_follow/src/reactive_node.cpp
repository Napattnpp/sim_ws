#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

// Dynamic parameter
#include <rcl_interfaces/msg/set_parameters_result.hpp>

// Visualizae
#include "visualization_msgs/msg/marker.hpp"

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

using std::placeholders::_1;

class ReactiveFollowGap : public rclcpp::Node {
public:
    ReactiveFollowGap() : Node("reactive_node_cpp") {
        // Topics
        std::string lidarscan_topic = "/scan";
        std::string drive_topic = "/drive";

        // --- Subscribers & Publishers --- //
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, _1));
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        // --- VISUALIZATIONS --- //
        virtual_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/virtual_scan", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/steering_marker", 10);    
        gap_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/gap_boundaries", 10);
        target_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/target_point", 10);
        bubble_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/safety_bubble", 10);

        // --- Dynamic Parameters --- //
        this->declare_parameter("moving_avg_window_", 1);
        this->declare_parameter("max_dist_", 6.0);                  // In meters
        this->declare_parameter("car_length_", 0.50);               // In meters
        this->declare_parameter("car_width_", 0.28);                // In meters
        this->declare_parameter("bubble_radius_", 0.22);             // In meters
        this->declare_parameter("min_obs_dist_threshold_", 1.2);    // In meters
        this->declare_parameter("disparity_threshold_", 0.5);       // In meters

        this->get_parameter("moving_avg_window_", moving_avg_window_);
        this->get_parameter("max_dist_", max_dist_);
        this->get_parameter("car_length_", car_length_);
        this->get_parameter("car_width_", car_width_);
        this->get_parameter("bubble_radius_", bubble_radius_);
        this->get_parameter("min_obs_dist_threshold_", min_obs_dist_threshold_);
        this->get_parameter("disparity_threshold_", disparity_threshold_);

        param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ReactiveFollowGap::parametersCallback, this, _1));

        front_view_start_idx_ = 0;
        front_view_end_idx_ = 0;
        angle_increment_ = 0.0;
        angle_min_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "Reactive Follow Gap Node Started");
    }

private:
    // ROS 2 objects
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    // --- VISUALIZATIONS --- //
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr virtual_scan_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gap_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bubble_pub_;
    // --- Dynamic Parameters --- //
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Class variables
    int moving_avg_window_;
    double max_dist_;
    double car_length_;
    double car_width_;
    double bubble_radius_;
    double min_obs_dist_threshold_;
    double disparity_threshold_;

    int front_view_start_idx_;
    int front_view_end_idx_;
    double angle_increment_;
    double angle_min_;

    double range_index_to_angle(int index) {
        return angle_min_ + index * angle_increment_;
    }

    int angle_to_index(double angle) {
        return static_cast<int>((angle - angle_min_) / angle_increment_);
    }

    double velocity_mapping(double angle, double gap_depth) {
        double velocity_min = 1.0;
        double velocity_max = 4.0;

        double gap_depth_step1 = 2.0;
        double velocity_step1 = 2.0;

        double gap_depth_step2 = 3.5;
        double velocity_step2 = 3.0;

        double gap_depth_step3 = 5.0;
        double velocity_step3 = velocity_max;

        double velocity = velocity_max;
        if (gap_depth < gap_depth_step1) {
            velocity = velocity_min;
        } else if (gap_depth < gap_depth_step2) {
            velocity = velocity_step1 + (gap_depth - gap_depth_step1) * (velocity_step2 - velocity_step1) / (gap_depth_step2 - gap_depth_step1);
        } else if (gap_depth < gap_depth_step3) {
            velocity = velocity_step2 + (gap_depth - gap_depth_step2) * (velocity_step3 - velocity_step2) / (gap_depth_step3 - gap_depth_step2);
        } else {
            velocity = velocity_step3;
        }

        // Adjust velocity based on steering angle
        if (std::abs(angle) > 20.0 * M_PI / 180.0) {
            velocity *= 0.5;
        } else if (std::abs(angle) > 10.0 * M_PI / 180.0) {
            velocity *= 0.75;
        }

        return velocity;
    }

    double steering_angle_to_velocity_mapping(double angle) {
        double curr_velocity = 1.0;
        if (std::abs(angle) < 10.0 * M_PI / 180.0) {
            curr_velocity = 0.63;
        } else if (std::abs(angle) < 20.0 * M_PI / 180.0) {
            curr_velocity = 0.4;
        } else {
            curr_velocity = 0.2;
        }
        return curr_velocity;
    }

    // --- PARAMETER CALLBACK FUNCTION --- //
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters) {
            if (param.get_name() == "moving_avg_window_" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                moving_avg_window_ = param.as_int();
                RCLCPP_INFO(this->get_logger(), "Updated moving_avg_window_ to: %d", moving_avg_window_);
            } 
            else if (param.get_name() == "max_dist_" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                max_dist_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated max_dist_ to: %.2f", max_dist_);
            }
            else if (param.get_name() == "car_length_" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                car_length_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated car_length_ to: %.2f", car_length_);
            }
            else if (param.get_name() == "car_width_" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                car_width_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated car_width_ to: %.2f", car_width_);
            }
            else if (param.get_name() == "bubble_radius_" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                bubble_radius_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated bubble_radius_ to: %.2f", bubble_radius_);
            }
            else if (param.get_name() == "min_obs_dist_threshold_" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                min_obs_dist_threshold_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated min_obs_dist_threshold_ to: %.2f", min_obs_dist_threshold_);
            }
            else if (param.get_name() == "disparity_threshold_" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                disparity_threshold_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated disparity_threshold_ to: %.2f", disparity_threshold_);
            }
        }

        return result;
    }

    std::vector<double> preprocess_lidar(const std::vector<float>& ranges) {
        std::vector<double> proc_ranges(ranges.size());

        // Convert nan values to zero, and higher than max_dist to max_dist
        for (size_t i = 0; i < ranges.size(); ++i) {
            if (std::isnan(ranges[i])) {
                proc_ranges[i] = 0.0;
            } else if (std::isinf(ranges[i]) || ranges[i] > max_dist_) {
                proc_ranges[i] = max_dist_;
            } else {
                proc_ranges[i] = ranges[i];
            }
        }

        // Apply a moving average filter
        std::vector<double> smoothed_ranges(proc_ranges.size(), 0.0);
        int window = moving_avg_window_;
        int pad = window / 2;

        for (size_t i = 0; i < proc_ranges.size(); ++i) {
            double sum = 0.0;
            for (int j = -pad; j <= pad; ++j) {
                int idx = i + j;
                if (idx >= 0 && idx < static_cast<int>(proc_ranges.size())) {
                    sum += proc_ranges[idx];
                }
            }
            smoothed_ranges[i] = sum / window;
        }

        return smoothed_ranges;
    }

    void bubble_extend(std::vector<double>& ranges, int center_idx, bool extend_value) {
        if (ranges[center_idx] < bubble_radius_) return;

        double dtheta = std::asin(bubble_radius_ / ranges[center_idx]);
        int d_index = static_cast<int>(dtheta / angle_increment_);
        int start_idx = center_idx - d_index;
        int end_idx = center_idx + d_index;

        for (int i = start_idx; i < end_idx; ++i) {
            if (i >= 0 && i < static_cast<int>(ranges.size())) {
                if (extend_value) {
                    ranges[i] = std::min(ranges[i], ranges[center_idx]);
                } else {
                    ranges[i] = 0.0;
                }
            }
        }
    }

    void find_deepest_valid_gap(const std::vector<double>& free_space_ranges, int& best_start_i, int& best_end_i) {
        double max_gap_depth = 0;
        best_start_i = front_view_start_idx_;
        best_end_i = front_view_start_idx_;
        
        int gap_length = 0;
        double gap_depth = 0;
        double gap_min_depth = 1000.0;

        for (int i = front_view_start_idx_; i < front_view_end_idx_; ++i) {
            if (free_space_ranges[i] > min_obs_dist_threshold_) {
                gap_length += 1;
                gap_depth += free_space_ranges[i];
                gap_min_depth = std::min(gap_min_depth, free_space_ranges[i]);
            } else {
                double gap_width = gap_length * gap_min_depth * angle_increment_;
                if (gap_width > 0.8 * car_width_) {
                    double mean_gap_depth = gap_depth / gap_length;
                    if (mean_gap_depth > max_gap_depth) {
                        max_gap_depth = mean_gap_depth;
                        best_start_i = i - gap_length;
                        best_end_i = i;
                    }
                }
                gap_length = 0;
                gap_depth = 0;
                gap_min_depth = 1000.0;
            }
        }

        // Check the last gap
        if (gap_length > 0) {
            double gap_width = gap_length * gap_min_depth * angle_increment_;
            if (gap_width > car_width_) {
                double mean_gap_depth = gap_depth / gap_length;
                if (mean_gap_depth > max_gap_depth) {
                    max_gap_depth = mean_gap_depth;
                    best_start_i = front_view_end_idx_ - gap_length;
                    best_end_i = front_view_end_idx_;
                }
            }
        }
    }

    void find_max_gap(const std::vector<double>& free_space_ranges, int& best_start_i, int& best_end_i) {
        int max_gap = 0;
        best_start_i = front_view_start_idx_;
        best_end_i = front_view_start_idx_;
        int gap = 0;

        for (int i = front_view_start_idx_; i < front_view_end_idx_; ++i) {
            if (free_space_ranges[i] > min_obs_dist_threshold_) {
                gap += 1;
            } else {
                if (gap > max_gap) {
                    max_gap = gap;
                    best_start_i = i - gap;
                    best_end_i = i;
                }
                gap = 0;
            }
        }

        // Check the last gap
        if (gap > max_gap) {
            best_start_i = front_view_end_idx_ - gap;
            best_end_i = front_view_end_idx_;
        }
    }

    int find_best_point(int start_i, int end_i, const std::vector<double>& ranges) {
        (void)ranges; // unused in original python script as well
        // Go towards the center of the gap (65% towards start)
        return static_cast<int>(start_i * 0.65 + 0.35 * end_i);
    }

    std::vector<double> disparity_extending(const std::vector<double>& ranges) {
        std::vector<double> virtual_ranges = ranges;
        for (int i = front_view_start_idx_ + 1; i < front_view_end_idx_; ++i) {
            if (std::abs(ranges[i] - ranges[i - 1]) > disparity_threshold_) {
                if (ranges[i] < ranges[i - 1]) {
                    bubble_extend(virtual_ranges, i, true);
                } else {
                    bubble_extend(virtual_ranges, i - 1, true);
                }
            }
        }
        return virtual_ranges;
    }

    // ========================================== //
    //           VISUALIZATION HELPERS            //
    // ========================================== //
    void publish_virtual_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, const std::vector<double>& virtual_ranges) {
        auto virtual_scan = sensor_msgs::msg::LaserScan();
        virtual_scan.header = scan_msg->header;
        virtual_scan.angle_min = scan_msg->angle_min;
        virtual_scan.angle_max = scan_msg->angle_max;
        virtual_scan.angle_increment = scan_msg->angle_increment;
        virtual_scan.time_increment = scan_msg->time_increment;
        virtual_scan.scan_time = scan_msg->scan_time;
        virtual_scan.range_min = scan_msg->range_min;
        virtual_scan.range_max = scan_msg->range_max;
        
        // Convert std::vector<double> back to std::vector<float> for the message
        std::vector<float> float_ranges(virtual_ranges.begin(), virtual_ranges.end());
        virtual_scan.ranges = float_ranges;
        this->virtual_scan_pub_->publish(virtual_scan);
    }

    void publish_steering_marker(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, double steering_angle, double speed) {
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

        if (speed >= 3.0) {
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; // Green
        } else if (speed >= 1.5) {
            marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0; // Yellow
        } else {
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; // Red
        }

        this->marker_pub_->publish(marker);
    }

    // Modified to accept virtual_ranges as an argument
    void publish_gap_boundaries(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, int gap_start_i, int gap_length, const std::vector<double>& virtual_ranges) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = scan_msg->header.frame_id;
        marker.header.stamp = this->now();
        marker.ns = "gap_boundaries";
        marker.id = 1; 
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST; 
        marker.action = visualization_msgs::msg::Marker::ADD;

        double start_angle = scan_msg->angle_min + (gap_start_i * scan_msg->angle_increment);
        double start_dist = virtual_ranges[gap_start_i];
        geometry_msgs::msg::Point p1;
        p1.x = start_dist * std::cos(start_angle);
        p1.y = start_dist * std::sin(start_angle);
        p1.z = 0.0;

        int gap_end_i = std::min(static_cast<int>(virtual_ranges.size()) - 1, gap_start_i + gap_length);
        double end_angle = scan_msg->angle_min + (gap_end_i * scan_msg->angle_increment);
        double end_dist = virtual_ranges[gap_end_i];
        geometry_msgs::msg::Point p2;
        p2.x = end_dist * std::cos(end_angle);
        p2.y = end_dist * std::sin(end_angle);
        p2.z = 0.0;

        marker.points.push_back(p1);
        marker.points.push_back(p2);

        marker.scale.x = 0.2; marker.scale.y = 0.2; marker.scale.z = 0.2;
        marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.8; marker.color.b = 0.3;

        this->gap_marker_pub_->publish(marker);
    }

    void publish_target_point(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, int target_index, const std::vector<double>& virtual_ranges) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = scan_msg->header.frame_id;
        marker.header.stamp = this->now();
        marker.ns = "target_point";
        marker.id = 2; 
        marker.type = visualization_msgs::msg::Marker::SPHERE; 
        marker.action = visualization_msgs::msg::Marker::ADD;

        double target_angle = scan_msg->angle_min + (target_index * scan_msg->angle_increment);
        double target_dist = virtual_ranges[target_index];

        marker.pose.position.x = target_dist * std::cos(target_angle);
        marker.pose.position.y = target_dist * std::sin(target_angle);
        marker.pose.position.z = 0.0;

        // Make it slightly larger and bright Red
        marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3;
        marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;

        this->target_pub_->publish(marker);
    }

    void publish_bubble(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, int center_index, double distance) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = scan_msg->header.frame_id;
        marker.header.stamp = this->now();
        marker.ns = "safety_bubble";
        marker.id = 3;
        marker.type = visualization_msgs::msg::Marker::CYLINDER; // A flat disk
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Calculate the (x, y) position of the closest point
        double angle = scan_msg->angle_min + (center_index * scan_msg->angle_increment);
        marker.pose.position.x = distance * std::cos(angle);
        marker.pose.position.y = distance * std::sin(angle);
        marker.pose.position.z = 0.0; // Keep it flat on the ground

        // RViz scale is diameter, so we multiply the radius by 2
        marker.scale.x = bubble_radius_ * 2.0; 
        marker.scale.y = bubble_radius_ * 2.0; 
        marker.scale.z = 0.05; // Make it a thin disk

        // Color it semi-transparent Orange/Red
        marker.color.a = 0.4; // 40% opaque
        marker.color.r = 1.0; 
        marker.color.g = 0.3; 
        marker.color.b = 0.0;

        this->bubble_pub_->publish(marker);
    }

    // ========================================== //
    //               LIDAR CALLBACK               //
    // ========================================== //
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr data) {
        std::vector<double> proc_ranges = preprocess_lidar(data->ranges);

        angle_increment_ = data->angle_increment;
        angle_min_ = data->angle_min;
        front_view_start_idx_ = angle_to_index(-M_PI / 2.0); // -90 deg
        front_view_end_idx_ = angle_to_index(M_PI / 2.0);    //  90 deg

        // Extend disparities
        std::vector<double> virtual_ranges = disparity_extending(proc_ranges);

        // Find closest point to LiDAR
        int min_index = front_view_start_idx_;
        double min_value = virtual_ranges[front_view_start_idx_];
        for (int i = front_view_start_idx_; i < front_view_end_idx_; ++i) {
            if (virtual_ranges[i] < min_value) {
                min_value = virtual_ranges[i];
                min_index = i;
            }
        }

        double angle = 0.0;
        double velocity = steering_angle_to_velocity_mapping(angle);

        if (min_value != max_dist_) {
            bubble_extend(virtual_ranges, min_index, false);

            int start_i, end_i;
            find_deepest_valid_gap(virtual_ranges, start_i, end_i);
            int best_index = find_best_point(start_i, end_i, virtual_ranges);

            if ((end_i - start_i) < 5) {
                // means no gap - just go straight
                angle = 0.0;
            } else {
                angle = range_index_to_angle(best_index);
            }

            velocity = velocity_mapping(angle, virtual_ranges[best_index]);

            // --- VISUALIZATIONS --- //
            publish_virtual_scan(data, virtual_ranges);
            publish_steering_marker(data, angle, velocity);
            publish_gap_boundaries(data, start_i, (end_i - start_i), virtual_ranges);
            if (min_value != max_dist_) {
                publish_target_point(data, best_index, virtual_ranges);
                publish_bubble(data, min_index, min_value);
            }

        }

        // Publish Drive message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header = data->header;
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = velocity;
        drive_pub_->publish(drive_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReactiveFollowGap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
