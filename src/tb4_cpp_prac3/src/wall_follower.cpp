#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#define PI 3.14159265358
using rcl_interfaces::msg::ParameterType;

class WallFollower : public rclcpp::Node
{
public:
    WallFollower() : Node("wall_follower")
    {
        /*TODO TASK - MILESTONE # 4.1
            1. Declare all parameters used for configuring the "following distance", "following angle", and all control gains. Their default values should be given as well.
            2. Get all parameter values from the constructor, and save them to private class element variables.
            3. Print all parameter values here.
            4. Set the value of "following_angle_" after initialising all parameters
        */
        auto wall_side_desc = rcl_interfaces::msg::ParameterDescriptor{};
        wall_side_desc.description = "A positive vlaue indicates that the wall will be on the left side of the robot, otherwise on the right";
        auto buffer_zone_desc = rcl_interfaces::msg::ParameterDescriptor{};
        buffer_zone_desc.description = "A positive value used to determine whether the tracking control is on or off";

        // Declare parameters
        this->declare_parameter<double>("following_angle", 0);
        this->declare_parameter<double>("following_distance", 0.7);
        this->declare_parameter<int64_t>("wall_side", 1, wall_side_desc);
        this->declare_parameter<double>("buffer_zone", 0.4, buffer_zone_desc);
        this->declare_parameter<double>("forward_velocity", 0.4);
        this->declare_parameter<double>("angle_control_gain_1", 1.0);
        this->declare_parameter<double>("angle_control_gain_2", 1.0);
        this->declare_parameter<double>("distance_control_gain", 0.5);

        // Get parameter values
        this->get_parameter("following_angle", following_angle_);
        this->get_parameter("following_distance", following_distance_);
        this->get_parameter("wall_side", wall_side_);
        this->get_parameter("buffer_zone", buffer_zone_);
        this->get_parameter("forward_velocity", forward_velocity_);
        this->get_parameter("angle_control_gain_1", angle_control_gain_1_);
        this->get_parameter("angle_control_gain_2", angle_control_gain_2_);
        this->get_parameter("distance_control_gain", distance_control_gain_);

        // Print parameter values
        RCLCPP_INFO(this->get_logger(), "following_angle: %.2f", following_angle_);
        RCLCPP_INFO(this->get_logger(), "following_distance: %.2f", following_distance_);
        RCLCPP_INFO(this->get_logger(), "wall_side: %1ld", wall_side_);
        RCLCPP_INFO(this->get_logger(), "buffer_zone: %.2f", buffer_zone_);
        RCLCPP_INFO(this->get_logger(), "forward_velocity: %.2f", forward_velocity_);
        RCLCPP_INFO(this->get_logger(), "angle_control_gain_1: %.2f", angle_control_gain_1_);
        RCLCPP_INFO(this->get_logger(), "angle_control_gain_2: %.2f", angle_control_gain_2_);
        RCLCPP_INFO(this->get_logger(), "distance_control_gain: %.2f", distance_control_gain_);

        // Setting value for following angle
        if (wall_side_ > 0)
            following_angle_ = PI / 2;
        else
            following_angle_ = -PI / 2;

        /* TODO TASK - MILESTONE #4.3
            Initialise dynamic parameter handler by the rclcpp node method "add_on_set_parameters_callback"
        */

        dyn_params_handler_ = this->add_on_set_parameters_callback(
            std::bind(
                &WallFollower::dynamicParametersCallback,
                this, std::placeholders::_1));

        this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel_unstamped",
            rclcpp::SystemDefaultsQoS());
        using namespace std::placeholders;
        this->scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(&WallFollower::scan_callback, this, _1));
    }

private:
    std::recursive_mutex mutex_;
    // Define a command velocity publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    // Define a laser scan subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    /* TODO TASK - MILESTONE #4.2
        define dynamic parameter call back handle.
    */

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

    rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    double following_angle_;
    double following_distance_;
    int64_t wall_side_;
    double buffer_zone_;
    double forward_velocity_;
    double angle_control_gain_1_;
    double angle_control_gain_2_;
    double distance_control_gain_;
};

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    std::lock_guard<std::recursive_mutex> cfl(mutex_);
    /*TODO TASKS
        MILESTONE # 6.1. Process the received scan_msg to get the location of the closest object in robot's environment.
        NOTE: the four pillars of will be visible from the Lidar sensor, you have to remove the distance
        measurements of these four pillars by ignoring any measurement less than 0.2 meter.
    */
    auto valid_range = [scan_msg](float range)
    {
        return std::isfinite(range) && range >= 0.2f && range < scan_msg->range_max;
    };

    auto valid_distances = [valid_range](double a, double b)
    {
        if (!valid_range(a))
            return false; // if a is less than 0.2, choose b
        if (!valid_range(b))
            return true; // if b is less than 0.2, choose a
        return a < b;
    };
    // finds the minimum element in the ranges vector of the scan_msg. It returns an iterator (min_distance) pointing to the smallest value in the range.
    auto min_distance = std::min_element(scan_msg->ranges.begin(), scan_msg->ranges.end(), valid_distances); // chose closest object

    // Extracts the actual minimum value from the iterator obtained in the previous step.
    float min_value = *min_distance;

    // Calculates the index of the minimum value in the ranges vector by finding the distance between the beginning of the vector and the iterator pointing to the minimum value.
    int min_index = std::distance(scan_msg->ranges.begin(), min_distance);

    /*

        MILESTONE # 6.2. You have to calculate the bearing and the range of the closest object with respect to the robot frame. You have
        to check the LaserScan message definition, and how the Lidar sensor is mounted with respective to  the robot's coordinate.

    */

    float theta = scan_msg->angle_min + (min_index * scan_msg->angle_increment);

    float theta_o = theta + PI / 2;

    geometry_msgs::msg::Twist cmd_vel_msg;

    // Milestone 6.3.1 , If the robot is far away from the wall, it should move towards its nearest wall at a constant speed until the robot arrives at a distance of desired value + buffer zone, with respect to its closest wall.

    /*
The magic number 12 is from the simulation setup, i.e., the range of the lidar sensor is from
0.164 to 12.
If min_value<12, the lidar sensor has a valid measurement.
*/

    if (min_value < 12)
    {
        // The robot moving towards to the closed target at speed of forward_velocity_
        if (min_value > (following_distance_ + buffer_zone_))
        {
            cmd_vel_msg.angular.z = 0;
            cmd_vel_msg.linear.x = forward_velocity_;
        }

        // Milestone 6.3.2 Next, the robot enter the wall follow mode with the control lawy in in Algorithm 1
        else
        {
            double theta_tilde;
            if (wall_side_ > 0)
            {
                theta_tilde = theta_o - PI / 2;
            }
            else
            {
                theta_tilde = theta_o + PI / 2;
            }

            // Assign constant linear velocity
            cmd_vel_msg.linear.x = forward_velocity_;

            double omega = 0.0;

            if (wall_side_ == 1) // wall on the left
            {
                if (std::abs(theta_tilde) > PI / 10)
                {
                    omega = angle_control_gain_1_ * theta_tilde + angle_control_gain_2_ * (min_value - following_distance_) * std::sin(theta_tilde) / theta_tilde;
                }
                else
                {
                    omega = angle_control_gain_1_ * theta_tilde + angle_control_gain_2_ * (min_value - following_distance_);
                }
            }
            else if (wall_side_ == -1) // wall on the right
            {
                if (std::abs(theta_tilde) > PI / 10)
                {
                    omega = angle_control_gain_1_ * theta_tilde - angle_control_gain_2_ * (min_value - following_distance_) * std::sin(theta_tilde) / theta_tilde;
                }
                else
                {
                    omega = angle_control_gain_1_ * theta_tilde - angle_control_gain_2_ * (min_value - following_distance_);
                }
            }

            cmd_vel_msg.angular.z = omega;
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No Object is Detected");
        cmd_vel_msg.linear.x = 0.0; // stop the robot
    }
    cmd_vel_publisher_->publish(cmd_vel_msg);
}

rcl_interfaces::msg::SetParametersResult
WallFollower::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
    std::lock_guard<std::recursive_mutex> cfl(mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    /*TODO TASK - MILESTONE #5.1
      Check whether update of a parameter in the node is requested, if yes and save the updated
      parameter value.
    */

    for (auto parameter : parameters)
    {
        const auto &param_type = parameter.get_type();
        const auto &param_name = parameter.get_name();
        if (param_type == ParameterType::PARAMETER_DOUBLE)
        {
            if (param_name == "following_distance")
            {
                following_distance_ = parameter.as_double();
                if (following_distance_ < 0.0)
                {
                    RCLCPP_WARN(this->get_logger(), "You've set following_distance_ to be negative,"
                                                    " this isn't allowed, so the value will be set to default.");
                    following_distance_ = 0.7;
                }
            }

            if (param_name == "following_angle")
            {
                following_angle_ = parameter.as_double();
            }

            if (param_name == "buffer_zone")
            {
                buffer_zone_ = parameter.as_double();
                if (buffer_zone_ < 0.0)
                {
                    RCLCPP_WARN(this->get_logger(), "You've set buffer_zone_ to be negative,"
                                                    " this isn't allowed, so the value will be set to default.");
                    buffer_zone_ = 0.4;
                }
            }

            if (param_name == "forward_velocity")
            {
                forward_velocity_ = parameter.as_double();
                if (forward_velocity_ < 0.0)
                {
                    RCLCPP_WARN(this->get_logger(), "You've set forward_velocity_ to be negative,"
                                                    " this isn't allowed, so the value will be set to default.");
                    forward_velocity_ = 0.4;
                }
            }

            if (param_name == "angle_control_gain_1")
            {
                angle_control_gain_1_ = parameter.as_double();
                if (angle_control_gain_1_ < 0.0)
                {
                    RCLCPP_WARN(this->get_logger(), "You've set angle_control_gain_1 to be negative,"
                                                    " this isn't allowed, so the value will be set to default.");
                    angle_control_gain_1_ = 1.0;
                }
            }

            if (param_name == "angle_control_gain_2")
            {
                angle_control_gain_2_ = parameter.as_double();
                if (angle_control_gain_2_ < 0.0)
                {
                    RCLCPP_WARN(this->get_logger(), "You've set angle_control_gain_2 to be negative,"
                                                    " this isn't allowed, so the value will be set to default.");
                    angle_control_gain_2_ = 1.0;
                }
            }

            if (param_name == "distance_control_gain")
            {
                distance_control_gain_ = parameter.as_double();
                if (distance_control_gain_ < 0.0)
                {
                    RCLCPP_WARN(this->get_logger(), "You've set distance_control_gain to be negative,"
                                                    " this isn't allowed, so the value will be set to default.");
                    distance_control_gain_ = 0.5;
                }
            }
        }

        if (param_type == ParameterType::PARAMETER_INTEGER)
        {
            if (param_name == "wall_side")
            {
                wall_side_ = parameter.as_int();
            }
        }
    }
    result.successful = true;
    return result;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollower>());
    rclcpp::shutdown();
    return 0;
}
