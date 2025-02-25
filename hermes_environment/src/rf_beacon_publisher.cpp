#include <sstream>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "rclcpp_components/register_node_macro.hpp"

class RFBeaconPublisher : public rclcpp::Node{
public:
    explicit RFBeaconPublisher(const rclcpp::NodeOptions & options): 
    Node("rf_beacon_publisher_node", options),
    generator_(std::random_device{}()){
        // Topic parameter to publish IR intensity vector to
        std::string publisher_topic_ = this->declare_parameter("publisher_topic", "rf_signal");

        // Subscription topics parameter
        std::string subscription_topic_ = this->declare_parameter("subscription_topic", "model_states");

        // Create tf2 listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publisher for RF signal data
        rf_signal_pub_ = this->create_publisher<std_msgs::msg::String>(publisher_topic_, rclcpp::SensorDataQoS().reliable());

        // Subscribe to Gazebo's /model_states to get beacon position
        model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            subscription_topic_, rclcpp::SensorDataQoS(),
            std::bind(&RFBeaconPublisher::model_states_callback, this, std::placeholders::_1));

        // Publish rate parameter in Hz
        const double publish_rate = this->declare_parameter("publish_rate", 10.0);

        path_loss_ = this->declare_parameter("path_loss", 2.0);
        noise_mean_ = this->declare_parameter("noise_mean", 0.0);
        noise_std_ = this->declare_parameter("noise_std", 1.0);

        // Timer to get robot position and publish RF signal
        timer_ = rclcpp::create_timer(
            this,
            this->get_clock(),
            rclcpp::Duration(std::chrono::duration<double>(1 / publish_rate)),
            std::bind(&RFBeaconPublisher::publish_rf_signal, this)
        );
    }

private:
    void model_states_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg){
        if (!beacon_position_initialized_) {
            // Get beacon position once (assuming beacon is static)
            for (size_t i = 0; i < msg->name.size(); ++i){
                if (msg->name[i] == this->get_name()){
                    beacon_position_ = msg->pose[i].position;
                    beacon_position_initialized_ = true;
                    break;
                }
            }
        }
    }

    void publish_rf_signal(){
        if (!beacon_position_initialized_){
            RCLCPP_WARN(this->get_logger(), "Beacon position not initialized yet.");
            return;
        }

        try{
            // Get the transform from "odom" to "base_link"
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "odom", "base_link", tf2::TimePointZero);

            double robot_x = transform.transform.translation.x;
            double robot_y = transform.transform.translation.y;
            double robot_z = transform.transform.translation.z;

            // Compute signal strength
            double signal_strength = calculate_signal_strength(
                robot_x, robot_y, robot_z,
                beacon_position_.x, beacon_position_.y, beacon_position_.z);

            // Apply Gaussian noise to the signal strength
            signal_strength += generate_gaussian_noise();

            // Publish RF signal data
            auto msg = std::make_shared<std_msgs::msg::String>();
            std::ostringstream oss;
            oss << "Beacon=" << this->get_name() << ";SignalStrength=" << signal_strength;
            msg->data = oss.str();

            rf_signal_pub_->publish(*msg);
        }
        catch (const tf2::TransformException &ex){
            //RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }

    double calculate_signal_strength(double robot_x, double robot_y, double robot_z,
                                     double beacon_x, double beacon_y, double beacon_z){
        // Compute Euclidean distance between robot and beacon
        double dx = robot_x - beacon_x;
        double dy = robot_y - beacon_y;
        double dz = robot_z - beacon_z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        // RSSI calculation (decays with distance)
        double rssi = -10 * this->path_loss_ * std::log10(distance);

        return rssi;
    }

    // Function to generate Gaussian noise
    double generate_gaussian_noise() {
        std::normal_distribution<double> distribution(this->noise_mean_, this->noise_std_); 
        return distribution(generator_);
    }

    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rf_signal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::Point beacon_position_;
    bool beacon_position_initialized_ = false;
    double path_loss_;
    double noise_mean_;
    double noise_std_;

    // Random number generator and Gaussian distribution
    std::default_random_engine generator_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(RFBeaconPublisher)