#include "rclcpp/rclcpp.hpp"
#include "aura_msg/msg/actuator_outputs.hpp"
#include "aura_msg/msg/waypoint.hpp"
#include "aura_msg/msg/parameter.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <GeographicLib/UTMUPS.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

template <typename T>
constexpr const T& clamp(const T& value, const T& low, const T& high)
{
    return (value < low) ? low : (value > high ? high : value);
}

class ActuatorPublisher : public rclcpp::Node
{
public:
    ActuatorPublisher()
        : Node("actuator_publisher"),
          k(0), x(0.0), y(0.0), psi(0.0), received_(false),
          acceptance_radius(3.0), Kp(300.0), Kd(0.0),
          default_thrust(50), max_steer(100), max_thrust_diff(0.1), max_steer_diff(0.1)
    {
        // RCLCPP_INFO(this->get_logger(), "Initializing ActuatorPublisher...");

        // Publishers
        publisher_ = this->create_publisher<aura_msg::msg::ActuatorOutputs>("/mavros/actuator_outputs", 10);

        // Subscribers
        subscriber_state_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/ekf/estimated_state", 10,
            std::bind(&ActuatorPublisher::estimated_state_callback, this, std::placeholders::_1));

        subscriber_waypoints_ = this->create_subscription<aura_msg::msg::Waypoint>(
            "/waypoints", 10,
            std::bind(&ActuatorPublisher::waypoints_callback, this, std::placeholders::_1));

        subscriber_params_ = this->create_subscription<aura_msg::msg::Parameter>(
            "/control_parameters", 10,
            std::bind(&ActuatorPublisher::parameters_callback, this, std::placeholders::_1));

        // Timer for periodic publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ActuatorPublisher::timer_callback, this));

        // Initialize control parameters
        // acceptance_radius = 3.0;
        // Kp = 300.0;
        // Kd = 0.0;
        // default_thrust = 50;
        // max_steer = 100;
        diff_thrust_before = 0.0;
        before_error_angle = 0.0;
        last_steering = 0.0;
        last_thrust = 0.0;

        // RCLCPP_INFO(this->get_logger(), "ActuatorPublisher node started successfully");
    }

private:
    void estimated_state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 3)
        {
            double lat = msg->data[0];
            double lon = msg->data[1];
            psi = msg->data[2];
            // RCLCPP_INFO(this->get_logger(), "Received state: lat=%.2f, lon=%.2f, Psi=%.2f", lat, lon, psi);

            // Convert latitude and longitude to UTM coordinates
            double utm_easting, utm_northing;
            int zone;
            bool northp;

            GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, utm_easting, utm_northing);

            // Assign the converted coordinates
            x = utm_easting;
            y = utm_northing;

            // Normalize psi to be within [-pi, pi]
            if (psi > M_PI)
                psi -= 2 * M_PI; 
            else if (psi < -M_PI)
                psi += 2 * M_PI;

            // RCLCPP_INFO(this->get_logger(), "Received state: x=%.2f, y=%.2f, Psi=%.2f", x, y, psi);
            received_ = true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "State message does not have enough data. Ignoring...");
        }
    }

    void waypoints_callback(const aura_msg::msg::Waypoint::SharedPtr msg)
    {
        waypoints.clear();
        // RCLCPP_INFO(this->get_logger(), "Received waypoints message");

        if (msg->x_lat.size() == msg->y_long.size())
        {
            for (size_t i = 0; i < msg->x_lat.size(); i++)
            {
                double utm_easting, utm_northing;
                int zone;
                bool northp;

                // Convert lat/lon to UTM/UPS
                GeographicLib::UTMUPS::Forward(
                    msg->x_lat[i], msg->y_long[i], zone, northp, utm_easting, utm_northing
                );
                waypoints.emplace_back(utm_easting, utm_northing);

                // RCLCPP_INFO(this->get_logger(), "Waypoint %zu: Lat=%.6f, Lon=%.6f -> UTM Easting=%.2f, Northing=%.2f", i, msg->x_lat[i], msg->y_long[i], utm_easting, utm_northing);
            }
            // RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints", waypoints.size());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Mismatch in x_lat and y_long sizes. Ignoring waypoints.");
        }
    }

    void parameters_callback(const aura_msg::msg::Parameter::SharedPtr msg)
    {

        acceptance_radius = msg->acceptance_radius;
        Kp = msg->kp;
        Kd = msg->kd;
        default_thrust = msg->default_thrust;
        max_steer = msg->max_steer;
        max_steer_diff = msg->max_steer_diff;
        max_thrust_diff = msg->max_thrust_diff;


    }

    double convertSteeringToPwm(double steer) {
        // Map steering value to PWM based on the given formula
        const double pwm_center = 1500.0;

        if (steer >= 300) {
            // Steer above 300 maps directly to PWM 2000
            return 2000.0;
        } else if (steer >= 0 && steer < 300) {
            // Steer in the range [0, 300] maps linearly between PWM = 1500 and PWM = 2000
            return 1500.0 + (steer * 1.6667);
        } else if (steer >= -300 && steer < 0) {
            // Steer in the range [-300, 0] maps linearly between PWM = 1000 and PWM = 1500
            return 1500.0 + (steer * 1.6667);
        } else if (steer < -300) {
            // Steer below -300 maps directly to PWM 1000
            return 1000.0;
        }

    }

    // Convert thrust level to PWM signal
    double convertThrustToPwm(double thrust) {
        if (thrust <= 0) {
            return 1500; // Any value <= 0 thrust maps to PWM 1000
        } else {
            // Calculate PWM based on the thrust
            double pwm = 5.0 * thrust + 1500;
            return clamp(pwm, 1500.0, 2000.0); // Ensure PWM is within the bounds
        }
    }

    void timer_callback()
    {
        if (!received_)
        {
            RCLCPP_WARN(this->get_logger(), "State data not yet received. Skipping timer callback.");
            return;
        }

        if (k >= waypoints.size())
        {
            RCLCPP_INFO(this->get_logger(), "All waypoints reached. Resetting to first waypoint.");
            k = 0;
        }

        if (!waypoints.empty())
        {
            auto target_wpt = waypoints[k];
            double dx = target_wpt.first - x;
            double dy = target_wpt.second - y;
            double distance_to_wpt = std::sqrt(dx * dx + dy * dy);

            if (psi > M_PI)
                psi -= 2 * M_PI;
            else if (psi < -M_PI)
                psi += 2 * M_PI;

            // Line of Sight (LOS) angle
            double LOS = std::atan2(dy, dx) - psi;
            if (LOS > M_PI)
                LOS -= 2 * M_PI;
            else if (LOS < -M_PI)
                LOS += 2 * M_PI;

            double error_angle = LOS;
            double steer_input = Kp * error_angle + Kd * (error_angle - before_error_angle);
            steer_input = clamp(steer_input, -max_steer, max_steer);
            before_error_angle = error_angle;

            // Apply rate limiting
            double steer_change = steer_input - last_steering;
            steer_change = clamp(steer_change, -max_steer_diff, max_steer_diff);
            double steer = last_steering + steer_change;

            double proposed_thrust = default_thrust;  // Example calculation for thrust
            double thrust_change = proposed_thrust - last_thrust;
            thrust_change = clamp(thrust_change, -max_thrust_diff, max_thrust_diff);
            double thrust = last_thrust + thrust_change;

            // Store the last commands
            last_steering = steer;
            last_thrust = thrust;

            // Convert to PWM signals
            double pwm_steer = convertSteeringToPwm(steer);
            double pwm_thrust = convertThrustToPwm(thrust);

            auto actuator_msg = aura_msg::msg::ActuatorOutputs();
            std::fill(actuator_msg.actuator.begin(), actuator_msg.actuator.end(), 0.0f);
            actuator_msg.actuator[1] = pwm_steer;
            actuator_msg.actuator[3] = pwm_thrust;
            RCLCPP_INFO(this->get_logger(), "steer=%.2f, thrust=%.2f", steer, thrust);

            // actuator_msg.actuator[1] = static_cast<float>(1000); // Steering control
            // actuator_msg.actuator[3] = static_cast<float>(-1);            
            // RCLCPP_INFO(this->get_logger(), "dx=%.2f, dy=%.2f", dx, steer);

            publisher_->publish(actuator_msg);

            // RCLCPP_INFO(this->get_logger(), "###################################################################");
            // RCLCPP_INFO(this->get_logger(), "wpt: x=%.2f, y=%.2f", target_wpt.first, target_wpt.second);
            // RCLCPP_INFO(this->get_logger(), "Actuators: Steering=%.2f, Thrust=%.2f", steer, thrust);
            // RCLCPP_INFO(this->get_logger(), "Distance=%.2f, LOS=%.2f", distance_to_wpt, LOS);
            // RCLCPP_INFO(this->get_logger(), "dx=%.2f, dy=%.2f", dx, dy);

            // Check if waypoint reached
            if (distance_to_wpt < acceptance_radius)
            {
                k++;
                RCLCPP_INFO(this->get_logger(), "Waypoint %zu reached. Moving to next waypoint.", k - 1);
            }
        }
    }

    // Publishers
    rclcpp::Publisher<aura_msg::msg::ActuatorOutputs>::SharedPtr publisher_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_state_;
    rclcpp::Subscription<aura_msg::msg::Waypoint>::SharedPtr subscriber_waypoints_;
    rclcpp::Subscription<aura_msg::msg::Parameter>::SharedPtr subscriber_params_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Data variables
    size_t k;
    double x, y, psi;
    std::vector<std::pair<double, double>> waypoints;
    double acceptance_radius, Kp, Kd, default_thrust, max_steer, diff_thrust_before, before_error_angle, max_thrust_diff, max_steer_diff, last_steering, last_thrust;
    bool received_;
};

int main(int argc, char **argv)
{
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting ROS2 node...");
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ActuatorPublisher>();
    rclcpp::spin(node);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down ROS2 node...");
    rclcpp::shutdown();
    return 0;
}
