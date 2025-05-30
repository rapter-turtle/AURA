#include "rclcpp/rclcpp.hpp"
#include "aura_msg/msg/actuator_outputs.hpp"
#include "aura_msg/msg/waypoint.hpp"
#include "aura_msg/msg/parameter.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


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
          k(0), x(0.0), y(0.0), u(0.0), v(0.0), r(0.0), Xu(0.081), LLOS(0.0), psi(0.0), received_(false),
          acceptance_radius(3.0), Kp(300.0), Kd(0.0), Kup(10.0), Kud(0.0), Kui(0.1), max_I(1),
          desired_velocity(5/1.94384), max_steer(100), max_thrust(70), max_thrust_diff(0.5), max_steer_diff(0.5)
    {
        // RCLCPP_INFO(this->get_logger(), "Initializing ActuatorPublisher...");

        // Publishers
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/actuator_outputs", 10);
        utm_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ship/utm", 10);

        // Subscribers
        subscriber_state_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/ekf/estimated_state", 10, std::bind(&ActuatorPublisher::state_callback, this, std::placeholders::_1));

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

        diff_thrust_before = 0.0;
        before_error_angle = 0.0;
        last_steering = 0.0;
        last_thrust = 0.0;
        before_velocity_e = 0.0;
        I_thrust = 0.0;
    }

private:

    void state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Extract yaw (psi) from quaternion
        x = msg->data[0];
        y = msg->data[1];
        psi = msg->data[2];
        u = msg->data[3];
        v = msg->data[4];
        r = msg->data[5];
        
        gps_received_ = true;
        publishUtmCoordinates();

        imu_received_ = true;
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
            }
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
        desired_velocity = msg->desired_velocity/1.94384;
        max_steer = msg->max_steer;
        max_steer_diff = msg->max_steer_diff;
        max_thrust_diff = msg->max_thrust_diff;
        Kup = msg->kup;
        Kud = msg->kud;
        Kui = msg->kui;
        max_thrust = msg->max_thrust;

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
            double pwm = 3.9 * thrust + 1550;
            // double pwm = 5.0 * thrust + 1500;
            return clamp(pwm, 1500.0, 2000.0); // Ensure PWM is within the bounds
        }
    }

    void timer_callback()
    {

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
            LLOS = LOS;
 
            double error_angle = LOS;
            double steer_input = Kp * error_angle + Kd * (error_angle - before_error_angle);
            steer_input = clamp(steer_input, -max_steer, max_steer);
            before_error_angle = error_angle;


            double velocity_e = u - desired_velocity;
            I_thrust = I_thrust + velocity_e*0.1;
            I_thrust = clamp(I_thrust, -70.0, 70.0);
            // double proposed_thrust = (100.0*desired_velocity);  // Example calculation for thrust
            double proposed_thrust = (5.0*desired_velocity + Xu*u- Kup*velocity_e- Kud*(velocity_e - before_velocity_e) - Kui*I_thrust);  // Example calculation for thrust
            // double proposed_thrust = (Xu*u - Kup*velocity_e - Kud*(velocity_e - before_velocity_e) - Kui*I_thrust)/0.0002;  // Example calculation for thrust
            
            proposed_thrust = sqrt(clamp(proposed_thrust, 0.0, max_thrust*max_thrust));
            before_velocity_e = velocity_e;
            RCLCPP_INFO(this->get_logger(), "vel_e =%f, proposed_thrust =%f, I_thrust =%f", velocity_e, proposed_thrust, I_thrust);
            // RCLCPP_INFO(this->get_logger(), "vel_e =%f, proposed_thrust =%f, I_thrust =%f", velocity_e, proposed_thrust, I_thrust);


            // Apply rate limiting
            double steer_change = steer_input - last_steering;
            steer_change = clamp(steer_change, -max_steer_diff, max_steer_diff);
            double steer = last_steering + steer_change;

            double thrust_change = proposed_thrust - last_thrust;
            thrust_change = clamp(thrust_change, -max_thrust_diff, max_thrust_diff);
            double thrust = last_thrust + thrust_change;

            // Store the last commands
            last_steering = steer;
            last_thrust = thrust;

            // Convert to PWM signals
            // double pwm_steer = convertSteeringToPwm(steer);

            double pwm_steer = convertSteeringToPwm(steer);
            double pwm_thrust = convertThrustToPwm(thrust);

            // auto actuator_msg = std_msg::msg::Float64MultiaRRAY();
            std_msgs::msg::Float64MultiArray actuator_msg;
            // std::fill(actuator_msg.actuator.begin(), actuator_msg.actuator.end(), 0.0f);
            actuator_msg.data.push_back(pwm_steer);// = [pwm_steer, pwm_thrust];
            actuator_msg.data.push_back(pwm_thrust);
            actuator_msg.data.push_back(0.0);
            actuator_msg.data.push_back(0.0);
            // actuator_msg.actuator[3] = pwm_thrust;
            RCLCPP_INFO(this->get_logger(), "WPT #=%d", k);
            RCLCPP_INFO(this->get_logger(), "steer=%.2f, thrust=%.2f", steer, thrust);
            RCLCPP_INFO(this->get_logger(), "LOS=%.2f, Distance=%.2f", LOS*180/M_PI, distance_to_wpt);
            RCLCPP_INFO(this->get_logger(), "u=%.2f m/s, %.2f knots", u, u*1.94384);


            publisher_->publish(actuator_msg);
            // printf("pub2");
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

    void publishUtmCoordinates()
    {
        std_msgs::msg::Float64MultiArray utm_msg;
        utm_msg.data = {x, y, LLOS*180/M_PI};
        utm_publisher_->publish(utm_msg);
        // RCLCPP_INFO(this->get_logger(), "Published UTM Coordinates: x=%.2f, y=%.2f", x, y);
    }

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    // rclcpp::Publisher<aura_msg::msg::ActuatorOutputs>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr utm_publisher_;



    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_state_;
    rclcpp::Subscription<aura_msg::msg::Waypoint>::SharedPtr subscriber_waypoints_;
    rclcpp::Subscription<aura_msg::msg::Parameter>::SharedPtr subscriber_params_;
    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu_;
    // rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_gps_;

    bool imu_received_ = false;
    bool gps_received_ = false;


    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Data variables
    size_t k;
    double x, y, psi, u, v, r, LLOS;
    std::vector<std::pair<double, double>> waypoints;
    double acceptance_radius, Kp, Kd, Kup, Kud, Kui, Xu, desired_velocity, max_thrust, max_steer, diff_thrust_before, before_error_angle, max_thrust_diff, max_steer_diff, last_steering, last_thrust, before_velocity_e, I_thrust, max_I;
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
