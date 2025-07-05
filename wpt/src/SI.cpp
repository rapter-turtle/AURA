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

class SI : public rclcpp::Node
{
public:
    SI()
        : Node("actuator_publisher"),
          k(0), x(0.0), y(0.0), u(0.0), v(0.0), r(0.0), Xu(0.081), LLOS(0.0), psi(0.0), received_(false),
          acceptance_radius(0), Kp(0.0), Kd(0.0), Kup(0.0), Kud(0.0), Kui(0.0), max_I(0),
          desired_velocity(4), max_steer(100), max_thrust(70), max_thrust_diff(0.5), max_steer_diff(0.5)
    {

        // Publishers
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/actuator_outputs", 10);
        utm_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ship/utm", 10);

        // Subscribers
        subscriber_state_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/ekf/estimated_state", 10, std::bind(&SI::state_callback, this, std::placeholders::_1));

        subscriber_params_ = this->create_subscription<aura_msg::msg::Parameter>(
            "/control_parameters", 10,
            std::bind(&SI::parameters_callback, this, std::placeholders::_1));

        // Timer for periodic publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SI::timer_callback, this));

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


    void parameters_callback(const aura_msg::msg::Parameter::SharedPtr msg)
    {

        acceptance_radius = msg->acceptance_radius;
        Kp = msg->kp*3.141592/180;
        Kd = msg->kd;
        desired_velocity = msg->desired_velocity;
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
        {
            
            if (psi > M_PI)
                psi -= 2 * M_PI;
            else if (psi < -M_PI)
                psi += 2 * M_PI;

            LLOS = psi - Kud*M_PI/180.0;

            if (LLOS > M_PI)
                LLOS -= 2 * M_PI;
            else if (LLOS < -M_PI)
                LLOS += 2 * M_PI;


            // SI type
            double steer_input = 0;
            double proposed_thrust = 0;

            // SI type selection
            if (acceptance_radius == 1) // acceleration-deceleration
            {  
                if (max_I == 0)
                {
                    steer_input = max_steer;
                    proposed_thrust = desired_velocity;
                    if (last_thrust >= desired_velocity)
                    {
                        max_I = 1;
                    }
                }
                else if (max_I == 1)
                {
                    steer_input = max_steer;
                    proposed_thrust = Kui;
                    if (last_thrust <= Kui)
                    {
                        max_I = 0;
                    }
                }
                RCLCPP_INFO(this->get_logger(), "steer input =%f, proposed thrust =%f", steer_input , proposed_thrust);
            }
            else if (acceptance_radius == 2) // zig-zag
            {  
                // LLOS
                if (max_I == 0)
                {
                    steer_input = max_steer;
                    proposed_thrust = desired_velocity;
                    if (LLOS >= Kp)
                    {
                        max_I = 1;
                    }
                }
                else if (max_I == 1)
                {
                    steer_input = -max_steer;
                    proposed_thrust = desired_velocity;
                    if (LLOS <= -Kp)
                    {
                        max_I = 0;
                    }
                }

                // if (max_I == 0)
                // {
                //     steer_input = max_steer;
                //     proposed_thrust = desired_velocity;
                //     if (psi >= Kp)
                //     {
                //         max_I = 1;
                //     }
                // }
                // else if (max_I == 1)
                // {
                //     steer_input = -max_steer;
                //     proposed_thrust = desired_velocity;
                //     if (psi <= -Kp)
                //     {
                //         max_I = 0;
                //     }
                // }
                // RCLCPP_INFO(this->get_logger(), "steer input =%f, proposed thrust =%f", steer_input , proposed_thrust);
           
            }
            else if (acceptance_radius == 3) // circle
            {  
                steer_input = max_steer;
                proposed_thrust = desired_velocity;
            }
            else // Default case
            {  
                steer_input = 0;
                proposed_thrust = 0;
            }

            RCLCPP_INFO(this->get_logger(), "steer input =%f, proposed thrust =%f", steer_input , proposed_thrust);
            // Apply rate limiting
            double steer_change = steer_input - last_steering;
            steer_change = clamp(steer_change, -max_steer_diff, max_steer_diff);
            double steer = last_steering + steer_change;

            double thrust_change = proposed_thrust - last_thrust;
            thrust_change = clamp(thrust_change, -max_thrust_diff, max_thrust_diff);
            double thrust = last_thrust + thrust_change;

            RCLCPP_INFO(this->get_logger(), "steer =%f, thrust =%f", steer , thrust);
            // Store the last commands
            last_steering = steer;
            last_thrust = thrust;

            // Convert to PWM signals
            double pwm_steer = convertSteeringToPwm(steer);
            double pwm_thrust = convertThrustToPwm(thrust);

            // auto actuator_msg = std_msg::msg::Float64MultiaRRAY();
            std_msgs::msg::Float64MultiArray actuator_msg;
            // std::fill(actuator_msg.actuator.begin(), actuator_msg.actuator.end(), 0.0f);
            actuator_msg.data.push_back(pwm_steer);// = [pwm_steer, pwm_thrust];
            actuator_msg.data.push_back(pwm_thrust);
            actuator_msg.data.push_back(abs(50*Kd));
            actuator_msg.data.push_back(Kd);
            // actuator_msg.actuator[3] = pwm_thrust;
            RCLCPP_INFO(this->get_logger(), "SI type =%f", acceptance_radius);



            publisher_->publish(actuator_msg);
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
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr utm_publisher_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_state_;
    rclcpp::Subscription<aura_msg::msg::Parameter>::SharedPtr subscriber_params_;

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

    auto node = std::make_shared<SI>();
    rclcpp::spin(node);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down ROS2 node...");
    rclcpp::shutdown();
    return 0;
}
