#include "rclcpp/rclcpp.hpp"
#include "aura_msg/msg/actuator_outputs.hpp"
#include "aura_msg/msg/parameter.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "L1_control.h"

#include <cmath>
#include <vector>
#include <algorithm>
#include <array>

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
          x(0.0), y(0.0), psi(0.0), u(0.0), v(0.0), r(0.0), v_int(0.0), uu_int(0.0),
          last_steering(0.0), last_thrust(0.0),
          max_steer(1000), max_thrust(700), max_thrust_diff(500), max_steer_diff(500),
          state({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
          state_estim({0.0, 0.0}),
          param_filtered({0.0, 0.0}),
          param_estim({0.0, 0.0}),
          x_t_plus({0.0, 0.0}),
          updated_param_estim({0.0, 0.0}),
          updated_param_filtered({0.0, 0.0}),
          L1_thruster({0.0, 0.0})
    {
        // Publishers
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/actuator_outputs", 10);

        // Subscribers
        subscriber_state_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/ekf/estimated_state", 10, std::bind(&ActuatorPublisher::state_callback, this, std::placeholders::_1));

        subscriber_params_ = this->create_subscription<aura_msg::msg::Parameter>(
            "/control_parameters", 10,
            std::bind(&ActuatorPublisher::parameters_callback, this, std::placeholders::_1));

        // Timer for periodic publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ActuatorPublisher::timer_callback, this));
    }

private:
    void state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Extract states from message
        x = msg->data[0];
        y = msg->data[1];
        psi = msg->data[2];
        u = msg->data[3];
        v = msg->data[4];
        r = msg->data[5];
    }

    void parameters_callback(const aura_msg::msg::Parameter::SharedPtr msg)
    {
        max_steer = msg->max_steer;
        max_steer_diff = msg->max_steer_diff;
        max_thrust_diff = msg->max_thrust_diff;
        max_thrust = msg->max_thrust;
    }
    
    double bow_switch(double s) {
        if (s >= 1)
        {
            return 1;

        }
        else if (s <=1 && s>= -1)
        {
            return 0;
        }
        else
        {
            return -1;
        }

    }    

    void timer_callback()
    {

        uu_int = uu_int + 0.1*u;
        v_int = v_int + 0.1*v;
        double s_v = v + 0.5*v_int;
        double s_u = u + 0.5*uu_int;

        // Update state array with latest values
        state = {x, y, psi, u, v, r, s_u, last_steering};

        double dt = 0.1; // Sampling time
        double w_cutoff = 1.0; // Filter cutoff frequency

        // Call L1 Adaptive Control function
        L1Control::L1_control(state, state_estim, param_filtered, dt, param_estim, w_cutoff,
                            x_t_plus, updated_param_estim, updated_param_filtered, L1_thruster);

        // Update estimated and filtered parameters
        state_estim = x_t_plus;
        param_estim = updated_param_estim;
        param_filtered = updated_param_filtered;
        
        // std::cout << "state estim: " << state_estim << std::endl;

        // Extract adaptive control outputs
        double steer = L1_thruster[1];  // Adaptive steering command
        double thrust = L1_thruster[0]; // Adaptive thrust command
        double bow = -1*bow_switch(s_v/2);
        RCLCPP_INFO(this->get_logger(), "VS: %.2f, bow: %.2f", s_v, bow);


        // Apply rate limiting
        // steer = clamp(last_steering + clamp(steer - last_steering, -max_steer_diff, max_steer_diff), -max_steer, max_steer);
        // thrust = clamp(last_thrust + clamp(thrust - last_thrust, -max_thrust_diff, max_thrust_diff), -max_thrust, max_thrust);

        // Store last control values
        last_steering = steer;
        last_thrust = thrust;

        // Create and publish actuator message
        std_msgs::msg::Float64MultiArray actuator_msg;
        actuator_msg.data = {steer, thrust, bow, 0.0};

        publisher_->publish(actuator_msg);
    }

    // ROS2 Publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

    // ROS2 Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_state_;
    rclcpp::Subscription<aura_msg::msg::Parameter>::SharedPtr subscriber_params_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // System states
    double x, y, psi, u, v, r, v_int, uu_int;
    double last_steering, last_thrust;
    double max_steer, max_thrust, max_steer_diff, max_thrust_diff;

    // L1 Adaptive Control variables
    std::array<double, 8> state;
    std::array<double, 2> state_estim;
    std::array<double, 2> param_filtered;
    std::array<double, 2> param_estim;
    std::array<double, 2> x_t_plus;
    std::array<double, 2> updated_param_estim;
    std::array<double, 2> updated_param_filtered;
    std::array<double, 2> L1_thruster;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActuatorPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
