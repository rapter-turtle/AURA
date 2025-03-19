#include "rclcpp/rclcpp.hpp"
#include "aura_msg/msg/actuator_outputs.hpp"
#include "aura_msg/msg/waypoint.hpp"
#include "aura_msg/msg/parameter.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
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
          k(0), x(0.0), y(0.0), u(0.0), v(0.0), r(0.0), Xu(0.10531), LLOS(0.0), psi(0.0), received_(false),
          acceptance_radius(3.0), Kp(0.0), Kd(0.0), Kup(1.0), Kud(0.0), Kui(0.01), max_I(1),
          desired_velocity(0.0), max_steer(100), max_thrust(70), max_thrust_diff(0.1), max_steer_diff(0.5)
    {
        // RCLCPP_INFO(this->get_logger(), "Initializing ActuatorPublisher...");

        // Publishers
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/actuator_outputs", 10);
        utm_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ship/utm", 10);

        // Subscribers
        subscriber_state_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/ekf/estimated_state", 10, std::bind(&ActuatorPublisher::state_callback, this, std::placeholders::_1));

        subscriber_desired_velocity_ = this->create_subscription<std_msgs::msg::Float64>(
            "/desired_velocity", 10, std::bind(&ActuatorPublisher::velocity_callback, this, std::placeholders::_1));

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
        param_estim = 0.0;
        state_estim = 0.0;
        param_filtered = 0.0; 
        before_param_filtered = 0.0;
        x_error = 0.0;
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

    void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // Extract yaw (psi) from quaternion
        desired_velocity = msg->data/1.94384;
        
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
        
        double vel = msg->desired_velocity/1.94384;
        int index = static_cast<int>(vel);

        acceptance_radius = msg->acceptance_radius;
        Kp_schedule[index] = msg->kp;
        Kd_schedule[index] = msg->kd;
        
        max_steer_schedule[index] = msg->max_steer;
        max_steer_diff_schedule[index] = msg->max_steer_diff;
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

    void update_gains_based_on_velocity(double desired_velocity)
    {
        // Determine the index based on the integer value of desired_velocity
        int index = static_cast<int>(desired_velocity);  // Integer part of the velocity value

        // Ensure the index is within bounds (0 to 20)
        index = std::min(index, static_cast<int>(Kp_schedule.size()) - 1);

        // Set the gains based on the velocity
        Kp = Kp_schedule[index];
        Kd = Kd_schedule[index];
        max_steer = max_steer_schedule[index];
        max_steer_diff = max_steer_diff_schedule[index];

        // Optionally, log the updated values for debugging
        RCLCPP_INFO(this->get_logger(), "Updated gains: Kp=%.2f, Kd=%.2f, max_steer=%.2f, max_steer_diff=%.2f", Kp, Kd, max_steer, max_steer_diff);
    }

    void timer_callback()
    {
        // Update gains based on the current desired velocity
        update_gains_based_on_velocity(desired_velocity);

        // Proceed with the rest of your existing control logic
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
            double steer_input = -Kp * error_angle - Kd * (error_angle - before_error_angle);
            steer_input = clamp(steer_input, -max_steer, max_steer);
            before_error_angle = error_angle;

            double velocity_e = 0.0;
            //Decrease mode 
            if (distance_to_wpt <= 10){
                velocity_e = u - 0.7*desired_velocity;
            }
            else{
                velocity_e = u - desired_velocity;
            }

            // velocity_e = u - desired_velocity

            // L1 Adaptive control
            double w_cutoff = 5;
            x_error = state_estim - u;
            double xdot = param_estim + param_filtered - Kup*velocity_e;
            state_estim = xdot*0.1 + state_estim;
            double gain = -1.0;
            double pi = (1/gain)*(exp(gain*0.1)-1.0);
            param_estim = -exp(gain*0.1)*x_error/pi;
            double disturb_max = 1.5;

            if (param_estim >= disturb_max){
                param_estim = disturb_max;
                state_estim = u;
            }
            else if (param_estim <= -disturb_max){
                param_estim = -disturb_max;
                state_estim = u;
            }

            if (param_filtered >= disturb_max){
                param_filtered = disturb_max;
                state_estim = u;
            }
            else if (param_filtered <= -disturb_max){
                param_filtered = -disturb_max;
                state_estim = u;
            }

            before_param_filtered = param_filtered;
            param_filtered = before_param_filtered*exp(-w_cutoff*0.1) - param_estim*(1-exp(-w_cutoff*0.1));
            
            // param_filtered = 0.0;

            double proposed_thrust = (0.10531 * u + 0.018405*u*sqrt(u*u + 0.00001) - Kup * velocity_e + param_filtered);  // Thrust calculation

            
            before_velocity_e = velocity_e;

            // Apply rate limiting
            double steer_change = steer_input - last_steering;
            steer_change = clamp(steer_change, -max_steer_diff, max_steer_diff);
            double steer = last_steering + steer_change;

            double thrust_change = proposed_thrust - last_thrust;
            thrust_change = clamp(thrust_change, -max_thrust_diff, max_thrust_diff);
            double thrust = last_thrust + thrust_change;
            double remap_thrust = thrust/(0.00058466*std::cos(0.0040635*steer));
            if (remap_thrust <= 0)
            {
                remap_thrust = sqrt(-remap_thrust);
            }
            else
            {
                remap_thrust = sqrt(remap_thrust);
            }


            // Store the last commands
            last_steering = steer;
            last_thrust = thrust;

            remap_thrust = clamp(remap_thrust, 0.0, max_thrust);
            // Convert to PWM signals
            double pwm_steer = convertSteeringToPwm(steer);
            double pwm_thrust = convertThrustToPwm(remap_thrust);

            std_msgs::msg::Float64MultiArray actuator_msg;
            actuator_msg.data.push_back(pwm_steer);
            actuator_msg.data.push_back(pwm_thrust);
            actuator_msg.data.push_back(0.0);
            actuator_msg.data.push_back(0.0);

            RCLCPP_INFO(this->get_logger(), "WPT #=%d", k);
            RCLCPP_INFO(this->get_logger(), "steer=%.2f, thrust=%.2f", steer, remap_thrust);
            RCLCPP_INFO(this->get_logger(), "LOS=%.2f, Distance=%.2f", LOS * 180 / M_PI, distance_to_wpt);
            RCLCPP_INFO(this->get_logger(), "desired u =%.2f knots, u=%.2f m/s, %.2f knots", desired_velocity* 1.94384,u , u * 1.94384);
            RCLCPP_INFO(this->get_logger(), "Kp=%.2f, Kd=%.2f, max_steer=%.2f, max_steer_diff=%.2f",Kp, Kd, max_steer, max_steer_diff);
            RCLCPP_INFO(this->get_logger(), "L1 thrust=%.2f", -param_filtered);

            publisher_->publish(actuator_msg);

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
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_desired_velocity_;
    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu_;
    // rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_gps_;

    bool imu_received_ = false;
    bool gps_received_ = false;


    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> Kp_schedule = {
        800.0, 800.0, 800.0, 800.0, 800.0, 800.0, 800.0, 800.0, 800.0, 800.0, 
        300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 
        300.0
    };

    std::vector<double> Kd_schedule = {
        2200.0, 2200.0, 2200.0, 2200.0, 2200.0, 2200.0, 2200.0, 2200.0, 2200.0, 2200.0, 
        2200.0, 2200.0, 2200.0, 2200.0, 2200.0, 2200.0, 2200.0, 2200.0, 2200.0, 2200.0, 
        2200.0
    };

    std::vector<double> max_steer_schedule = {
        300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 
        300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 
        300.0
    };

    std::vector<double> max_steer_diff_schedule = {
        10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 
        20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 
        20.0
    };

    // Data variables
    size_t k;
    double x, y, psi, u, v, r, LLOS;
    std::vector<std::pair<double, double>> waypoints;
    double acceptance_radius, Kp, Kd, Kup, Kud, Kui, Xu, desired_velocity, max_thrust, max_steer, diff_thrust_before, before_error_angle, max_thrust_diff, max_steer_diff, last_steering, last_thrust, before_velocity_e, I_thrust, max_I;
    double param_estim, param_filtered, before_param_filtered, x_error, state_estim;
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
