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

//  Kp = heading angle
//  Kd = steer
//  acceptance radius = LOS angle

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
          k(0), x(0.0), y(0.0), LLOS(0.0), psi(0.0), received_(false),
          LOS_angle(0.0), heading_angle(0.0), steering(0.0), fixed(30.0),
          default_thrust(30), max_steer(100), max_thrust_diff(0.5), max_steer_diff(0.5)
    {
        // RCLCPP_INFO(this->get_logger(), "Initializing ActuatorPublisher...");

        // Publishers
        publisher_ = this->create_publisher<aura_msg::msg::ActuatorOutputs>("/mavros/actuator_outputs", 10);
        utm_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ship/utm", 10);

        // Subscribers
        // subscriber_state_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        //     "/ekf/estimated_state", 10,
        //     std::bind(&ActuatorPublisher::estimated_state_callback, this, std::placeholders::_1));

        subscriber_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&ActuatorPublisher::imu_callback, this, std::placeholders::_1));

        subscriber_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix", 10, std::bind(&ActuatorPublisher::gps_callback, this, std::placeholders::_1));



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
        diff_thrust_before = 0.0;
        before_error_angle = 0.0;
        last_steering = 0.0;
        last_thrust = 0.0;

        // RCLCPP_INFO(this->get_logger(), "ActuatorPublisher node started successfully");
    }

private:

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract yaw (psi) from quaternion
        tf2::Quaternion quat(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        psi = yaw;

        // Normalize psi to be within [-pi, pi]
        if (psi > M_PI)
            psi -= 2 * M_PI;
        else if (psi < -M_PI)
            psi += 2 * M_PI;
        // RCLCPP_INFO(this->get_logger(), "psi=%.2f",psi);
        // Indicate that we have received imu data
        imu_received_ = true;
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (msg->status.status >= 0) // Check if GPS data is valid
        {
            double lat = msg->latitude;
            double lon = msg->longitude;

            // Convert latitude and longitude to UTM coordinates
            double utm_easting, utm_northing;
            int zone;
            bool northp;

            GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, utm_easting, utm_northing);

            // Assign the converted coordinates
            x = utm_easting;
            y = utm_northing;
            // RCLCPP_INFO(this->get_logger(), "x=%.2f, y=%.2f", x, y);
            // Indicate that we have received GPS data
            gps_received_ = true;
            publishUtmCoordinates();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Invalid GPS data received. Ignoring...");
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

        LOS_angle = msg->acceptance_radius;
        heading_angle = msg->kp;
        steering = msg->kd;
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
        if (!gps_received_ || !imu_received_)
        {
            RCLCPP_WARN(this->get_logger(), "GPS or IMU data not yet received. Skipping timer callback.");
            return;
        }



        if (psi > M_PI)
            psi -= 2 * M_PI;
        else if (psi < -M_PI)
            psi += 2 * M_PI;

        // Line of Sight (LOS) angle
        double LOS =  psi - heading_angle*M_PI/180;
        if (LOS > M_PI)
            LOS -= 2 * M_PI;
        else if (LOS < -M_PI)
            LOS += 2 * M_PI;
        LLOS = LOS;


        
        
        if (LOS >= LOS_angle)
        {
             fixed = steering;
        }
        else if (LOS < LOS_angle)
        {
             fixed = -steering;
        }
        
        double steer_input = fixed;
        steer_input = clamp(steer_input, -max_steer, max_steer);
        

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
        // RCLCPP_INFO(this->get_logger(), "WPT #=%d", k);
        RCLCPP_INFO(this->get_logger(), "steer=%.2f, thrust=%.2f", steer, thrust);
        // RCLCPP_INFO(this->get_logger(), "LOS=%.2f, Distance=%.2f", LOS*180/M_PI, distance_to_wpt);

        publisher_->publish(actuator_msg);

        
    }

    void publishUtmCoordinates()
    {
        std_msgs::msg::Float64MultiArray utm_msg;
        utm_msg.data = {x, y, LLOS*180/M_PI};
        utm_publisher_->publish(utm_msg);
        // RCLCPP_INFO(this->get_logger(), "Published UTM Coordinates: x=%.2f, y=%.2f", x, y);
    }

    // Publishers
    rclcpp::Publisher<aura_msg::msg::ActuatorOutputs>::SharedPtr publisher_;
    // rclcpp::Publisher<aura_msg::msg::ActuatorOutputs>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr utm_publisher_;



    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_state_;
    rclcpp::Subscription<aura_msg::msg::Waypoint>::SharedPtr subscriber_waypoints_;
    rclcpp::Subscription<aura_msg::msg::Parameter>::SharedPtr subscriber_params_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_gps_;

    bool imu_received_ = false;
    bool gps_received_ = false;


    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Data variables
    size_t k;
    double x, y, psi, LLOS, fixed;
    std::vector<std::pair<double, double>> waypoints;
    double heading_angle, LOS_angle, steering, default_thrust, max_steer, diff_thrust_before, before_error_angle, max_thrust_diff, max_steer_diff, last_steering, last_thrust;
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
