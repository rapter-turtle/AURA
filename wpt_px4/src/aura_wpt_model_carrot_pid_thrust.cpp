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
          k(0), start_n(0.0), x(0.0), y(0.0), u(0.0), v(0.0), r(0.0), Xu(0.10531), LLOS(0.0), psi(0.0), received_(false), ll_thrust(0.0), delta(50.0),
          acceptance_radius(8.0), Kp(300.0), Kd(2000.0), Kup(0.2), Kud(0.0), Kui(0.05), max_I(1), get_vel(0.0), n(0.0),
          desired_velocity(0.0), max_steer(150.0), max_thrust(48), max_thrust_diff(0.04), max_steer_diff(30.0), before_proposed_thrust(0.0)
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
            std::chrono::milliseconds(333),
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

    void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // Extract yaw (psi) from quaternion
        get_vel = msg->data/1.94384;

        if (get_vel!= msg->data/1.94384)
        {
            n = 0.0;
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
        // int index = static_cast<int>(desired_velocity);  // Integer part of the velocity value
        // // Ensure the index is within bounds (0 to 20)
        // index = std::min(index, static_cast<int>(Kp_schedule.size()) - 1);
        // // Set the gains based on the velocity
        // Kp = Kp_schedule[index];
        // Kd = Kd_schedule[index];
        // max_steer = max_steer_schedule[index];
        // max_steer_diff = max_steer_diff_schedule[index];
        double du = desired_velocity*1.94384;
        // Kp, Kd, Kui
        if (du <= 10.0){
            Kp = 500.0;
            Kd = 1000.0;
            max_steer = 150.0;
            max_steer_diff = 7.0*3.5;
            Kui = 0.05;
            RCLCPP_INFO(this->get_logger(), "went in1");
        }
        else if (du > 10.0 && du <= 16.0 ){
            Kp = -60.0*(du - 10) + 500.0;
            Kd = 200.0;
            max_steer = 150.0;
            max_steer_diff = 15.0*3.5;
            Kui = 0.05;
            RCLCPP_INFO(this->get_logger(), "went in2");
        }
        else{
            Kp = 140.0;
            Kd = 200.0;
            max_steer = 150.0;
            max_steer_diff = 15.0*3.5;
            Kui = 0.05;
            RCLCPP_INFO(this->get_logger(), "went in3");
        }

        // Kup
        if (du <= 8.0){
            Kup = 0.5;
        }
        else if (du > 8.0 && du <= 13.0 ){
            Kup = -0.06*(du - 8.0) + 0.5;
        }
        else{
            Kup = 0.2;
        }        
        // RCLCPP_INFO(this->get_logger(), "Desired velocity : %.2f", du);

        // Optionally, log the updated values for debugging
        RCLCPP_INFO(this->get_logger(), "Kp=%.2f, Kd=%.2f, max_steer=%.2f, max_steer_diff=%.2f", Kp, Kd, max_steer, max_steer_diff);
        RCLCPP_INFO(this->get_logger(), "Kup=%.2f, Kui=%.2f", Kup, Kui);
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
            if (start_n == 0.0){
                start_n = 1;
                
                // Initialize variables to track the closest waypoint
                double min_distance = std::numeric_limits<double>::max();  // Start with a large value
                size_t closest_waypoint_index = 0;

                // Loop through all the waypoints to find the closest one
                for (size_t i = 0; i < waypoints.size(); ++i) {
                    double x_wpt = waypoints[i].first;
                    double y_wpt = waypoints[i].second;
                    
                    // Calculate Euclidean distance from the current position (x, y) to the waypoint (x_wpt, y_wpt)
                    double distance = std::sqrt((x - x_wpt) * (x - x_wpt) + (y - y_wpt) * (y - y_wpt));
                    
                    // If the current waypoint is closer, update the minimum distance and the closest waypoint index
                    if (distance < min_distance) {
                        min_distance = distance;
                        closest_waypoint_index = i;
                    }
                }

                double xp1 = waypoints[closest_waypoint_index].first;
                double yp1 = waypoints[closest_waypoint_index].second;
                double xp2 = waypoints[closest_waypoint_index + 1].first;
                double yp2 = waypoints[closest_waypoint_index + 1].second;
                double fp = (yp1 - yp2)/(xp1 - xp2);
                double xoc = (x + fp*y - fp*yp1 + fp*fp*xp1)/(1+fp*fp);
                double yoc = (xoc-xp1)*fp + yp1;

                double distance_check1 = std::sqrt((xp1 - xoc) * (xp1 - xoc) + (yp1 - yoc) * (yp1 - yoc));
                double distance_check2 = std::sqrt((xp1 - xp2) * (xp1 - xp2) + (yp1 - yp2) * (yp1 - yp2));
                if (distance_check1 <= distance_check2){
                    k = closest_waypoint_index;
                }
                else{
                    if (closest_waypoint_index == 0){
                        k = waypoints.size()-1;
                    }
                }
                


                // f= ()

            }

            auto target_wpt = waypoints[k];
            double dx = 0.0;
            double dy = 0.0;
            double distance_to_wpt = 0.0;

            double x1 = 0.0;
            double y1 = 0.0;
            double x2 = 0.0;
            double y2 = 0.0;
            double wpt_angle = 0.0;
            double f = 0.0;
            double xo = 0.0;
            double yo = 0.0;

            if (k >= 0 && k < waypoints.size()-1)
            {
                x1 = waypoints[k].first;
                y1 = waypoints[k].second;
                x2 = waypoints[k+1].first;
                y2 = waypoints[k+1].second;
            }
            else if (k == waypoints.size()-1){             
                x1 = waypoints[waypoints.size()-1].first;
                y1 = waypoints[waypoints.size()-1].second;
                x2 = waypoints[0].first;
                y2 = waypoints[0].second;
            }

            wpt_angle = std::atan2(y2-y1, x2-x1);
            f = (y2-y1)/(x2-x1);

            xo = (f*f*x1 - f*y1 + x + f*y)/(f*f + 1);
            yo = f*(xo - x1) + y1;
            
            dx = xo + delta*std::cos(wpt_angle) - x;
            dy = yo + delta*std::sin(wpt_angle) - y;

            distance_to_wpt = std::sqrt(dx * dx + dy * dy);
            double wpt_end_distance = std::sqrt((x - x2)*(x - x2) + (y - y2)*(y - y2));
            delta = 50.0;///////////////////////////////////////////////////////////////////// safe area
            if (wpt_end_distance <= delta){
                k = k + 1;
            }


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

            double dt = 0.33333;


            double error_angle = LOS;
            double steer_input = -Kp * error_angle - Kd * (error_angle - before_error_angle)*0.1/dt;
            steer_input = clamp(steer_input, -max_steer, max_steer);
            before_error_angle = error_angle;


            desired_velocity = get_vel*(1 - std::exp(-0.01*n))*(1 - 0.5*clamp((steer_input*steer_input)/150000, 0.0, 0.06));
            double velocity_e = 0.0;
            velocity_e = u - desired_velocity;
            double proposed_thrust = 0.0;
            
            // set_thrust = (0.10531*desired_velocity + 0.0181*desired_velocity*sqrt(desired_velocity*desired_velocity));

            // Thrust Regulation
            double safe_area = 40.0;    ///////////////////////////////////////////////////////////////////// safe area
            // double waypoint_start_distance = std::sqrt((xo - x1)*(xo - x1) + (yo - y1)*(yo - y1));
            double waypoint_start_distance = std::sqrt((x - x1)*(x - x1) + (y - y1)*(y - y1));
            // //Decrease mode 
            if (waypoint_start_distance <= safe_area){
                proposed_thrust = before_proposed_thrust;
                RCLCPP_INFO(this->get_logger(), "##### proposed thrust ######");
            }
            else{
                proposed_thrust = (0.21*desired_velocity + 0.0*desired_velocity*sqrt(desired_velocity*desired_velocity));
                before_proposed_thrust = proposed_thrust;
            }

            before_velocity_e = velocity_e;
 
            // Apply rate limiting
            double steer_change = steer_input - last_steering;
            steer_change = clamp(steer_change, -max_steer_diff, max_steer_diff);
            double steer = last_steering + steer_change;

            double thrust_change = proposed_thrust - last_thrust;
            thrust_change = clamp(thrust_change, -max_thrust_diff, max_thrust_diff);
            
            double thrust = 0.0;
            if (waypoint_start_distance <= safe_area){
                thrust = 1.0*(1-0.1*(1 - std::exp(-0.1*nn)))*ll_thrust;
                last_thrust = thrust;
            }
            else{
                thrust = last_thrust + thrust_change;
                last_thrust = thrust;
                ll_thrust = thrust;
                nn = 0;
            }            
            
            // double thrust = last_thrust + thrust_change;
            // double remap_thrust = thrust/(0.00058466*std::cos(0.0040635*steer));
            double remap_thrust = thrust/(0.00058466);
            if (remap_thrust <= 0)
            {
                remap_thrust = -sqrt(-remap_thrust);
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
            actuator_msg.data.push_back(LLOS);
            actuator_msg.data.push_back(0.0);

            // RCLCPP_INFO(this->get_logger(), "check=%.2f", clamp((steer_input*steer_input)/150000, 0.0, 0.3));
            RCLCPP_INFO(this->get_logger(), "WPT #=%d, distance = %2.f", k, wpt_end_distance);
            RCLCPP_INFO(this->get_logger(), "steer=%.2f, thrust=%.2f", steer, remap_thrust);
            // RCLCPP_INFO(this->get_logger(), "LOS=%.2f", LOS * 180 / M_PI);
            RCLCPP_INFO(this->get_logger(), "desired u =%.2f knots, u=%.2f m/s, %.2f knots", desired_velocity* 1.94384,u , u * 1.94384);
            RCLCPP_INFO(this->get_logger(), "Kp=%.2f, Kd=%.2f, max_steer=%.2f, max_steer_diff=%.2f",Kp, Kd, max_steer, max_steer_diff);
            RCLCPP_INFO(this->get_logger(), "I thrust=%.2f", I_thrust);

            publisher_->publish(actuator_msg);

            // Check if waypoint reached
            // if (distance_to_wpt < acceptance_radius)
            // {
            //     k++;
            //     RCLCPP_INFO(this->get_logger(), "Waypoint %zu reached. Moving to next waypoint.", k - 1);
            // }

            n = n + 1;
            nn = nn + 1;
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
        500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 
        500.0, 400.0, 400.0, 400.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 
        200.0
    };

    std::vector<double> Kd_schedule = {
        2200.0, 2200.0, 2200.0, 2200.0, 2200.0, 2200.0, 2200.0, 2200.0, 2200.0, 2200.0, 
        2200.0, 2200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 
        200.0
    };

    std::vector<double> max_steer_schedule = {
        150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 
        150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 
        150.0
    };

    std::vector<double> max_steer_diff_schedule = {
        7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 
        7.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 
        15.0
    };

    // Data variables
    size_t k;
    double x, y, psi, u, v, r, LLOS, n, nn, delta, start_n;
    std::vector<std::pair<double, double>> waypoints;
    double acceptance_radius, Kp, Kd, Kup, Kud, Kui, Xu, desired_velocity, max_thrust, max_steer, diff_thrust_before, before_error_angle, max_thrust_diff, max_steer_diff, last_steering, last_thrust, before_velocity_e, I_thrust, max_I, before_proposed_thrust;
    double get_vel, ll_thrust;
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
