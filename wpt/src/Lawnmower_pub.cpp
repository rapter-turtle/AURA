#include "rclcpp/rclcpp.hpp"
#include "aura_msg/msg/waypoint.hpp"
#include "LawnMower.h"
#include <vector>
#include <utility>
#include <GeographicLib/UTMUPS.hpp>
#include <sstream>

class WaypointPublisher : public rclcpp::Node
{
public:
    WaypointPublisher()
        : Node("waypoint_publisher"),
          pattern_generator(5.0, 100.0, 100.0, 3, 0.0) // Default parameters for the pattern
    {
        // Publisher for GPS waypoints
        gps_publisher_ = this->create_publisher<aura_msg::msg::Waypoint>("/waypoints", 10);

        // Publisher for UTM waypoints
        utm_publisher_ = this->create_publisher<aura_msg::msg::Waypoint>("/waypoints_utm", 10);

        // Timer for periodic publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Publish every 0.1 seconds
            std::bind(&WaypointPublisher::publishWaypoints, this));

        RCLCPP_INFO(this->get_logger(), "Waypoint Publisher Node has been started.");

        // Define the start point (latitude and longitude)
        double start_lat = 37.1870503;
        double start_lon = 126.6463455;

        // Convert the start point to UTM coordinates
        double start_utm_x, start_utm_y;
        int zone;
        bool northp;

        GeographicLib::UTMUPS::Forward(start_lat, start_lon, zone, northp, start_utm_x, start_utm_y);
        RCLCPP_INFO(this->get_logger(), "Start UTM: x=%.3f, y=%.3f, zone=%d, hemisphere=%s",
                    start_utm_x, start_utm_y, zone, northp ? "North" : "South");

        // Generate waypoints using the LawnMowerPattern
        pattern_generator.generateWaypoints();
        const auto &generated_waypoints = pattern_generator.getWaypoints();

        // Convert generated waypoints to latitude/longitude and UTM
        for (const auto &point : generated_waypoints)
        {
            double utm_x = start_utm_x + point.first;
            double utm_y = start_utm_y + point.second;

            wpt_utm_x_.push_back(utm_x);
            wpt_utm_y_.push_back(utm_y);

            double lat = 0.0, lon = 0.0;
            try
            {
                GeographicLib::UTMUPS::Reverse(zone, northp, utm_x, utm_y, lat, lon);
                wpt_x_.push_back(lat);
                wpt_y_.push_back(lon);
            }
            catch (const GeographicLib::GeographicErr &e)
            {
                RCLCPP_ERROR(this->get_logger(), "GeographicLib error: %s", e.what());
            }
        }
    }

private:
    void publishWaypoints()
    {
        // Prepare the GPS waypoint message
        aura_msg::msg::Waypoint gps_msg;
        gps_msg.x_lat = wpt_x_;
        gps_msg.y_long = wpt_y_;
        gps_publisher_->publish(gps_msg);

        // Prepare the UTM waypoint message
        aura_msg::msg::Waypoint utm_msg;
        utm_msg.x_lat = wpt_utm_x_; // Use UTM x-coordinates
        utm_msg.y_long = wpt_utm_y_; // Use UTM y-coordinates
        utm_publisher_->publish(utm_msg);

        RCLCPP_INFO(this->get_logger(), "Published GPS waypoints and UTM waypoints.");
    }

    std::string formatVector(const std::vector<double> &vec)
    {
        std::ostringstream oss;
        for (size_t i = 0; i < vec.size(); ++i)
        {
            oss << vec[i];
            if (i < vec.size() - 1)
                oss << ", ";
        }
        return oss.str();
    }

    rclcpp::Publisher<aura_msg::msg::Waypoint>::SharedPtr gps_publisher_;
    rclcpp::Publisher<aura_msg::msg::Waypoint>::SharedPtr utm_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> wpt_x_;
    std::vector<double> wpt_y_;
    std::vector<double> wpt_utm_x_;
    std::vector<double> wpt_utm_y_;

    LawnMowerPattern pattern_generator; // Instance of the LawnMowerPattern class
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<WaypointPublisher>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
