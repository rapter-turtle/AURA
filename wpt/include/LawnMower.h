#ifndef LAWN_MOWER_PATTERN_H
#define LAWN_MOWER_PATTERN_H

#include <vector>
#include <utility>
#include <iostream>
#include <cmath>  // For trigonometric functions

class LawnMowerPattern
{
private:
    double distance;   // Distance between waypoints
    double height_;         // Height of the area to cover
    double width_;          // Width of the area to cover
    int num_patterns_;      // Number of 'ㄹ' patterns to generate
    double tilt_angle_;     // Angle in degrees to tilt the pattern

    std::vector<std::pair<double, double>> waypoints_; // Stores generated waypoints

public:
    // Constructor
    LawnMowerPattern(double distance, double height, double width, int num_patterns = 1, double tilt_angle = 0.0)
        : distance(distance), height_(height), width_(width), num_patterns_(num_patterns), tilt_angle_(tilt_angle)
    {
    }

    // Function to rotate a point (x, y) by tilt_angle around the origin (0, 0)
    void rotatePoint(double& x, double& y) const
    {
        // Convert tilt angle from degrees to radians
        double angle_rad = tilt_angle_ * M_PI / 180.0;

        // Rotation matrix transformation
        double x_new = x * cos(angle_rad) - y * sin(angle_rad);
        double y_new = x * sin(angle_rad) + y * cos(angle_rad);

        // Update the point
        x = x_new;
        y = y_new;
    }

    // Generate waypoints in a 'ㄹ' shaped pattern, repeated for 'num_patterns' times
    void generateWaypoints()
    {
        waypoints_.clear(); // Clear any existing waypoints

        double vertical_step = height_ / distance; // Vertical step size (height-wise)
        double horizontal_step = width_ / distance; // Horizontal step size (width-wise)
        
        for (int i = 0; i < vertical_step; ++i)
        {
            waypoints_.emplace_back(0, i * distance);  // Offset by the pattern index
        }

        // Generate 'num_patterns' of 'ㄹ' shaped patterns
        for (int pattern = 0; pattern < num_patterns_; ++pattern)
        {
            // Horizontal movement at the start of the pattern
            for (int i = 0; i < horizontal_step; ++i)
            {
                waypoints_.emplace_back(2*pattern*width_ + i * distance, height_);  // Offset by the pattern index
            }

            // Move vertically down for the current pattern
            for (int i = 0; i < vertical_step; ++i)
            {
                waypoints_.emplace_back((2*pattern+1)*width_, height_ - i * distance);
            }

            // Move horizontally (left to right)
            for (int i = 0; i < horizontal_step; ++i)
            {
                waypoints_.emplace_back((2*pattern+1)*width_ + i * distance, 0);  // Offset by the pattern index
            }

            // Move vertically up for the current pattern
            for (int i = 0; i < vertical_step; ++i)
            {
                waypoints_.emplace_back(2*(pattern+1)*width_, i * distance);
            }
        }

        waypoints_.emplace_back(2*(num_patterns_)*width_, height_);
        
        // Rotate all waypoints
        for (auto& waypoint : waypoints_)
        {
            rotatePoint(waypoint.first, waypoint.second);  // Rotate each point
        }
    }

    // Get generated waypoints
    const std::vector<std::pair<double, double>>& getWaypoints() const
    {
        return waypoints_;
    }

    // Set parameters dynamically
    void setParameters(int distance, double height, double width, int num_patterns, double tilt_angle)
    {
        distance = distance;
        height_ = height;
        width_ = width;
        num_patterns_ = num_patterns;
        tilt_angle_ = tilt_angle;
    }
};

#endif // LAWN_MOWER_PATTERN_H
