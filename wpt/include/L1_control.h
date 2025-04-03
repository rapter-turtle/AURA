#ifndef L1_CONTROL_HPP
#define L1_CONTROL_HPP

#include <array>
#include <cmath>
#include <vector>

class L1Control {
public:
    static double bow_switch(double s) {
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

    static double smc_sat(double s) {
        if (s >= 1)
        {
            return 1;

        }
        else if (s <=1 && s>= -1)
        {
            return s;
        }
        else
        {
            return -1;
        }

    }        

    static void L1_control(const std::array<double, 8>& state,
                           const std::array<double, 2>& state_estim,
                           std::array<double, 2>& param_filtered,
                           double dt,
                           std::array<double, 2>& param_estim,
                           double w_cutoff,
                           std::array<double, 2>& x_t_plus,
                           std::array<double, 2>& updated_param_estim,
                           std::array<double, 2>& updated_param_filtered,
                           std::array<double, 2>& L1_thruster)
    {
        // System Constants
        constexpr double M = 1;
        constexpr double I = 1;
        constexpr double Xu = 0.1;
        constexpr double Xuu = 0.05;
        constexpr double Nr = 0.081*2;
        constexpr double Nrrr = 0.05;
        constexpr double Yv = 0.06*2;
        constexpr double Yvv = 0.05;
        constexpr double Yr = 0.081*0.5;
        constexpr double Nv = 0.081*0.5;
        constexpr double dist = 0.3;
        constexpr double head_dist = 1.0;


        // Extract state variables
        double x = state[0];
        double y = state[1];
        double psi = state[2];
        double u = state[3];
        double v = state[4];
        double r = state[5];
        double s_u = state[6];


        double x_actual_min = 289577.66;
        double x_actual_max = 291591.05;
        double y_actual_min = 4117065.30;
        double y_actual_max = 4118523.52;

        // Compute the station-keeping point
        std::array<double, 2> station_keeping_point = {
            (x_actual_min + x_actual_max) * 0.5,
            (y_actual_min + y_actual_max) * 0.5
        };


        // Compute forces
        std::array<double, 3> f_usv = {
            (- (Xu + Xuu * std::sqrt(u * u)) * u) / M,
            (-Yv * v - Yvv * std::sqrt(v * v) * v - Yr * r) / M,
            (- (Nr + Nrrr * r * r) * r - Nv * v) / I
        };


        // Compute error
        std::array<double, 2> x_error = {
            state_estim[0] - u,
            state_estim[1] - r
        };

        // Adaptive control
        std::array<double, 2> adaptive_control = param_filtered;

        // Compute L1 thruster
        L1_thruster[0] = adaptive_control[0] - 0.1*u - smc_sat(s_u) + Xuu * std::sqrt(u * u) * u;

        L1_thruster[1] = adaptive_control[1] - 0.1*r - psi + Nrrr * r * r * r;
        
        // L1_thruster[0] = adaptive_control[0] - 0.1*u - bow_switch(s_u);

        // L1_thruster[1] = adaptive_control[1] - 0.1*r - psi;

        // Compute xdot
        std::array<double, 2> xdot = {
            param_estim[0] + L1_thruster[0] + f_usv[0] - x_error[0],
            param_estim[1] + L1_thruster[1] + f_usv[2] - x_error[1]
        };

        // Compute updated state
        x_t_plus[0] = xdot[0] * dt + state_estim[0];
        x_t_plus[1] = xdot[1] * dt + state_estim[1];

        // Compute parameter update
        constexpr double gain = -1.0;
        double pi = (1 / gain) * (std::exp(gain * dt) - 1.0);
        updated_param_estim[0] = -std::exp(gain * dt) * x_error[0] / pi;
        updated_param_estim[1] = -std::exp(gain * dt) * x_error[1] / pi;

        // Compute filtered parameter update
        updated_param_filtered[0] = param_filtered[0] * std::exp(-w_cutoff * dt) - updated_param_estim[0] * (1 - std::exp(-w_cutoff * dt));
        updated_param_filtered[1] = param_filtered[1] * std::exp(-w_cutoff * dt) - updated_param_estim[1] * (1 - std::exp(-w_cutoff * dt));
    }
};

#endif // L1_CONTROL_HPP
