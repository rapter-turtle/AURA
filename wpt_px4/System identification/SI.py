import numpy as np
import scipy.optimize as opt
import pandas as pd
import matplotlib.pyplot as plt

# Load experimental data
# Replace with actual data
data = pd.read_csv("/home/user/aura_ws/src/wpt/System identification/recorded_data2.csv")  # Ensure the file contains time-series data of u, ns, nt

u_exp = data["u"].values  # Observed velocity
nt = data["throttle"].values  # Control input (thruster force)
ns = data["steering"].values  # Control input (steering)

# dt = 0.1  # Compute sampling time
dt = 0.1  # Sampling time (adjust if needed)
time = np.arange(len(u_exp)) * dt  # Generate time values


# Define system dynamics
def ship_dynamics(u, nt, ns, params):
    """
    Discretized ship dynamics based on:
        M * u_dot = -Xu*u - Xu|u|*|u|*u + c*nt*cos(alpha*ns)
    """
    M = 1000
    Xu, Xuu, c, alpha = params
    u_dot = (-Xu * u - Xuu * np.abs(u) * u + c * nt * np.cos(alpha * ns)) / M
    return u + u_dot * dt  # Forward Euler integration

# Define cost function for optimization
# Define cost function for optimization
def cost_function(params, u_exp, nt, ns):
    """
    Computes the cost function J(P) = sum (xi - x̄i)^T W (xi - x̄i)
    """
    u_sim = np.zeros_like(u_exp)
    u_sim[0] = u_exp[0]  # Initial condition

    for i in range(len(u_exp) - 1):
        u_sim[i + 1] = ship_dynamics(u_sim[i], nt[i], ns[i], params)

    # Compute error
    error = u_sim - u_exp  # Shape (N,)
    
    # Use sum of squared errors instead of matrix multiplication
    return np.sum(error ** 2)  # Scalar loss function


# Initial guesses for parameters
initial_guess = [1.0, 0.1, 1.0, 0.1]

# Perform nonlinear optimization
result = opt.minimize(cost_function, initial_guess, args=(u_exp, nt, ns), method='BFGS')

# Extract estimated parameters
Xu_est, Xuu_est, c_est, alpha_est = result.x

print("Estimated Parameters:")
print(f"Xu = {Xu_est:.4f}, Xuu = {Xuu_est:.4f}, c = {c_est:.4f}, alpha = {alpha_est:.4f}")

# Plot results
plt.figure(figsize=(10, 5))
plt.plot(time, u_exp, label="Experimental Data", linestyle="dashed", color="r")
plt.plot(time, [ship_dynamics(u_exp[0], nt[i], ns[i], result.x) for i in range(len(u_exp))], label="Estimated Model", color="b")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (u)")
plt.legend()
plt.title("Ship System Identification: Estimated vs Experimental Data")
plt.show()
