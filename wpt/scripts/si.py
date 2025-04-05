import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

# Define the model function
def model(u, Xu, Xuu):
    return Xu * u + Xuu * np.sqrt(u * u) * u

def thrust(n):
    return 0.00058466 * n * n

# Given data
u_data = np.array([1.6, 3.3, 4.9, 6.6, 8.3])
n_data = thrust(np.array([20.0, 30.0, 40.0, 50.0, 60.0]))

# Fit the model to the data
params, covariance = curve_fit(model, u_data, n_data, p0=[0, 0]) 
Xu_fitted, Xuu_fitted = params
print(f"Fitted Xu: {Xu_fitted}")
print(f"Fitted Xuu: {Xuu_fitted}")

# Plot: x-axis = n (Thrust), y-axis = u (Velocity)
plt.plot(n_data, u_data, 'bo', label='Measured Data (u vs n)')
plt.plot(model(u_data, *params), u_data, 'r-', label='Fitted Model')
plt.xlabel('n (Thrust)')
plt.ylabel('u (Velocity)')
plt.title('Velocity vs. Thrust Fit')
plt.legend()
plt.grid(True)
plt.show()
