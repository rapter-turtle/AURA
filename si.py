import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

# Define the model function
def model(u, Xu, Xuu):
    # return Xu * u + Xuu * u**3
    return Xu * u + Xuu * abs(u)*u

# Given data
# u_data = np.array([1.32, 1.55, 2.54, 2.9, 3.2, 3.77, 3.98, 4.04, 4.23, 4.63])
# n_data = 0.1*np.array([22.31, 25.35, 27.38, 30.41, 32.0, 35.49, 36.5, 38.53, 40.56, 42.59])
u_data = np.array([2.54, 2.9, 3.2, 3.77, 3.98, 4.04, 4.23, 4.63])
n_data = 0.1*np.array([27.38, 30.41, 32.0, 35.49, 36.5, 38.53, 40.56, 42.59])

thrust_data = n_data**2

# Fit the model to the data with positivity constraints
params, covariance = curve_fit(
    model,
    u_data,
    thrust_data,
    p0=[0.1, 0.1],
    bounds=([0, 0], [np.inf, np.inf])  # Xu ≥ 0, Xuu ≥ 0
)
Xu_fitted, Xuu_fitted = params
print(f"Fitted Xu: {Xu_fitted}")
print(f"Fitted Xuu: {Xuu_fitted}")

# Plot: x-axis = n (Thrust), y-axis = u (Velocity)
plt.plot(thrust_data, u_data, 'bo', label='Measured Data (u vs n)')
plt.plot(model(u_data, *params), u_data, 'r-', label='Fitted Model')
plt.xlabel('n^2 (Thrust)')
plt.ylabel('u (Velocity)')
plt.title('Velocity vs. Thrust Fit')
plt.legend()
plt.grid(True)
plt.show()
