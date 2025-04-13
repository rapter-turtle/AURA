import cvxpy as cp

# Time step
dt = 0.1

# Initial values
x_init = 0.0
y_init = 0.0

# Decision variables
u1 = cp.Variable()
u2 = cp.Variable()

# Next state
x_next = (x_init + u1) * dt
y_next = (y_init + u2) * dt

# Objective
objective = cp.Minimize(u1**2 + u2**2)

# Constraints
constraints = [
    x_next + y_next <= -1,
    x_next - y_next <= 100
]

# Solve
problem = cp.Problem(objective, constraints)
problem.solve()

# Results
print(f"Optimal u1: {u1.value:.4f}")
print(f"Optimal u2: {u2.value:.4f}")
print(f"x_next: {x_next.value:.4f}")
print(f"y_next: {y_next.value:.4f}")
print(f"x_next + y_next: {(x_next.value + y_next.value):.4f}")
print(f"x_next - y_next: {(x_next.value - y_next.value):.4f}")
