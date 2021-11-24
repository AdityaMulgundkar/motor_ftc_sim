import numpy as np

### All quadrotor values/ variables stored here

# Position x, y, z
x = 0
y = 0
z = 0

xi_matrix = np.array([x, y, z]).transpose()

# Rotation phi, theta, psi
phi = 1
theta = 2
psi = 3

eta_matrix = np.array([phi, theta, psi]).transpose()

R_BtoE_matrix = np.array([
[np.cos(phi)*np.cos(psi), (np.cos(psi)*np.sin(theta)*np.sin(phi)) - (np.cos(phi)*np.sin(psi)), (np.cos(phi)*np.cos(psi)*np.sin(theta)) + (np.sin(phi)*np.sin(psi))],
[np.cos(phi)*np.sin(psi), (np.sin(theta)*np.sin(phi)*np.sin(psi)) - (np.cos(phi)*np.cos(psi)), (np.cos(phi)*np.sin(psi)*np.sin(theta)) + (np.cos(phi)*np.sin(psi))],
[-np.sin(theta), np.cos(theta)*np.sin(phi), np.cos(theta)*np.cos(phi)]
])

# Angular Velocity around axes p, q, r
p = 0
q = 0
r = 0

omega_matrix = np.array([p, q, r]).transpose()

W_n = np.array([
[1, 0, -np.sin(theta)],
[0, np.cos(phi), np.sin(phi)*np.cos(theta)],
[0, -np.sin(phi), np.cos(phi)*np.cos(theta)]
])

print("eta: \n", eta_matrix)
print("R BtoE: \n", R_BtoE_matrix)
print("omega: \n", omega_matrix)
print("W_n: \n", W_n)