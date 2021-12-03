import numpy as np

### All quadrotor values/ variables stored here
### Dot values are for quadrotor
### e.g. x, y, z are Earth/ origin/ home co-ords
### whereas x_dot, ... are quadcopter co-ords

### u = l_d * [f1 f2 f3]
### u * inv(l_d) = [f1 f2 f3]

# Orphans
mass = 0
Ixx = 0
Iyy = 0
Izz = 0
gravity = 9.8
f1 = 0
f2 = 0
f3 = 0
f4 = 0
k_t = 0 # Translational drag co-efficient
k_r = 0 # Rotational drag co-efficient
l = 1.5 # arm length
d = 0.25 # ratio between drag and thrust co-efficients of blade

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

# Rotation from Body to Earth frame; eq(2)
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

# Wn; eq(4)
W_n = np.array([
[1, 0, -np.sin(theta)],
[0, np.cos(phi), np.sin(phi)*np.cos(theta)],
[0, -np.sin(phi), np.cos(phi)*np.cos(theta)]
])

# Translational component; eq(5)
T_trans = 0.5 * mass * xi_matrix.transpose() * xi_matrix
def get_T_trans(xi_current):
    return 0.5 * mass * xi_current.transpose() * xi_current

# Inertia matrix; eq(7)
I_matrix = [[Ixx, 0, 0], [0, Iyy, 0], [0, 0, Izz]]

# J matrix; eq(8)
J_matrix = W_n.transpose() * I_matrix * W_n

# Rotational component; eq(6)
T_rot = 0.5 * eta_matrix.transpose() * J_matrix * eta_matrix
def get_T_rot(eta_current):
    return 0.5 * eta_current.transpose() * J_matrix * eta_current

# Potential Energy U; eq(9)
U_potential_energy = -mass * gravity * z

# State vector q
q_state = np.array([x, y, z, phi, theta, psi]).transpose

# Lagrangian of q state, eq(10)
lagrangian_of_q = T_trans + T_rot + U_potential_energy
def get_lagrangian_of_q():
    return (get_T_trans() + get_T_rot + U_potential_energy)

# TODO, eq(11)
# Write code

# Upward force, eq(12)
U_f = f1 + f2 + f3 + f4

# Translational generalized force F_xi; eq(13)
F_xi = R_BtoE_matrix * np.array([[0], [0], [U_f]]) - k_t * xi_matrix

tau_p = l * (f4 - f2)
tau_q = l * (f3 - f1)
tau_r = d * (f1 - f2 + f3 - f4)
# Torques around body frame axis
tau_B = np.array([[tau_p], [tau_q], [tau_r]])

print("eta: \n", eta_matrix)
print("R BtoE: \n", R_BtoE_matrix)
print("omega: \n", omega_matrix)
print("F_xi: \n", F_xi)