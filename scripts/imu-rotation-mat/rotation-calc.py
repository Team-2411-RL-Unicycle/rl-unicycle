import numpy as np

z_vec = np.array([1, -0.03, 0])
x_vec = np.array([0, 1.03, 0])

# Normalize vectors
z_vec = z_vec / np.linalg.norm(z_vec)

# Get the two vectors orthogonal to the z_vec
x_vec = np.array([0.03, 1, 0])
x_vec = x_vec / np.linalg.norm(x_vec)

y_vec = np.cross(x_vec, z_vec)

mat = np.array([x_vec, y_vec, z_vec])

# check unitary matrix
check = np.abs(mat @ mat.T - np.eye(3)) < 1e-10
print(check)

print(mat)
