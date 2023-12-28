import math

x = 46
y = 3
z = 136
width = 640
height = 480
h_fov = 1.085595
v_fov = h_fov / (width / height)
actual_radius = 0.075

u = x / (width / 2)
v = y / (height / 2)

print(u, v)

z_2d = z / width

print(z_2d)

theta_ball = z_2d * h_fov
d = actual_radius / math.atan(theta_ball / 2)

print(d)

theta_y = v * v_fov / 2
y_3d = d * math.sin(theta_y)
d_dash = d * math.cos(theta_y)

print(y_3d)

theta_x = u * h_fov / 2
x_3d = d_dash * math.sin(theta_x)
z_3d = d_dash * math.cos(theta_x)

print(x_3d, z_3d)