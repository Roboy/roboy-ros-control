#%% 
from __future__ import print_function, division
from sympy import symbols, simplify, Matrix
from sympy import trigsimp
from sympy.physics.mechanics import dynamicsymbols, ReferenceFrame, Point, Particle, inertia, RigidBody, KanesMethod
from numpy import deg2rad, rad2deg, array, zeros, linspace
from sympy.physics.vector import init_vprinting, vlatex
import numpy as np

from sympy.physics.vector import init_vprinting
init_vprinting(use_latex='mathjax', pretty_print=False)
#%% 
# define all reference frames of all individually moving links of the robot
print("Defining reference frames")
inertial_frame = ReferenceFrame('I')
lower_leg_left_frame = ReferenceFrame('R_1')
upper_leg_left_frame = ReferenceFrame('R_2')
hip_frame = ReferenceFrame('R_3')
upper_leg_right_frame = ReferenceFrame('R_4')
lower_leg_right_frame = ReferenceFrame('R_5')
#%% Angles
# everything is symbolic, so create all angles of your robot
# NOTE: the angle phi is the angle between your robot and the inertial frame
theta0, theta1, theta2, theta3, phi = dynamicsymbols('theta0, theta1, theta2, theta3, phi')

lower_leg_left_frame.orient(inertial_frame, 'Axis', (phi, inertial_frame.z))
simplify(lower_leg_left_frame.dcm(inertial_frame))

upper_leg_left_frame.orient(lower_leg_left_frame, 'Axis', (theta0, -lower_leg_left_frame.z))
simplify(upper_leg_left_frame.dcm(inertial_frame))

hip_frame.orient(upper_leg_left_frame, 'Axis', (theta1, -upper_leg_left_frame.z))
hip_frame.dcm(inertial_frame)

upper_leg_right_frame.orient(hip_frame, 'Axis', (theta2, -hip_frame.z))
simplify(upper_leg_right_frame.dcm(inertial_frame))

lower_leg_right_frame.orient(upper_leg_right_frame, 'Axis', (theta3, -upper_leg_right_frame.z))
simplify(lower_leg_right_frame.dcm(inertial_frame))
#%% Points and Locations
# define the kinematical chain of your robot
print("Defining kinematical chain")

# these can be arbitray points on each of you robots links
# later these points are visualized, so you can verify the correct build of your robot
origin = Point('Origin')
ankle_left = Point('AnkleLeft')
knee_left = Point('KneeLeft')
hip_left = Point('HipLeft')
hip_center = Point('HipCenter')
hip_right = Point('HipRight')
knee_right = Point('KneeRight')
ankle_right = Point('AnkleRight')

# here go the lengths of your robots links
lower_leg_length, upper_leg_length, hip_length = symbols('l1, l2, l3')

ankle_left.set_pos(origin, (0 * inertial_frame.y)+(0 * inertial_frame.x))
#ankle_left.pos_from(origin).express(inertial_frame).simplify()

knee_left.set_pos(ankle_left, lower_leg_length * lower_leg_left_frame.y)
#knee_left.pos_from(origin).express(inertial_frame).simplify()

hip_left.set_pos(knee_left, upper_leg_length * upper_leg_left_frame.y)
#hip_left.pos_from(origin).express(inertial_frame).simplify()

hip_center.set_pos(hip_left, hip_length/2 * hip_frame.x)
#hip_center.pos_from(origin).express(inertial_frame).simplify()

hip_right.set_pos(hip_center, hip_length/2 * hip_frame.x)
#hip_right.pos_from(origin).express(inertial_frame).simplify()

knee_right.set_pos(hip_right, upper_leg_length * -upper_leg_right_frame.y)
#knee_right.pos_from(origin).express(inertial_frame).simplify()

ankle_right.set_pos(knee_right, lower_leg_length * -lower_leg_right_frame.y)
#ankle_right.pos_from(origin).express(inertial_frame).simplify()

#%%
# The following defines the full robots kinematics, if you only want to do 
# inverse kinematics you can skip this whole part
print("calculating full kinematics, this might take a while ...")
#%% COMs
print("Defining center of mass")

# we set them to be in the middle of each link
lower_leg_left_com_length = lower_leg_length/2
upper_leg_left_com_length = upper_leg_length/2
hip_com_length = hip_length/2
upper_leg_right_com_length = upper_leg_length/2
lower_leg_right_com_length = lower_leg_length/2

# these points are visualized later as black dots
lower_leg_left_mass_center = Point('L_COMleft')
upper_leg_left_mass_center = Point('U_COMleft')
hip_mass_center = Point('H_COMleft')
upper_leg_right_mass_center = Point('U_COMright')
lower_leg_right_mass_center = Point('L_COMright')

lower_leg_left_mass_center.set_pos(ankle_left, lower_leg_left_com_length * lower_leg_left_frame.y)
#lower_leg_left_mass_center.pos_from(origin).express(inertial_frame).simplify()

upper_leg_left_mass_center.set_pos(knee_left, upper_leg_left_com_length * upper_leg_left_frame.y)
#upper_leg_left_mass_center.pos_from(origin).express(inertial_frame).simplify()

hip_mass_center.set_pos(hip_center, 0 * hip_frame.x)
#hip_mass_center.pos_from(origin).express(inertial_frame).simplify()

upper_leg_right_mass_center.set_pos(knee_right, upper_leg_right_com_length * upper_leg_right_frame.y)
#upper_leg_right_mass_center.pos_from(origin).express(inertial_frame).simplify()

lower_leg_right_mass_center.set_pos(ankle_right, lower_leg_right_com_length * lower_leg_right_frame.y)
#lower_leg_right_mass_center.pos_from(origin).express(inertial_frame).simplify()

#%% Kinematical Differential Equations
omega0, omega1, omega2, omega3, psi = dynamicsymbols('omega0, omega1, omega2, omega3, psi')
kinematical_differential_equations = [omega0 - theta0.diff(),
                                      omega1 - theta1.diff(),
                                      omega2 - theta2.diff(),
                                      omega3 - theta3.diff(),
                                        psi - phi.diff(),                                
                                        ]

kinematical_differential_equations
#%% Angular Velocities
print("Defining angular velocities")

lower_leg_left_frame.set_ang_vel(inertial_frame,         0 * inertial_frame.z)
#lower_leg_left_frame.ang_vel_in(inertial_frame)

upper_leg_left_frame.set_ang_vel(upper_leg_left_frame,   omega0     * inertial_frame.z)
#upper_leg_left_frame.ang_vel_in(inertial_frame)

hip_frame.set_ang_vel(hip_frame,   omega1     * inertial_frame.z)
#hip_frame.ang_vel_in(inertial_frame)

upper_leg_right_frame.set_ang_vel(upper_leg_right_frame, omega2     * inertial_frame.z)
#upper_leg_right_frame.ang_vel_in(inertial_frame)

lower_leg_right_frame.set_ang_vel(lower_leg_right_frame, omega3     * inertial_frame.z)
#lower_leg_right_frame.ang_vel_in(inertial_frame)
#%% Linear Velocities
print("Defining linear velocities")
origin.set_vel(inertial_frame, 0)

ankle_left.set_vel(inertial_frame, 0)

knee_left.v2pt_theory(ankle_left, inertial_frame, lower_leg_left_frame)
#knee_left.vel(inertial_frame)

hip_left.v2pt_theory(knee_left, inertial_frame, upper_leg_left_frame)
#hip_left.vel(inertial_frame)

hip_center.v2pt_theory(hip_left, inertial_frame, hip_frame)
#hip_center.vel(inertial_frame)

hip_mass_center.v2pt_theory(hip_center, inertial_frame, hip_frame)
#hip_mass_center.vel(inertial_frame)

hip_right.v2pt_theory(hip_center, inertial_frame, hip_frame)
#hip_right.vel(inertial_frame)

knee_right.v2pt_theory(hip_right, inertial_frame, upper_leg_right_frame)
#knee_right.vel(inertial_frame)

ankle_right.v2pt_theory(knee_right, inertial_frame, lower_leg_right_frame)
#ankle_right.vel(inertial_frame)

lower_leg_left_mass_center.v2pt_theory(ankle_left, inertial_frame, lower_leg_left_frame)
#lower_leg_left_mass_center.vel(inertial_frame)

lower_leg_right_mass_center.v2pt_theory(ankle_right, inertial_frame, lower_leg_right_frame)
#lower_leg_right_mass_center.vel(inertial_frame)

upper_leg_left_mass_center.v2pt_theory(knee_left, inertial_frame, upper_leg_left_frame)
#upper_leg_left_mass_center.vel(inertial_frame)

upper_leg_right_mass_center.v2pt_theory(knee_right, inertial_frame, upper_leg_right_frame)
#upper_leg_right_mass_center.vel(inertial_frame)
#%% Masses, Inertia, Rigid Bodies
print("Defining Masses, Inertia, Rigid Bodies")
lower_leg_mass, upper_leg_mass, hip_mass = symbols('m_L, m_U, m_H')

lower_leg_inertia, upper_leg_inertia, hip_inertia = symbols('I_Lz, I_Uz, I_Hz')

lower_leg_left_inertia_dyadic = inertia(lower_leg_left_frame, lower_leg_inertia, lower_leg_inertia, lower_leg_inertia)
lower_leg_left_central_inertia = (lower_leg_left_inertia_dyadic, lower_leg_left_mass_center)
lower_leg_left_inertia_dyadic.to_matrix(lower_leg_left_frame)

upper_leg_left_inertia_dyadic = inertia(upper_leg_left_frame, upper_leg_inertia, upper_leg_inertia, upper_leg_inertia)
upper_leg_left_central_inertia = (upper_leg_left_inertia_dyadic, upper_leg_left_mass_center)
upper_leg_left_inertia_dyadic.to_matrix(upper_leg_left_frame)

hip_inertia_dyadic = inertia(hip_frame, hip_inertia, hip_inertia, hip_inertia)
hip_central_inertia = (hip_inertia_dyadic, hip_mass_center)
hip_inertia_dyadic.to_matrix(hip_frame)

upper_leg_right_inertia_dyadic = inertia(upper_leg_right_frame, upper_leg_inertia, upper_leg_inertia, upper_leg_inertia)
upper_leg_right_central_inertia = (upper_leg_right_inertia_dyadic, upper_leg_right_mass_center)
upper_leg_right_inertia_dyadic.to_matrix(upper_leg_right_frame)

lower_leg_right_inertia_dyadic = inertia(lower_leg_right_frame, lower_leg_inertia, lower_leg_inertia, lower_leg_inertia)
lower_leg_right_central_inertia = (lower_leg_right_inertia_dyadic, lower_leg_right_mass_center)
lower_leg_right_inertia_dyadic.to_matrix(lower_leg_right_frame)

lower_leg_left = RigidBody('Lower Leg Left', lower_leg_left_mass_center, lower_leg_left_frame, lower_leg_mass, lower_leg_left_central_inertia)
                      
upper_leg_left = RigidBody('Upper Leg Left', upper_leg_left_mass_center, upper_leg_left_frame, upper_leg_mass, upper_leg_left_central_inertia)

hip = RigidBody('Hip', hip_mass_center, hip_frame, hip_mass, hip_central_inertia)

upper_leg_right = RigidBody('Upper Leg Right', upper_leg_right_mass_center, upper_leg_right_frame, upper_leg_mass, upper_leg_right_central_inertia)

lower_leg_right = RigidBody('Lower Leg Right', lower_leg_right_mass_center, lower_leg_right_frame, lower_leg_mass, lower_leg_right_central_inertia)


particles = []
particles.append(Particle('ankle_left', ankle_left, 0))
particles.append(Particle('knee_left', knee_left, 0))
particles.append(Particle('hip_left', hip_left, 0))
particles.append(Particle('hip_center', hip_center, 0))
particles.append(Particle('hip_right', hip_right, 0))
particles.append(Particle('knee_right', knee_right, 0))
particles.append(Particle('ankle_right', ankle_right, 0))
particles

mass_centers = []
mass_centers.append(Particle('lower_leg_left_mass_center', lower_leg_left_mass_center, lower_leg_mass))
mass_centers.append(Particle('upper_leg_left_mass_center', upper_leg_left_mass_center, upper_leg_mass))
mass_centers.append(Particle('hip_mass_center', hip_mass_center, hip_mass))
mass_centers.append(Particle('hip_mass_center', hip_mass_center, hip_mass))
mass_centers.append(Particle('hip_mass_center', hip_mass_center, hip_mass))
mass_centers.append(Particle('lower_leg_left_mass_center', lower_leg_right_mass_center, lower_leg_mass))
mass_centers.append(Particle('upper_leg_left_mass_center', upper_leg_right_mass_center, upper_leg_mass))
mass_centers
#%% Forces and Torques
print("Defining Forces and Torques")
g = symbols('g')

lower_leg_left_grav_force = (lower_leg_left_mass_center, -lower_leg_mass * g * inertial_frame.y)
upper_leg_left_grav_force = (upper_leg_left_mass_center, -upper_leg_mass * g * inertial_frame.y)
hip_grav_force = (hip_mass_center, -hip_mass * g * inertial_frame.y)
upper_leg_right_grav_force = (upper_leg_right_mass_center, -upper_leg_mass * g * inertial_frame.y)
lower_leg_right_grav_force = (lower_leg_right_mass_center, -lower_leg_mass * g * inertial_frame.y)

ankle_torque0, knee_torque0, hip_torque0, hip_torque1, knee_torque1, ankle_torque1 = dynamicsymbols('T_a0, T_k0, T_h0, T_h1, T_k1, T_a1')

lower_leg_left_torque_vector = ankle_torque0 * inertial_frame.z - knee_torque0 * inertial_frame.z
upper_leg_left_torque_vector = knee_torque0 * inertial_frame.z - hip_torque0 * inertial_frame.z
hip_left_torque_vector = hip_torque0 * inertial_frame.z - hip_torque1 * inertial_frame.z
hip_right_torque_vector = hip_torque1 * inertial_frame.z - knee_torque1 * inertial_frame.z
upper_leg_right_torque_vector = knee_torque1 * inertial_frame.z - ankle_torque1 * inertial_frame.z
lower_leg_right_torque_vector = ankle_torque1 * inertial_frame.z

lower_leg_left_torque = (lower_leg_left_frame, lower_leg_left_torque_vector)
upper_leg_left_torque = (upper_leg_left_frame, upper_leg_left_torque_vector)
hip_left_torque = (hip_frame, hip_left_torque_vector)
hip_right_torque = (hip_frame, hip_right_torque_vector)
upper_leg_right_torque = (upper_leg_right_frame, upper_leg_right_torque_vector)
lower_leg_right_torque = (lower_leg_right_frame, lower_leg_right_torque_vector)
#%% Equations of Motion
# you only need this if you want a full dynamics simulation, with forces etc.
print("Calculating equations of motion")
coordinates = [theta0, theta1, theta2, theta3, phi]
coordinates

speeds = [omega0, omega1, omega2, omega3, psi]
speeds

kinematical_differential_equations

kane = KanesMethod(inertial_frame, coordinates, speeds, kinematical_differential_equations)

loads = [lower_leg_left_grav_force,
         upper_leg_left_grav_force,
         hip_grav_force, 
         upper_leg_right_grav_force,
         lower_leg_right_grav_force,
         lower_leg_left_torque,
         upper_leg_left_torque,
         hip_left_torque,
         hip_right_torque,
         upper_leg_right_torque,
         lower_leg_right_torque]
loads

bodies = [lower_leg_left, upper_leg_left, hip, upper_leg_right, lower_leg_right]
bodies

print("evaluating kanes equation")
fr, frstar = kane.kanes_equations(loads, bodies)

#print("simplifying kanes equation")
#trigsimp(fr + frstar)

print("simplifying mass_matrix")
mass_matrix = trigsimp(kane.mass_matrix_full)
#print(mass_matrix)

print("simplifying forcing_vector")
forcing_vector = trigsimp(kane.forcing_full)
#print(forcing_vector)
