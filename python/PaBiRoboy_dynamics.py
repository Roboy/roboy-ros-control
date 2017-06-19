#!/usr/bin/env python
from PaBiRoboy_dynamics_derivation import *
from PaBiRoboy_dynamics_simulation import *

#%% Plot
print("Plotting")
plot(t, rad2deg(y[:, :5]))
xlabel('Time [s]')
ylabel('Angle [deg]')
legend(["${}$".format(vlatex(c)) for c in coordinates])
                     
#plot(t, rad2deg(y[:, 5:]))
#xlabel('Time [s]')
#ylabel('Angular Rate [deg/s]')
#legend(["${}$".format(vlatex(s)) for s in speeds])

#%% Visualization
print("Visualize")
# external
from pydy.viz.shapes import Cylinder, Sphere
from pydy.viz.scene import Scene
from pydy.viz.visualization_frame import VisualizationFrame

# A cylinder will be attached to each link and a sphere to each bob for the
# visualization.
lengths = [lower_leg_length, upper_leg_length, hip_length, hip_length, hip_length, upper_leg_length, lower_leg_length]
bodies = [lower_leg_left, upper_leg_left, hip, hip, hip, upper_leg_right, lower_leg_right]

viz_frames = []
colors = ['yellow','green','red','red','red','green','blue']

for i, (body, particle, mass_center) in enumerate(zip(bodies, particles, mass_centers)):
#        body_shape = Cylinder(name='cylinder{}'.format(i),
#                              radius=0.05,
#                              length=lengths[i],
#                              color='red')
#    
#        viz_frames.append(VisualizationFrame('link_frame{}'.format(i), body,
#                                             body_shape))
            
    particle_shape = Sphere(name='sphere{}'.format(i),
                            radius=0.06,
                            color=colors[i])
                            
    
    viz_frames.append(VisualizationFrame('particle_frame{}'.format(i),
                                         body.frame,
                                         particle,
                                         particle_shape))
                                         
    mass_center_shape = Sphere(name='sphere{}'.format(i),
                            radius=0.02,
                            color='black')
                                         
    viz_frames.append(VisualizationFrame('mass_center_frame{}'.format(i),
                                         body.frame,
                                         mass_center,
                                         mass_center_shape))
                                             
target_shape = Sphere(name='sphere{}'.format(i),
                            radius=0.02,
                            color='green')

target_left_leg = Point('target_left_leg')
target_left_leg.set_pos(origin, (x_des[0] * inertial_frame.x)+(x_des[1] * inertial_frame.y))         

target_right_leg = Point('target_left_leg')
target_right_leg.set_pos(origin, (x_des[2] * inertial_frame.x)+(x_des[3] * inertial_frame.y))                              
                                   
viz_frames.append(VisualizationFrame('target_frame_left',
                                     inertial_frame,
                                     target_left_leg,
                                     target_shape))
                                     
viz_frames.append(VisualizationFrame('target_frame_right',
                                     inertial_frame,
                                     target_right_leg,
                                     target_shape))

## Now the visualization frames can be passed in to create a scene.
scene = Scene(inertial_frame, origin, *viz_frames)

# Provide the data to compute the trajectories of the visualization frames.
scene.constants = dict(zip(constants, numerical_constants))
scene.states_symbols = coordinates+speeds
scene.states_trajectories = y

scene.display()
#%%

y