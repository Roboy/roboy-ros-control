from __future__ import division
import numpy as np
import math
from matplotlib.cbook import dedent


open('tendon.sdf','w').close()
f = open('tendon.sdf','a')

link = '''\

  <link name={link_name}>
   <gravity>true</gravity>
   <pose>0.1 0 0 0 0 0</pose>
    <inertial>
      <mass>0.1</mass>
      <pose>0.1 0 0 0 0 0</pose>
      <inertia>
        <ixx>0.01</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.01</iyy>
        <iyz>0</iyz>
        <izz>0.01</izz>
      </inertia>
    </inertial>
    <collision name="collision">
      <pose>{pose}</pose>
      <geometry>
        <cylinder>
          <length>{length}</length>
          <radius>{radius}</radius>
        </cylinder>
      </geometry>
      <surface>
        <contact>
          <ode>
            <min_depth>0.005</min_depth>
          </ode>
        </contact>
        <friction>
          <ode>
            <mu>1.0</mu>
            <mu2>1.0</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
    <visual name="visual">
      <pose>{pose}</pose>
      <geometry>
        <cylinder>
          <length>{length}</length>
          <radius>{radius}</radius>
        </cylinder>
      </geometry>
    </visual>
  </link>
\
'''
joint = '''\

  <joint name={joint_name} type="universal">
    <child>{child}</child>
    <parent>{parent}</parent>
    <axis>
      <xyz>0 1 0</xyz>
      <limit>
        <lower>-1.57</lower>
        <upper>1.57</upper>
      </limit>
      <dynamics>
        <damping>1.0</damping>
      </dynamics>
      <use_parent_model_frame>true</use_parent_model_frame>
    </axis>
    <axis2>
      <xyz>0 0 1</xyz>
      <limit>
        <lower>-1.57</lower>
        <upper>1.57</upper>
      </limit>
      <dynamics>
        <damping>1.0</damping>
      </dynamics>
      <use_parent_model_frame>true</use_parent_model_frame>
    </axis2>
    <physics>
      <ode>
        <cfm_damping>1</cfm_damping>
      </ode>
    </physics>
  </joint>
\
'''

def transform(a,b):
  a = a/np.linalg.norm(a)
  b = b/np.linalg.norm(b)

  v = np.cross(a,b)
  c = np.dot(a,b) # cos
  s = np.linalg.norm(v)  # sin

  t_x = b[0] - a[0]
  t_y = b[1] - a[1]
  t_z = b[2] - a[2]

  v_x = np.matrix([
        [0, -v[2], v[1]], 
        [v[2], 0, -v[0]], 
        [-v[1], v[0], 0]
        ])

  identity_mat = np.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]])

  #print v,c,s,identity_mat
  
  rotation = identity_mat + v_x + v_x*v_x*(1-c)/s*s
  return np.matrix([[rotation.item(0,0), rotation.item(0,0), rotation.item(0,2), t_x],
                    [rotation.item(1,0), rotation.item(1,1), rotation.item(1,2), t_y],
                    [rotation.item(2,0), rotation.item(2,1), rotation.item(2,2), t_z],
                    [0,0,0,1]])

def add_links(insertion_point, fixation_point):

 initial_point = np.array(insertion_point)
 terminal_point = np.array(fixation_point)
 cylinder_lenght = 0.5
 cylinder_radius = 0.005

 distance = np.linalg.norm(initial_point - terminal_point)

 pieces_count = int(distance/cylinder_lenght)

 print "num of pieces ", pieces_count

 def angle(v1,v2):
   return math.acos(np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v1)))

 def orientation():
   tendon_vector = terminal_point - initial_point
   tendon_proj_x = np.array([0, tendon_vector[1], tendon_vector[2]])
   tendon_proj_y = np.array([tendon_vector[0], 0, tendon_vector[2]])
   tendon_proj_z = np.array([tendon_vector[0], tendon_vector[1], 0])
   pitch = angle(tendon_vector, tendon_proj_x)
   roll = angle(tendon_vector, tendon_proj_y)
   yaw = angle(tendon_vector, tendon_proj_z)
   return pitch,roll,yaw

 piece_orient = orientation()

 for piece_num in range(pieces_count):
   piece_pos = initial_point + (terminal_point - initial_point)*(piece_num + 1)/pieces_count
   print piece_pos
   '''
   for i in range(len(piece_pos)):
     print initial_point[i],terminal_point[i],pieces_count,piece_num
     piece_pos[i] = initial_point[i] + (terminal_point[i] - initial_point[i])*(piece_num + 1)/pieces_count
     print piece_pos[i]
   ''' 
   
   f.write(link.format(link_name = "link_"+str(piece_num), length = str(cylinder_lenght), 
     radius = str(cylinder_radius), 
     pose = str(piece_pos[0]) + " " + str(piece_pos[1]) + " " + str(piece_pos[2]) + " " +# "0 0 0"))
           str(piece_orient[0]) + " 0" + " " + str(piece_orient[2])))
 
 for piece_num in range(pieces_count-1):
  joint_name_ = "joint_" + str(piece_num)
  parent_ = "link_" + str(piece_num)
  child_ = "link_" + str(piece_num+1)
  f.write(joint.format(joint_name = joint_name_, parent = parent_, child = child_))

terminal_p =  np.array([5,0,0])#0.14,0.197/2,0.335])
initial_p = np.array([0,0,0])#0,0.22,0.55/2])
#T = transform(np.array([0,1,0]), np.array([1,0,0])) 
add_links(initial_p,terminal_p)