#Run if new world is needed (coordinates of a1 or squaresize change)
# This script generates a Gazebo world file for a chessboard and chess pieces.
import rclpy
from rclpy.node import Node
import os
from ament_index_python import get_package_share_directory
import yaml
import numpy as np

rclpy.init()
node = Node("chess_world_gen")
node.declare_parameter("board_layout", get_package_share_directory("ur_chess") + "/config/board_layout.yaml")
board_layout_path = node.get_parameter("board_layout").value

with open(board_layout_path, 'r') as f:
    board_layout = yaml.safe_load(f)
a1_x_0, a1_y_0, a1_z = board_layout['a1']
square_size = board_layout['tile_size']
o_x, o_y  = board_layout['orientation']

#TODO Assertions
node.get_logger().debug("Read config")

a1_x = a1_x_0 + o_x*(square_size / 2)
a1_y = a1_y_0 + o_y*(square_size / 2)
scale = 1.2

#Chess piece collision params (for a cylinder)
#TODO different height for the pieces for realism (king is 1.5 while pawn is 1.1)
piece_col_lenght = square_size*1.4
piece_col_radius = square_size/4

table_pose_x = a1_x_0 + o_x*2*square_size
table_pose_y = a1_y_0 + o_y*8*square_size
table_pose_z = a1_z-1-0.1
chessboard_pose_x = a1_x_0 + o_x*4*square_size
chessboard_pose_y = a1_y_0 + o_y*4*square_size
chessboard_pose_z = a1_z-0.01
base_pose_x = chessboard_pose_x
base_pose_y = chessboard_pose_y
base_pose_z = a1_z-0.1
table_leg_length = a1_z-0.1

pieces = [
    ("w_pawn_1", "model://pawn_white/meshes/pawn.dae", "a2", "0"),
    ("w_pawn_2", "model://pawn_white/meshes/pawn.dae", "b2", "0"),
    ("w_pawn_3", "model://pawn_white/meshes/pawn.dae", "c2", "0"),
    ("w_pawn_4", "model://pawn_white/meshes/pawn.dae", "d2", "0"),
    ("w_pawn_5", "model://pawn_white/meshes/pawn.dae", "e2", "0"),
    ("w_pawn_6", "model://pawn_white/meshes/pawn.dae", "f2", "0"),
    ("w_pawn_7", "model://pawn_white/meshes/pawn.dae", "g2", "0"),
    ("w_pawn_8", "model://pawn_white/meshes/pawn.dae", "h2", "0"),
    ("w_rook_1", "model://rook_white/meshes/rook.dae", "a1", "0"),
    ("w_knight_1", "model://knight_white/meshes/knight.dae", "b1", "0"),
    ("w_bishop_1", "model://bishop_white/meshes/bishop.dae", "c1", "0"),
    ("w_queen", "model://queen_white/meshes/queen.dae", "d1", "0"),
    ("w_king", "model://king_white/meshes/king.dae", "e1", "0"),
    ("w_bishop_2", "model://bishop_white/meshes/bishop.dae", "f1", "0"),
    ("w_knight_2", "model://knight_white/meshes/knight.dae", "g1", "0"),
    ("w_rook_2", "model://rook_white/meshes/rook.dae", "h1", "0"),
    ("b_pawn_1", "model://pawn_black/meshes/pawn.dae", "a7", "0"),
    ("b_pawn_2", "model://pawn_black/meshes/pawn.dae", "b7", "0"),
    ("b_pawn_3", "model://pawn_black/meshes/pawn.dae", "c7", "0"),
    ("b_pawn_4", "model://pawn_black/meshes/pawn.dae", "d7", "0"),
    ("b_pawn_5", "model://pawn_black/meshes/pawn.dae", "e7", "0"),
    ("b_pawn_6", "model://pawn_black/meshes/pawn.dae", "f7", "0"),
    ("b_pawn_7", "model://pawn_black/meshes/pawn.dae", "g7", "0"),
    ("b_pawn_8", "model://pawn_black/meshes/pawn.dae", "h7", "0"),
    ("b_rook_1", "model://rook_black/meshes/rook.dae", "a8", "0"),
    ("b_knight_1", "model://knight_black/meshes/knight.dae", "b8", "3.14"),
    ("b_bishop_1", "model://bishop_black/meshes/bishop.dae", "c8", "0"),
    ("b_queen", "model://queen_black/meshes/queen.dae", "d8", "0"),
    ("b_king", "model://king_black/meshes/king.dae", "e8", "0"),
    ("b_bishop_2", "model://bishop_black/meshes/bishop.dae", "f8", "0"),
    ("b_knight_2", "model://knight_black/meshes/knight.dae", "g8", "3.14"),
    ("b_rook_2", "model://rook_black/meshes/rook.dae", "h8", "0"),
]

def square_to_xy(square):
    col = "abcdefgh".index(square[0])
    row = int(square[1]) - 1
    x = a1_x - (col) * square_size
    y = a1_y - (row) * square_size
    return x, y

def generate_detailed_model(name, mesh_path, pose_x, pose_y , pose_z, yaw):
    color = "black" if "b_" in name else "white"
    if color == "black":
        ambient = diffuse = "0.1 0.1 0.1 1"
    else:
        ambient = diffuse = "0.8 0.8 0.8 1"
    return f"""
    <model name='{name}'>
      <static>0</static>
      <link name='link_0'>
        <inertial>
          <mass>0.01</mass>
          <inertia>
            <ixx>1e-06</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1e-06</iyy>
            <iyz>0</iyz>
            <izz>5e-07</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <gravity>1</gravity>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <visual name='visual'>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>{mesh_path}</uri>
              <scale>{scale} {scale} {scale}</scale>
            </mesh>
          </geometry>
          <material>
            <lighting>1</lighting>
            <shader type='vertex'>
              <normal_map>__default__</normal_map>
            </shader>
            <ambient>{ambient}</ambient>
            <diffuse>{diffuse}</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <emissive>0 0 0 1</emissive>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>0</laser_retro>
          <max_contacts>10</max_contacts>
          <pose>0 0 0.005 0 -0 0</pose>
          <geometry>
            <cylinder>
                <radius>{piece_col_radius}</radius>
                <length>{piece_col_lenght}</length>
              </cylinder>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <coefficient>1</coefficient>
                <patch_radius>0</patch_radius>
                <surface_radius>0</surface_radius>
                <use_patch_radius>1</use_patch_radius>
                <ode>
                  <slip>0</slip>
                </ode>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
              <threshold>1e+06</threshold>
            </bounce>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <ode>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
              <bullet>
                <split_impulse>1</split_impulse>
                <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
              </bullet>
            </contact>
          </surface>
        </collision>
      </link>
      <static>0</static>
      <allow_auto_disable>1</allow_auto_disable>
      <pose>{pose_x:.6f} {pose_y:.6f} {pose_z:.6f} 0 -0 {yaw}</pose>
    </model>"""

detailed_models = ["""
<sdf version='1.7'>
<world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose frame=''>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <contact>
              <collide_bitmask>65535</collide_bitmask>
              <ode/>
            </contact>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics name='default_physics' default='0' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>0</shadows>
    </scene>
    <audio>
      <device>default</device>
    </audio>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>"""]

def generate_environment(table_pose_x, table_pose_y, table_pose_z, chessboard_pose_x, chessboard_pose_y, chessboard_pose_z, table_leg_length):
    return f"""
<model name='table'>
      <static>1</static>
      <link name='link'>
        <collision name='surface'>
          <pose frame=''>0 0 1 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.5 0.8 0.03</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.6</mu>
                <mu2>0.6</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual1'>
          <pose frame=''>0 0 1 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.5 0.8 0.03</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='front_left_leg'>
          <pose frame=''>0.68 0.38 {1-table_leg_length/2:.6f} 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>{table_leg_length}</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='front_left_leg'>
          <pose frame=''>0.68 0.38 {1-table_leg_length/2:.6f} 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>{table_leg_length}</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='front_right_leg'>
          <pose frame=''>0.68 -0.38 {1-table_leg_length/2:.6f} 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>{table_leg_length}</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='front_right_leg'>
          <pose frame=''>0.68 -0.38 {1-table_leg_length/2:.6f} 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>{table_leg_length}</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='back_right_leg'>
          <pose frame=''>-0.68 -0.38 {1-table_leg_length/2:.6f} 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>{table_leg_length}</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='back_right_leg'>
          <pose frame=''>-0.68 -0.38 {1-table_leg_length/2:.6f} 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>{table_leg_length}</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='back_left_leg'>
          <pose frame=''>-0.68 0.38 {1-table_leg_length/2:.6f} 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>{table_leg_length}</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='back_left_leg'>
          <pose frame=''>-0.68 0.38 {1-table_leg_length/2:.6f} 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>{table_leg_length}</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose frame=''>{table_pose_x:.6f} {table_pose_y:.6f} {table_pose_z:.6f} 0 -0 0</pose>
    </model>
    <model name='chessboard'>
      <static>1</static>
      <link name='link_0'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose frame=''>0 0 0 0 -0 0</pose>
        </inertial>
        <pose frame=''>0 0 0 0 0 0</pose>
        <gravity>1</gravity>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <enable_wind>0</enable_wind>
        <visual name='visual'>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://chessboard/meshes/chessboard.dae</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>0</laser_retro>
          <max_contacts>10</max_contacts>
          <pose frame=''>0 0 0.005 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.368 0.368 0.01</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode>
                <kp>1e+07</kp>
                <kd>1</kd>
                <max_vel>0.001</max_vel>
                <min_depth>0.1</min_depth>
              </ode>
            </contact>
            <bounce/>
          </surface>
        </collision>
      </link>
      <pose frame=''>{chessboard_pose_x:.6f} {chessboard_pose_y:.6f} {chessboard_pose_z:.6f} 0 -0 1.57</pose>
    </model>
    <model name='chessboard_base'>
      <static>1</static>
      <link name='link_0'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose frame=''>0 0 0 0 -0 0</pose>
        </inertial>
        <pose frame=''>0 0 0 0 0 0</pose>
        <gravity>1</gravity>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <enable_wind>0</enable_wind>
        <visual name='visual'>
          <pose frame=''>0 0 0.045 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.68 0.45 0.09</size>
            </box>
          </geometry>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>0</laser_retro>
          <max_contacts>10</max_contacts>
          <pose frame=''>0 0 0.045 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.68 0.45 0.09</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode>
                <kp>1e+07</kp>
                <kd>1</kd>
                <max_vel>0.001</max_vel>
                <min_depth>0.1</min_depth>
              </ode>
            </contact>
            <bounce/>
          </surface>
        </collision>
      </link>
      <pose frame=''>{base_pose_x:.6f} {base_pose_y:.6f} {base_pose_z:.6f} 0 -0 0</pose>
    </model>"""   

def main():
    detailed_models.append(generate_environment(table_pose_x, table_pose_y, table_pose_z, chessboard_pose_x, chessboard_pose_y, chessboard_pose_z, table_leg_length))
    for name, mesh_path, square, yaw in pieces:
        x, y = square_to_xy(square)
        z = chessboard_pose_z + 0.01
        detailed_models.append(generate_detailed_model(name, mesh_path, x, y, z, yaw))
    detailed_models.append("""
                              <!-- A gazebo links attacher -->
                              <plugin name="ros_link_attacher_plugin" filename="libgazebo_ros_link_attacher.so"/>
                            </world>
                          </sdf>""")
    with open("/home/appuser/ros2_ws/src/my_packages/chess_gazebo_world/worlds/chess_room.world", "w") as f:
        f.write("\n".join(detailed_models) + "\n</sdf>\n")
    node.get_logger().info("Generated chess world")

if __name__ == "__main__":
    main()

