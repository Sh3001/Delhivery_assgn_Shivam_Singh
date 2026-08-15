#!/usr/bin/env python3
# Copyright 2026 Delhivery RSE Assignment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Generates amr_gazebo/worlds/warehouse.sdf."""

import math
import os

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "worlds", "warehouse.sdf")

WALL_H = 6.0
WALL_T = 0.3
X_MIN, X_MAX = -28.0, 26.0
Y_MIN, Y_MAX = -16.0, 16.0

MEZZ_H = 1.0
MEZZ_X0, MEZZ_X1 = 0.0, 8.0
MEZZ_Y0, MEZZ_Y1 = -6.0, 6.0

RAMP_HALF_W = 2.0          # ramps are 4 m wide, y in [-2, 2]
RAMP_RUN = 10.0
RAMP_LEN = (RAMP_RUN ** 2 + MEZZ_H ** 2) ** 0.5
RAMP_PITCH = 0.09966865      # atan(1.0/10.0)
RAMP_T = 0.2               # deck slab thickness

RAMP_DZ = (RAMP_T / 2.0) * math.cos(RAMP_PITCH)
RAMP_Z = MEZZ_H / 2.0 - RAMP_DZ

KERB_H = 0.5               # kerb height ABOVE the deck it follows
KERB_Z = RAMP_Z + (RAMP_T / 2.0 + KERB_H / 2.0) * math.cos(RAMP_PITCH)

RAMP_UP = ("ramp_up", -10.0, 0.0)
RAMP_DOWN = ("ramp_down", 18.0, 8.0)

RACK_PITCH = 3.0
BAY_HALF_X = 0.7           # pallet_rack is 1.4 x 0.9
BAY_HALF_Y = 0.45

parts = []
add = parts.append


def box_model(name, x, y, z, sx, sy, sz, rgb, yaw=0.0):
    r, g, b = rgb
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="visual"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def paint(name, x, y, sx, sy, rgb):
    """Floor marking: visual only, never an obstacle, 5 mm proud of the floor."""
    r, g, b = rgb
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0.005 0 0 0</pose>
      <link name="link">
        <visual name="visual"><geometry><box><size>{sx} {sy} 0.01</size></box></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def pedestrian(name, x, y, z, yaw, torso_rgb, comment=""):
    r, g, b = torso_rgb
    return f"""    <model name="{name}">{comment}
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
      <link name="link">
        <inertial><pose>0 0 -0.65 0 0 0</pose><mass>12</mass><inertia><ixx>0.43</ixx><iyy>0.43</iyy><izz>0.54</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="collision"><pose>0 0 -0.65 0 0 0</pose><geometry><cylinder><radius>0.3</radius><length>0.4</length></cylinder></geometry><surface><contact><ode><max_vel>1.0</max_vel><min_depth>0.001</min_depth></ode></contact><bounce><restitution_coefficient>0.0</restitution_coefficient></bounce><friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction></surface></collision>
        <visual name="legs"><pose>0 0.09 -0.4 0 0 0</pose><geometry><cylinder><radius>0.07</radius><length>0.9</length></cylinder></geometry>
          <material><ambient>0.15 0.15 0.2 1</ambient><diffuse>0.15 0.15 0.2 1</diffuse></material></visual>
        <visual name="legs_2"><pose>0 -0.09 -0.4 0 0 0</pose><geometry><cylinder><radius>0.07</radius><length>0.9</length></cylinder></geometry>
          <material><ambient>0.15 0.15 0.2 1</ambient><diffuse>0.15 0.15 0.2 1</diffuse></material></visual>
        <visual name="torso"><pose>0 0 0.335 0 0 0</pose><geometry><cylinder><radius>0.18</radius><length>0.55</length></cylinder></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material></visual>
        <visual name="head"><pose>0 0 0.73 0 0 0</pose><geometry><sphere><radius>0.11</radius></sphere></geometry>
          <material><ambient>0.82 0.65 0.5 1</ambient><diffuse>0.82 0.65 0.5 1</diffuse></material></visual>
      </link>
      <plugin filename="ignition-gazebo-velocity-control-system" name="ignition::gazebo::systems::VelocityControl">
        <topic>/model/{name}/cmd_vel</topic>
      </plugin>
    </model>
"""


def obstacle_robot(name, x, y, z, comment=""):
    return f"""    <model name="{name}">{comment}
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <inertial><mass>18</mass><inertia><ixx>0.35</ixx><iyy>0.35</iyy><izz>0.35</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="collision"><geometry><box><size>0.5 0.4 0.3</size></box></geometry><surface><contact><ode><max_vel>1.0</max_vel><min_depth>0.001</min_depth></ode></contact><bounce><restitution_coefficient>0.0</restitution_coefficient></bounce><friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction></surface></collision>
        <visual name="chassis"><geometry><box><size>0.5 0.4 0.3</size></box></geometry>
          <material><ambient>0.14 0.42 0.42 1</ambient><diffuse>0.14 0.42 0.42 1</diffuse></material></visual>
        <visual name="wheel_fl"><pose>0.17 0.21 -0.09 1.5708 0 0</pose><geometry><cylinder><radius>0.09</radius><length>0.05</length></cylinder></geometry>
          <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material></visual>
        <visual name="wheel_fr"><pose>0.17 -0.21 -0.09 1.5708 0 0</pose><geometry><cylinder><radius>0.09</radius><length>0.05</length></cylinder></geometry>
          <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material></visual>
        <visual name="wheel_rl"><pose>-0.17 0.21 -0.09 1.5708 0 0</pose><geometry><cylinder><radius>0.09</radius><length>0.05</length></cylinder></geometry>
          <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material></visual>
        <visual name="wheel_rr"><pose>-0.17 -0.21 -0.09 1.5708 0 0</pose><geometry><cylinder><radius>0.09</radius><length>0.05</length></cylinder></geometry>
          <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material></visual>
        <visual name="beacon"><pose>0 0 0.22 0 0 0</pose><geometry><sphere><radius>0.05</radius></sphere></geometry>
          <material><ambient>1.0 0.15 0.1 1</ambient><diffuse>1.0 0.15 0.1 1</diffuse></material></visual>
      </link>
      <plugin filename="ignition-gazebo-velocity-control-system" name="ignition::gazebo::systems::VelocityControl">
        <topic>/model/{name}/cmd_vel</topic>
      </plugin>
    </model>
"""


def rack_row(tag, y, x_start, x_end, skip_centres=()):
    """Tiles pallet_rack bays along y, leaving a navigable north-south cross."""
    out = []
    n = int(round((x_end - x_start) / RACK_PITCH))
    idx = 0
    for k in range(n + 1):
        x = x_start + k * RACK_PITCH
        if any(abs(x - s) < 1.5 for s in skip_centres):
            continue
        out.append(f'    <include><name>rack_{tag}_{idx}</name><uri>model://pallet_rack</uri>'
                   f'<pose>{x} {y} 0 0 0 0</pose></include>\n')
        idx += 1
    return out


add(f"""    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{X_MAX - X_MIN} {Y_MAX - Y_MIN}</size></plane></geometry>
          <surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{X_MAX - X_MIN} {Y_MAX - Y_MIN}</size></plane></geometry>
          <material><ambient>0.62 0.62 0.63 1</ambient><diffuse>0.66 0.66 0.67 1</diffuse></material>
        </visual>
      </link>
    </model>
""")

WALL = (0.74, 0.74, 0.72)
cx = (X_MIN + X_MAX) / 2
add(box_model("wall_north", cx, Y_MAX + WALL_T / 2, WALL_H / 2, X_MAX - X_MIN + 2 * WALL_T, WALL_T, WALL_H, WALL))
add(box_model("wall_south", cx, Y_MIN - WALL_T / 2, WALL_H / 2, X_MAX - X_MIN + 2 * WALL_T, WALL_T, WALL_H, WALL))
add(box_model("wall_east", X_MAX + WALL_T / 2, 0, WALL_H / 2, WALL_T, Y_MAX - Y_MIN, WALL_H, WALL))
add(box_model("wall_west", X_MIN - WALL_T / 2, 0, WALL_H / 2, WALL_T, Y_MAX - Y_MIN, WALL_H, WALL))

add(box_model("mezzanine", (MEZZ_X0 + MEZZ_X1) / 2, (MEZZ_Y0 + MEZZ_Y1) / 2, MEZZ_H / 2,
              MEZZ_X1 - MEZZ_X0, MEZZ_Y1 - MEZZ_Y0, MEZZ_H, (0.55, 0.55, 0.57)))

for name, x_low, x_high in (RAMP_UP, RAMP_DOWN):
    pitch = -RAMP_PITCH if x_high > x_low else RAMP_PITCH
    add(f"""    <model name="{name}">
      <static>true</static>
      <pose>{(x_low + x_high) / 2} 0 {RAMP_Z:.4f} 0 {pitch} 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{RAMP_LEN:.3f} {2 * RAMP_HALF_W} {RAMP_T}</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="visual"><geometry><box><size>{RAMP_LEN:.3f} {2 * RAMP_HALF_W} {RAMP_T}</size></box></geometry>
          <material><ambient>0.5 0.5 0.52 1</ambient><diffuse>0.54 0.54 0.56 1</diffuse></material></visual>
      </link>
    </model>
""")
    for side, sy in (("n", RAMP_HALF_W - 0.06), ("s", -RAMP_HALF_W + 0.06)):
        add(f"""    <model name="{name}_kerb_{side}">
      <static>true</static>
      <pose>{(x_low + x_high) / 2} {sy} {KERB_Z:.4f} 0 {pitch} 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{RAMP_LEN:.3f} 0.12 {KERB_H}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{RAMP_LEN:.3f} 0.12 {KERB_H}</size></box></geometry>
          <material><ambient>0.85 0.68 0.05 1</ambient><diffuse>0.95 0.76 0.08 1</diffuse></material>
        </visual>
      </link>
    </model>
""")

add(box_model("mezz_rail_n", (MEZZ_X0 + MEZZ_X1) / 2, MEZZ_Y1 - 0.06, MEZZ_H + 0.55,
              MEZZ_X1 - MEZZ_X0, 0.12, 1.1, (0.85, 0.68, 0.05)))
add(box_model("mezz_rail_s", (MEZZ_X0 + MEZZ_X1) / 2, MEZZ_Y0 + 0.06, MEZZ_H + 0.55,
              MEZZ_X1 - MEZZ_X0, 0.12, 1.1, (0.85, 0.68, 0.05)))
for tag, ex in (("w", MEZZ_X0 + 0.06), ("e", MEZZ_X1 - 0.06)):
    for half, y0, y1 in (("n", RAMP_HALF_W, MEZZ_Y1), ("s", MEZZ_Y0, -RAMP_HALF_W)):
        add(box_model(f"mezz_rail_{tag}{half}", ex, (y0 + y1) / 2, MEZZ_H + 0.55,
                      0.12, y1 - y0, 1.1, (0.85, 0.68, 0.05)))

for y in (-12.0, -8.0, -4.0, 4.0, 8.0, 12.0):
    add_rows = rack_row(f"w{int(y)}".replace("-", "m"), y, -26.0, -14.0, skip_centres=(-19.0,))
    for r in add_rows:
        add(r)

for y in (13.0, -13.0):
    for r in rack_row(f"m{int(y)}".replace("-", "m"), y, -8.0, 16.0, skip_centres=(4.0,)):
        add(r)

for y in (-12.0, -8.0, 8.0, 12.0):
    for r in rack_row(f"e{int(y)}".replace("-", "m"), y, 20.0, 24.0):
        add(r)

for i, x in enumerate([-26.5, -25.0, -23.5]):
    for j, y in enumerate([-15.0, 15.0]):
        add(f'    <include><name>stack_{i}_{j}</name><uri>model://pallet_stack</uri>'
            f'<pose>{x} {y} 0 0 0 0</pose></include>\n')

GREEN = (0.15, 0.62, 0.25)
for tag, yc in (("n", 8.0), ("s", -8.0)):
    add(paint(f"bypass_{tag}_a", 4.0, yc + 1.9, 28.0, 0.12, GREEN))
    add(paint(f"bypass_{tag}_b", 4.0, yc - 1.9, 28.0, 0.12, GREEN))

add(pedestrian("ped_1", -20.0, 0.0, 0.85, 0.0, (0.95, 0.55, 0.05),
               "\n      <!-- west central corridor, on the Ramp Up run-up -->"))
add(pedestrian("ped_2", -20.0, 6.0, 0.85, 0.0, (0.9, 0.85, 0.1),
               "\n      <!-- west cross aisle -->"))
add(pedestrian("ped_3", 12.0, 8.0, 0.85, 0.0, (0.95, 0.15, 0.1),
               "\n      <!-- north bypass aisle: contests the flat route -->"))
add(pedestrian("ped_4", 4.0, 3.0, 0.85 + MEZZ_H, 0.0, (0.15, 0.55, 0.9),
               "\n      <!-- up on the mezzanine -->"))

add(obstacle_robot("robot_1", -16.0, -8.0, 0.15,
                   "\n      <!-- west block aisle -->"))
add(obstacle_robot("robot_2", 12.0, -8.0, 0.15,
                   "\n      <!-- south bypass aisle: contests the flat route -->"))
add(obstacle_robot("robot_3", 22.0, 3.0, 0.15,
                   "\n      <!-- east block, by the packing bay -->"))
add(obstacle_robot("robot_4", 4.0, -3.0, 0.15 + MEZZ_H,
                   "\n      <!-- up on the mezzanine, south half -->"))

HEADER = f"""<?xml version="1.0" ?>
<!--
  Large multi-level warehouse with a complex, initially-unknown aisle grid.

  The point of the layout is that the ramp cost function has something to
  decide. "Packing Bay 4" on the east ground floor is reachable TWO ways:
  over the bridge (shorter, but 20 m of it on ramps) or around it along a
  flat bypass aisle (longer). A correct terrain cost takes the flat detour.
  "Mezzanine Dock" sits on the elevated deck and can ONLY be reached by ramp,
  so the same cost function must still be willing to climb.

  Layout (world frame, metres):
    - Hall x in [{X_MIN}, {X_MAX}], y in [{Y_MIN}, {Y_MAX}], {WALL_H} m walls.
    - Ramp Up   x in [-10, 0], y in [-2, 2], climbs 0 -> {MEZZ_H} (~10% grade).
    - Mezzanine x in [{MEZZ_X0}, {MEZZ_X1}], y in [{MEZZ_Y0}, {MEZZ_Y1}], deck at z = {MEZZ_H}.
      Solid, so it genuinely blocks ground traffic; railed except at the
      two ramp landings.
    - Ramp Down x in [8, 18], y in [-2, 2], descends {MEZZ_H} -> 0.
    - Bypass aisles y in [6, 10] and y in [-10, -6] run clear across the
      whole bridge span - the flat alternative route.
    - West block: six rack rows (y = +/-4, +/-8, +/-12) broken by a
      north-south cross aisle at x = -19, with the y in [-3.5, 3.5] corridor
      left open as the run-up to Ramp Up. Outer rows at y = +/-13 across the
      middle, and an east block at x = 20..24.
    - Eight mobile obstacles (four pedestrians, four third-party robots),
      two of them deliberately patrolling the bypass aisles so the flat
      route is contested rather than free.

  Generated by amr_gazebo/scripts/generate_warehouse_world.py - edit that and
  regenerate rather than hand-patching the repetitive blocks below.
-->
<sdf version="1.9">
  <world name="warehouse">

    <gui fullscreen="false">
      <camera name="user_camera">
        <camera_pose>-4 -26 14 0 0.5 1.5708</camera_pose>
      </camera>
    </gui>

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-contact-system" name="ignition::gazebo::systems::Contact"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.78 0.82 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 25 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <attenuation><range>900</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.3 0.35 -0.9</direction>
    </light>

"""

with open(OUT, "w") as f:
    f.write(HEADER)
    f.writelines(parts)
    f.write("  </world>\n</sdf>\n")

print(f"wrote {OUT}")
print(f"ramp deck length {RAMP_LEN:.3f} m, pitch {RAMP_PITCH:.6f} rad (~10% grade)")
