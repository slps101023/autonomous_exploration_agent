#!/usr/bin/env python3
"""Drive XLeRobot around a 3 m radius circle in the MuJoCo scene."""

import argparse
import math
from pathlib import Path
import time

import mujoco
import mujoco.viewer


SCENE = Path(__file__).resolve().with_name("scene.xml")


def named_id(model, kind, name):
    item_id = mujoco.mj_name2id(model, kind, name)
    if item_id < 0:
        raise ValueError(f"MuJoCo model does not contain {kind.name}: {name}")
    return item_id


def pose_xy_yaw(data, body_id):
    x, y = data.xpos[body_id, :2]
    rotation = data.xmat[body_id].reshape(3, 3)
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return float(x), float(y), yaw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--radius", type=float, default=3.0, help="Circle radius in metres")
    parser.add_argument("--speed", type=float, default=0.5, help="Forward speed in m/s")
    parser.add_argument("--direction", choices=("ccw", "cw"), default="ccw")
    args = parser.parse_args()
    if args.radius <= 0 or args.speed <= 0:
        parser.error("--radius and --speed must be positive")

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    chassis = named_id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    left_body = named_id(model, mujoco.mjtObj.mjOBJ_BODY, "left_wheel")
    right_body = named_id(model, mujoco.mjtObj.mjOBJ_BODY, "right_wheel")
    left_joint = named_id(model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_drive_joint")
    right_joint = named_id(model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_drive_joint")
    left_actuator = named_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_wheel_velocity")
    right_actuator = named_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_wheel_velocity")
    left_contact = named_id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_wheel_contact")
    right_contact = named_id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_wheel_contact")

    wheel_separation = abs(model.body_pos[left_body, 1] - model.body_pos[right_body, 1])
    wheel_radius = (model.geom_size[left_contact, 0] + model.geom_size[right_contact, 0]) / 2
    if wheel_separation <= 0 or wheel_radius <= 0:
        raise ValueError("Wheel separation and contact radius must be positive")

    # A positive +Y wheel rotation drives a wheel forward along chassis +X.
    left_axis_sign = math.copysign(1.0, model.jnt_axis[left_joint, 1])
    right_axis_sign = math.copysign(1.0, model.jnt_axis[right_joint, 1])
    turn_sign = 1 if args.direction == "ccw" else -1

    x0, y0, yaw0 = pose_xy_yaw(data, chassis)
    center_x = x0 - turn_sign * args.radius * math.sin(yaw0)
    center_y = y0 + turn_sign * args.radius * math.cos(yaw0)
    lookahead = min(0.6, args.radius * 0.5)
    steps_per_frame = max(1, round(0.02 / model.opt.timestep))

    print(f"Scene: {SCENE}")
    print(f"Circle centre: ({center_x:.3f}, {center_y:.3f}) m; radius: {args.radius:.3f} m")
    print(f"Speed: {args.speed:.3f} m/s; direction: {args.direction}")
    print(f"Wheel radius: {wheel_radius:.3f} m; wheel separation: {wheel_separation:.3f} m")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Keep the whole 6 m diameter path visible from above.
        with viewer.lock():
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer.cam.lookat[:] = (center_x, center_y, 0.1)
            viewer.cam.distance = max(10.0, args.radius * 3.3)
            viewer.cam.elevation = -65

        next_frame = time.monotonic()
        while viewer.is_running():
            for _ in range(steps_per_frame):
                x, y, yaw = pose_xy_yaw(data, chassis)

                # Follow a point ahead on the circle to correct for wheel slip.
                angle = math.atan2(y - center_y, x - center_x)
                target_angle = angle + turn_sign * lookahead / args.radius
                target_x = center_x + args.radius * math.cos(target_angle)
                target_y = center_y + args.radius * math.sin(target_angle)
                dx, dy = target_x - x, target_y - y
                target_left = -math.sin(yaw) * dx + math.cos(yaw) * dy
                distance_sq = max(dx * dx + dy * dy, 1e-6)
                curvature = 2 * target_left / distance_sq
                yaw_rate = args.speed * curvature

                left_linear = args.speed - yaw_rate * wheel_separation / 2
                right_linear = args.speed + yaw_rate * wheel_separation / 2
                data.ctrl[left_actuator] = left_axis_sign * left_linear / wheel_radius
                data.ctrl[right_actuator] = right_axis_sign * right_linear / wheel_radius
                mujoco.mj_step(model, data)

            viewer.sync()
            next_frame += steps_per_frame * model.opt.timestep
            remaining = next_frame - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            else:
                next_frame = time.monotonic()


if __name__ == "__main__":
    main()
