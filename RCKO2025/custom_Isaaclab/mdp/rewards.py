# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Common functions that can be used to define rewards for the learning environment.

The functions can be passed to the :class:`isaaclab.managers.RewardTermCfg` object to
specify the reward function and its parameters.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor
from isaaclab.utils.math import quat_rotate_inverse, yaw_quat

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def feet_air_time(
    env: ManagerBasedRLEnv, command_name: str, sensor_cfg: SceneEntityCfg, threshold: float
) -> torch.Tensor:
    """Reward long steps taken by the feet using L2-kernel.

    This function rewards the agent for taking steps that are longer than a threshold. This helps ensure
    that the robot lifts its feet off the ground and takes steps. The reward is computed as the sum of
    the time for which the feet are in the air.

    If the commands are small (i.e. the agent is not supposed to take a step), then the reward is zero.
    """
    # extract the used quantities (to enable type-hinting)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # compute the reward
    first_contact = contact_sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids]
    last_air_time = contact_sensor.data.last_air_time[:, sensor_cfg.body_ids]
    reward = torch.sum((last_air_time - threshold) * first_contact, dim=1)
    # no reward for zero command
    reward *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1
    return reward


def feet_air_time_positive_biped(env, command_name: str, threshold: float, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """Reward long steps taken by the feet for bipeds.

    This function rewards the agent for taking steps up to a specified threshold and also keep one foot at
    a time in the air.

    If the commands are small (i.e. the agent is not supposed to take a step), then the reward is zero.
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # compute the reward
    air_time = contact_sensor.data.current_air_time[:, sensor_cfg.body_ids]
    contact_time = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids]
    in_contact = contact_time > 0.0
    in_mode_time = torch.where(in_contact, contact_time, air_time)
    single_stance = torch.sum(in_contact.int(), dim=1) == 1
    reward = torch.min(torch.where(single_stance.unsqueeze(-1), in_mode_time, 0.0), dim=1)[0]
    reward = torch.clamp(reward, max=threshold)
    # no reward for zero command
    reward *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1
    return reward


# def feet_slide(env, sensor_cfg: SceneEntityCfg, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
#     """Penalize feet sliding.

#     This function penalizes the agent for sliding its feet on the ground. The reward is computed as the
#     norm of the linear velocity of the feet multiplied by a binary contact sensor. This ensures that the
#     agent is penalized only when the feet are in contact with the ground.
#     """
#     # Penalize feet sliding
#     contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
#     contacts = contact_sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids, :].norm(dim=-1).max(dim=1)[0] > 1.0
#     asset = env.scene[asset_cfg.name]

#     body_vel = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :2]
#     reward = torch.sum(body_vel.norm(dim=-1) * contacts, dim=1)
#     return reward


def track_lin_vel_xy_yaw_frame_exp(
    env, std: float, command_name: str, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Reward tracking of linear velocity commands (xy axes) in the gravity aligned robot frame using exponential kernel."""
    # extract the used quantities (to enable type-hinting)
    asset = env.scene[asset_cfg.name]
    vel_yaw = quat_rotate_inverse(yaw_quat(asset.data.root_quat_w), asset.data.root_lin_vel_w[:, :3])
    lin_vel_error = torch.sum(
        torch.square(env.command_manager.get_command(command_name)[:, :2] - vel_yaw[:, :2]), dim=1
    )
    return torch.exp(-lin_vel_error / std**2)


def track_ang_vel_z_world_exp(
    env, command_name: str, std: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Reward tracking of angular velocity commands (yaw) in world frame using exponential kernel."""
    # extract the used quantities (to enable type-hinting)
    asset = env.scene[asset_cfg.name]
    ang_vel_error = torch.square(env.command_manager.get_command(command_name)[:, 2] - asset.data.root_ang_vel_w[:, 2])
    return torch.exp(-ang_vel_error / std**2)

def feet_slide(
    env,
    sensor_cfg: SceneEntityCfg,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    force_threshold: float = 0.4, 
) -> torch.Tensor:
    """
    Penalize feet sliding with configurable force threshold.

    Parameters:
    - env: 환경 인스턴스
    - sensor_cfg: 접촉 센서 설정
    - asset_cfg: 로봇 본체 설정
    - force_threshold: 접촉 판단 임계값

    Returns:
    - slide 보상 (패널티 값)
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # configurable threshold 사용
    contact_force = contact_sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids, :]
    contacts = contact_force.norm(dim=-1).max(dim=1)[0] > force_threshold

    asset = env.scene[asset_cfg.name]
    body_vel = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :2]  # XY 평면 속도
    reward = torch.sum(body_vel.norm(dim=-1) * contacts, dim=1)
    return reward


# 커스텀 보상함수 추가
# def center_of_mass_over_feet(env, asset_cfg: SceneEntityCfg) -> torch.Tensor:
#     asset = env.scene[asset_cfg.name]
#     com_xy = asset.data.root_pos_w[:, :2]
#     left_foot = asset.data.body_pos_w[:, asset_cfg.body_ids[0], :2]
#     right_foot = asset.data.body_pos_w[:, asset_cfg.body_ids[1], :2]
#     mid_point = 0.5 * (left_foot + right_foot)
#     offset = torch.norm(com_xy - mid_point, dim=1)
#     return torch.exp(-offset * 10.0)

# def penalize_wide_stance(env, asset_cfg: SceneEntityCfg) -> torch.Tensor:
#     asset = env.scene[asset_cfg.name]
#     left_y = asset.data.body_pos_w[:, asset_cfg.body_ids[0], 1]
#     right_y = asset.data.body_pos_w[:, asset_cfg.body_ids[1], 1]
#     return -torch.abs(left_y - right_y)

