# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg, RewardsCfg

##
# Pre-defined configs
##
from isaaclab_assets import ROBIT_CFG  # 로빛 로봇 설정 반영


@configclass
class RobitRewards(RewardsCfg):

    # 바닥에 넘어짐에 대한 페널티
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    lin_vel_z_l2 = None

    # 목표 선속도를 유지하면 보상 >> weight = 1.0 //오차율 0.25로 인하
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_yaw_frame_exp,
        weight=1.0, #5.0
        params={"command_name": "base_velocity", "std": 0.15},  # threshold 변경 0.5 >> 0.15
    )

    # 목표 각속도를 유지하면 보상 >> weight = 1.0 // 오차율 0.25로 인하
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_world_exp, 
        weight=1.0, 
        params={"command_name": "base_velocity", "std": 0.15}  # threshold 변경 0.5 >> 0.15
    )

    # 지면 접촉 피드백 보상
    feet_air_time = RewTerm(
        func=mdp.feet_air_time_positive_biped,
        weight=0.25, # 0.25
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["Foot_R_1", "Foot_L_1"]),
            "threshold": 0.15, # 원본 0.4 에서 0.15로 변경 (스케일다운)
        },
    )

    # 발이 미끄러짐에 대한 페널티
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.25,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["Foot_R_1", "Foot_L_1"]),
            "asset_cfg": SceneEntityCfg("robot", body_names=["Foot_R_1", "Foot_L_1"]),
        },
    )


    # 제한 관절 범위를 넘어가면 페널티
    dof_pos_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=1e-4,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[
                "rotate_10", "rotate_11", "rotate_12", "rotate_13", "rotate_14", "rotate_15",
                "rotate_16", "rotate_17", "rotate_18", "rotate_19", "rotate_20", "rotate_21"
            ])
        }
    )

    #     # 무게중심이 양발 사이에 위치하면 보상
    # centered_com = RewTerm(
    #     func=mdp.center_of_mass_over_feet,
    #     weight=1.0,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names=["MX_21_1", "MX_20_1"])},
    # )

    # # 발 사이 거리 벌어짐을 패널티
    # stance_penalty = RewTerm(
    #     func=mdp.penalize_wide_stance,
    #     weight=0.2,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names=["MX_21_1", "MX_20_1"])},
    # )



@configclass
class RobitRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    rewards: RobitRewards = RobitRewards()

    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # Scene
        self.scene.robot = ROBIT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Randomization
        self.events.push_robot = None
        self.events.add_base_mass = None
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)

        self.events.base_external_force_torque.params["asset_cfg"].body_names = ["base_link"]
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5) ,"yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        }

        # Terminations
        self.terminations.base_contact.params["sensor_cfg"].body_names = ["base_link"]

        # Rewards
        self.rewards.undesired_contacts = None
        self.rewards.flat_orientation_l2.weight = -1.0
        # self.rewards.dof_torques_l2.weight = 1e-4
        self.rewards.action_rate_l2.weight = -0.005
        self.rewards.dof_acc_l2.weight = -1.25e-7
        self.rewards.dof_pos_limits.weight = 0.001

        # Commands
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.54, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.3, 0.3)

        # self.commands.base_velocity.control_type = "random"
        self.commands.base_velocity.visualize = True
        # self.commands.base_velocity.visualization_cfg.prim_path = "{ENV_REGEX_NS}/base_velocity_arrow"

@configclass
class RobitRoughEnvCfg_PLAY(RobitRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.episode_length_s = 40.0
        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.max_init_terrain_level = None
        # reduce the number of terrains to save memory
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)
        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing
        self.events.base_external_force_torque = None
        self.events.push_robot = None
