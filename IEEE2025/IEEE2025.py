# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for Robit IEEE 2025 humanoid robot."""

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

IEEE2025_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/lwj/usd_ws/robit/robit_humanoid/masterHumanoid2025/masterHumanoid2025.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
			linear_damping=0.5, # unit N/(m/s) >> 유니트리 기준 0.0
            angular_damping=0.5, # unit N/(m/s) >> 유니트리 기준 0.0  액추에이터 댐핑값이랑 다름
            max_linear_velocity=5.0, # unit m/s >> 유니트리 기준 1000.0
            max_angular_velocity=20.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=4, solver_velocity_iteration_count=4
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0), # z = 1.05
        joint_pos={
			# 고관절 / 발목 롤 축 offset 5 deg (0.0872665 rad)
			"rotate_10": -0.0872665, "rotate_11": 0.0872665,
			"rotate_20": 0.0872665, "rotate_21": -0.0872665,

            # 고관절 요 축
            "rotate_12": 0.0, "rotate_13": 0.0,

            # 고관절 / 무릎 / 발목 피치 축 27 / 45 / 23 degree
            "rotate_14": -0.471239, "rotate_15": 0.471239,
			"rotate_16": 0.785398, "rotate_17": -0.785398,
			"rotate_18": -0.401426, "rotate_19": 0.401426
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.95, # 95% of the joint position limits >> 좀더 널널하게 해도 ㄱㅊ
    actuators={
        "DYNAMIXEL_MX_106": ImplicitActuatorCfg(
            joint_names_expr=[
                "rotate_12", "rotate_13"
            ],
            effort_limit=10.0,
            velocity_limit=5.76,
            stiffness={
                "rotate_12": 6.640625, "rotate_13": 6.640625 # DYNAMIXEL_RDK_ROS2 P_Gain = 6.640625
            },
            damping={
                "rotate_12": 0.05, "rotate_13": 0.05 # LET. DYNAMIXEL_RDK_ROS2 P_DGain = 0.05
            },
        ),
		"DUAL_MOTOR": ImplicitActuatorCfg(
            joint_names_expr=[
                "rotate_14", "rotate_15",
				"rotate_16", "rotate_17",
				"rotate_18", "rotate_19"
            ],
            effort_limit=20.0,
            velocity_limit=5.76,
            stiffness={
                "rotate_14": 6.640625, "rotate_15": 6.640625,
                "rotate_16": 6.640625, "rotate_17": 6.640625,
                "rotate_18": 6.640625, "rotate_19": 6.640625
            },
            damping={
                "rotate_14": 0.11, "rotate_15": 0.11,
                "rotate_16": 0.11, "rotate_17": 0.11,
                "rotate_18": 0.11, "rotate_19": 0.11
            },
        ),
		"GEAR_BOX": ImplicitActuatorCfg(
            joint_names_expr=[
                "rotate_10", "rotate_11",
                "rotate_20", "rotate_21",
            ],
            effort_limit=15.0,
            velocity_limit=3.84,
            stiffness={
                "rotate_10": 6.640625, "rotate_11": 6.640625,
                "rotate_20": 6.640625, "rotate_21": 6.640625
            },
            damping={
                "rotate_10": 0.1125, "rotate_11": 0.1125,
                "rotate_20": 0.1125, "rotate_21": 0.1125
            },
        ),
    },
)

"""Configuration for the Robit IEEE 2025 humanoid robot with custom actuator settings."""

