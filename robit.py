# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for Robit humanoid robot."""

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

ROBIT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/lwj/usd_ws/robit/robit_humanoid/robit_humanoid/robit_humanoid.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0, # 1000.0
            max_angular_velocity=1000.0,  # 1000.0
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=4
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0), # z = 1.05
        joint_pos={
            "rotate_10": 0.0,       # 하체 요 & 롤 축 6개 관절
            "rotate_11": 0.0,
            "rotate_12": 0.0,
            "rotate_13": 0.0,
            "rotate_20": 0.0,
            "rotate_21": 0.0,

            "rotate_14": 0.0,       # 하체 피치축 고관절 >> 0.4363 rad = 25도
            "rotate_15": 0.0,
            "rotate_16": 0.0,       # 하체 롤축 무릎 >> 0.6981 rad = 40도
            "rotate_17": 0.0,
            "rotate_18": 0.0,        # 하체 롤축 발목 >> 0.2618 rad = 15도
            "rotate_19": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "lower_body": ImplicitActuatorCfg(
            joint_names_expr=[
                "rotate_10", "rotate_11", "rotate_12", "rotate_13", "rotate_14", "rotate_15",
                "rotate_16", "rotate_17", "rotate_18", "rotate_19", "rotate_20", "rotate_21"
            ],
            effort_limit=300,
            velocity_limit=100.0,
            stiffness={
                "rotate_10": 150.0, "rotate_11": 150.0, "rotate_12": 150.0, "rotate_13": 150.0,
                "rotate_14": 200.0, "rotate_15": 200.0, "rotate_16": 200.0, "rotate_17": 200.0,
                "rotate_18": 200.0, "rotate_19": 200.0, "rotate_20": 150.0, "rotate_21": 150.0,
            },
            damping={
                "rotate_10": 5.0, "rotate_11": 5.0, "rotate_12": 5.0, "rotate_13": 5.0,
                "rotate_14": 5.0, "rotate_15": 5.0, "rotate_16": 5.0, "rotate_17": 5.0,
                "rotate_18": 5.0, "rotate_19": 5.0, "rotate_20": 5.0, "rotate_21": 5.0,
            },
        ),
    },
)

"""Configuration for the Robit humanoid robot with custom actuator settings."""

