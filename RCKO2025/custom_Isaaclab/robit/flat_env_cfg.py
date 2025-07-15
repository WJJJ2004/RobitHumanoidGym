# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

from .rough_env_cfg import RobitRoughEnvCfg


@configclass
class RobitFlatEnvCfg(RobitRoughEnvCfg):
    def __post_init__(self):
        # 부모 클래스의 설정을 먼저 적용
        super().__post_init__()

        # Terrain 설정을 평지로 변경
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None

        # Height Scanner 사용 안 함
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None

        # Terrain Curriculum 제거
        self.curriculum.terrain_levels = None
        self.rewards.feet_air_time.weight = 1.0
        self.rewards.feet_air_time.params["threshold"] = 0.3 # 0.6

        # self.commands.base_velocity.ranges.lin_vel_x = (0.0, 1.0)
        # self.commands.base_velocity.ranges.lin_vel_y = (-0.5, 0.5)
        # self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)

@configclass
class RobitFlatEnvCfg_PLAY(RobitFlatEnvCfg):
    def __post_init__(self) -> None:
        # 부모 클래스의 설정을 먼저 적용
        super().__post_init__()

        # Play 모드를 위한 환경 크기 조정
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5

        # 랜덤 노이즈 제거
        self.observations.policy.enable_corruption = False

        # 랜덤 외부 힘 이벤트 제거
        self.events.base_external_force_torque = None
        self.events.push_robot = None
