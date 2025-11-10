# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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

# XLeRobot integration based on
#   https://www.hackster.io/brainbot/brainbot-big-brain-with-xlerobot-ad1b4c
#   https://github.com/Astera-org/brainbot
#   https://github.com/Vector-Wangel/XLeRobot
#   https://github.com/bingogome/lerobot

from dataclasses import dataclass, field
from typing import Any

from lerobot.cameras import CameraConfig

from ..bi_so101_follower.config_bi_so101_follower import BiSO101FollowerConfig
from ..biwheel_base.config_biwheel_base import BiWheelBaseConfig
from ..config import RobotConfig
from ..lekiwi_base.config import LeKiwiBaseConfig
from ..xlerobot_mount.config import XLeRobotMountConfig


@dataclass
class SharedBusAssemblyConfig:
    type: str = "feetech"
    port: str = ""
    protocol_version: int = 0
    handshake_on_connect: bool = True


@dataclass
class SharedComponentAssemblyConfig:
    name: str
    type: str
    role: str
    bus: str
    prefix: str = ""
    motor_id_offset: int = 0
    action_aliases: list[str] = field(default_factory=list)
    config: dict[str, Any] = field(default_factory=dict)


@RobotConfig.register_subclass("xlerobot")
@dataclass
class XLeRobotConfig(RobotConfig):
    BASE_TYPE_LEKIWI = "lekiwi_base"
    BASE_TYPE_BIWHEEL = "biwheel_base"
    BUS_MODE_SEPARATE = "separate_buses"
    BUS_MODE_SHARED = "shared_buses"

    arms: dict[str, Any] = field(default_factory=dict)
    base: dict[str, Any] = field(default_factory=dict)
    mount: dict[str, Any] = field(default_factory=dict)
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    base_type: str | None = BASE_TYPE_LEKIWI
    bus_mode: str = BUS_MODE_SEPARATE
    shared_buses: dict[str, Any] = field(default_factory=dict)
    shared_components: list[dict[str, Any]] = field(default_factory=list)

    def __post_init__(self) -> None:
        super().__post_init__()
        self.bus_mode = self.bus_mode or self.BUS_MODE_SEPARATE
        if self.bus_mode == self.BUS_MODE_SHARED:
            self._init_shared_mode()
        else:
            self._init_separate_mode()

    def _init_separate_mode(self) -> None:
        arms_cfg: BiSO101FollowerConfig | None = None
        if isinstance(self.arms, BiSO101FollowerConfig):
            arms_cfg = self.arms
        elif self.arms:
            arms_cfg = BiSO101FollowerConfig(**self.arms)

        base_cfg: LeKiwiBaseConfig | BiWheelBaseConfig | None = None
        base_type = self.base_type or self.BASE_TYPE_LEKIWI
        if self.base:
            if base_type == self.BASE_TYPE_LEKIWI:
                base_cfg = (
                    self.base if isinstance(self.base, LeKiwiBaseConfig) else LeKiwiBaseConfig(**self.base)
                )
            elif base_type == self.BASE_TYPE_BIWHEEL:
                base_cfg = (
                    self.base if isinstance(self.base, BiWheelBaseConfig) else BiWheelBaseConfig(**self.base)
                )
            else:
                raise ValueError(f"Unsupported XLeRobot base type: {base_type}")
        else:
            base_type = None

        mount_cfg: XLeRobotMountConfig | None = None
        if isinstance(self.mount, XLeRobotMountConfig):
            mount_cfg = self.mount
        elif self.mount:
            mount_cfg = XLeRobotMountConfig(**self.mount)

        self.arms = arms_cfg
        self.base = base_cfg
        self.mount = mount_cfg
        self.arms_config: BiSO101FollowerConfig | None = arms_cfg
        self.base_config: LeKiwiBaseConfig | BiWheelBaseConfig | None = base_cfg
        self.mount_config: XLeRobotMountConfig | None = mount_cfg
        self.base_type = base_type

        if self.id:
            if arms_cfg and arms_cfg.id is None:
                arms_cfg.id = f"{self.id}_arms"
            if base_cfg and getattr(base_cfg, "id", None) is None:
                base_cfg.id = f"{self.id}_base"
            if mount_cfg and mount_cfg.id is None:
                mount_cfg.id = f"{self.id}_mount"

        if self.calibration_dir:
            if arms_cfg and arms_cfg.calibration_dir is None:
                arms_cfg.calibration_dir = self.calibration_dir
            if base_cfg and getattr(base_cfg, "calibration_dir", None) is None:
                base_cfg.calibration_dir = self.calibration_dir
            if mount_cfg and mount_cfg.calibration_dir is None:
                mount_cfg.calibration_dir = self.calibration_dir

        self.shared_bus_configs: dict[str, Any] = {}
        self.shared_component_configs: list[Any] = []

    def _init_shared_mode(self) -> None:
        if not self.shared_buses:
            raise ValueError("shared_buses must be provided when bus_mode is 'shared_buses'.")
        if not self.shared_components:
            raise ValueError("shared_components must be provided when bus_mode is 'shared_buses'.")

        self.arms = None
        self.base = None
        self.mount = None
        self.arms_config = None
        self.base_config = None
        self.mount_config = None
        self.base_type = None

        shared_buses: dict[str, SharedBusAssemblyConfig] = {}
        for name, cfg in self.shared_buses.items():
            if isinstance(cfg, SharedBusAssemblyConfig):
                shared_buses[name] = cfg
            else:
                shared_buses[name] = SharedBusAssemblyConfig(**cfg)
            if shared_buses[name].type != "feetech":
                raise ValueError("Only 'feetech' shared buses are supported currently.")

        shared_components: list[SharedComponentAssemblyConfig] = []
        for item in self.shared_components:
            if isinstance(item, SharedComponentAssemblyConfig):
                shared_components.append(item)
            else:
                shared_components.append(SharedComponentAssemblyConfig(**item))
            if shared_components[-1].bus not in shared_buses:
                raise ValueError(
                    f"Component '{shared_components[-1].name}' references unknown bus '{shared_components[-1].bus}'."
                )

        self.shared_bus_configs = shared_buses
        self.shared_component_configs = shared_components
