"""Helpers to build shared-bus component assemblies for XLeRobot."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from lerobot.robots.lekiwi_base.config import LeKiwiBaseConfig
from lerobot.robots.lekiwi_base.lekiwi_base import LeKiwiBase
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.robots.xlerobot_mount.config import XLeRobotMountConfig
from lerobot.robots.xlerobot_mount.xlerobot_mount import XLeRobotMount

from ..config_xlerobot import XLeRobotConfig
from .shared_bus import FeetechBusManager


@dataclass
class ComponentBinding:
    name: str
    role: str
    prefix: str
    aliases: list[str]
    robot: Any
    bus_manager: FeetechBusManager


COMPONENT_REGISTRY: dict[str, tuple[type[Any], type[Any]]] = {
    "so101_follower": (SO101Follower, SO101FollowerConfig),
    "lekiwi_base": (LeKiwiBase, LeKiwiBaseConfig),
    "xlerobot_mount": (XLeRobotMount, XLeRobotMountConfig),
}


def _make_config(config_cls: type[Any], raw_cfg: dict[str, Any], port: str) -> Any:
    cfg_copy = dict(raw_cfg)
    cfg_copy.setdefault("port", port)
    return config_cls(**cfg_copy)


def build_shared_components(
    cfg: XLeRobotConfig,
) -> tuple[list[ComponentBinding], dict[str, FeetechBusManager]]:
    bus_managers: dict[str, FeetechBusManager] = {
        name: FeetechBusManager(name, bus_cfg) for name, bus_cfg in cfg.shared_bus_configs.items()
    }

    bindings: list[ComponentBinding] = []
    for component_spec in cfg.shared_component_configs:
        registry_entry = COMPONENT_REGISTRY.get(component_spec.type)
        if not registry_entry:
            raise ValueError(f"Unsupported shared component type '{component_spec.type}'.")
        robot_cls, config_cls = registry_entry
        bus_manager = bus_managers.get(component_spec.bus)
        if bus_manager is None:
            raise ValueError(f"Component '{component_spec.name}' references unknown bus '{component_spec.bus}'.")

        component_config = _make_config(config_cls, component_spec.config, bus_manager.config.port)
        robot = robot_cls(component_config)

        # Allocate shared bus view replacing the default bus
        view = bus_manager.register_component(
            component_name=component_spec.name,
            local_motors=robot.bus.motors,
            prefix=component_spec.prefix,
            motor_id_offset=component_spec.motor_id_offset,
            initial_calibration=robot.calibration,
        )
        robot.bus = view

        aliases = list(component_spec.action_aliases or [])
        if component_spec.prefix and component_spec.prefix not in aliases:
            aliases.append(component_spec.prefix)

        bindings.append(
            ComponentBinding(
                name=component_spec.name,
                role=component_spec.role,
                prefix=component_spec.prefix,
                aliases=aliases,
                robot=robot,
                bus_manager=bus_manager,
            )
        )

    return bindings, bus_managers
