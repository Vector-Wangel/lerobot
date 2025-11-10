"""Infrastructure for sharing a single Feetech bus across multiple XLeRobot sub-components."""

from __future__ import annotations

from collections import defaultdict
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any, Dict, Iterable

from lerobot.motors import Motor, MotorCalibration
from lerobot.motors.feetech import FeetechMotorsBus

from ..config_xlerobot import SharedBusAssemblyConfig


@dataclass
class FeetechBusView:
    """Proxy that exposes a motor subset on a shared Feetech bus."""

    manager: "FeetechBusManager"
    local_motors: Dict[str, Motor]
    local_to_bus: Dict[str, str]

    def __post_init__(self) -> None:
        self._connected = False

    # ------------------------------------------------------------------
    # Helpers
    def _translate_motor(self, motor: str | int | None) -> str | int | None:
        if motor is None:
            return None
        if isinstance(motor, int):
            return motor
        return self.local_to_bus[motor]

    def _translate_motors(self, motors: str | list[str] | None) -> list[str] | None:
        if motors is None:
            return [self.local_to_bus[name] for name in self.local_motors]
        if isinstance(motors, str):
            return [self.local_to_bus[motors]]
        return [self.local_to_bus[name] for name in motors]

    def _translate_dict(self, values: dict[str, Any]) -> dict[str, Any]:
        return {self.local_to_bus[name]: value for name, value in values.items()}

    def _map_dict_to_local(self, values: dict[str, Any]) -> dict[str, Any]:
        reverse = {bus_name: local for local, bus_name in self.local_to_bus.items()}
        return {reverse[name]: value for name, value in values.items() if name in reverse}

    # ------------------------------------------------------------------
    # Properties mirroring MotorsBus
    @property
    def motors(self) -> dict[str, Motor]:
        return self.local_motors

    @property
    def calibration(self) -> dict[str, MotorCalibration]:
        bus = self.manager.bus
        if bus is None:
            return {}
        return self._map_dict_to_local(bus.calibration)

    @property
    def port(self) -> str:
        return self.manager.config.port

    @property
    def is_connected(self) -> bool:
        return bool(self.manager.bus and self.manager.bus.is_connected)

    @property
    def is_calibrated(self) -> bool:
        bus = self.manager.bus
        if bus is None or not bus.is_connected:
            return False
        return all(self.local_to_bus[name] in bus.calibration for name in self.local_motors)

    # ------------------------------------------------------------------
    # Connection lifecycle
    def connect(self, handshake: bool = True) -> None:
        handshake_flag = handshake
        self.manager.acquire(handshake_flag)
        self._connected = True

    def disconnect(self, disable_torque: bool = True) -> None:
        if not self._connected:
            return
        if disable_torque and self.is_connected:
            self.disable_torque()
        self.manager.release(disable_torque=disable_torque)
        self._connected = False

    # ------------------------------------------------------------------
    # Delegated bus operations
    def disable_torque(self, motors: int | str | list[str] | None = None, num_retry: int = 0) -> None:
        subset = self._translate_motors(motors)
        if subset and self.manager.bus:
            self.manager.bus.disable_torque(subset, num_retry=num_retry)

    def enable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        subset = self._translate_motors(motors)
        if subset and self.manager.bus:
            self.manager.bus.enable_torque(subset, num_retry=num_retry)

    @contextmanager
    def torque_disabled(self, motors: int | str | list[str] | None = None):
        subset = self._translate_motors(motors)
        if subset and self.manager.bus:
            with self.manager.bus.torque_disabled(subset):
                yield
        else:
            yield

    def configure_motors(self, motors: str | list[str] | None = None) -> None:
        subset = self._translate_motors(motors)
        if subset and self.manager.bus:
            self.manager.bus.configure_motors(subset)

    def write(self, data_name: str, motor: str | int, value: Any, *, normalize: bool = True, num_retry: int = 0) -> None:
        if self.manager.bus is None:
            raise RuntimeError("Shared bus not initialized.")
        target = self._translate_motor(motor)
        self.manager.bus.write(data_name, target, value, normalize=normalize, num_retry=num_retry)

    def sync_write(self, data_name: str, values: dict[str, Any], *, normalize: bool = True, num_retry: int = 0) -> None:
        if self.manager.bus is None:
            raise RuntimeError("Shared bus not initialized.")
        payload = self._translate_dict(values)
        self.manager.bus.sync_write(data_name, payload, normalize=normalize, num_retry=num_retry)

    def sync_read(
        self,
        data_name: str,
        motors: str | list[str] | None = None,
        *,
        normalize: bool = True,
        num_retry: int = 0,
    ) -> dict[str, Any]:
        if self.manager.bus is None:
            raise RuntimeError("Shared bus not initialized.")
        subset = self._translate_motors(motors)
        result = self.manager.bus.sync_read(data_name, subset, normalize=normalize, num_retry=num_retry)
        return self._map_dict_to_local(result)

    def set_half_turn_homings(self, motors: list[str] | None = None) -> dict[str, int]:
        if self.manager.bus is None:
            raise RuntimeError("Shared bus not initialized.")
        subset = self._translate_motors(motors)
        result = self.manager.bus.set_half_turn_homings(subset)
        return self._map_dict_to_local(result)

    def record_ranges_of_motion(self, motors: list[str] | None = None) -> tuple[dict[str, int], dict[str, int]]:
        if self.manager.bus is None:
            raise RuntimeError("Shared bus not initialized.")
        subset = self._translate_motors(motors)
        mins, maxes = self.manager.bus.record_ranges_of_motion(subset)
        return self._map_dict_to_local(mins), self._map_dict_to_local(maxes)

    def write_calibration(self, calibration: dict[str, MotorCalibration]) -> None:
        if self.manager.bus is None:
            raise RuntimeError("Shared bus not initialized.")
        translated = self._translate_dict(calibration)
        self.manager.bus.write_calibration(translated)

    def setup_motor(self, motor: str, initial_baudrate: int | None = None, initial_id: int | None = None) -> None:
        if self.manager.bus is None:
            raise RuntimeError("Shared bus not initialized.")
        target = self._translate_motor(motor)
        self.manager.bus.setup_motor(target, initial_baudrate=initial_baudrate, initial_id=initial_id)


class FeetechBusManager:
    """Tracks a shared Feetech bus and provides component-specific views."""

    def __init__(self, name: str, config: SharedBusAssemblyConfig):
        self.name = name
        self.config = config
        self._motor_defs: dict[str, Motor] = {}
        self._calibration: dict[str, MotorCalibration] = {}
        self.views: dict[str, FeetechBusView] = {}
        self.bus: FeetechMotorsBus | None = None
        self._refcount = 0

    # ------------------------------------------------------------------
    def register_component(
        self,
        component_name: str,
        local_motors: dict[str, Motor],
        prefix: str,
        motor_id_offset: int,
        initial_calibration: dict[str, MotorCalibration] | None = None,
    ) -> FeetechBusView:
        local_copy: dict[str, Motor] = {}
        mapping: dict[str, str] = {}
        for motor_name, motor in local_motors.items():
            bus_name = f"{prefix}{motor_name}"
            if bus_name in self._motor_defs:
                raise ValueError(f"Motor name collision on shared bus '{self.name}': {bus_name}")
            updated_motor = Motor(
                id=motor.id + motor_id_offset,
                model=motor.model,
                norm_mode=motor.norm_mode,
            )
            self._motor_defs[bus_name] = updated_motor
            local_copy[motor_name] = Motor(
                id=updated_motor.id,
                model=updated_motor.model,
                norm_mode=updated_motor.norm_mode,
            )
            mapping[motor_name] = bus_name
            if initial_calibration and motor_name in initial_calibration:
                self._calibration[bus_name] = initial_calibration[motor_name]

        view = FeetechBusView(self, local_copy, mapping)
        self.views[component_name] = view
        return view

    def _ensure_bus(self) -> None:
        if self.bus is None:
            calibration = self._calibration or None
            self.bus = FeetechMotorsBus(
                port=self.config.port,
                motors=self._motor_defs,
                calibration=calibration,
                protocol_version=self.config.protocol_version,
            )

    # ------------------------------------------------------------------
    def acquire(self, handshake: bool = True) -> None:
        self._ensure_bus()
        self._refcount += 1
        if self._refcount == 1 and self.bus and not self.bus.is_connected:
            handshake_flag = self.config.handshake_on_connect if handshake is None else handshake
            self.bus.connect(handshake=handshake_flag)

    def release(self, disable_torque: bool = True) -> None:
        if self._refcount == 0:
            return
        self._refcount -= 1
        if self._refcount == 0 and self.bus and self.bus.is_connected:
            self.bus.disconnect(disable_torque=disable_torque)

    @property
    def is_connected(self) -> bool:
        return bool(self.bus and self.bus.is_connected)
