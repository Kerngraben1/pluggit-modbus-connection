"""Modbus implementation."""

from __future__ import annotations

import asyncio
from enum import IntEnum
import logging
import re
from typing import TYPE_CHECKING, Any

from modbus_connection import ModbusError, ModbusUnit
from modbus_connection.decode import (
    decode_float32,
    decode_int32,
    decode_uint32,
    decode_uint64,
)
from modbus_connection.encode import encode_float32, encode_uint32

# All Dantherm/Pluggit registers are little-endian word order
# (identical to the previous pymodbus `convert_from_registers(..., "little")`).
WORD_ORDER = "little"

if TYPE_CHECKING:
    from .device_map import DanthermEntityDescription

# Modbus Register Constants
MODBUS_REGISTER_AB_SWITCH_POSITION_HAL_LEFT = 84
MODBUS_REGISTER_AB_SWITCH_POSITION_HAL_RIGHT = 86
MODBUS_REGISTER_ACTIVE_MODE = 168
MODBUS_REGISTER_AIR_QUALITY = 430
MODBUS_REGISTER_AIR_QUALITY_LOW_THRESHOLD = 562
MODBUS_REGISTER_AIR_QUALITY_MIDDLE_THRESHOLD = 564
MODBUS_REGISTER_AIR_QUALITY_HIGH_THRESHOLD = 566
MODBUS_REGISTER_ALARM = 516
MODBUS_REGISTER_ALARM_RESET = 514
MODBUS_REGISTER_BYPASS_DAMPER = 198
MODBUS_REGISTER_BYPASS_MAX_TEMP = 446
MODBUS_REGISTER_BYPASS_MIN_TEMP = 444
MODBUS_REGISTER_BYPASS_MAX_TEMP_SUMMER = 764
MODBUS_REGISTER_BYPASS_MIN_TEMP_SUMMER = 766
MODBUS_REGISTER_CURRENT_MODE = 472
MODBUS_REGISTER_DATETIME = 108
MODBUS_REGISTER_DATETIME_SET = 110
MODBUS_REGISTER_EXHAUST_TEMP = 138
MODBUS_REGISTER_EXTRACT_TEMP = 136
MODBUS_REGISTER_FAN_LEVEL = 324
MODBUS_REGISTER_FAN1_SPEED = 100
MODBUS_REGISTER_FAN2_SPEED = 102
MODBUS_REGISTER_FILTER_DIRTINESS = 612
MODBUS_REGISTER_FILTER_LIFETIME = 556
MODBUS_REGISTER_FILTER_REMAIN = 554
MODBUS_REGISTER_FILTER_RESET = 558
MODBUS_REGISTER_FIRMWARE_VERSION = 24
MODBUS_REGISTER_HUMIDITY = 196
MODBUS_REGISTER_HUMIDITY_SETPOINT = 340
MODBUS_REGISTER_HUMIDITY_SETPOINT_SUMMER = 768
MODBUS_REGISTER_INTERNAL_PREHEATER_DUTYCYCLE = 160
MODBUS_REGISTER_MANUAL_BYPASS_DURATION = 264
MODBUS_REGISTER_NIGHT_MODE_END_HOUR = 336
MODBUS_REGISTER_NIGHT_MODE_END_MINUTE = 338
MODBUS_REGISTER_NIGHT_MODE_START_HOUR = 332
MODBUS_REGISTER_NIGHT_MODE_START_MINUTE = 334
MODBUS_REGISTER_OUTDOOR_TEMP = 132
MODBUS_REGISTER_ROOM_TEMP = 140
MODBUS_REGISTER_SERIAL_NUMBER = 4
MODBUS_REGISTER_SERVOFLOW_ENABLED = 448
MODBUS_REGISTER_SUPPLY_TEMP = 134
MODBUS_REGISTER_SYSTEM_ID = 2
MODBUS_REGISTER_SYSTEM_ID_COMPONENTS = 610
MODBUS_REGISTER_WEEK_PROGRAM_SELECTION = 466
MODBUS_REGISTER_WORK_TIME = 624


class DataClass(IntEnum):
    """Dantherm modbus data class."""

    Int32 = 1
    UInt32 = 2
    UInt64 = 3
    Float32 = 4


class ABSwitchPosition(IntEnum):
    """Dantherm A/B switch position class."""

    Unknown = 0
    A = 1
    B = 2


_LOGGER = logging.getLogger(__name__)


class DanthermModbus:
    """Dantherm Modbus."""

    def __init__(
        self,
        name: str,
        unit: ModbusUnit,
    ) -> None:
        """Initialize Modbus wrapper around a shared connection unit.

        The unit comes from the modbus integration (`async_get_unit` /
        `async_get_temporary_unit`). Connection lifecycle (open, reopen,
        close) is managed by the framework, never here.
        """
        self._name = name
        self._unit = unit
        self._attr_available = False
        self._read_errors = 0
        # coordinator will be set by subclass

    @property
    def coordinator(self) -> Any:
        """Coordinator property - to be implemented by subclass."""
        # This will be overridden in DanthermDevice
        return None

    @coordinator.setter
    def coordinator(self, value: Any) -> None:
        """Set coordinator - for testing purposes."""
        # This setter is mainly for test mocking

    async def connect_and_verify(self) -> bool:
        """Verify the device answers on the shared connection.

        Asking for a unit does no I/O; the first request opens the link
        and a dropped link is reopened by the framework on the next
        request. So this only probes the device, it never opens a
        connection itself.
        """

        _LOGGER.debug("Verifying Modbus response for %s", self._name)
        for _ in range(5):
            result = await self._read_holding_uint32(MODBUS_REGISTER_SYSTEM_ID)
            if result is not None:
                _LOGGER.debug("Modbus device is responding!")
                self._attr_available = True
                return True
            await asyncio.sleep(1)

        _LOGGER.error("Modbus device failed to respond for %s", self._name)
        raise ValueError("Modbus device failed to respond")

    async def disconnect_and_close(self) -> None:
        """Kept for backward compatibility with existing call sites.

        The shared connection is owned by the modbus integration and is
        closed automatically when the last config entry using it unloads,
        or when the config flow's temporary unit context exits. Do not
        close it here.
        """
        _LOGGER.debug(
            "Connection lifecycle for %s is managed by the modbus integration",
            self._name,
        )

    async def read_holding_registers(
        self,
        description: DanthermEntityDescription | None = None,
        address: int | None = None,
        count: int = 1,
        precision: int | None = None,
        scale: int = 1,
    ) -> int | float | str | list[bool] | list[int] | list[float] | None:
        """Read modbus holding registers."""
        result: int | float | str | list[bool] | list[int] | list[float] | None = None
        if description:
            if address is None:
                address = description.data_address
            if address is not None:
                match description.data_class:
                    case DataClass.Int32:
                        result = await self._read_holding_int32(address)
                    case DataClass.UInt32:
                        result = await self._read_holding_uint32(address)
                    case DataClass.UInt64:
                        result = await self._read_holding_uint64(address)
                    case DataClass.Float32:
                        if precision is None:
                            precision = getattr(description, "data_precision", None)
                        result = await self._read_holding_float32(address, precision)
        elif address:
            match count:
                case 1:
                    result = await self._read_holding_uint16(address)
                case 2:
                    result = await self._read_holding_uint32(address)
                case 4:
                    result = await self._read_holding_uint64(address)
        if result is None:
            _LOGGER.debug("Reading holding register=%s failed", str(address))
            return None
        result *= scale
        _LOGGER.debug("Reading holding register=%s result=%s", str(address), result)
        return result

    async def write_holding_registers(
        self,
        description: DanthermEntityDescription | None = None,
        address: int | None = None,
        value: int = 0,
        scale: int = 1,
    ) -> None:
        """Write modbus holding registers."""
        value *= scale
        if description:
            if not address:
                address = description.data_setaddress or description.data_address
            data_class = description.data_setclass or description.data_class
            match data_class:
                case DataClass.UInt32:
                    if address is not None:
                        # Note: coordinator queue handling is done internally in _write_holding_uint32
                        await self._write_holding_uint32(address, value)
                case DataClass.Float32:
                    if address is not None:
                        # Note: coordinator queue handling is done internally in _write_holding_float32
                        await self._write_holding_float32(address, value)
        else:
            if address is not None:
                if self.coordinator is not None:
                    await self.coordinator.enqueue_backend(
                        self.__write_holding_registers, address, [value]
                    )
                else:
                    await self.__write_holding_registers(address, [value])

    async def get_device_ab_switch_position(self) -> ABSwitchPosition:
        """Get device A/B switch position."""

        HALLeft = await self._read_holding_uint32(
            MODBUS_REGISTER_AB_SWITCH_POSITION_HAL_LEFT
        )
        _LOGGER.debug("HALLeft = %s", HALLeft)
        HALRight = await self._read_holding_uint32(
            MODBUS_REGISTER_AB_SWITCH_POSITION_HAL_RIGHT
        )
        _LOGGER.debug("HALRight = %s", HALRight)

        result = ABSwitchPosition.Unknown
        if HALRight == 1 and HALLeft == 0:
            result = ABSwitchPosition.A
        elif HALRight == 0 and HALLeft == 1:
            result = ABSwitchPosition.B
        _LOGGER.debug("Device A/B switch position = %s", result.name)
        return result

    async def __read_holding_registers_with_retry(
        self, address: int, count: int, retries: int = 3, initial_delay: float = 0.5
    ) -> list[int] | None:
        """Read holding registers with retry using exponential backoff."""
        delay = initial_delay
        for _ in range(retries):
            result = await self.__read_holding_registers(address, count)
            if result is not None:
                return result
            if self._attr_available is False:
                return None
            await asyncio.sleep(delay)
            delay *= 2
        _LOGGER.error("Failed to read holding registers for address %s", address)
        return None

    async def __read_holding_registers(
        self, address: int, count: int
    ) -> list[int] | None:
        """Read holding registers through the shared connection unit."""
        try:
            registers = await self._unit.read_holding_registers(address, count)
        except ModbusError as err:
            _LOGGER.error("Read holding registers failed: %s", err)
            self._read_errors += 1
            return None
        self._read_errors = 0
        return registers

    async def __write_holding_registers(self, address: int, values: list[int]) -> None:
        """Write holding registers through the shared connection unit."""
        try:
            await self._unit.write_registers(address, values)
            _LOGGER.debug("Written %s to register address %d", values, address)
        except ModbusError as err:
            _LOGGER.warning("Write holding registers failed: %s", err)
            self._attr_available = False

    async def _read_holding_uint16(self, address: int) -> int | None:
        result = await self._read_holding_uint32(address)
        return result & 0xFFFF if result is not None else None

    async def _read_holding_int32(self, address: int) -> int | None:
        result = await self.__read_holding_registers_with_retry(address, 2)
        if result is None:
            return None
        return decode_int32(result, word_order=WORD_ORDER)

    async def _read_holding_uint32(self, address: int) -> int | None:
        result = await self.__read_holding_registers_with_retry(address, 2)
        if result is None:
            return None
        return decode_uint32(result, word_order=WORD_ORDER)

    async def _write_holding_uint32(self, address: int, value: int | None) -> None:
        if value is None:
            return
        payload = encode_uint32(int(value), word_order=WORD_ORDER)
        await self.__enqueue_write(address, payload)

    async def _read_holding_uint64(self, address: int) -> int | None:
        result = await self.__read_holding_registers_with_retry(address, 4)
        if result is None:
            return None
        return decode_uint64(result, word_order=WORD_ORDER)

    async def _read_holding_float32(
        self, address: int, precision: int | None
    ) -> float | None:
        result = await self.__read_holding_registers_with_retry(address, 2)
        if result is None:
            return None
        value = decode_float32(result, word_order=WORD_ORDER)
        if precision is not None:
            if precision >= 0:
                value = round(value, precision)
            if precision == 0:
                value = int(value)
        return value

    async def _write_holding_float32(self, address: int, value: float | None) -> None:
        if value is None:
            return
        payload = encode_float32(float(value), word_order=WORD_ORDER)
        await self.__enqueue_write(address, payload)

    async def __enqueue_write(self, address: int, payload: list[int]) -> None:
        """Route a write through the coordinator queue when available."""
        coordinator = self.coordinator
        if coordinator is not None:
            await coordinator.enqueue_backend(
                self.__write_holding_registers, address, payload
            )
        else:
            await self.__write_holding_registers(address, payload)

    def _to_hex(self, value: int | str) -> str:
        """Convert value to hex string."""
        if isinstance(value, int):
            return hex(value)
        # value must be str at this point
        # Check if the string is a valid hexadecimal number
        if re.match(r"^0x[0-9a-fA-F]+$", value):
            return value
        # If not, convert it to an integer and then to hex
        return hex(int(value))

    async def _read_hac_controller(self) -> None:
        """Read HAC controller data."""

        class HacComponentClass(int):
            """Dantherm HAC components."""

            CO2Sensor = 0x0001
            PreHeater = 0x0004
            PreCooler = 0x0008
            AfterHeater = 0x0010
            AfterCooler = 0x0020
            Hygrostat = 0x0040

        _LOGGER.critical(
            "HAC controller found, please reach out for support collaboration"
        )

        result = await self._read_holding_uint32(574)
        _LOGGER.debug("HAC CO2 Level = %s ppm (574)", result)
        result = await self._read_holding_uint32(568)
        _LOGGER.debug("Low Threshold of CO2 = %s ppm (568)", result)
        result = await self._read_holding_uint32(570)
        _LOGGER.debug("Middle Threshold of CO2 = %s ppm (570)", result)
        result = await self._read_holding_uint32(572)
        _LOGGER.debug("High Threshold of CO2 = %s ppm (572)", result)
        result = await self._read_holding_uint32(244)
        if result is not None:
            _LOGGER.debug("Installed Hac components = %s (244)", hex(result))
            if result & HacComponentClass.CO2Sensor == HacComponentClass.CO2Sensor:
                _LOGGER.debug("CO2 sensor found")
            if result & HacComponentClass.PreHeater == HacComponentClass.PreHeater:
                _LOGGER.debug("Pre-heater found")
            if result & HacComponentClass.PreCooler == HacComponentClass.PreCooler:
                _LOGGER.debug("Pre-cooler found")
            if result & HacComponentClass.AfterHeater == HacComponentClass.AfterHeater:
                _LOGGER.debug("After-heater found")
            if result & HacComponentClass.AfterCooler == HacComponentClass.AfterCooler:
                _LOGGER.debug("After-cooler found")
        result = await self._read_holding_uint32(300)
        if result is not None:
            _LOGGER.debug("Hac active component = %s (300)", hex(result))
        result = await self._read_holding_int32(344)
        _LOGGER.debug("Setpoint of the T2 = %s °C (344)", result)
        result = await self._read_holding_int32(346)
        _LOGGER.debug("Setpoint of the T3 = %s °C (346)", result)
        result = await self._read_holding_int32(348)
        _LOGGER.debug("Setpoint of the T5 = %s °C (348)", result)
