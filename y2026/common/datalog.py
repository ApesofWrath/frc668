from typing import Any, Dict, Optional, Type, TypeVar

import wpilib
from wpiutil import log

T = TypeVar("T")


class DataLogger:
    def __init__(self):
        self._log: Optional[log.DataLog] = None
        # Map of topic name to LogEntry object.
        self._entries: Dict[str, Any] = {}

    def _get_log(self) -> log.DataLog:
        """Get the DataLog instance, initializing it if needed."""
        if self._log is None:
            self._log = wpilib.DataLogManager.getLog()
        return self._log

    def flush(self) -> None:
        self._get_log().flush()

    def log_struct(
        self,
        topic_name: str,
        value: T,
        struct_type: Type[T],
        on_change: bool = False,
    ) -> None:
        """Log a single struct value (eg: Pose2d).

        Args:
            topic_name: The name of the topic to log to.
            value: The value to log.
            struct_type: The type of the value.
            on_change: If True, only logs the value if it changed since the last
                value provided for this topic. Defaults to False.
        """
        if topic_name not in self._entries:
            self._entries[topic_name] = log.StructLogEntry(
                self._get_log(), topic_name, struct_type
            )
        if on_change:
            self._entries[topic_name].update(value)
        else:
            self._entries[topic_name].append(value)

    def log_struct_array(
        self,
        topic_name: str,
        values: list[T],
        struct_type: Type[T],
        on_change: bool = False,
    ) -> None:
        """Log an array of structs (eg: Pose2d[]).

        Args:
            topic_name: The name of the topic to log to.
            values: The list of values to log.
            struct_type: The type of the value.
            on_change: If True, only logs the value if it changed since the last
                value provided for this topic. Defaults to False.
        """
        if topic_name not in self._entries:
            self._entries[topic_name] = log.StructArrayLogEntry(
                self._get_log(), topic_name, struct_type
            )
        if on_change:
            self._entries[topic_name].update(values)
        else:
            self._entries[topic_name].append(values)

    def log_string(
        self, topic_name: str, value: str, on_change: bool = False
    ) -> None:
        """Log a string value.

        Args:
            topic_name: The name of the topic to log to.
            value: The string value to log.
            on_change: If True, only logs the value if it changed since the last
                value provided for this topic. Defaults to False.
        """
        if topic_name not in self._entries:
            self._entries[topic_name] = log.StringLogEntry(
                self._get_log(), topic_name
            )
        if on_change:
            self._entries[topic_name].update(value)
        else:
            self._entries[topic_name].append(value)

    def log_string_array(
        self, topic_name: str, values: list[str], on_change: bool = False
    ) -> None:
        """Log an array of strings.

        Args:
            topic_name: The name of the topic to log to.
            values: The list of string values to log.
            on_change: If True, only logs the value if it changed since the last
                value provided for this topic. Defaults to False.
        """
        if topic_name not in self._entries:
            self._entries[topic_name] = log.StringArrayLogEntry(
                self._get_log(), topic_name
            )
        if on_change:
            self._entries[topic_name].update(values)
        else:
            self._entries[topic_name].append(values)

    def log_double(
        self, topic_name: str, value: float, on_change: bool = True
    ) -> None:
        """Log a double value.

        Args:
            topic_name: The name of the topic to log to.
            value: The double value to log.
            on_change: If True, only logs the value if it changed since the last
                value provided for this topic. Defaults to False.
        """
        if topic_name not in self._entries:
            self._entries[topic_name] = log.DoubleLogEntry(
                self._get_log(), topic_name
            )
        if on_change:
            self._entries[topic_name].update(value)
        else:
            self._entries[topic_name].append(value)

    def log_boolean(
        self, topic_name: str, value: bool, on_change: bool = True
    ) -> None:
        """Log a boolean value.

        Args:
            topic_name: The name of the topic to log to.
            value: The boolean value to log.
            on_change: If True, only logs the value if it changed since the last
                value provided for this topic. Defaults to False.
        """
        if topic_name not in self._entries:
            self._entries[topic_name] = log.BooleanLogEntry(
                self._get_log(), topic_name
            )
        if on_change:
            self._entries[topic_name].update(value)
        else:
            self._entries[topic_name].append(value)


def log_primary_motor_data(
    data_logger: DataLogger,
    topic_prefix: str,
    motor: phoenix6.hardware.TalonFX,
    position: bool = False,
    velocity: bool = False,
) -> None:
    """Log primary motor data.

    This includes things like currents, position, and velocity, which are
    typically intended to be logged at a high rate.

    Args:
        data_logger:
            The data logger to use. Assumes the log is open for writing.
        topic_prefix:
            The prefix for the topic names.
        motor:
            The motor to log data for.
        position:
            If True, log position data. Defaults to False.
        velocity:
            If True, log velocity data. Defaults to False.
    """
    data_logger.log_double(
        f"{topic_prefix}/supply_current", motor.get_supply_current().value
    )
    data_logger.log_double(
        f"{topic_prefix}/stator_current", motor.get_stator_current().value
    )
    if position:
        data_logger.log_double(
            f"{topic_prefix}/position_rotations", motor.get_position().value
        )
        data_logger.log_double(
            f"{topic_prefix}/rotor_position_rotations",
            motor.get_rotor_position().value,
        )
    if velocity:
        data_logger.log_double(
            f"{topic_prefix}/velocity_rotations_per_second",
            motor.get_velocity().value,
        )
        data_logger.log_double(
            f"{topic_prefix}/rotor_velocity_rotations_per_second",
            motor.get_rotor_velocity().value,
        )


def log_secondary_motor_data(
    data_logger: DataLogger,
    topic_prefix: str,
    motor: phoenix6.hardware.TalonFX,
) -> None:
    """Log secondary motor data.

    This includes things like temperatures and faults, which are typically
    logged at a lower rate.

    Args:
        data_logger: The data logger to use.
        topic_prefix: The prefix for the channel names.
        motor: The motor to log data for.
    """
    data_logger.log_double(
        f"{topic_prefix}/device_temp", motor.get_device_temp().value
    )
    data_logger.log_double(
        f"{topic_prefix}/processor_temp", motor.get_processor_temp().value
    )
    data_logger.log_boolean(
        f"{topic_prefix}/device_temp_fault",
        motor.get_fault_device_temp().value,
    )
    data_logger.log_boolean(
        f"{topic_prefix}/processor_temp_fault",
        motor.get_fault_proc_temp().value,
    )
    data_logger.log_boolean(
        f"{topic_prefix}/supply_current_limit_fault",
        motor.get_fault_supply_curr_limit().value,
    )
    data_logger.log_boolean(
        f"{topic_prefix}/stator_current_limit_fault",
        motor.get_fault_stator_curr_limit().value,
    )
