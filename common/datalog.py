from typing import Any, Dict, Optional, Type, TypeVar

import wpilib
from wpiutil import log

T = TypeVar("T")


class DataLogger:
    def __init__(self):
        self._log: Optional[log.DataLog] = None
        # Map of topic name to LogEntry object.
        self._entries: Dict[str, Any] = {}

    def _getLog(self) -> log.DataLog:
        """Get the DataLog instance, initializing it if needed."""
        if self._log is None:
            self._log = wpilib.DataLogManager.getLog()
        return self._log

    def flush(self) -> None:
        self._getLog().flush()

    def logStruct(
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
                self._getLog(), topic_name, struct_type
            )
        if on_change:
            self._entries[topic_name].update(value)
        else:
            self._entries[topic_name].append(value)

    def logStructArray(
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
                self._getLog(), topic_name, struct_type
            )
        if on_change:
            self._entries[topic_name].update(values)
        else:
            self._entries[topic_name].append(values)

    def logString(
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
                self._getLog(), topic_name
            )
        if on_change:
            self._entries[topic_name].update(value)
        else:
            self._entries[topic_name].append(value)

    def logStringArray(
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
                self._getLog(), topic_name
            )
        if on_change:
            self._entries[topic_name].update(values)
        else:
            self._entries[topic_name].append(values)

    def logDouble(
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
                self._getLog(), topic_name
            )
        if on_change:
            self._entries[topic_name].update(value)
        else:
            self._entries[topic_name].append(value)

    def logBoolean(
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
                self._getLog(), topic_name
            )
        if on_change:
            self._entries[topic_name].update(value)
        else:
            self._entries[topic_name].append(value)
