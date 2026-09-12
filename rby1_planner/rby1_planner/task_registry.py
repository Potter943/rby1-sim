"""Load planner-only runtime Tasks directly from the editable source module."""
from __future__ import annotations

import importlib
from collections.abc import Mapping

from .task_commands import (
    DynamicTaskDefinition,
    RunnableTaskDefinition,
    Task,
    TaskDefinition,
)


def load_tasks_from_source() -> dict[str, RunnableTaskDefinition]:
    """Reload ``task.py`` and validate the planner UI Task registry."""

    from . import task as task_source

    module = importlib.reload(task_source)
    authored = module.build_tasks()
    if not isinstance(authored, Mapping):
        raise ValueError('build_tasks() must return a name-to-Task mapping')

    result: dict[str, RunnableTaskDefinition] = {}
    for name, value in authored.items():
        if not isinstance(name, str) or not name.strip():
            raise ValueError('Task registry names must be nonempty strings')
        if isinstance(value, Task):
            definition = value.build()
        elif isinstance(value, (TaskDefinition, DynamicTaskDefinition)):
            definition = value
        else:
            raise ValueError(f'Task {name!r} is not a planner Task')
        if name != definition.name:
            raise ValueError(
                f'Task registry key {name!r} does not match '
                f'{definition.name!r}'
            )
        if name in result:
            raise ValueError(f'duplicate Task name: {name}')
        result[name] = definition
    return result


__all__ = ['load_tasks_from_source']
