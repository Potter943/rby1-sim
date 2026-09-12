"""Qt operator UI that runs planner Tasks with execution-time perception."""
from __future__ import annotations

from typing import Dict

from rby1_control.main_window import MainWindow
from rby1_control.qt_compat import QLabel, QWidget
from rby1_control.scenario_ui import ScenarioPanel

from .task_commands import (
    CameraLinearAbsoluteStep,
    RunnableTaskDefinition,
)
from .task_registry import load_tasks_from_source


class PlannerScenarioPanel(ScenarioPanel):
    """Scenario panel backed by the PlannerNode's camera-aware runner."""

    def __init__(
        self,
        backend,
        *,
        on_log,
        on_active_changed,
        parent=None,
    ) -> None:
        # ScenarioPanel.__init__ creates the canonical control TaskRunner, so
        # initialize only QWidget and reuse PlannerNode.task_runner instead.
        QWidget.__init__(self, parent)
        self.backend = backend
        self.on_log = on_log
        self.on_active_changed = on_active_changed
        self.tasks: Dict[str, RunnableTaskDefinition] = {}
        self.runner = backend.task_runner
        self._node_status_callback = self.runner.on_status
        self.runner.on_status = self._runner_status
        self.runner.on_active_changed = self._runner_active_changed

        self._build_ui()
        self._update_source_note()
        self._connect_cancel_to_planner_runner()
        self.reload_tasks(initial=True)

    def _update_source_note(self) -> None:
        for label in self.findChildren(QLabel):
            if label.text().startswith('Edit task.py in the IDE'):
                label.setText(
                    'Edit planner task.py in the IDE, then Reload Tasks. '
                    'Delete removes only the current UI list entry.'
                )
                break

    def _connect_cancel_to_planner_runner(self) -> None:
        try:
            self.cancel_button.clicked.disconnect()
        except (RuntimeError, TypeError):
            pass
        self.cancel_button.clicked.connect(self._cancel_selected_task)

    def _cancel_selected_task(self) -> None:
        if self.runner.active:
            self.runner.stop('Task stopped by planner UI operator')
        else:
            self.backend.cancel_motion()

    def _runner_status(self, message: str) -> None:
        self._node_status_callback(message)
        super()._runner_status(message)

    def reload_tasks(self, initial: bool = False) -> None:
        del initial
        if self.runner.active:
            self._log('warning', 'Stop the running Task before reloading.')
            return
        selected = self._selected_name()
        try:
            self.tasks = load_tasks_from_source()
        except Exception as exc:
            self.tasks = {}
            self._log('error', f'Planner Task source load failed: {exc}')
        else:
            self._log(
                'info',
                f'Planner Task source loaded: {len(self.tasks)} Task(s)',
            )
        self._fill_task_list(selected)
        if not self.tasks:
            self._log('warning', 'No runnable planner Tasks are registered.')

    def delete_selected(self) -> None:
        if self.runner.active:
            self._log('warning', 'Stop the running Task before deleting.')
            return
        name = self._selected_name()
        if not name:
            self._log('warning', 'Select a Task to delete.')
            return
        self.tasks.pop(name, None)
        self._fill_task_list()
        self._log(
            'info',
            f'Removed {name!r} from the current list. Reload restores it.',
        )

    def _show_selected(self, name: str) -> None:
        task = self.tasks.get(name)
        if task is None:
            self.task_preview.clear()
            return

        lines = [task.name]
        if task.description:
            lines.append(task.description)
        lines.append('')
        for index, command in enumerate(task.commands, 1):
            if isinstance(command, CameraLinearAbsoluteStep):
                detail = (
                    f'{command.group} <- {command.object_id}; '
                    f'timeout={command.detection_timeout_sec:.3f}s; '
                    f'object_to_ee_xyz='
                    f'{list(command.object_to_end_effector_position)}'
                )
                lines.append(
                    f'{index}. camera_linear_absolute: {detail}'
                )
                continue
            if command.kind.value == 'delay':
                detail = f'{float(command.seconds or 0.0):.3f} s'
            elif command.joint_targets:
                detail = '; '.join(
                    f'{group} {list(values)}'
                    for group, values in command.joint_targets
                )
            else:
                detail = f'{command.group}  {list(command.values)}'
            lines.append(f'{index}. {command.kind.value}: {detail}')
        self.task_preview.setPlainText('\n'.join(lines))


class PlannerMainWindow(MainWindow):
    """Reuse the control UI while replacing its Scenario Task source."""

    def __init__(self, backend) -> None:
        super().__init__(backend)
        self.setWindowTitle('RB-Y1 Planner')

    def _build_scenario_tab(self) -> QWidget:
        self.scenario_panel = PlannerScenarioPanel(
            self.backend,
            on_log=self.append_log,
            on_active_changed=self._scenario_active_changed,
        )
        return self.scenario_panel


__all__ = ['PlannerMainWindow', 'PlannerScenarioPanel']
