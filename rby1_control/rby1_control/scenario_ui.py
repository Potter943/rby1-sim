"""Minimal Scenario tab: select/run/delete Tasks and capture Q/TCP values."""
from __future__ import annotations

import math
from typing import Callable, Dict, Optional

from .qt_compat import (
    QApplication,
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from .task_commands import (
    TaskDefinition,
    compile_task_source,
    delete_catalog_task,
    load_catalog,
)
from .task_runner import TaskRunner


class ScenarioPanel(QWidget):
    def __init__(
        self,
        backend,
        *,
        on_log: Callable[[str, str], None],
        on_active_changed: Callable[[bool], None],
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.on_log = on_log
        self.on_active_changed = on_active_changed
        self.tasks: Dict[str, TaskDefinition] = {}
        self.runner = TaskRunner(
            backend,
            on_status=self._runner_status,
            on_active_changed=self._runner_active_changed,
        )
        self._build_ui()
        self.reload_tasks(initial=True)

    def _build_ui(self) -> None:
        root = QHBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        tasks_group = QGroupBox("Task List")
        tasks_layout = QVBoxLayout(tasks_group)
        self.task_list = QListWidget()
        self.task_list.currentTextChanged.connect(self._show_selected)
        tasks_layout.addWidget(self.task_list, 1)

        source_note = QLabel(
            "Edit task.py in the IDE, then Reload Tasks. "
            "Delete removes only the compiled list entry."
        )
        source_note.setObjectName("secondaryNote")
        source_note.setWordWrap(True)
        tasks_layout.addWidget(source_note)

        task_buttons = QHBoxLayout()
        self.reload_button = QPushButton("Reload Tasks")
        self.reload_button.clicked.connect(self.reload_tasks)
        self.delete_button = QPushButton("Delete Task")
        self.delete_button.clicked.connect(self.delete_selected)
        task_buttons.addWidget(self.reload_button)
        task_buttons.addWidget(self.delete_button)
        tasks_layout.addLayout(task_buttons)

        self.run_button = QPushButton("Run Selected Task")
        self.run_button.clicked.connect(self.run_selected)
        tasks_layout.addWidget(self.run_button)

        self.cancel_button = QPushButton("CANCEL")
        self.cancel_button.clicked.connect(self.backend.cancel_motion)
        tasks_layout.addWidget(self.cancel_button)
        root.addWidget(tasks_group, 2)

        details = QWidget()
        details_layout = QVBoxLayout(details)
        details_layout.setContentsMargins(0, 0, 0, 0)
        details_layout.setSpacing(8)

        capture_group = QGroupBox("Get Q / Get TCP")
        capture_layout = QVBoxLayout(capture_group)

        q_row = QHBoxLayout()
        self.q_group = QComboBox()
        for label, value in (
            ("Right Arm", "right_arm"),
            ("Left Arm", "left_arm"),
            ("Torso", "torso"),
            ("Head", "head"),
        ):
            self.q_group.addItem(label, value)
        self._style_combo_popup(self.q_group)
        self.get_q_button = QPushButton("Get Q")
        self.get_q_button.clicked.connect(self.get_q)
        q_row.addWidget(self.q_group, 1)
        q_row.addWidget(self.get_q_button)
        capture_layout.addLayout(q_row)

        tcp_row = QHBoxLayout()
        self.tcp_arm = QComboBox()
        self.tcp_arm.addItem("Right Arm TCP (ee_right)", "right_arm")
        self.tcp_arm.addItem("Left Arm TCP (ee_left)", "left_arm")
        self._style_combo_popup(self.tcp_arm)
        self.tcp_arm.setToolTip(
            "TCP is the selected arm end-effector pose relative to base."
        )
        self.get_tcp_button = QPushButton("Get TCP")
        self.get_tcp_button.clicked.connect(self.get_tcp)
        tcp_row.addWidget(self.tcp_arm, 1)
        tcp_row.addWidget(self.get_tcp_button)
        capture_layout.addLayout(tcp_row)

        self.capture_value = QPlainTextEdit()
        self.capture_value.setReadOnly(True)
        self.capture_value.setPlaceholderText(
            "Get Q: joint angles (deg)\n"
            "Get TCP: base → selected ee link [x, y, z, roll, pitch, yaw] "
            "(m / deg)"
        )
        self.capture_value.setMaximumHeight(78)
        capture_layout.addWidget(self.capture_value)
        self.copy_button = QPushButton("Copy Value")
        self.copy_button.clicked.connect(self.copy_value)
        capture_layout.addWidget(self.copy_button)
        details_layout.addWidget(capture_group)

        preview_group = QGroupBox("Selected Task")
        preview_layout = QVBoxLayout(preview_group)
        self.task_preview = QPlainTextEdit()
        self.task_preview.setReadOnly(True)
        preview_layout.addWidget(self.task_preview)
        details_layout.addWidget(preview_group, 1)

        log_group = QGroupBox("Scenario Log")
        log_layout = QVBoxLayout(log_group)
        log_layout.setContentsMargins(6, 6, 6, 6)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(700)
        self.log_view.setMinimumHeight(110)
        self.log_view.setMaximumHeight(170)
        log_layout.addWidget(self.log_view)
        details_layout.addWidget(log_group)
        root.addWidget(details, 3)

    @staticmethod
    def _style_combo_popup(combo: QComboBox) -> None:
        """Keep every Scenario combo item readable in the dark theme."""

        combo.setStyleSheet(
            """
            QComboBox {
                background-color: #171a1f;
                color: #e8edf2;
                border: 1px solid #4b5563;
                border-radius: 4px;
                padding: 3px;
            }
            """
        )
        combo.view().setStyleSheet(
            """
            QAbstractItemView {
                background-color: #171a1f;
                color: #e8edf2;
                border: 1px solid #657184;
                outline: 0;
                selection-background-color: #2c6e9b;
                selection-color: #ffffff;
            }
            QAbstractItemView::item {
                min-height: 24px;
                padding: 3px 7px;
                background-color: #171a1f;
                color: #e8edf2;
            }
            QAbstractItemView::item:hover,
            QAbstractItemView::item:selected {
                background-color: #2c6e9b;
                color: #ffffff;
            }
            """
        )

    def _log(self, level: str, message: str) -> None:
        prefix = {
            "error": "[ERROR] ",
            "warning": "[WARN] ",
        }.get(level, "")
        self.log_view.appendPlainText(prefix + message)
        self.on_log(level, message)

    def _runner_status(self, message: str) -> None:
        if message.startswith("Task failed"):
            level = "error"
        elif "stopped" in message.lower() or "cancel" in message.lower():
            level = "warning"
        else:
            level = "info"
        self._log(level, message)

    def reload_tasks(self, initial: bool = False) -> None:
        selected = self.task_list.currentItem().text() if self.task_list.currentItem() else ""
        try:
            if initial:
                self.tasks = load_catalog()
                self._log("info", f"Task catalog loaded: {len(self.tasks)} Task(s)")
            else:
                self.tasks = compile_task_source()
                self._log("info", f"Task source loaded: {len(self.tasks)} Task(s)")
        except Exception as exc:
            try:
                if initial:
                    self.tasks = compile_task_source()
                    self._log("info", f"Initial Task source loaded: {len(self.tasks)} Task(s)")
                else:
                    raise exc
            except Exception as source_exc:
                self._log("error", f"Task source load failed: {source_exc}")
                try:
                    self.tasks = load_catalog()
                    self._log("warning", "Using the last valid compiled Task catalog.")
                except Exception:
                    self.tasks = {}
        self._fill_task_list(selected)
        if not initial and not self.tasks:
            self._log("warning", "No runnable Tasks are registered in task.py.")

    def _fill_task_list(self, selected: str = "") -> None:
        self.task_list.clear()
        for name in sorted(self.tasks):
            self.task_list.addItem(name)
        selected_row = -1
        for row in range(self.task_list.count()):
            if self.task_list.item(row).text() == selected:
                selected_row = row
                break
        if selected_row >= 0:
            self.task_list.setCurrentRow(selected_row)
        elif self.task_list.count():
            self.task_list.setCurrentRow(0)
        else:
            self.task_preview.clear()

    def _selected_name(self) -> str:
        item = self.task_list.currentItem()
        return item.text() if item is not None else ""

    def _show_selected(self, name: str) -> None:
        task = self.tasks.get(name)
        if task is None:
            self.task_preview.clear()
            return
        lines = [task.name]
        if task.description:
            lines.append(task.description)
        lines.append("")
        for index, command in enumerate(task.commands, 1):
            if command.kind.value == "delay":
                detail = f"{float(command.seconds or 0.0):.3f} s"
            elif command.joint_targets:
                detail = "; ".join(
                    f"{group} {list(values)}"
                    for group, values in command.joint_targets
                )
            else:
                detail = f"{command.group}  {list(command.values)}"
            lines.append(f"{index}. {command.kind.value}: {detail}")
        self.task_preview.setPlainText("\n".join(lines))

    def run_selected(self) -> None:
        name = self._selected_name()
        task = self.tasks.get(name)
        if task is None:
            self._log("warning", "Select a Task first.")
            return
        try:
            self.runner.start(task)
        except Exception as exc:
            self._log("error", f"Task start rejected: {exc}")

    def delete_selected(self) -> None:
        if self.runner.active:
            self._log("warning", "Stop the running Task before deleting.")
            return
        name = self._selected_name()
        if not name:
            self._log("warning", "Select a Task to delete.")
            return
        try:
            self.tasks = delete_catalog_task(name)
        except Exception as exc:
            self._log("error", f"Task delete failed: {exc}")
            return
        self._fill_task_list()
        self._log(
            "info",
            f"Deleted compiled Task '{name}'. Reload Tasks restores IDE source entries.",
        )

    def get_q(self) -> None:
        group = str(self.q_group.currentData())
        try:
            state = self.backend.task_state()
            values = state.joint_groups.get(group)
            self._require_fresh(state.joint_updated_at.get(group), state.captured_at)
            if values is None or not state.joint_order_verified.get(group, False):
                raise RuntimeError("joint names/order are not verified")
            self._set_capture(values)
            self._log("info", f"Get Q captured {group} in degrees.")
        except Exception as exc:
            self._log("error", f"Get Q failed: {exc}")

    def get_tcp(self) -> None:
        arm = str(self.tcp_arm.currentData())
        try:
            state = self.backend.task_state()
            values = state.cartesian.get(arm)
            self._require_fresh(state.cartesian_updated_at.get(arm), state.captured_at)
            if values is None:
                raise RuntimeError("Cartesian pose is unavailable")
            self._set_capture(values)
            self._log("info", f"Get TCP captured {arm} in base frame (m / deg).")
        except Exception as exc:
            self._log("error", f"Get TCP failed: {exc}")

    @staticmethod
    def _require_fresh(updated_at: Optional[float], captured_at: float) -> None:
        if updated_at is None:
            raise RuntimeError("state is unavailable")
        age = captured_at - updated_at
        if not math.isfinite(age) or age < 0.0 or age > 1.0:
            raise RuntimeError("state is stale")

    def _set_capture(self, values) -> None:
        formatted = "[" + ", ".join(f"{float(value):.6f}" for value in values) + "]"
        self.capture_value.setPlainText(formatted)

    def copy_value(self) -> None:
        value = self.capture_value.toPlainText().strip()
        if not value:
            self._log("warning", "Capture a Q or TCP value before copying.")
            return
        QApplication.clipboard().setText(value)
        self._log("info", "Captured value copied to the clipboard.")

    def _runner_active_changed(self, active: bool) -> None:
        self.task_list.setEnabled(not active)
        self.reload_button.setEnabled(not active)
        self.delete_button.setEnabled(not active)
        self.run_button.setEnabled(not active)
        self.get_q_button.setEnabled(not active)
        self.get_tcp_button.setEnabled(not active)
        self.on_active_changed(active)

    def emergency_stop(self) -> None:
        self.runner.stop("Task stopped by EMERGENCY STOP")

    def close(self) -> bool:
        self.runner.close()
        return super().close()
