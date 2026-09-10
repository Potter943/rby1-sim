"""Qt operator UI for RB-Y1 M v1.3.

Layout policy
-------------
Always visible:
- Power / Servo / Stream controls
- Control Manager command area
- compact robot / safety state
- hardware EMO / E-Stop status area (no software E-Stop command)

Each tab provides a software MOTION STOP.
Space triggers the same MOTION STOP action.
UI event logging is written to the terminal rather than an on-screen log panel.
"""

from __future__ import annotations

from typing import Dict, Set

from .scenario_ui import ScenarioPanel
from .qt_compat import (
    QAbstractSpinBox,
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFont,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QTabWidget,
    QTimer,
    QVBoxLayout,
    QWidget,
    alignment,
    event_type,
    focus_policy,
    qt_key,
)


class MainWindow(QMainWindow):

    CONTROL_STATE_NAMES = {
        0: "NONE",
        1: "IDLE",
        2: "ENABLE",
        3: "EXECUTING",
        4: "MAJOR FAULT",
        5: "MINOR FAULT",
    }

    ACTION_FORWARD = "forward"
    ACTION_BACKWARD = "backward"
    ACTION_LEFT = "left"
    ACTION_RIGHT = "right"
    ACTION_ROTATE_CCW = "rotate_ccw"
    ACTION_ROTATE_CW = "rotate_cw"

    # UI-level grouping for RB-Y1 body components.
    # Actual ROS joint names and limits are resolved in RosBackend on Day 3.
    JOINT_GROUPS = {
        "right_arm": ("Right Arm", 7),
        "left_arm": ("Left Arm", 7),
        "torso": ("Torso", 6),
        "head": ("Head", 2),
    }

    CARTESIAN_ARMS = {
        "right_arm": "Right Arm",
        "left_arm": "Left Arm",
    }

    def __init__(self, backend) -> None:
        super().__init__()

        self.backend = backend
        self.backend_name = getattr(
            backend,
            "backend_name",
            "ROS2",
        )
        self._pressed_actions: Set[str] = set()
        self._action_buttons: Dict[str, QPushButton] = {}
        self._closing = False
        self._scenario_active = False

        self._joint_target_cache = {
            key: [0.0] * dof
            for key, (_, dof) in self.JOINT_GROUPS.items()
        }
        self._cartesian_target_cache = {
            key: [0.0] * 6
            for key in self.CARTESIAN_ARMS
        }
        self._latest_motion_state = {}

        # Press-and-hold Cartesian direction control.
        # Only one Cartesian hold command is active at a time because the
        # backend intentionally allows one Joint/Cartesian motion goal at once.
        self._cartesian_hold_command = None
        self.cartesian_hold_timer = QTimer(self)
        self.cartesian_hold_timer.setInterval(100)
        self.cartesian_hold_timer.timeout.connect(
            self._continue_cartesian_hold
        )

        self.setWindowTitle("RB-Y1")

        # Wider operator layout: both Cartesian arms are visible together.
        self.setMinimumSize(980, 700)
        self.resize(1120, 820)
        self.setFocusPolicy(focus_policy("StrongFocus"))

        central = QWidget(self)
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # Always-visible robot operation / safety panel.
        root.addWidget(self._build_fixed_top_panel())

        # Task-specific controls.
        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_base_tab(), "Base")
        self.tabs.addTab(self._build_joint_tab(), "Joints")
        self.tabs.addTab(
            self._build_scenario_tab(),
            "Scenario",
        )
        self.tabs.addTab(
            self._build_diagnostics_tab(),
            "Diagnostics",
        )
        root.addWidget(self.tabs, 1)


        self._apply_style()

        # Base commands are refreshed at 25 Hz while a control is held.
        self.command_timer = QTimer(self)
        self.command_timer.setInterval(40)
        self.command_timer.timeout.connect(
            self._refresh_command
        )
        self.command_timer.start()

        # UI status refresh can be slower than the command path.
        self.status_timer = QTimer(self)
        self.status_timer.setInterval(200)
        self.status_timer.timeout.connect(
            self.refresh_backend_status
        )
        self.status_timer.start()

        self.append_log(
            "info",
            f"UI initialized in {self.backend_name} mode.",
        )
        self.refresh_backend_status()

    # ==================================================================
    # Fixed top operator / safety panel
    # ==================================================================
    def _build_fixed_top_panel(self) -> QWidget:
        frame = QFrame()
        frame.setObjectName("headerFrame")

        layout = QHBoxLayout(frame)
        layout.setContentsMargins(7, 6, 7, 6)
        layout.setSpacing(6)

        # --------------------------------------------------------------
        # Power
        # --------------------------------------------------------------
        power_group = QGroupBox("Power")
        power_layout = QVBoxLayout(power_group)
        power_layout.setContentsMargins(5, 6, 5, 5)
        power_layout.setSpacing(4)

        self.power_on_button = QPushButton("ON")
        self.power_off_button = QPushButton("OFF")
        self.power_on_button.clicked.connect(
            lambda: self.backend.request_power(True)
        )
        self.power_off_button.clicked.connect(
            lambda: self.backend.request_power(False)
        )

        power_layout.addWidget(self.power_on_button)
        power_layout.addWidget(self.power_off_button)

        # --------------------------------------------------------------
        # Servo
        # --------------------------------------------------------------
        servo_group = QGroupBox("Servo")
        servo_layout = QVBoxLayout(servo_group)
        servo_layout.setContentsMargins(5, 6, 5, 5)
        servo_layout.setSpacing(4)

        self.servo_on_button = QPushButton("ON")
        self.servo_off_button = QPushButton("OFF")
        self.servo_on_button.clicked.connect(
            lambda: self.backend.request_servo(True)
        )
        self.servo_off_button.clicked.connect(
            lambda: self.backend.request_servo(False)
        )

        servo_layout.addWidget(self.servo_on_button)
        servo_layout.addWidget(self.servo_off_button)

        # --------------------------------------------------------------
        # Stream
        # --------------------------------------------------------------
        stream_group = QGroupBox("Stream")
        stream_layout = QVBoxLayout(stream_group)
        stream_layout.setContentsMargins(5, 6, 5, 5)
        stream_layout.setSpacing(4)

        self.stream_on_button = QPushButton("ON")
        self.stream_off_button = QPushButton("OFF")
        self.stream_on_button.clicked.connect(
            lambda: self.backend.request_stream(True)
        )
        self.stream_off_button.clicked.connect(self._stream_off)

        stream_layout.addWidget(self.stream_on_button)
        stream_layout.addWidget(self.stream_off_button)

        # --------------------------------------------------------------
        # Control Manager
        # --------------------------------------------------------------
        control_manager_group = QGroupBox("Control Manager")
        control_manager_layout = QGridLayout(control_manager_group)
        control_manager_layout.setContentsMargins(5, 6, 5, 5)
        control_manager_layout.setSpacing(4)

        self.control_manager_enable_button = QPushButton("ENABLE")
        self.control_manager_disable_button = QPushButton("DISABLE")
        self.control_manager_reset_button = QPushButton("RESET")

        self.control_manager_enable_button.clicked.connect(
            lambda: self.backend.request_control_manager("enable")
        )
        self.control_manager_disable_button.clicked.connect(
            lambda: self.backend.request_control_manager("disable")
        )
        self.control_manager_reset_button.clicked.connect(
            lambda: self.backend.request_control_manager("reset")
        )

        # refresh_backend_status() enables these only when the ROS service
        # is actually available.
        for button in (
            self.control_manager_enable_button,
            self.control_manager_disable_button,
            self.control_manager_reset_button,
        ):
            button.setEnabled(False)

        control_manager_layout.addWidget(
            self.control_manager_enable_button, 0, 0
        )
        control_manager_layout.addWidget(
            self.control_manager_disable_button, 1, 0
        )
        control_manager_layout.addWidget(
            self.control_manager_reset_button, 0, 1, 2, 1
        )

        # --------------------------------------------------------------
        # State: horizontal operator view
        #
        # Power/Servo/Stream | emphasized Control | EMO/Collision
        # --------------------------------------------------------------
        state_group = QGroupBox("State")
        state_layout = QHBoxLayout(state_group)
        state_layout.setContentsMargins(8, 7, 8, 7)
        state_layout.setSpacing(12)

        # Power / Servo do not currently have independent feedback in
        # BackendSnapshot, so they intentionally remain UNKNOWN for now.
        self.power_state_value = self._status_indicator()
        self.servo_state_value = self._status_indicator()
        self.stream_value = self._status_indicator()

        # Left column: Power / Servo / Stream.
        left_state_widget = QWidget()
        left_state_layout = QGridLayout(left_state_widget)
        left_state_layout.setContentsMargins(0, 0, 0, 0)
        left_state_layout.setHorizontalSpacing(7)
        left_state_layout.setVerticalSpacing(5)

        left_state_layout.addWidget(QLabel("Power"), 0, 0)
        left_state_layout.addWidget(self.power_state_value, 0, 1)
        left_state_layout.addWidget(QLabel("Servo"), 1, 0)
        left_state_layout.addWidget(self.servo_state_value, 1, 1)
        left_state_layout.addWidget(QLabel("Stream"), 2, 0)
        left_state_layout.addWidget(self.stream_value, 2, 1)

        # Center: Control is intentionally the most prominent state.
        control_frame = QFrame()
        control_frame.setObjectName("controlStateCard")
        control_frame.setMinimumWidth(170)
        control_frame.setMinimumHeight(78)

        control_layout = QVBoxLayout(control_frame)
        control_layout.setContentsMargins(12, 7, 12, 7)
        control_layout.setSpacing(2)

        control_title = QLabel("CONTROL")
        control_title.setObjectName("controlStateTitle")
        control_title.setAlignment(alignment("AlignCenter"))

        self.state_value = QLabel("UNKNOWN")
        self.state_value.setObjectName("controlStateValue")
        self.state_value.setAlignment(alignment("AlignCenter"))

        control_layout.addWidget(control_title)
        control_layout.addWidget(self.state_value, 1)

        # Right column: EMO / Collision.
        right_state_widget = QWidget()
        right_state_layout = QGridLayout(right_state_widget)
        right_state_layout.setContentsMargins(0, 0, 0, 0)
        right_state_layout.setHorizontalSpacing(7)
        right_state_layout.setVerticalSpacing(8)

        self.emo_value = QLabel("UNKNOWN")
        self.collision_value = QLabel("UNKNOWN")
        self.emo_value.setObjectName("stateValue")
        self.collision_value.setObjectName("stateValue")

        right_state_layout.addWidget(QLabel("EMO"), 0, 0)
        right_state_layout.addWidget(self.emo_value, 0, 1)
        right_state_layout.addWidget(QLabel("Collision"), 1, 0)
        right_state_layout.addWidget(self.collision_value, 1, 1)

        state_layout.addWidget(left_state_widget)
        state_layout.addWidget(control_frame, 2)
        state_layout.addWidget(right_state_widget)

        # --------------------------------------------------------------
        # Hardware EMO / E-Stop status
        #
        # The public rby1-ros2 / rby1-sdk interfaces expose EMO state
        # feedback, but no supported software E-Stop command.  Keep this
        # control non-clickable so it cannot be confused with MOTION STOP.
        # --------------------------------------------------------------
        self.estop_button = QPushButton("E-STOP\nHW ONLY")
        self.estop_button.setObjectName("eStopButton")
        self.estop_button.setMinimumWidth(95)
        self.estop_button.setMinimumHeight(74)
        self.estop_button.setEnabled(False)
        self.estop_button.setProperty("emoActive", False)
        self.estop_button.setToolTip(
            "Hardware EMO status only. Use the physical EMO remote for "
            "emergency stop."
        )

        layout.addWidget(power_group)
        layout.addWidget(servo_group)
        layout.addWidget(stream_group)
        layout.addWidget(control_manager_group)
        layout.addWidget(state_group, 2)
        layout.addWidget(self.estop_button)

        return frame

    @staticmethod
    def _status_indicator() -> QLabel:
        label = QLabel("● UNKNOWN")
        label.setMinimumWidth(82)
        label.setObjectName("statusUnknown")
        return label

    @staticmethod
    def _set_status_indicator(
        label: QLabel,
        state,
    ) -> None:
        if state is True:
            label.setText("● ON")
            label.setObjectName("statusOn")
        elif state is False:
            label.setText("● OFF")
            label.setObjectName("statusOff")
        else:
            label.setText("● UNKNOWN")
            label.setObjectName("statusUnknown")

        # Re-polish so object-name based style updates immediately.
        label.style().unpolish(label)
        label.style().polish(label)

    @staticmethod
    def _empty_vector(count: int) -> str:
        return "[ " + ", ".join("-" for _ in range(count)) + " ]"

    @staticmethod
    def _format_joint_vector(values, count: int) -> str:
        rendered = []
        for index in range(count):
            if index < len(values):
                rendered.append(f"{float(values[index]):+.2f}")
            else:
                rendered.append("-")
        return "[ " + ", ".join(rendered) + " ]"

    @staticmethod
    def _format_tcp_vector(values) -> str:
        rendered = []
        for index in range(6):
            if index >= len(values):
                rendered.append("-")
                continue

            value = float(values[index])
            rendered.append(
                f"{value:+.4f}" if index < 3 else f"{value:+.2f}"
            )

        return "[ " + ", ".join(rendered) + " ]"

    def _set_estop_status(self, emo_active) -> None:
        """Mirror physical EMO feedback without exposing a fake E-Stop command."""
        if emo_active is True:
            self.estop_button.setText("EMO\nACTIVE")
            self.estop_button.setProperty("emoActive", True)
            self.estop_button.setToolTip(
                "Physical EMO is ACTIVE. Release the hardware EMO only after "
                "the robot and workspace are safe."
            )
        elif emo_active is False:
            self.estop_button.setText("E-STOP\nHW ONLY")
            self.estop_button.setProperty("emoActive", False)
            self.estop_button.setToolTip(
                "No supported software E-Stop command is exposed by the "
                "current public RB-Y1 ROS2/SDK interface. Use the physical "
                "EMO remote for emergency stop."
            )
        else:
            self.estop_button.setText("E-STOP\nUNKNOWN")
            self.estop_button.setProperty("emoActive", False)
            self.estop_button.setToolTip(
                "EMO feedback is unavailable. Verify /rby1/robot_state and "
                "keep the physical EMO remote accessible."
            )

        # This is intentionally never clickable: MOTION STOP remains the
        # software stop path, while this widget mirrors hardware EMO state.
        self.estop_button.setEnabled(False)
        self.estop_button.style().unpolish(self.estop_button)
        self.estop_button.style().polish(self.estop_button)

    def _build_current_command_group(self) -> QGroupBox:
        group = QGroupBox("Current Base Command")
        form = QFormLayout(group)

        self.vx_value = self._command_label()
        self.vy_value = self._command_label()
        self.wz_value = self._command_label()

        form.addRow("linear.x", self.vx_value)
        form.addRow("linear.y", self.vy_value)
        form.addRow("angular.z", self.wz_value)

        return group

    def _build_connection_group(self) -> QGroupBox:
        group = QGroupBox("Connection")
        form = QFormLayout(group)

        self.backend_value = QLabel(self.backend_name)
        self.namespace_value = QLabel("-")
        self.topic_value = QLabel("-")
        self.subscriber_value = QLabel("0")
        self.service_value = QLabel("unknown")
        self.service_value.setWordWrap(True)

        form.addRow("Backend", self.backend_value)
        form.addRow("Namespace", self.namespace_value)
        form.addRow("cmd_vel", self.topic_value)
        form.addRow("Subscribers", self.subscriber_value)
        form.addRow("Services", self.service_value)

        return group


    # ==================================================================
    # Base tab
    # ==================================================================
    def _build_base_tab(self) -> QWidget:
        tab = QWidget()
        layout = QHBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        left = QVBoxLayout()
        right = QVBoxLayout()
        left.setSpacing(8)
        right.setSpacing(8)

        # Main operator control: direction buttons first.
        motion = QGroupBox("Base Jog")
        motion_grid = QGridLayout(motion)
        motion_grid.setHorizontalSpacing(6)
        motion_grid.setVerticalSpacing(6)

        ccw = self._motion_button(
            "↶ CCW",
            self.ACTION_ROTATE_CCW,
        )
        forward = self._motion_button(
            "↑ Forward",
            self.ACTION_FORWARD,
        )
        cw = self._motion_button(
            "↷ CW",
            self.ACTION_ROTATE_CW,
        )
        left_button = self._motion_button(
            "← Left",
            self.ACTION_LEFT,
        )
        right_button = self._motion_button(
            "Right →",
            self.ACTION_RIGHT,
        )
        backward = self._motion_button(
            "↓ Backward",
            self.ACTION_BACKWARD,
        )

        center_stop = self._motion_stop_button("MOTION STOP")
        center_stop.setMinimumHeight(40)

        motion_grid.addWidget(ccw, 0, 0)
        motion_grid.addWidget(forward, 0, 1)
        motion_grid.addWidget(cw, 0, 2)
        motion_grid.addWidget(left_button, 1, 0)
        motion_grid.addWidget(center_stop, 1, 1)
        motion_grid.addWidget(right_button, 1, 2)
        motion_grid.addWidget(backward, 2, 1)

        left.addWidget(motion, 1)

        speed = QGroupBox("Speed")
        speed_grid = QGridLayout(speed)

        self.linear_speed = self._speed_spin(
            0.15,
            1.50,
            " m/s",
        )
        self.lateral_speed = self._speed_spin(
            0.15,
            1.50,
            " m/s",
        )
        self.angular_speed = self._speed_spin(
            0.25,
            2.50,
            " rad/s",
        )

        speed_grid.addWidget(QLabel("Forward"), 0, 0)
        speed_grid.addWidget(self.linear_speed, 0, 1)
        speed_grid.addWidget(QLabel("Lateral"), 1, 0)
        speed_grid.addWidget(self.lateral_speed, 1, 1)
        speed_grid.addWidget(QLabel("Rotation"), 2, 0)
        speed_grid.addWidget(self.angular_speed, 2, 1)

        right.addWidget(speed)

        # Current command belongs with Base, not in a permanent sidebar.
        right.addWidget(self._build_current_command_group())

        options = QGroupBox("Options")
        option_layout = QVBoxLayout(options)
        option_layout.setSpacing(4)

        self.keyboard_enable = QCheckBox("Keyboard control")
        self.keyboard_enable.setChecked(True)

        self.require_stream = QCheckBox(
            "Require Stream ON for base motion"
        )
        self.require_stream.setChecked(True)

        self.stop_on_focus_loss = QCheckBox(
            "Stop base when window loses focus"
        )
        self.stop_on_focus_loss.setChecked(True)

        # Kept for command logic, intentionally hidden from the compact UI.
        self.invert_lateral = QCheckBox()
        self.invert_lateral.setChecked(False)
        self.invert_lateral.setVisible(False)

        self.arrow_mode = QComboBox()
        self.arrow_mode.addItem(
            "Arrow keys: lateral",
            "strafe",
        )
        self.arrow_mode.addItem(
            "Arrow keys: rotation",
            "rotate",
        )

        option_layout.addWidget(self.keyboard_enable)
        option_layout.addWidget(self.require_stream)
        option_layout.addWidget(self.stop_on_focus_loss)
        option_layout.addWidget(self.arrow_mode)

        right.addWidget(options)
        right.addStretch(1)

        layout.addLayout(left, 3)
        layout.addLayout(right, 2)

        return tab

    @staticmethod
    def _speed_spin(
        value: float,
        maximum: float,
        suffix: str,
    ) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setDecimals(2)
        spin.setRange(0.01, maximum)
        spin.setSingleStep(0.01)
        spin.setValue(value)
        spin.setSuffix(suffix)
        spin.setKeyboardTracking(False)
        return spin

    def _motion_button(
        self,
        text: str,
        action: str,
    ) -> QPushButton:
        button = QPushButton(text)
        button.setMinimumHeight(40)
        button.setAutoRepeat(False)

        button.pressed.connect(
            lambda selected=action: self._press_action(selected)
        )
        button.released.connect(
            lambda selected=action: self._release_action(selected)
        )

        self._action_buttons[action] = button
        return button

    def _motion_stop_button(
        self,
        text: str = "MOTION STOP",
    ) -> QPushButton:
        button = QPushButton(text)
        button.setObjectName("motionStop")
        button.setMinimumHeight(36)
        button.clicked.connect(self.motion_stop)
        return button

    @staticmethod
    def _command_label() -> QLabel:
        label = QLabel("0.00")
        font = QFont("Monospace")
        font.setPointSize(12)
        font.setBold(True)
        label.setFont(font)
        label.setAlignment(alignment("AlignRight"))
        return label

    # ==================================================================
    # Placeholder tabs for later implementation
    # ==================================================================
    def _placeholder(
        self,
        title: str,
        text: str,
    ) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        group = QGroupBox(title)
        group_layout = QVBoxLayout(group)

        label = QLabel(text)
        label.setWordWrap(True)

        group_layout.addWidget(label)
        group_layout.addStretch(1)

        layout.addWidget(group, 1)

        stop_row = QHBoxLayout()
        stop_row.addStretch(1)
        stop_row.addWidget(self._motion_stop_button())
        layout.addLayout(stop_row)

        return tab

    # ==================================================================
    # Day 2: Joint / Cartesian motion tab
    # ==================================================================
    def _build_joint_tab(self) -> QWidget:
        tab = QWidget()
        root = QVBoxLayout(tab)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(7)

        # One motion mode is visible at a time.  This is the main width
        # reduction compared with the previous dual-panel layout.
        mode_group = QGroupBox("Motion")
        mode_layout = QHBoxLayout(mode_group)

        mode_layout.addWidget(QLabel("Type"))
        self.motion_type_selector = QComboBox()
        self.motion_type_selector.addItem("Joint Space", "joint")
        self.motion_type_selector.addItem(
            "Cartesian Space",
            "cartesian",
        )
        mode_layout.addWidget(self.motion_type_selector, 1)

        root.addWidget(mode_group)

        self.joint_panel = self._build_joint_space_panel()
        self.cartesian_panel = self._build_cartesian_space_panel()

        root.addWidget(self.joint_panel, 1)
        root.addWidget(self.cartesian_panel, 1)

        action_group = QGroupBox("Command")
        action_layout = QHBoxLayout(action_group)

        self.copy_current_button = QPushButton(
            "Copy Current → Target"
        )
        self.move_target_button = QPushButton("MOVE TARGET")
        self.cancel_motion_button = QPushButton("CANCEL")
        self.joint_motion_stop_button = self._motion_stop_button()

        self.copy_current_button.clicked.connect(
            self._copy_selected_current_to_target
        )
        self.move_target_button.clicked.connect(
            self._move_selected_target
        )
        self.cancel_motion_button.clicked.connect(
            self._cancel_arm_motion
        )

        action_layout.addWidget(self.copy_current_button, 2)
        action_layout.addWidget(self.move_target_button, 2)
        action_layout.addWidget(self.cancel_motion_button, 1)
        action_layout.addWidget(self.joint_motion_stop_button, 1)

        root.addWidget(action_group)

        self.motion_type_selector.currentIndexChanged.connect(
            self._on_motion_type_changed
        )
        self._on_motion_type_changed()

        return tab

    def _on_motion_type_changed(self, *args) -> None:
        del args

        mode = str(self.motion_type_selector.currentData())

        self.joint_panel.setVisible(mode == "joint")
        self.cartesian_panel.setVisible(mode == "cartesian")

        # Cartesian has dedicated Copy / MOVE buttons for each arm.
        if hasattr(self, "copy_current_button"):
            self.copy_current_button.setVisible(mode == "joint")
        if hasattr(self, "move_target_button"):
            self.move_target_button.setVisible(mode == "joint")

    def _build_joint_space_panel(self) -> QGroupBox:
        panel = QGroupBox("Joint Space Motion")
        root = QVBoxLayout(panel)
        root.setSpacing(9)

        # --------------------------------------------------------------
        # Component selector
        # --------------------------------------------------------------
        selector_group = QGroupBox("Joint Group")
        selector_layout = QFormLayout(selector_group)

        self.joint_group_selector = QComboBox()
        for key, (label, _) in self.JOINT_GROUPS.items():
            self.joint_group_selector.addItem(label, key)

        self.joint_group_selector.currentIndexChanged.connect(
            self._on_joint_group_changed
        )
        selector_layout.addRow("Component", self.joint_group_selector)
        root.addWidget(selector_group)

        # --------------------------------------------------------------
        # Get q snapshot: compact vector display
        # --------------------------------------------------------------
        snapshot_group = QGroupBox("Joint Snapshot")
        snapshot_layout = QHBoxLayout(snapshot_group)
        snapshot_layout.setContentsMargins(8, 6, 8, 6)
        snapshot_layout.setSpacing(8)

        self.get_q_button = QPushButton("Get q")
        self.get_q_button.clicked.connect(self._get_joint_snapshot)

        self.joint_snapshot_value = QLabel(
            self._empty_vector(self.JOINT_GROUPS[self._selected_joint_group()][1])
        )
        self.joint_snapshot_value.setObjectName("vectorSnapshot")
        self.joint_snapshot_value.setAlignment(alignment("AlignCenter"))
        self.joint_snapshot_value.setMinimumHeight(30)

        snapshot_layout.addWidget(self.get_q_button)
        snapshot_layout.addWidget(self.joint_snapshot_value, 1)

        # Snapshot and joint jog settings are placed side-by-side below.

        # --------------------------------------------------------------
        # Current / target / jog table
        # --------------------------------------------------------------
        state_group = QGroupBox("Joint Position")
        state_group.setMinimumHeight(235)
        grid = QGridLayout(state_group)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)
        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)
        grid.setColumnStretch(3, 0)
        grid.setColumnStretch(4, 0)

        joint_header = QLabel("Joint")
        current_header = QLabel("Current [deg]")
        target_header = QLabel("Target [deg]")
        jog_header = QLabel("Jog")

        current_header.setAlignment(alignment("AlignCenter"))
        target_header.setAlignment(alignment("AlignCenter"))
        jog_header.setAlignment(alignment("AlignCenter"))

        grid.addWidget(joint_header, 0, 0)
        grid.addWidget(current_header, 0, 1)
        grid.addWidget(target_header, 0, 2)
        grid.addWidget(jog_header, 0, 3, 1, 2)

        self.joint_name_labels = []
        self.joint_current_labels = []
        self.joint_target_spins = []
        self.joint_minus_buttons = []
        self.joint_plus_buttons = []
        self.joint_row_widgets = []

        # Maximum visible group is a 7-DOF arm.
        for index in range(7):
            row = index + 1

            name = QLabel(f"J{index + 1}")
            current = QLabel("--")
            current.setAlignment(alignment("AlignCenter"))
            current.setMinimumWidth(82)
            current.setMinimumHeight(26)

            target = QDoubleSpinBox()
            target.setMinimumWidth(105)
            target.setMinimumHeight(28)
            target.setDecimals(2)
            target.setRange(-360.0, 360.0)
            target.setSingleStep(1.0)
            target.setSuffix("°")
            target.setKeyboardTracking(False)
            target.valueChanged.connect(
                lambda value, i=index: self._on_joint_target_changed(i, value)
            )

            minus = QPushButton("−")
            plus = QPushButton("+")
            minus.setFixedSize(38, 28)
            plus.setFixedSize(38, 28)

            minus.clicked.connect(
                lambda checked=False, i=index: self._joint_jog(i, -1)
            )
            plus.clicked.connect(
                lambda checked=False, i=index: self._joint_jog(i, +1)
            )

            grid.addWidget(name, row, 0)
            grid.addWidget(current, row, 1)
            grid.addWidget(target, row, 2)
            grid.addWidget(minus, row, 3)
            grid.addWidget(plus, row, 4)

            self.joint_name_labels.append(name)
            self.joint_current_labels.append(current)
            self.joint_target_spins.append(target)
            self.joint_minus_buttons.append(minus)
            self.joint_plus_buttons.append(plus)
            self.joint_row_widgets.append(
                (name, current, target, minus, plus)
            )

        root.addWidget(state_group, 1)

        # --------------------------------------------------------------
        # Joint-space settings
        # --------------------------------------------------------------
        settings = QGroupBox("Joint Motion Settings")
        settings_layout = QGridLayout(settings)

        self.joint_jog_step = QDoubleSpinBox()
        self.joint_jog_step.setDecimals(1)
        self.joint_jog_step.setRange(0.1, 30.0)
        self.joint_jog_step.setSingleStep(0.5)
        self.joint_jog_step.setValue(2.0)
        self.joint_jog_step.setSuffix("° / click")

        self.joint_minimum_time = QDoubleSpinBox()
        self.joint_minimum_time.setDecimals(1)
        self.joint_minimum_time.setRange(0.1, 30.0)
        self.joint_minimum_time.setSingleStep(0.5)
        self.joint_minimum_time.setValue(3.0)
        self.joint_minimum_time.setSuffix(" s")

        settings_layout.addWidget(QLabel("Jog step"), 0, 0)
        settings_layout.addWidget(self.joint_jog_step, 0, 1)
        settings_layout.addWidget(QLabel("Minimum time"), 1, 0)
        settings_layout.addWidget(self.joint_minimum_time, 1, 1)

        # Keep the compact status/readback controls together at the top.
        joint_tools_row = QHBoxLayout()
        joint_tools_row.setSpacing(8)
        joint_tools_row.addWidget(snapshot_group, 3)
        joint_tools_row.addWidget(settings, 2)
        root.insertLayout(1, joint_tools_row)

        self._on_joint_group_changed()
        return panel

    def _build_cartesian_space_panel(self) -> QGroupBox:
        panel = QGroupBox("Cartesian Space Motion")
        root = QVBoxLayout(panel)
        root.setSpacing(8)

        # The selector is intentionally removed: Left and Right are shown
        # simultaneously and each arm owns its own snapshot / target controls.
        self.tcp_snapshot_values = {}
        self.get_tcp_buttons = {}

        self.cartesian_current_labels = {}
        self.cartesian_target_spins = {}
        self.cartesian_minus_buttons = {}
        self.cartesian_plus_buttons = {}

        # --------------------------------------------------------------
        # Top row: independent Left/Right TCP snapshots + shared jog settings
        # --------------------------------------------------------------
        top_row = QHBoxLayout()
        top_row.setSpacing(8)

        snapshot_group = QGroupBox("TCP Snapshot")
        snapshot_layout = QGridLayout(snapshot_group)
        snapshot_layout.setHorizontalSpacing(7)
        snapshot_layout.setVerticalSpacing(5)
        snapshot_layout.setColumnStretch(1, 1)

        for row, (arm, label) in enumerate(
            (
                ("left_arm", "Left TCP"),
                ("right_arm", "Right TCP"),
            )
        ):
            snapshot_label = QLabel(self._empty_vector(6))
            snapshot_label.setObjectName("vectorSnapshot")
            snapshot_label.setAlignment(alignment("AlignCenter"))
            snapshot_label.setMinimumHeight(29)

            button = QPushButton("Get TCP")
            button.clicked.connect(
                lambda checked=False, arm_name=arm:
                self._get_cartesian_snapshot(arm_name)
            )

            snapshot_layout.addWidget(QLabel(label), row, 0)
            snapshot_layout.addWidget(snapshot_label, row, 1)
            snapshot_layout.addWidget(button, row, 2)

            self.tcp_snapshot_values[arm] = snapshot_label
            self.get_tcp_buttons[arm] = button

        settings = QGroupBox("Jog Settings")
        settings_layout = QGridLayout(settings)
        settings_layout.setHorizontalSpacing(7)
        settings_layout.setVerticalSpacing(5)

        self.cartesian_linear_step = QDoubleSpinBox()
        self.cartesian_linear_step.setDecimals(3)
        self.cartesian_linear_step.setRange(0.001, 0.200)
        self.cartesian_linear_step.setSingleStep(0.005)
        self.cartesian_linear_step.setValue(0.010)
        self.cartesian_linear_step.setSuffix(" m")

        self.cartesian_angular_step = QDoubleSpinBox()
        self.cartesian_angular_step.setDecimals(1)
        self.cartesian_angular_step.setRange(0.5, 30.0)
        self.cartesian_angular_step.setSingleStep(0.5)
        self.cartesian_angular_step.setValue(5.0)
        self.cartesian_angular_step.setSuffix("°")

        self.cartesian_minimum_time = QDoubleSpinBox()
        self.cartesian_minimum_time.setDecimals(1)
        self.cartesian_minimum_time.setRange(0.1, 30.0)
        self.cartesian_minimum_time.setSingleStep(0.5)
        self.cartesian_minimum_time.setValue(3.0)
        self.cartesian_minimum_time.setSuffix(" s")

        settings_layout.addWidget(QLabel("Linear"), 0, 0)
        settings_layout.addWidget(self.cartesian_linear_step, 0, 1)
        settings_layout.addWidget(QLabel("Angular"), 1, 0)
        settings_layout.addWidget(self.cartesian_angular_step, 1, 1)
        settings_layout.addWidget(QLabel("Min time"), 2, 0)
        settings_layout.addWidget(self.cartesian_minimum_time, 2, 1)

        # Keep the TCP snapshots and jog settings on one compact row,
        # matching the Joint Space layout.
        settings.setMaximumWidth(300)
        top_row.addWidget(snapshot_group, 4)
        top_row.addWidget(settings, 2)
        root.addLayout(top_row)

        # --------------------------------------------------------------
        # Main area: Left Arm and Right Arm are always visible together.
        # --------------------------------------------------------------
        arms_row = QHBoxLayout()
        arms_row.setSpacing(8)

        left_panel = self._build_cartesian_arm_panel(
            "left_arm",
            "Left Arm",
        )
        right_panel = self._build_cartesian_arm_panel(
            "right_arm",
            "Right Arm",
        )

        arms_row.addWidget(left_panel, 1)
        arms_row.addWidget(right_panel, 1)
        root.addLayout(arms_row, 1)

        return panel

    def _build_cartesian_arm_panel(
        self,
        arm: str,
        title: str,
    ) -> QGroupBox:
        """Build one arm's Current / Target / Jog and direction-key UI."""

        group = QGroupBox(title)
        root = QVBoxLayout(group)
        root.setSpacing(6)

        motion_widget = QWidget()
        grid = QGridLayout(motion_widget)
        grid.setContentsMargins(2, 2, 2, 2)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(5)

        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 0)
        grid.setColumnStretch(3, 0)
        grid.setColumnStretch(4, 0)

        axis_header = QLabel("Axis")
        current_header = QLabel("Current")
        target_header = QLabel("Target")
        jog_header = QLabel("Jog")

        current_header.setAlignment(alignment("AlignCenter"))
        target_header.setAlignment(alignment("AlignCenter"))
        jog_header.setAlignment(alignment("AlignCenter"))

        grid.addWidget(axis_header, 0, 0)
        grid.addWidget(current_header, 0, 1)
        grid.addWidget(target_header, 0, 2)
        grid.addWidget(jog_header, 0, 3, 1, 2)

        axis_specs = (
            ("X", " m", -2.0, 2.0, 3),
            ("Y", " m", -2.0, 2.0, 3),
            ("Z", " m", -2.0, 2.0, 3),
            ("R", "°", -180.0, 180.0, 1),
            ("P", "°", -180.0, 180.0, 1),
            ("Y", "°", -180.0, 180.0, 1),
        )

        current_labels = []
        target_spins = []
        minus_buttons = []
        plus_buttons = []

        for index, (axis, suffix, minimum, maximum, decimals) in enumerate(
            axis_specs
        ):
            row = index + 1

            axis_label = QLabel(axis)
            current = QLabel("--")
            current.setAlignment(alignment("AlignCenter"))
            current.setMinimumWidth(76)

            target = QDoubleSpinBox()
            target.setFixedWidth(88)
            target.setDecimals(decimals)
            target.setRange(minimum, maximum)
            target.setSingleStep(0.01 if index < 3 else 1.0)
            target.setSuffix(suffix)
            target.setKeyboardTracking(False)
            target.valueChanged.connect(
                lambda value, arm_name=arm, i=index:
                self._on_cartesian_target_changed(
                    arm_name,
                    i,
                    value,
                )
            )

            minus = QPushButton("−")
            plus = QPushButton("+")
            minus.setFixedSize(34, 28)
            plus.setFixedSize(34, 28)

            minus.clicked.connect(
                lambda checked=False, arm_name=arm, i=index:
                self._cartesian_jog(
                    arm_name,
                    i,
                    -1,
                )
            )
            plus.clicked.connect(
                lambda checked=False, arm_name=arm, i=index:
                self._cartesian_jog(
                    arm_name,
                    i,
                    +1,
                )
            )

            grid.addWidget(axis_label, row, 0)
            grid.addWidget(current, row, 1)
            grid.addWidget(target, row, 2)
            grid.addWidget(minus, row, 3)
            grid.addWidget(plus, row, 4)

            current_labels.append(current)
            target_spins.append(target)
            minus_buttons.append(minus)
            plus_buttons.append(plus)

        self.cartesian_current_labels[arm] = current_labels
        self.cartesian_target_spins[arm] = target_spins
        self.cartesian_minus_buttons[arm] = minus_buttons
        self.cartesian_plus_buttons[arm] = plus_buttons

        root.addWidget(motion_widget, 1)

        # Per-arm target commands replace the old global arm selector.
        command_row = QHBoxLayout()
        copy_button = QPushButton("Copy Current → Target")
        move_button = QPushButton("MOVE TARGET")

        copy_button.clicked.connect(
            lambda checked=False, arm_name=arm:
            self._copy_current_cartesian_to_target(arm_name)
        )
        move_button.clicked.connect(
            lambda checked=False, arm_name=arm:
            self._move_cartesian_target(arm_name)
        )

        command_row.addWidget(copy_button, 1)
        command_row.addWidget(move_button, 1)
        root.addLayout(command_row)

        # Direction keys use the same linear jog backend as X/Y/Z row buttons.
        direction_group = QGroupBox("Direction Key")
        direction_layout = QGridLayout(direction_group)
        direction_layout.setHorizontalSpacing(5)
        direction_layout.setVerticalSpacing(5)

        # XY buttons form a cross; Z buttons are a separate vertical pair.
        # This mirrors the operator sketch and keeps each key visually compact.
        direction_layout.setColumnMinimumWidth(3, 14)
        direction_specs = (
            ("+X", 0, +1, 0, 1),
            ("−Y", 1, -1, 1, 0),
            ("+Y", 1, +1, 1, 2),
            ("−X", 0, -1, 2, 1),
            ("+Z", 2, +1, 0, 4),
            ("−Z", 2, -1, 2, 4),
        )

        for text, axis_index, direction, row, column in direction_specs:
            button = QPushButton(text)
            button.setFixedSize(48, 28)

            # Direction keys are press-and-hold controls. The small +/- buttons
            # in the pose table remain discrete one-click jogs.
            button.pressed.connect(
                lambda arm_name=arm,
                i=axis_index,
                d=direction:
                self._start_cartesian_hold(
                    arm_name,
                    i,
                    d,
                )
            )
            button.released.connect(
                self._stop_cartesian_hold
            )
            direction_layout.addWidget(button, row, column)

        root.addWidget(direction_group)
        return group

    # ------------------------------------------------------------------
    # Joint-space UI callbacks
    # ------------------------------------------------------------------
    def _selected_joint_group(self) -> str:
        return str(self.joint_group_selector.currentData())

    def _get_joint_snapshot(self) -> None:
        group = self._selected_joint_group()

        if not hasattr(self.backend, "get_joint_snapshot"):
            self.append_log(
                "warning",
                "Get q is not available in this backend.",
            )
            return

        values = self.backend.get_joint_snapshot(group)

        if values is None:
            self.append_log(
                "warning",
                "No joint state is available.",
            )
            return

        _, dof = self.JOINT_GROUPS[group]
        self.joint_snapshot_value.setText(
            self._format_joint_vector(values, dof)
        )

    def _on_joint_group_changed(self, *args) -> None:
        del args
        if not hasattr(self, "joint_group_selector"):
            return

        group = self._selected_joint_group()
        label, dof = self.JOINT_GROUPS[group]
        cached_targets = self._joint_target_cache[group]

        for index, row_widgets in enumerate(self.joint_row_widgets):
            visible = index < dof
            for widget in row_widgets:
                widget.setVisible(visible)

            if not visible:
                continue

            self.joint_name_labels[index].setText(f"J{index + 1}")
            self.joint_target_spins[index].blockSignals(True)
            self.joint_target_spins[index].setValue(cached_targets[index])
            self.joint_target_spins[index].blockSignals(False)

        if hasattr(self, "joint_snapshot_value"):
            self.joint_snapshot_value.setText(
                self._empty_vector(dof)
            )

        self.append_log(
            "info",
            f"Joint group selected: {label}.",
        )

    def _on_joint_target_changed(
        self,
        index: int,
        value: float,
    ) -> None:
        group = self._selected_joint_group()
        _, dof = self.JOINT_GROUPS[group]
        if index < dof:
            self._joint_target_cache[group][index] = float(value)

    def _joint_jog(
        self,
        index: int,
        direction: int,
    ) -> None:
        group = self._selected_joint_group()
        _, dof = self.JOINT_GROUPS[group]
        if index >= dof:
            return

        step_deg = float(self.joint_jog_step.value()) * float(direction)

        if not hasattr(self.backend, "jog_joint"):
            self.append_log(
                "warning",
                "Joint jog is not available in this backend yet.",
            )
            return

        self.backend.jog_joint(
            group,
            index,
            step_deg,
        )

    def _move_joint_target(self) -> None:
        group = self._selected_joint_group()
        _, dof = self.JOINT_GROUPS[group]

        targets_deg = [
            float(self.joint_target_spins[i].value())
            for i in range(dof)
        ]
        self._joint_target_cache[group] = list(targets_deg)

        if not hasattr(self.backend, "move_joint_group"):
            self.append_log(
                "warning",
                "Joint target motion is not available in this backend yet.",
            )
            return

        self.backend.move_joint_group(
            group,
            targets_deg,
            float(self.joint_minimum_time.value()),
        )

    def _copy_current_joint_to_target(self) -> None:
        group = self._selected_joint_group()
        state = self._latest_motion_state.get("joint_groups", {})
        current = state.get(group)

        if current is None:
            self.append_log(
                "warning",
                "No current joint state is available to copy.",
            )
            return

        _, dof = self.JOINT_GROUPS[group]
        values = [float(v) for v in current[:dof]]
        self._joint_target_cache[group] = values

        for index, value in enumerate(values):
            self.joint_target_spins[index].blockSignals(True)
            self.joint_target_spins[index].setValue(value)
            self.joint_target_spins[index].blockSignals(False)

    # ------------------------------------------------------------------
    # Cartesian-space UI callbacks
    # ------------------------------------------------------------------
    def _get_cartesian_snapshot(self, arm: str) -> None:
        if arm not in self.CARTESIAN_ARMS:
            return

        if not hasattr(
            self.backend,
            "request_cartesian_snapshot",
        ):
            self.append_log(
                "warning",
                "Get TCP is not available in this backend.",
            )
            return

        requested = self.backend.request_cartesian_snapshot(
            arm
        )

        if not requested:
            self.append_log(
                "warning",
                f"Get TCP request could not be started for {arm}.",
            )
            return

        self.tcp_snapshot_values[arm].setText(
            "[ ..., ..., ..., ..., ..., ... ]"
        )

    def _on_cartesian_target_changed(
        self,
        arm: str,
        index: int,
        value: float,
    ) -> None:
        if arm not in self._cartesian_target_cache:
            return
        self._cartesian_target_cache[arm][index] = float(value)

    def _start_cartesian_hold(
        self,
        arm: str,
        axis_index: int,
        direction: int,
    ) -> None:
        """Start press-and-hold Cartesian translation control."""

        if arm not in self.CARTESIAN_ARMS:
            return
        if axis_index < 0 or axis_index >= 3:
            return

        if (
            self._cartesian_hold_command is not None
            and self._cartesian_hold_command
            != (arm, axis_index, direction)
            and hasattr(self.backend, "cancel_active_motion")
        ):
            self.backend.cancel_active_motion()

        self._cartesian_hold_command = (
            arm,
            int(axis_index),
            int(direction),
        )

        if not self.cartesian_hold_timer.isActive():
            self.cartesian_hold_timer.start()

        self._continue_cartesian_hold()

    def _continue_cartesian_hold(self) -> None:
        """Issue the next jog step after the previous action has finished."""

        command = self._cartesian_hold_command
        if command is None:
            self.cartesian_hold_timer.stop()
            return

        if (
            hasattr(self.backend, "is_motion_busy")
            and self.backend.is_motion_busy()
        ):
            return

        arm, axis_index, direction = command
        self._cartesian_jog(
            arm,
            axis_index,
            direction,
        )

    def _stop_cartesian_hold(
        self,
        *args,
        cancel_motion: bool = True,
    ) -> None:
        """Stop a held direction key and cancel its current action goal."""

        del args

        was_active = self._cartesian_hold_command is not None
        self._cartesian_hold_command = None
        self.cartesian_hold_timer.stop()

        # Action-level cancel stops the current arm command without using the
        # stronger global cancel_control service.
        if (
            was_active
            and cancel_motion
            and hasattr(self.backend, "cancel_active_motion")
        ):
            self.backend.cancel_active_motion()

    def _cartesian_jog(
        self,
        arm: str,
        axis_index: int,
        direction: int,
    ) -> None:
        if arm not in self.CARTESIAN_ARMS:
            return

        if axis_index < 3:
            delta = float(self.cartesian_linear_step.value())
        else:
            delta = float(self.cartesian_angular_step.value())

        delta *= float(direction)

        axis_names = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        self.append_log(
            "info",
            f"[UI JOG] {arm} "
            f"axis={axis_names[axis_index]}, "
            f"delta={delta:+.3f}",
        )

        if not hasattr(self.backend, "jog_cartesian"):
            self.append_log(
                "warning",
                "Cartesian jog is not available in this backend yet.",
            )
            return

        self.backend.jog_cartesian(
            arm,
            axis_index,
            delta,
            reference_frame="base",
        )

    def _move_cartesian_target(self, arm: str) -> None:
        if arm not in self.CARTESIAN_ARMS:
            return

        target = [
            float(spin.value())
            for spin in self.cartesian_target_spins[arm]
        ]
        self._cartesian_target_cache[arm] = list(target)

        if not hasattr(self.backend, "move_cartesian"):
            self.append_log(
                "warning",
                "Cartesian target motion is not available in this backend yet.",
            )
            return

        self.backend.move_cartesian(
            arm,
            target,
            float(self.cartesian_minimum_time.value()),
            reference_frame="base",
        )

    def _copy_current_cartesian_to_target(self, arm: str) -> None:
        if arm not in self.CARTESIAN_ARMS:
            return

        state = self._latest_motion_state.get("cartesian", {})
        current = state.get(arm)

        if current is None:
            self.append_log(
                "warning",
                f"No current Cartesian state is available for {arm}.",
            )
            return

        values = [float(v) for v in current[:6]]
        self._cartesian_target_cache[arm] = values

        for index, value in enumerate(values):
            spin = self.cartesian_target_spins[arm][index]
            spin.blockSignals(True)
            spin.setValue(value)
            spin.blockSignals(False)

    def _copy_selected_current_to_target(self) -> None:
        # Joint mode keeps the existing selected-group workflow.
        # Cartesian mode has independent per-arm buttons.
        mode = str(self.motion_type_selector.currentData())

        if mode == "joint":
            self._copy_current_joint_to_target()

    def _move_selected_target(self) -> None:
        # Joint mode keeps the existing selected-group workflow.
        # Cartesian mode has independent per-arm buttons.
        mode = str(self.motion_type_selector.currentData())

        if mode == "joint":
            self._move_joint_target()

    def _cancel_arm_motion(self) -> None:
        if hasattr(self.backend, "cancel_motion"):
            self.backend.cancel_motion()
        else:
            self.append_log(
                "warning",
                "Arm motion cancel is not available in this backend yet.",
            )

    def _refresh_motion_state(self) -> None:
        if not hasattr(self.backend, "get_motion_state"):
            return

        state = self.backend.get_motion_state()
        if not isinstance(state, dict):
            return

        self._latest_motion_state = state

        # Joint state: only the selected group is displayed.
        group = self._selected_joint_group()
        _, dof = self.JOINT_GROUPS[group]
        joint_values = state.get("joint_groups", {}).get(group)

        if joint_values is not None:
            for index in range(dof):
                if index < len(joint_values):
                    self.joint_current_labels[index].setText(
                        f"{float(joint_values[index]):+.2f}"
                    )
                else:
                    self.joint_current_labels[index].setText("--")

        # Cartesian state: Left and Right are rendered simultaneously.
        cartesian_state = state.get("cartesian", {})
        for arm in ("left_arm", "right_arm"):
            cartesian = cartesian_state.get(arm)
            labels = self.cartesian_current_labels.get(arm, [])

            if cartesian is None:
                continue

            for index in range(6):
                if index >= len(labels):
                    continue
                if index >= len(cartesian):
                    labels[index].setText("--")
                    continue

                value = float(cartesian[index])
                if index < 3:
                    labels[index].setText(
                        f"{value:+.4f} m"
                    )
                else:
                    labels[index].setText(
                        f"{value:+.2f}°"
                    )

        # Get TCP snapshots are independent for Left and Right.
        if hasattr(
            self.backend,
            "get_cartesian_snapshot",
        ):
            for arm in ("left_arm", "right_arm"):
                snapshot = self.backend.get_cartesian_snapshot(
                    arm
                )

                if snapshot is not None:
                    self.tcp_snapshot_values[arm].setText(
                        self._format_tcp_vector(snapshot)
                    )

    def _build_scenario_tab(self) -> QWidget:
        self.scenario_panel = ScenarioPanel(
            self.backend,
            on_log=self.append_log,
            on_active_changed=self._scenario_active_changed,
        )
        return self.scenario_panel

    def _scenario_active_changed(self, active: bool) -> None:
        self._scenario_active = bool(active)
        if active:
            self._stop_cartesian_hold(cancel_motion=False)
            self._stop_base_only()

        # Manual Base and Joint/Cartesian controls cannot race a Task.
        self.tabs.setTabEnabled(0, not active)
        self.tabs.setTabEnabled(1, not active)

    def _build_diagnostics_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # ROS/backend details live here instead of occupying operator space.
        layout.addWidget(self._build_connection_group())

        group = QGroupBox("Diagnostics")
        group_layout = QVBoxLayout(group)

        run_button = QPushButton("Run Diagnostics")
        run_button.clicked.connect(self.run_diagnostics)

        self.diagnostic_summary = QPlainTextEdit()
        self.diagnostic_summary.setReadOnly(True)

        group_layout.addWidget(run_button)
        group_layout.addWidget(self.diagnostic_summary)

        layout.addWidget(group, 1)

        stop_row = QHBoxLayout()
        stop_row.addStretch(1)
        stop_row.addWidget(self._motion_stop_button())
        layout.addLayout(stop_row)

        return tab

    def run_diagnostics(self) -> None:
        if hasattr(self.backend, "run_diagnostics"):
            data = self.backend.run_diagnostics()
            lines = [
                f"{key}: {value}"
                for key, value in data.items()
            ]
        else:
            snapshot = self.backend.snapshot()
            lines = [
                f"backend: {self.backend_name}",
                f"namespace: {snapshot.namespace}",
                f"cmd_vel_topic: {snapshot.cmd_vel_topic}",
                (
                    "cmd_vel_subscribers: "
                    f"{snapshot.cmd_vel_subscribers}"
                ),
                f"control_state: {snapshot.control_state}",
                f"stream_enabled: {snapshot.stream_enabled}",
                f"emo_active: {snapshot.emo_active}",
                (
                    "collision_active: "
                    f"{snapshot.collision_active}"
                ),
                (
                    "services_enabled: "
                    f"{snapshot.services_enabled}"
                ),
                f"service_ready: {snapshot.service_ready}",
            ]

        self.diagnostic_summary.setPlainText(
            "\n".join(lines)
        )
        self.append_log(
            "info",
            "Diagnostics refreshed.",
        )

    # ==================================================================
    # Terminal-only UI logging
    # ==================================================================
    def append_log(
        self,
        level: str,
        message: str,
    ) -> None:
        prefix = {
            "error": "[UI ERROR]",
            "warning": "[UI WARN]",
            "info": "[UI INFO]",
        }.get(level, "[UI]")

        print(
            f"{prefix} {message}",
            flush=True,
        )

    # ==================================================================
    # Base command handling
    # ==================================================================
    def _press_action(
        self,
        action: str,
    ) -> None:
        if self._scenario_active:
            return
        self._pressed_actions.add(action)
        self._refresh_command()

    def _release_action(
        self,
        action: str,
    ) -> None:
        self._pressed_actions.discard(action)
        self._refresh_command()

    def _stream_off(self) -> None:
        # Stop first, then disable stream.
        self.motion_stop()
        self.backend.request_stream(False)

    def _stop_base_only(self) -> None:
        """Stop only mobile-base motion."""

        self._pressed_actions.clear()

        self.backend.stop(
            publish_immediately=True
        )

        self._update_command_labels(
            0.0,
            0.0,
            0.0,
        )

    def motion_stop(self) -> None:
        """Software motion stop: base zero + active motion cancel."""

        # Prevent a held Cartesian direction key from issuing another goal.
        self._stop_cartesian_hold(cancel_motion=False)

        # Stop mobile base first.
        self._stop_base_only()

        scenario_was_active = (
            hasattr(self, "scenario_panel")
            and self.scenario_panel.runner.active
        )
        if scenario_was_active:
            self.scenario_panel.emergency_stop()

        # A running Scenario already requests cancellation through its runner.
        if not scenario_was_active and hasattr(self.backend, "cancel_motion"):
            self.backend.cancel_motion()

    def _refresh_command(self) -> None:
        if self._closing or self._scenario_active:
            return

        vx, vy, wz = self._calculate_command()

        snapshot = self.backend.snapshot()

        if (
            self.require_stream.isChecked()
            and snapshot.services_enabled
            and snapshot.stream_enabled is not True
        ):
            vx = 0.0
            vy = 0.0
            wz = 0.0

        self.backend.set_velocity(
            vx,
            vy,
            wz,
        )
        self._update_command_labels(
            vx,
            vy,
            wz,
        )

    def _calculate_command(self):
        linear = float(
            self.linear_speed.value()
        )
        lateral = float(
            self.lateral_speed.value()
        )
        angular = float(
            self.angular_speed.value()
        )

        vx = linear * (
            int(
                self.ACTION_FORWARD
                in self._pressed_actions
            )
            - int(
                self.ACTION_BACKWARD
                in self._pressed_actions
            )
        )

        lateral_direction = (
            int(
                self.ACTION_LEFT
                in self._pressed_actions
            )
            - int(
                self.ACTION_RIGHT
                in self._pressed_actions
            )
        )

        if self.invert_lateral.isChecked():
            lateral_direction *= -1

        vy = lateral * lateral_direction

        wz = angular * (
            int(
                self.ACTION_ROTATE_CCW
                in self._pressed_actions
            )
            - int(
                self.ACTION_ROTATE_CW
                in self._pressed_actions
            )
        )

        return vx, vy, wz

    def _update_command_labels(
        self,
        vx: float,
        vy: float,
        wz: float,
    ) -> None:
        self.vx_value.setText(
            f"{vx:+.2f} m/s"
        )
        self.vy_value.setText(
            f"{vy:+.2f} m/s"
        )
        self.wz_value.setText(
            f"{wz:+.2f} rad/s"
        )

    # ==================================================================
    # Backend state rendering
    # ==================================================================
    def refresh_backend_status(self) -> None:
        snapshot = self.backend.snapshot()

        self.backend_value.setText(
            self.backend_name
        )
        self.namespace_value.setText(
            snapshot.namespace
        )
        self.topic_value.setText(
            snapshot.cmd_vel_topic
        )
        self.subscriber_value.setText(
            str(snapshot.cmd_vel_subscribers)
        )

        if snapshot.control_state is None:
            control_state_text = "UNKNOWN"
        else:
            control_state_text = self.CONTROL_STATE_NAMES.get(
                int(snapshot.control_state),
                f"UNKNOWN ({snapshot.control_state})",
            )

        self.state_value.setText(control_state_text)

        self._set_status_indicator(
            self.power_state_value,
            snapshot.power_enabled,
        )
        self._set_status_indicator(
            self.servo_state_value,
            snapshot.servo_enabled,
        )
        self._set_status_indicator(
            self.stream_value,
            snapshot.stream_enabled,
        )

        self.emo_value.setText(
            "ACTIVE"
            if snapshot.emo_active is True
            else "RELEASED"
            if snapshot.emo_active is False
            else "UNKNOWN"
        )

        self._set_estop_status(snapshot.emo_active)

        self.collision_value.setText(
            "ACTIVE"
            if snapshot.collision_active is True
            else "CLEAR"
            if snapshot.collision_active is False
            else "unknown"
        )

        control_manager_ready = False

        if snapshot.services_enabled:
            ready = snapshot.service_ready
            control_manager_ready = bool(
                ready.get("control_manager", False)
            )
            service_items = (
                ("power", "PWR"),
                ("servo", "SRV"),
                ("stream", "STR"),
                ("control_manager", "CM"),
                ("joint_action", "JNT"),
                ("cartesian_action", "TCP"),
                ("cartesian_pose", "POSE"),
                ("cancel_control", "CANCEL"),
                ("scenario_safety", "SAFE"),
            )
            parts = [
                f'{label}:{"ready" if ready.get(key) else "wait"}'
                for key, label in service_items
                if key in ready
            ]
            self.service_value.setText(
                "  ".join(parts)
                if parts
                else "no interface status"
            )
        else:
            self.service_value.setText(
                "cmd_vel-only mode"
            )

        service_widgets = (
            self.power_on_button,
            self.power_off_button,
            self.servo_on_button,
            self.servo_off_button,
            self.stream_on_button,
            self.stream_off_button,
        )

        for widget in service_widgets:
            widget.setEnabled(
                snapshot.services_enabled
            )

        for widget in (
            self.control_manager_enable_button,
            self.control_manager_disable_button,
            self.control_manager_reset_button,
        ):
            widget.setEnabled(control_manager_ready)

        # Day 2 motion state, when the backend provides it.
        self._refresh_motion_state()

        backend_events = self.backend.drain_events()
        if self.backend_name != "ROS2":
            for level, message in backend_events:
                self.append_log(level, message)

    # ==================================================================
    # Keyboard / focus safety
    # ==================================================================
    def eventFilter(
        self,
        watched,
        event,
    ):  # noqa: N802
        event_kind = event.type()

        if event_kind == event_type(
            "WindowDeactivate"
        ):
            if watched is self:
                # A mouse release can be missed if focus is lost while a
                # Cartesian direction key is held.
                self._stop_cartesian_hold()
                if self.stop_on_focus_loss.isChecked():
                    self._stop_base_only()
            return False
        if (
            not self.keyboard_enable.isChecked()
            or not self.isActiveWindow()
        ):
            return False

        focused = QApplication.focusWidget()

        if isinstance(
            focused,
            (
                QAbstractSpinBox,
                QLineEdit,
                QComboBox,
            ),
        ):
            return False

        if event_kind not in (
            event_type("KeyPress"),
            event_type("KeyRelease"),
        ):
            return False

        if event.isAutoRepeat():
            return True

        key = event.key()

        if key == qt_key("Key_Space"):
            if event_kind == event_type(
                "KeyPress"
            ):
                self.motion_stop()
            return True

        action = self._action_for_key(key)

        if action is None:
            return False

        if event_kind == event_type(
            "KeyPress"
        ):
            self._press_action(action)
        else:
            self._release_action(action)

        return True

    def _action_for_key(
        self,
        key: int,
    ):
        mapping = {
            qt_key("Key_Up"): self.ACTION_FORWARD,
            qt_key("Key_W"): self.ACTION_FORWARD,
            qt_key("Key_Down"): self.ACTION_BACKWARD,
            qt_key("Key_S"): self.ACTION_BACKWARD,
            qt_key("Key_A"): self.ACTION_LEFT,
            qt_key("Key_D"): self.ACTION_RIGHT,
            qt_key("Key_Q"): self.ACTION_ROTATE_CCW,
            qt_key("Key_E"): self.ACTION_ROTATE_CW,
        }

        if self.arrow_mode.currentData() == "rotate":
            mapping[
                qt_key("Key_Left")
            ] = self.ACTION_ROTATE_CCW
            mapping[
                qt_key("Key_Right")
            ] = self.ACTION_ROTATE_CW
        else:
            mapping[
                qt_key("Key_Left")
            ] = self.ACTION_LEFT
            mapping[
                qt_key("Key_Right")
            ] = self.ACTION_RIGHT

        return mapping.get(key)

    # ==================================================================
    # Shutdown
    # ==================================================================
    def closeEvent(self, event):  # noqa: N802
        self._closing = True
        self.command_timer.stop()
        self.status_timer.stop()
        self.cartesian_hold_timer.stop()
        self._cartesian_hold_command = None
        self._pressed_actions.clear()

        if hasattr(self, "scenario_panel"):
            self.scenario_panel.runner.close()

        try:
            self.backend.shutdown_safely(
                turn_stream_off=True
            )
        finally:
            event.accept()

    # ==================================================================
    # Style
    # ==================================================================
    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow {
                background: #20242b;
                color: #e8edf2;
            }

            QWidget {
                color: #e8edf2;
                font-size: 12px;
            }

            QGroupBox {
                border: 1px solid #4b5563;
                border-radius: 6px;
                margin-top: 10px;
                padding-top: 6px;
                font-weight: 600;
            }

            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 4px;
            }

            QPushButton {
                background: #343b46;
                border: 1px solid #657184;
                border-radius: 5px;
                padding: 5px 8px;
                font-weight: 600;
            }

            QPushButton:hover {
                background: #414b59;
            }

            QPushButton:pressed {
                background: #2c6e9b;
            }

            QPushButton:disabled {
                color: #7d8794;
                background: #2a2f36;
            }

            QPushButton#motionStop {
                background: #7f4a24;
                color: white;
                font-size: 12px;
                font-weight: 700;
            }

            QPushButton#eStopButton {
                background: #b42323;
                color: white;
                border: 2px solid #ef5555;
                border-radius: 8px;
                font-size: 15px;
                font-weight: 800;
            }

            QPushButton#eStopButton:disabled {
                background: #5f3030;
                color: #c7a0a0;
                border: 2px solid #754343;
            }

            QPushButton#eStopButton[emoActive="true"]:disabled {
                background: #b42323;
                color: white;
                border: 2px solid #ff6b6b;
            }

            QDoubleSpinBox,
            QComboBox,
            QListWidget,
            QPlainTextEdit {
                background: #171a1f;
                border: 1px solid #4b5563;
                border-radius: 4px;
                padding: 3px;
            }

            QListWidget::item:selected {
                background: #2c6e9b;
                color: #ffffff;
            }

            QPushButton#emergencyStop {
                background: #9b2c2c;
                color: white;
                font-size: 13px;
                font-weight: 700;
            }

            QFrame#headerFrame {
                background: #292f38;
                border-radius: 7px;
            }

            QTabWidget::pane {
                border: 1px solid #4b5563;
                border-radius: 5px;
            }

            QTabBar::tab {
                background: #2a2f36;
                padding: 7px 14px;
                margin-right: 2px;
            }

            QTabBar::tab:selected {
                background: #414b59;
            }


            QFrame#controlStateCard {
                background: #1f2630;
                border: 1px solid #607086;
                border-radius: 6px;
            }

            QLabel#controlStateTitle {
                color: #aeb8c4;
                font-size: 11px;
                font-weight: 700;
            }

            QLabel#controlStateValue {
                font-size: 17px;
                font-weight: 800;
            }

            QLabel#vectorSnapshot {
                background: #171a1f;
                border: 1px solid #4b5563;
                border-radius: 4px;
                padding: 4px 7px;
                font-family: Monospace;
            }

            QLabel#stateValue {
                font-weight: 700;
            }

            QLabel#statusOn {
                color: #43c463;
                font-weight: 700;
            }

            QLabel#statusOff {
                color: #e05252;
                font-weight: 700;
            }

            QLabel#statusUnknown {
                color: #9ca3af;
                font-weight: 700;
            }

            QLabel#secondaryNote {
                color: #aeb8c4;
                font-size: 11px;
            }
            """
        )
