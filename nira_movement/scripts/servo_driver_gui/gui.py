#!/usr/bin/env python3
"""FEETECH Servo Command Center. Run with --demo to explore without hardware."""

import sys
import time
from datetime import datetime

import serial.tools.list_ports
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QKeySequence, QPalette, QShortcut
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QFrame,
    QLabel, QPushButton, QComboBox, QSpinBox, QMessageBox,
    QListWidget, QListWidgetItem, QPlainTextEdit, QTabWidget, QProgressBar,
    QScrollArea, QSizePolicy,
)

from dashboard_widgets import STYLESHEET, PositionDial, PositionHistory, PositionSlider
from servo_bus import ANGLE_MAX, SPEED_MAX, ACC_MAX, BAUD_RATES, BusWorker, command_limits, decode_status

FEEDBACK_STALE_SECONDS = 3.0

def label(text, name=None):
    widget = QLabel(text)
    if name:
        widget.setObjectName(name)
    return widget


def button(text, callback, name=None):
    widget = QPushButton(text)
    if name:
        widget.setObjectName(name)
    widget.setCursor(Qt.CursorShape.PointingHandCursor)
    widget.clicked.connect(callback)
    return widget


def spin(minimum, maximum, value=0, suffix=""):
    widget = QSpinBox()
    widget.setRange(minimum, maximum)
    widget.setValue(value)
    widget.setSuffix(suffix)
    widget.setKeyboardTracking(False)
    return widget


def card():
    widget = QFrame()
    widget.setObjectName("card")
    layout = QVBoxLayout(widget)
    layout.setContentsMargins(20, 16, 20, 16)
    layout.setSpacing(10)
    return widget, layout


class Window(QWidget):
    def __init__(self, demo=False, worker=None, ros_service=None):
        super().__init__()
        self.setWindowTitle("FEETECH · Servo Command Center")
        self.resize(1320, 920)
        self.setMinimumSize(980, 720)
        palette = self.palette()
        for role, color in [(QPalette.ColorRole.Window, "#0d131c"),
                            (QPalette.ColorRole.Base, "#101824"),
                            (QPalette.ColorRole.Button, "#223044"),
                            (QPalette.ColorRole.ButtonText, "#e8eef6"),
                            (QPalette.ColorRole.Text, "#e8eef6"),
                            (QPalette.ColorRole.WindowText, "#e8eef6"),
                            (QPalette.ColorRole.Highlight, "#28574f")]:
            palette.setColor(role, QColor(color))
        self.setPalette(palette)
        self.setStyleSheet(STYLESHEET)
        self._connected = False
        self._scanning = False
        self._active_sid = None
        self._settings_ready = False
        self._actual = self._requested = None
        self._last_feedback = 0
        self._pending = False
        self._draft_dirty = False
        self._last_error = ""
        self._limits = (0, ANGLE_MAX)
        self.worker = worker or BusWorker()
        self.ros_service = ros_service

        root = QVBoxLayout(self)
        root.setContentsMargins(24, 20, 24, 16)
        root.setSpacing(14)
        header = QHBoxLayout()
        brand = QVBoxLayout()
        brand.addWidget(label("FEETECH  /  MOTION SYSTEMS", "eyebrow"))
        brand.addWidget(label("Servo command center", "title"))
        header.addLayout(brand)
        header.addStretch()
        self.connectionBadge = label("●  OFFLINE", "badge")
        header.addWidget(self.connectionBadge)
        self.holdBtn = button("Ⅱ  Hold position", self.hold_position, "hold")
        self.holdBtn.setToolTip("Command the selected servo to its latest measured position. Shortcut: Space. This is not an emergency stop or torque release.")
        header.addWidget(self.holdBtn)
        root.addLayout(header)

        body = QHBoxLayout()
        body.setSpacing(18)
        sidebar = self._build_sidebar()
        sidebar.setMinimumHeight(730)
        sidebar_scroll = QScrollArea()
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setFixedWidth(260)
        sidebar_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        sidebar_scroll.setWidget(sidebar)
        body.addWidget(sidebar_scroll)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 6, 0)
        content_layout.setSpacing(12)
        self.activeLabel = label("Your workspace is ready", "section")
        content_layout.addWidget(self.activeLabel)
        self.notice = label("Connect an adapter and scan, or open the demo to explore the controls.", "notice")
        if self.ros_service:
            self.notice.setText("Connect to the movement node to list the robot's configured servos.")
        self.notice.setWordWrap(True)
        content_layout.addWidget(self.notice)
        content_layout.addWidget(self._build_monitor())
        content_layout.addLayout(self._build_metrics())
        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_motion(), "Motion control")
        self.tabs.addTab(self._build_profile(), "Velocity / acceleration")
        self.tabs.addTab(self._build_settings(), "Device settings")
        self.tabs.addTab(self._build_activity(), "Activity log")
        content_layout.addWidget(self.tabs)
        content_layout.addStretch()
        scroll.setWidget(content)
        body.addWidget(scroll, 1)
        root.addLayout(body, 1)
        footer = QHBoxLayout()
        self.footerStatus = label("Disconnected", "muted")
        footer.addWidget(self.footerStatus)
        footer.addStretch()
        footer.addWidget(label("STS / HLS   ·   12-bit position   ·   0–4095 ticks", "muted"))
        root.addLayout(footer)

        self.worker.connected.connect(self._on_connected)
        self.worker.found.connect(self._add_servo)
        self.worker.scan_progress.connect(self._scan_progress)
        self.worker.scan_done.connect(self._scan_done)
        self.worker.selected.connect(self._on_settings)
        self.worker.telemetry.connect(self._on_telemetry)
        self.worker.failed.connect(self._on_error)
        self.worker.activity.connect(self._log)
        self.worker.changed_id.connect(self._on_id_changed)
        self.worker.completed.connect(self._on_completed)
        self.worker.profile_readback.connect(self._on_profile_readback)
        self.worker.start()
        self.watchdog = QTimer(self)
        self.watchdog.timeout.connect(self._check_freshness)
        self.watchdog.start(250)
        self.holdShortcut = QShortcut(QKeySequence("Space"), self)
        self.holdShortcut.activated.connect(self.hold_position)
        if not self.ros_service:
            self.refresh_ports()
        self._update_enabled()
        self._log("Command center ready. No movement is sent until you press Move to target.")
        if demo:
            QTimer.singleShot(0, self.start_demo)

    def _build_sidebar(self):
        panel, layout = card()
        layout.setContentsMargins(16, 18, 16, 18)
        layout.addWidget(label("01  /  CONNECTION", "eyebrow"))
        if self.ros_service:
            layout.addWidget(label("ROS service", "section"))
            layout.addWidget(label(self.ros_service))
        else:
            layout.addWidget(label("Serial bus", "section"))
            layout.addWidget(label("Adapter port", "muted"))
            port_row = QHBoxLayout()
            self.portBox = QComboBox()
            self.portBox.setEditable(True)
            self.portBox.setMinimumWidth(100)
            self.portBox.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed)
            self.portBox.setAccessibleName("Serial adapter port")
            port_row.addWidget(self.portBox, 1)
            self.refreshBtn = button("↻", self.refresh_ports)
            self.refreshBtn.setFixedWidth(36)
            self.refreshBtn.setToolTip("Refresh serial ports")
            port_row.addWidget(self.refreshBtn)
            layout.addLayout(port_row)
            layout.addWidget(label("Bus baud rate", "muted"))
            self.baudBox = QComboBox()
            self.baudBox.addItems(BAUD_RATES)
            self.baudBox.setAccessibleName("Connection baud rate")
            layout.addWidget(self.baudBox)
        self.connect_label = "Connect to robot" if self.ros_service else "Connect adapter"
        self.connectBtn = button(self.connect_label, self.connect_serial, "primary")
        layout.addWidget(self.connectBtn)
        self.demoBtn = button("Explore demo", self.start_demo)
        layout.addWidget(self.demoBtn)
        layout.addSpacing(12)
        layout.addWidget(label("02  /  DEVICES", "eyebrow"))
        scan_row = QHBoxLayout()
        self.scanFrom = spin(0, 253, 1)
        self.scanTo = spin(0, 253, 253 if self.ros_service else 30)
        self.scanFrom.setAccessibleName("First scan ID")
        self.scanTo.setAccessibleName("Last scan ID")
        scan_row.addWidget(self.scanFrom)
        scan_row.addWidget(label("to", "muted"))
        scan_row.addWidget(self.scanTo)
        layout.addLayout(scan_row)
        self.scanBtn = button("Scan for servos", self.scan_servos)
        layout.addWidget(self.scanBtn)
        self.scanProgress = QProgressBar()
        self.scanProgress.setTextVisible(False)
        self.scanProgress.setFixedHeight(4)
        layout.addWidget(self.scanProgress)
        self.deviceCount = label("No devices discovered", "muted")
        layout.addWidget(self.deviceCount)
        self.servoList = QListWidget()
        self.servoList.setMinimumHeight(120)
        self.servoList.setAccessibleName("Discovered servos")
        self.servoList.currentItemChanged.connect(self._select_servo)
        layout.addWidget(self.servoList, 1)
        manual = QHBoxLayout()
        self.idBox = spin(0, 253, 1)
        self.idBox.setAccessibleName("Manual servo ID")
        manual.addWidget(self.idBox)
        self.selectBtn = button("Open ID", self.open_id)
        manual.addWidget(self.selectBtn)
        layout.addLayout(manual)
        hint = label("Select one servo to inspect its feedback and prepare a move.", "muted")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        return panel

    def _build_monitor(self):
        panel, layout = card()
        title_row = QHBoxLayout()
        title_row.addWidget(label("Position monitor", "section"))
        title_row.addStretch()
        self.feedbackBadge = label("AWAITING DEVICE", "muted")
        title_row.addWidget(self.feedbackBadge)
        layout.addLayout(title_row)
        instruments = QHBoxLayout()
        instruments.setSpacing(22)
        self.dial = PositionDial()
        instruments.addWidget(self.dial, 1)
        readings = QVBoxLayout()
        readings.setSpacing(7)
        values = QHBoxLayout()
        for title, name, attr in [("●  ACTUAL", "actual", "posLabel"),
                                  ("●  REQUESTED", "requested", "goalLabel")]:
            column = QVBoxLayout()
            heading = label(title, "eyebrow")
            heading.setStyleSheet(f"color: {'#54ddc2' if name == 'actual' else '#ffbd69'}")
            column.addWidget(heading)
            value = label("—", name)
            setattr(self, attr, value)
            column.addWidget(value)
            column.addWidget(label("encoder ticks", "muted"))
            values.addLayout(column)
        readings.addLayout(values)
        self.errorLabel = label("Following error  —", "muted")
        readings.addWidget(self.errorLabel)
        readings.addSpacing(8)
        self.history = PositionHistory()
        readings.addWidget(self.history)
        instruments.addLayout(readings, 2)
        layout.addLayout(instruments)
        legend = label("Teal: measured feedback     ·     Amber: requested position read from the servo", "muted")
        legend.setWordWrap(True)
        layout.addWidget(legend)
        return panel

    def _build_metrics(self):
        row = QHBoxLayout()
        row.setSpacing(12)
        for title, attr, detail in [("BUS VOLTAGE", "voltageLabel", "Supply at the servo"),
                                     ("TEMPERATURE", "tempLabel", "Internal sensor"),
                                     ("DEVICE STATUS", "healthLabel", "Reported fault flags")]:
            panel, layout = card()
            layout.setContentsMargins(16, 13, 16, 13)
            layout.setSpacing(5)
            layout.addWidget(label(title, "eyebrow"))
            value = label("—", "metric")
            if attr == "healthLabel":
                value.setStyleSheet("font-size: 18px")
                value.setWordWrap(True)
            setattr(self, attr, value)
            layout.addWidget(value)
            layout.addWidget(label(detail, "muted"))
            row.addWidget(panel, 1)
        return row

    def _build_motion(self):
        panel, layout = card()
        row = QHBoxLayout()
        row.addWidget(label("Prepare a move", "section"))
        row.addStretch()
        self.draftLabel = label("Target is staged until you send it", "muted")
        row.addWidget(self.draftLabel)
        layout.addLayout(row)
        target = QHBoxLayout()
        target.addWidget(label("Target position", "muted"))
        self.goalEdit = spin(0, ANGLE_MAX, 2048, " ticks")
        self.goalEdit.setAccessibleName("Target position in encoder ticks")
        target.addWidget(self.goalEdit)
        self.degreeLabel = label("180.0°", "muted")
        target.addWidget(self.degreeLabel)
        target.addStretch()
        self.matchBtn = button("Use actual", self.use_actual)
        target.addWidget(self.matchBtn)
        layout.addLayout(target)
        self.posSlider = PositionSlider()
        self.posSlider.setRange(0, ANGLE_MAX)
        self.posSlider.setValue(2048)
        self.posSlider.setAccessibleName("Target position")
        self.posSlider.valueChanged.connect(self.goalEdit.setValue)
        self.goalEdit.valueChanged.connect(self._target_changed)
        layout.addWidget(self.posSlider)
        presets = QHBoxLayout()
        self.presetButtons = []
        for text, fraction in [("Min", 0), ("25%", 0.25), ("Center", 0.5), ("75%", 0.75), ("Max", 1)]:
            btn = button(text, lambda checked=False, f=fraction: self.stage_fraction(f))
            btn.setToolTip("Stage a target within the servo's configured position limits")
            self.presetButtons.append(btn)
            presets.addWidget(btn)
        presets.addStretch()
        self.jogMinus = button("−", lambda: self.jog(-1))
        self.jogPlus = button("+", lambda: self.jog(1))
        self.jogStep = spin(1, 1024, 32, " ticks")
        self.jogStep.setAccessibleName("Target adjustment step")
        presets.addWidget(self.jogMinus)
        presets.addWidget(self.jogStep)
        presets.addWidget(self.jogPlus)
        layout.addLayout(presets)
        self.profileSummary = label("Set the move profile in the Velocity / acceleration tab.", "muted")
        self.profileSummary.setWordWrap(True)
        layout.addWidget(self.profileSummary)
        action = QHBoxLayout()
        self.limitLabel = label("Travel limits  0–4095 ticks", "muted")
        action.addWidget(self.limitLabel)
        action.addStretch()
        self.moveBtn = button("Move to target  →", self.move_to_target, "primary")
        self.moveBtn.setMinimumWidth(185)
        action.addWidget(self.moveBtn)
        layout.addLayout(action)
        return panel

    def _build_profile(self):
        panel, layout = card()
        layout.addWidget(label("Velocity & acceleration", "section"))
        hint = label("Set the motion profile for this servo. Apply updates the profile without sending a new position target; it can affect a move already in progress.", "muted")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        self.liveVelocityLabel = label("Actual velocity  —", "section")
        layout.addWidget(self.liveVelocityLabel)
        self.speedEdit = spin(0, SPEED_MAX, 800, " ticks/s")
        self.accEdit = spin(0, ACC_MAX, 50)
        self.speedEdit.setAccessibleName("Requested velocity limit in ticks per second")
        self.accEdit.setAccessibleName("Requested acceleration register value")
        self.speedSlider = PositionSlider()
        self.speedSlider.setRange(0, SPEED_MAX)
        self.accSlider = PositionSlider()
        self.accSlider.setRange(0, ACC_MAX)
        self.speedSlider.setAccessibleName("Velocity limit")
        self.accSlider.setAccessibleName("Acceleration setting")
        self.speedReadback = label("Stored velocity limit  —", "muted")
        self.accReadback = label("Stored acceleration  —", "muted")
        for title, edit, slider, readback, hint_text in [
                ("Velocity limit", self.speedEdit, self.speedSlider, self.speedReadback, "0 = unlimited speed"),
                ("Acceleration", self.accEdit, self.accSlider, self.accReadback, "Raw setting · 0 = maximum acceleration")]:
            row = QHBoxLayout()
            row.addWidget(label(title))
            row.addStretch()
            row.addWidget(edit)
            layout.addLayout(row)
            slider.setValue(edit.value())
            slider.valueChanged.connect(edit.setValue)
            edit.valueChanged.connect(slider.setValue)
            edit.valueChanged.connect(self._profile_edited)
            layout.addWidget(slider)
            row = QHBoxLayout()
            row.addWidget(readback)
            row.addStretch()
            row.addWidget(label(hint_text, "muted"))
            layout.addLayout(row)
        note = label("Actual velocity is live signed feedback. Stored values are read from the servo, not the input fields. Acceleration is a configured register value, not measured acceleration.", "muted")
        note.setWordWrap(True)
        layout.addWidget(note)
        row = QHBoxLayout()
        self.profileStatus = label("Edit values, then apply or send a move.", "muted")
        self.profileStatus.setWordWrap(True)
        row.addWidget(self.profileStatus, 1)
        self.applyProfileBtn = button("Apply motion profile", self.apply_profile, "primary")
        row.addWidget(self.applyProfileBtn)
        layout.addLayout(row)
        return panel

    def _profile_edited(self):
        self.profileStatus.setText("Edited · not yet applied")
        self.profileSummary.setText(f"Next move: velocity {self.speedEdit.value()} ticks/s · acceleration {self.accEdit.value()} (edit in Velocity / acceleration).")

    def apply_profile(self):
        self.speedEdit.interpretText()
        self.accEdit.interpretText()
        if self._ready() and not self._pending:
            self.profileStatus.setText("Applying · waiting for readback…")
            self._send("profile", self.speedEdit.value(), self.accEdit.value())

    def _on_profile_readback(self, sid, data):
        if sid != self._active_sid:
            return
        speed, acc = data.get("speed"), data.get("acc")
        self.speedReadback.setText(f"Stored velocity limit  {speed} ticks/s" if speed is not None else "Stored velocity limit  —")
        self.accReadback.setText(f"Stored acceleration  {acc}" if acc is not None else "Stored acceleration  —")

    def _build_settings(self):
        panel, layout = card()
        layout.addWidget(label("Device configuration", "section"))
        note = label("Writes apply to the selected servo. ID and baud changes require confirmation. Limits are read back after saving; EEPROM protection may prevent changes.", "muted")
        note.setWordWrap(True)
        layout.addWidget(note)
        form = QGridLayout()
        form.setHorizontalSpacing(12)
        form.setVerticalSpacing(14)
        self.newId = spin(0, 253, 1)
        self.idWriteBtn = button("Change ID", self.write_id)
        form.addWidget(label("Device ID"), 0, 0)
        form.addWidget(self.newId, 0, 1)
        form.addWidget(self.idWriteBtn, 0, 2)
        self.newBaud = QComboBox()
        self.newBaud.addItems(BAUD_RATES)
        self.baudWriteBtn = button("Change baud", self.write_baud)
        if not self.ros_service:
            form.addWidget(label("Device baud rate"), 1, 0)
            form.addWidget(self.newBaud, 1, 1)
            form.addWidget(self.baudWriteBtn, 1, 2)
        # Display the complete raw words, including model-specific values.
        self.minAngle = spin(0, 65535, 0)
        self.maxAngle = spin(0, 65535, ANGLE_MAX)
        limits = QHBoxLayout()
        limits.addWidget(self.minAngle)
        limits.addWidget(label("to", "muted"))
        limits.addWidget(self.maxAngle)
        self.limitsBtn = button("Save limits", self.write_limits)
        form.addWidget(label("Travel limits"), 2, 0)
        form.addLayout(limits, 2, 1)
        form.addWidget(self.limitsBtn, 2, 2)
        layout.addLayout(form)
        hint = label("Position control assumes single-turn mode. The angle display uses 4096 ticks per revolution. Hold position sends a position command; it does not disable torque.", "muted")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        return panel

    def _build_activity(self):
        panel, layout = card()
        row = QHBoxLayout()
        row.addWidget(label("Session activity", "section"))
        row.addStretch()
        row.addWidget(button("Clear log", lambda: self.logView.clear()))
        layout.addLayout(row)
        self.logView = QPlainTextEdit()
        self.logView.setReadOnly(True)
        self.logView.setMaximumBlockCount(1000)
        self.logView.setMinimumHeight(220)
        layout.addWidget(self.logView)
        return panel

    def refresh_ports(self):
        current = self.portBox.currentText()
        self.portBox.clear()
        self.portBox.addItems([p.device for p in serial.tools.list_ports.comports()])
        self.portBox.setCurrentText(current or "/dev/ttyUSB0")

    def _log(self, text):
        self.logView.appendPlainText(f"{datetime.now():%H:%M:%S}  {text}")

    def _show_notice(self, text):
        self.notice.setText(text)
        self.notice.show()

    def connect_serial(self):
        if self._connected:
            self.worker.cancel_scan = True
            self._reset_selection()
            self._connected = False
            self._pending = True
            self._update_enabled()
            self.worker.submit("disconnect")
            return
        if self.ros_service:
            self._pending = True
            self._last_error = ""
            self._update_enabled()
            self._show_notice("Connecting to the movement node…")
            self.worker.submit("connect", self.ros_service, 0, False)
            return
        port = self.portBox.currentText().strip()
        if not port:
            self._show_notice("Choose or enter a serial adapter port first.")
            return
        self._pending = True
        self._last_error = ""
        self._update_enabled()
        self._show_notice("Opening serial adapter…")
        self.worker.submit("connect", port, int(self.baudBox.currentText()), False)

    def start_demo(self):
        if self._connected or self._pending:
            return
        self._pending = True
        self._update_enabled()
        self.worker.submit("connect", "demo", 1000000, True)

    def _on_connected(self, connected, description):
        self._connected = connected
        self._pending = False
        self._scanning = False
        self.connectBtn.setText("Disconnect" if connected else self.connect_label)
        self.connectionBadge.setText("●  DEMO MODE" if description.startswith("Demo") else
                                     "●  CONNECTED" if connected else "●  OFFLINE")
        self.footerStatus.setText(description)
        self._log(description)
        if not connected:
            self.servoList.clear()
            self.deviceCount.setText("No devices discovered")
            self._reset_selection()
            self._show_notice(self._last_error if self._last_error.startswith("Connect:") else
                              "Disconnected. Connect an adapter or explore the demo.")
        else:
            self._show_notice("Connected. Scan for servos or open a known device ID.")
        self._update_enabled()
        if connected and (self.ros_service or description.startswith("Demo")):
            self.scan_servos()

    def scan_servos(self):
        if self._scanning:
            self.worker.cancel_scan = True
            self.scanBtn.setText("Stopping…")
            self.scanBtn.setEnabled(False)
            return
        if not self._connected or self._pending:
            return
        start, end = self.scanFrom.value(), self.scanTo.value()
        if start > end:
            self._show_notice("The first scan ID must be less than or equal to the last ID.")
            return
        self._scanning = True
        self.worker.cancel_scan = False
        self.servoList.clear()
        self._reset_selection()
        self.scanBtn.setText("Cancel scan")
        self.scanProgress.setValue(0)
        self.deviceCount.setText("Searching the bus…")
        self._show_notice(f"Scanning device IDs {start}–{end}…")
        self._log(f"Scan started: IDs {start}–{end}")
        self._update_enabled()
        self.worker.submit("scan", start, end)

    def _add_servo(self, sid):
        for i in range(self.servoList.count()):
            if self.servoList.item(i).data(Qt.ItemDataRole.UserRole) == sid:
                return self.servoList.item(i)
        item = QListWidgetItem(f"◉   Servo {sid:02d}     /     ID {sid}")
        item.setData(Qt.ItemDataRole.UserRole, sid)
        self.servoList.addItem(item)
        self.deviceCount.setText(f"{self.servoList.count()} device(s) discovered")
        return item

    def _scan_progress(self, value, total):
        self.scanProgress.setValue(round(value / total * 100))

    def _scan_done(self):
        self._scanning = False
        self.scanBtn.setText("Scan for servos")
        count = self.servoList.count()
        self.deviceCount.setText(f"{count} device(s) discovered")
        self._log(f"Scan finished: {count} device(s) discovered")
        self._update_enabled()
        if count and self._connected:
            self.servoList.setCurrentRow(0)
        else:
            self._show_notice("No servos found. Check power, adapter wiring, baud rate, and the ID range.")

    def open_id(self):
        if self._connected and not self._pending and not self._scanning:
            item = self._add_servo(self.idBox.value())
            if item == self.servoList.currentItem():
                self._select_servo(item)
            else:
                self.servoList.setCurrentItem(item)

    def _reset_selection(self):
        self._active_sid = None
        self._settings_ready = False
        self._actual = self._requested = None
        self._last_feedback = 0
        self._draft_dirty = False
        self.activeLabel.setText("Your workspace is ready")
        self.feedbackBadge.setText("AWAITING DEVICE")
        self.posLabel.setText("—")
        self.goalLabel.setText("—")
        self.tempLabel.setText("—")
        self.voltageLabel.setText("—")
        self.healthLabel.setText("—")
        self._clear_profile_readback()
        self.errorLabel.setText("Following error  —")
        self.dial.set_positions(None, None)
        self.history.clear()
        self._update_enabled()

    def _select_servo(self, item, previous=None):
        self._reset_selection()
        if item is None or self._scanning or not self._connected:
            return
        sid = item.data(Qt.ItemDataRole.UserRole)
        self._active_sid = sid
        self.idBox.setValue(sid)
        self.activeLabel.setText(f"Servo {sid:02d}  /  Live workspace")
        self._show_notice(f"Reading servo {sid:02d} settings and feedback…")
        self.worker.submit("select", sid)

    def _on_settings(self, sid, settings):
        if sid != self._active_sid:
            return
        initial = not self._settings_ready
        raw_limits = (settings["minimum"], settings["maximum"])
        self._limits = command_limits(settings)
        self.goalEdit.setRange(*self._limits)
        self.posSlider.setRange(*self._limits)
        self.minAngle.setValue(raw_limits[0])
        self.maxAngle.setValue(raw_limits[1])
        self.speedEdit.setValue(settings["speed"])
        self.accEdit.setValue(settings["acc"])
        self._on_profile_readback(sid, settings)
        self.profileStatus.setText("Loaded from servo · edit values to change the profile")
        self.newId.setValue(sid)
        if not self.ros_service:
            self.newBaud.setCurrentText(self.baudBox.currentText())
        if self._limits == raw_limits:
            self.limitLabel.setText(f"Travel limits  {self._limits[0]}–{self._limits[1]} ticks")
        else:
            self.limitLabel.setText(f"Command range  0–4095  ·  Raw limits {raw_limits[0]}–{raw_limits[1]}")
        self.limitLabel.setWordWrap(True)
        self._settings_ready = True
        if initial:
            self._draft_dirty = False
        self._update_enabled()

    def _on_telemetry(self, sid, data):
        if sid != self._active_sid or self._scanning or not self._connected:
            return
        initial = self._requested is None
        self._actual, self._requested = data["position"], data["goal"]
        self._last_feedback = time.monotonic()
        self._last_error = ""
        self._on_profile_readback(sid, data)
        velocity = data.get("velocity")
        self.liveVelocityLabel.setText(f"Actual velocity  {velocity:+d} ticks/s" if velocity is not None else "Actual velocity  —")
        self.posLabel.setText(str(self._actual))
        self.goalLabel.setText(str(self._requested))
        delta = self._requested - self._actual
        self.errorLabel.setText(f"Following error  {delta:+d} ticks  /  {delta / 4096 * 360:+.1f}°")
        self.dial.set_positions(self._actual, self._requested)
        self.history.add_sample(self._actual, self._requested)
        self.voltageLabel.setText(f"{data['voltage']:.1f} V")
        self.tempLabel.setText(f"{data['temperature']} °C")
        self.healthLabel.setText(decode_status(data["status"]))
        self.healthLabel.setStyleSheet(f"font-size: 18px; color: {'#ffbd69' if data['status'] else '#54ddc2'}")
        valid = 0 <= self._actual <= ANGLE_MAX and 0 <= self._requested <= ANGLE_MAX
        self.feedbackBadge.setText("●  LIVE  ·  " + ("AT TARGET" if abs(delta) <= 8 else "TRACKING"))
        if not valid:
            self._show_notice("Feedback is outside the single-turn range. Check the servo's operating mode.")
        elif data["status"]:
            self._show_notice(f"Servo reports: {decode_status(data['status'])}. Check the device before moving.")
        elif self.notice.text().startswith(("Reading servo", "Feedback lost", "Command sent", "Hold requested", "Servo reports", "Feedback is outside")):
            self.notice.hide()
        if initial and not self._draft_dirty:
            self.goalEdit.setValue(max(self._limits[0], min(self._limits[1], self._requested)))
            self._draft_dirty = False
            self.draftLabel.setText("Target is staged until you send it")
        self._update_enabled()

    def _check_freshness(self):
        if self._active_sid is not None and self._last_feedback and time.monotonic() - self._last_feedback > FEEDBACK_STALE_SECONDS:
            self.feedbackBadge.setText("●  FEEDBACK LOST")
            self.posLabel.setText("—")
            self.goalLabel.setText("—")
            self.tempLabel.setText("—")
            self.voltageLabel.setText("—")
            self.healthLabel.setText("No response")
            self._clear_profile_readback()
            self.errorLabel.setText("Following error  —")
            self.dial.set_positions(None, None)
            self.history.add_sample(None, None)
            self._show_notice("Feedback lost. Motion controls are paused until the selected servo responds.")
        self.history.update()
        self._update_enabled()

    def _ready(self):
        return (self._connected and not self._scanning and self._settings_ready
                and self._active_sid is not None and self._last_feedback > 0
                and time.monotonic() - self._last_feedback <= FEEDBACK_STALE_SECONDS
                and self._actual is not None and 0 <= self._actual <= ANGLE_MAX
                and self._requested is not None and 0 <= self._requested <= ANGLE_MAX)

    def _update_enabled(self):
        ready = self._ready() and not self._pending
        for widget in [self.goalEdit, self.posSlider, self.speedEdit, self.accEdit,
                       self.speedSlider, self.accSlider, self.applyProfileBtn,
                       self.moveBtn, self.matchBtn, self.jogMinus, self.jogPlus,
                       self.jogStep, *self.presetButtons, self.newId, self.newBaud,
                       self.minAngle, self.maxAngle, self.idWriteBtn,
                       self.baudWriteBtn, self.limitsBtn]:
            widget.setEnabled(ready)
        self.holdBtn.setEnabled(ready)
        if hasattr(self, "holdShortcut"):
            self.holdShortcut.setEnabled(ready)
        connection_controls = [self.demoBtn] if self.ros_service else [self.portBox, self.baudBox, self.refreshBtn, self.demoBtn]
        for widget in connection_controls:
            widget.setEnabled(not self._connected and not self._pending)
        for widget in [self.scanFrom, self.scanTo, self.idBox, self.selectBtn, self.servoList]:
            widget.setEnabled(self._connected and not self._scanning and not self._pending)
        self.connectBtn.setEnabled(not self._pending)
        self.scanBtn.setEnabled(self._connected and not self._pending
                                and not (self._scanning and self.worker.cancel_scan))

    def _target_changed(self, value):
        self.posSlider.setValue(value)
        self.degreeLabel.setText(f"{value / 4096 * 360:.1f}°")
        self._draft_dirty = True
        self.draftLabel.setText("Staged · press Move to send")

    def stage_fraction(self, fraction):
        self.goalEdit.setValue(round(self._limits[0] + fraction * (self._limits[1] - self._limits[0])))

    def jog(self, direction):
        self.goalEdit.setValue(self.goalEdit.value() + direction * self.jogStep.value())

    def use_actual(self):
        if self._ready():
            self.goalEdit.setValue(self._actual)

    def _send(self, command, *args):
        if not self._ready() or self._pending:
            return
        self._pending = True
        self._update_enabled()
        self.worker.submit(command, self._active_sid, *args)

    def move_to_target(self):
        if not self._ready() or self._pending:
            return
        self.goalEdit.interpretText()
        self.speedEdit.interpretText()
        self.accEdit.interpretText()
        self._send("move", self.goalEdit.value(), self.speedEdit.value(), self.accEdit.value())
        self._show_notice("Command sent. Waiting for the servo's requested-position readback.")

    def hold_position(self):
        if self._ready() and not self._pending:
            self._send("hold")
            self._show_notice("Hold requested. The servo will be commanded to its measured position.")

    def write_limits(self):
        minimum, maximum = self.minAngle.value(), self.maxAngle.value()
        if not 0 <= minimum < maximum <= ANGLE_MAX:
            self._show_notice("Minimum travel must be less than maximum travel, both within 0–4095. Raw device limits are unchanged until a valid range is saved.")
            return
        self._send("limits", minimum, maximum)

    def write_id(self):
        sid, new_id = self._active_sid, self.newId.value()
        if sid is None or new_id == sid:
            return
        if QMessageBox.question(self, "Change device ID", f"Change servo {sid:02d} to ID {new_id:02d}?",
                                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                QMessageBox.StandardButton.No) == QMessageBox.StandardButton.Yes:
            self._send("id", new_id)

    def write_baud(self):
        baud = self.newBaud.currentText()
        if QMessageBox.question(self, "Change device baud rate",
                                f"Set the selected servo to {int(baud):,} baud? The adapter will disconnect. Reconnect at the new rate to verify the change.",
                                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                QMessageBox.StandardButton.No) == QMessageBox.StandardButton.Yes:
            self._send("baud", baud)

    def _on_id_changed(self, old, new):
        self._active_sid = new
        for i in range(self.servoList.count()):
            item = self.servoList.item(i)
            if item.data(Qt.ItemDataRole.UserRole) == old:
                item.setData(Qt.ItemDataRole.UserRole, new)
                item.setText(f"◉   Servo {new:02d}     /     ID {new}")
        self.idBox.setValue(new)
        self.newId.setValue(new)
        self.activeLabel.setText(f"Servo {new:02d}  /  Live workspace")

    def _on_completed(self, command):
        self._pending = False
        if command == "move":
            self._draft_dirty = False
            self.draftLabel.setText("Move sent · watching feedback")
        elif command == "baud":
            self.baudBox.setCurrentText(self.newBaud.currentText())
        elif command == "profile":
            self.profileStatus.setText("Applied · verified from servo readback")
        self._update_enabled()

    def _on_error(self, context, message):
        if context != "telemetry":
            self._pending = False
        text = f"{context.capitalize()}: {message}"
        if context == "profile":
            self.profileStatus.setText("Not verified · " + message)
        if text != self._last_error:
            self._log(text)
            self._last_error = text
        if context == "select":
            self._settings_ready = False
        elif context == "limits":
            # A two-register write may have partially succeeded. Reopen the
            # device before allowing another move with potentially stale limits.
            self._settings_ready = False
        self._show_notice(text)
        self._update_enabled()

    def _clear_profile_readback(self):
        self.liveVelocityLabel.setText("Actual velocity  —")
        self.speedReadback.setText("Stored velocity limit  —")
        self.accReadback.setText("Stored acceleration  —")

    def closeEvent(self, event):
        self.watchdog.stop()
        self.worker.cancel_scan = True
        self.worker.requestInterruption()
        if self.worker.wait(2000):
            event.accept()
        else:
            self._show_notice("Finishing the current serial operation before closing…")
            QTimer.singleShot(250, self.close)
            event.ignore()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = Window(demo="--demo" in sys.argv)
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
