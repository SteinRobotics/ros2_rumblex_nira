"""Native, resolution-independent instruments for the servo dashboard."""

import math
import time
from collections import deque
from pathlib import Path

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QSlider, QStyle, QStyleOptionSlider, QWidget

TEAL = "#54ddc2"
AMBER = "#ffbd69"
MUTED = "#8393a7"

STYLESHEET = """
QWidget { background: #0d131c; color: #e8eef6; font-family: 'Inter', 'DejaVu Sans'; font-size: 12px; }
QFrame#card { background: #151e2b; border: 1px solid #273345; border-radius: 12px; }
QFrame#card QLabel, QFrame#card QWidget#transparent { background: transparent; border: none; }
QLabel#eyebrow { color: #8393a7; font-size: 10px; font-weight: 700; letter-spacing: 2px; }
QLabel#title { font-size: 25px; font-weight: 700; }
QLabel#section { font-size: 16px; font-weight: 600; }
QLabel#muted { color: #8393a7; }
QLabel#actual { color: #54ddc2; font-size: 31px; font-weight: 600; }
QLabel#requested { color: #ffbd69; font-size: 31px; font-weight: 600; }
QLabel#metric { font-size: 23px; font-weight: 600; }
QLabel#badge { background: #19332f; color: #54ddc2; border-radius: 6px; padding: 7px 12px; }
QLabel#notice { background: #2e271d; color: #ffbd69; border-radius: 6px; padding: 9px; }
QPushButton { background: #223044; border: 1px solid #35465c; border-radius: 7px; padding: 9px 12px; font-weight: 600; }
QPushButton:hover { background: #2c3e55; border-color: #64809b; }
QPushButton:pressed { background: #172332; }
QPushButton:disabled { background: #192230; color: #536174; border-color: #263243; }
QPushButton#primary { background: #54ddc2; color: #0c2925; border-color: #54ddc2; }
QPushButton#primary:hover { background: #87ebd7; }
QPushButton#primary:disabled { background: #234640; color: #62827c; border-color: #234640; }
QPushButton#hold { background: #382a22; color: #ffbd69; border-color: #6d4a2e; }
QPushButton#hold:disabled { background: #201e1f; color: #706457; border-color: #38302b; }
QSpinBox, QComboBox { background: #101824; border: 1px solid #344358; border-radius: 6px; padding: 7px 9px; min-height: 19px; selection-background-color: #28574f; }
QSpinBox:focus, QComboBox:focus { border-color: #54ddc2; }
QSpinBox:disabled, QComboBox:disabled { color: #59687c; border-color: #273345; }
QSpinBox::up-button { subcontrol-origin: border; subcontrol-position: top right; width: 20px; border: none; background: transparent; }
QSpinBox::down-button { subcontrol-origin: border; subcontrol-position: bottom right; width: 20px; border: none; background: transparent; }
QSpinBox::up-arrow { image: url(__ASSETS__/chevron-up.svg); width: 10px; height: 6px; }
QSpinBox::down-arrow, QComboBox::down-arrow { image: url(__ASSETS__/chevron-down.svg); width: 10px; height: 6px; }
QComboBox::drop-down { border: none; width: 24px; }
QComboBox QAbstractItemView { background: #192536; selection-background-color: #28574f; padding: 5px; }
QListWidget { background: transparent; border: none; outline: none; }
QListWidget::item { border: 1px solid #2a384b; border-radius: 8px; padding: 16px 12px; margin: 4px 0px; color: #a9b8ca; }
QListWidget::item:selected { background: #1b3735; border-color: #438d80; color: #7aefd5; }
QListWidget::item:hover { background: #203044; }
QSlider { background: transparent; min-height: 22px; }
QSlider::groove:horizontal { height: 5px; background: #2a3749; border-radius: 2px; }
QSlider::sub-page:horizontal { background: #54ddc2; border-radius: 2px; }
QSlider::handle:horizontal { background: #d9fff7; border: 3px solid #54ddc2; width: 12px; height: 12px; margin: -6px 0; border-radius: 8px; }
QSlider:disabled::sub-page:horizontal { background: #42505d; }
QSlider:disabled::handle:horizontal { background: #65707d; border-color: #42505d; }
QTabWidget::pane { border: none; padding-top: 12px; }
QTabBar::tab { background: transparent; color: #8393a7; padding: 12px 20px; border-bottom: 2px solid #273345; }
QTabBar::tab:selected { color: #54ddc2; border-bottom-color: #54ddc2; }
QTabBar::tab:hover { color: #e8eef6; }
QPlainTextEdit { background: #101824; border: 1px solid #273345; border-radius: 8px; padding: 8px; font-family: 'DejaVu Sans Mono'; font-size: 11px; color: #a5b8cb; }
QProgressBar { background: #101824; border: none; border-radius: 3px; height: 5px; color: #8393a7; }
QProgressBar::chunk { background: #54ddc2; border-radius: 3px; }
QScrollArea { border: none; }
QScrollBar:vertical { background: #101824; width: 8px; }
QScrollBar::handle:vertical { background: #344358; border-radius: 4px; min-height: 25px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }
QToolTip { background: #223044; color: #e8eef6; border: 1px solid #465b73; padding: 6px; }
""".replace("__ASSETS__", (Path(__file__).parent / "assets").as_posix())


class PositionSlider(QSlider):
    """Keep Qt's keyboard/mouse behavior with a consistently painted track."""
    def __init__(self):
        super().__init__(Qt.Orientation.Horizontal)

    def paintEvent(self, event):
        option = QStyleOptionSlider()
        self.initStyleOption(option)
        handle = self.style().subControlRect(QStyle.ComplexControl.CC_Slider, option,
                                            QStyle.SubControl.SC_SliderHandle, self)
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        left, right = handle.width() / 2, self.width() - handle.width() / 2
        center = QPointF(handle.center().x(), self.height() / 2)
        p.setPen(QPen(QColor("#2a3749"), 5, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        p.drawLine(QPointF(left, center.y()), QPointF(right, center.y()))
        color = QColor(TEAL if self.isEnabled() else "#42505d")
        p.setPen(QPen(color, 5, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        p.drawLine(QPointF(left, center.y()), center)
        p.setPen(QPen(color, 3))
        p.setBrush(QColor("#d9fff7" if self.isEnabled() else "#65707d"))
        p.drawEllipse(center, 7, 7)
        if self.hasFocus():
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setPen(QPen(QColor(TEAL), 1, Qt.PenStyle.DotLine))
            p.drawRoundedRect(QRectF(self.rect()).adjusted(1, 1, -1, -1), 5, 5)
        p.end()


class PositionDial(QWidget):
    def __init__(self):
        super().__init__()
        self.actual = self.goal = None
        self.setMinimumSize(225, 210)
        self.setAccessibleName("Servo position dial: teal actual position and amber requested position")

    def set_positions(self, actual, goal):
        self.actual, self.goal = actual, goal
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        size = min(self.width(), self.height())
        p.translate(self.width() / 2, self.height() / 2)
        p.scale(size / 290, size / 290)
        p.setPen(QPen(QColor("#253346"), 13))
        p.drawArc(QRectF(-104, -104, 208, 208), -45 * 16, 270 * 16)
        for i in range(49):
            angle = math.radians(135 + i * 270 / 48)
            radius = 120 if i % 4 == 0 else 124
            p.setPen(QPen(QColor("#63758b" if i % 4 == 0 else "#34465c"), 1))
            p.drawLine(QPointF(math.cos(angle) * radius, math.sin(angle) * radius),
                       QPointF(math.cos(angle) * 130, math.sin(angle) * 130))
        for value, color, radius, width in [(self.goal, AMBER, 104, 3),
                                           (self.actual, TEAL, 88, 5)]:
            if value is None or not 0 <= value <= 4095:
                continue
            angle = math.radians(135 + value / 4095 * 270)
            if color == TEAL:
                p.setPen(QPen(QColor(TEAL), 6, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
                p.drawArc(QRectF(-104, -104, 208, 208), 225 * 16, -round(value / 4095 * 270 * 16))
            p.setPen(QPen(QColor(color), width, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            p.drawLine(QPointF(math.cos(angle) * 70, math.sin(angle) * 70),
                       QPointF(math.cos(angle) * radius, math.sin(angle) * radius))
            p.setBrush(QColor(color))
            p.drawEllipse(QPointF(math.cos(angle) * radius, math.sin(angle) * radius), 4, 4)
        p.setPen(QColor(TEAL if self.actual is not None else MUTED))
        p.setFont(QFont("DejaVu Sans", 29, QFont.Weight.DemiBold))
        value = f"{self.actual / 4096 * 360:.1f}°" if self.actual is not None else "—"
        p.drawText(QRectF(-90, -26, 180, 50), Qt.AlignmentFlag.AlignCenter, value)
        p.setPen(QColor(MUTED))
        p.setFont(QFont("DejaVu Sans", 9))
        p.drawText(QRectF(-80, 23, 160, 25), Qt.AlignmentFlag.AlignCenter, "ACTUAL POSITION")
        p.drawText(QRectF(-115, 94, 65, 23), Qt.AlignmentFlag.AlignCenter, "0")
        p.drawText(QRectF(50, 94, 65, 23), Qt.AlignmentFlag.AlignCenter, "4095")
        p.drawText(QRectF(-60, 106, 120, 22), Qt.AlignmentFlag.AlignCenter, "ENCODER TICKS")
        p.end()


class PositionHistory(QWidget):
    def __init__(self):
        super().__init__()
        self.samples = deque(maxlen=300)
        self.setMinimumHeight(110)
        self.setAccessibleName("Position history over the last 30 seconds")

    def clear(self):
        self.samples.clear()
        self.update()

    def add_sample(self, actual, goal):
        self.samples.append((time.monotonic(), actual, goal))
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        plot = QRectF(43, 10, max(1, self.width() - 55), max(1, self.height() - 34))
        p.setFont(QFont("DejaVu Sans", 8))
        for i, label in enumerate(["4095", "2048", "0"]):
            y = plot.top() + plot.height() * i / 2
            p.setPen(QPen(QColor("#29374a"), 1, Qt.PenStyle.DotLine))
            p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))
            p.setPen(QColor(MUTED))
            p.drawText(QRectF(0, y - 9, 35, 18), Qt.AlignmentFlag.AlignRight, label)
        for i, label in enumerate(["−30s", "−20s", "−10s", "now"]):
            x = plot.left() + plot.width() * i / 3
            p.drawText(QRectF(x - 20, plot.bottom() + 6, 40, 18), Qt.AlignmentFlag.AlignCenter, label)
        if not self.samples:
            p.drawText(plot, Qt.AlignmentFlag.AlignCenter, "Position history appears when a servo is selected")
        now = time.monotonic()
        p.setClipRect(plot.adjusted(-2, -2, 2, 2))
        for index, color in [(2, AMBER), (1, TEAL)]:
            path = QPainterPath()
            previous = None
            for stamp, actual, goal in self.samples:
                value = (stamp, actual, goal)[index]
                if stamp < now - 30 or value is None:
                    previous = None
                    continue
                x = plot.right() - (now - stamp) / 30 * plot.width()
                y = plot.bottom() - max(0, min(4095, value)) / 4095 * plot.height()
                if previous is None or stamp - previous > 1:
                    path.moveTo(x, y)
                else:
                    path.lineTo(x, y)
                previous = stamp
            pen = QPen(QColor(color), 2)
            if index == 2:
                pen.setStyle(Qt.PenStyle.DashLine)
            p.setPen(pen)
            p.drawPath(path)
        p.end()
