"""Qt compatibility layer supporting either PySide6 or PyQt5."""
from __future__ import annotations

QT_BINDING = ''
try:
    from PySide6.QtCore import QEvent, QObject, QTimer, Qt
    from PySide6.QtGui import QCloseEvent, QFont, QKeyEvent
    from PySide6.QtWidgets import (
        QAbstractSpinBox, QApplication, QCheckBox, QComboBox, QDoubleSpinBox,
        QFormLayout, QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
        QLineEdit, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton,
        QListWidget, QSizePolicy, QSpacerItem, QTabWidget, QVBoxLayout, QWidget,
    )
    QT_BINDING = 'PySide6'
except ImportError:
    try:
        from PyQt5.QtCore import QEvent, QObject, QTimer, Qt
        from PyQt5.QtGui import QCloseEvent, QFont, QKeyEvent
        from PyQt5.QtWidgets import (
            QAbstractSpinBox, QApplication, QCheckBox, QComboBox, QDoubleSpinBox,
            QFormLayout, QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
            QLineEdit, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton,
            QListWidget, QSizePolicy, QSpacerItem, QTabWidget, QVBoxLayout, QWidget,
        )
        QT_BINDING = 'PyQt5'
    except ImportError as exc:
        raise ImportError('No supported Qt binding was found. Install PySide6 or PyQt5.') from exc


def enum_value(container: object, scoped_name: str, flat_name: str):
    scoped = getattr(container, scoped_name, None)
    if scoped is not None:
        return getattr(scoped, flat_name)
    return getattr(container, flat_name)


def qt_key(name: str):
    return enum_value(Qt, 'Key', name)


def event_type(name: str):
    return enum_value(QEvent, 'Type', name)


def alignment(name: str):
    return enum_value(Qt, 'AlignmentFlag', name)


def focus_policy(name: str):
    return enum_value(Qt, 'FocusPolicy', name)


def app_exec(app: QApplication) -> int:
    runner = getattr(app, 'exec', None)
    return int(runner()) if runner is not None else int(app.exec_())
