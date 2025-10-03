"""Simulation configuration module for FluxhWeave Workbench."""

from __future__ import annotations

import json
import math
import os
import re
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple
import copy

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QDoubleValidator, QIntValidator, QTextOption
from PySide6.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QScrollArea,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)


ACTUATOR_TYPES = [
    ("ImplicitActuator", "隐式作动器"),
    ("IdealPDActuator", "理想 PD 作动器"),
    ("DCMotor", "直流电机"),
    ("DelayedPDActuator", "延迟 PD 作动器"),
    ("RemotizedPDActuator", "远程 PD 作动器"),
]

ALL_ACTUATOR_TYPES = {key for key, _ in ACTUATOR_TYPES}

ACTUATOR_PARAM_SPECS = [
    (
        "effort_limit_sim",
        {
            "label": "仿真力矩上限 / effort_limit_sim (N·m)",
            "types": ALL_ACTUATOR_TYPES,
            "widget": "line",
            "parser": "float",
            "placeholder": "留空使用 USD 默认",
        },
    ),
    (
        "velocity_limit_sim",
        {
            "label": "仿真角速度上限 / velocity_limit_sim (rad/s)",
            "types": ALL_ACTUATOR_TYPES,
            "widget": "line",
            "parser": "float",
            "placeholder": "留空使用 USD 默认",
        },
    ),
    (
        "stiffness",
        {
            "label": "位置刚度 / stiffness (N·m·rad⁻¹)",
            "types": ALL_ACTUATOR_TYPES,
            "widget": "line",
            "parser": "float",
        },
    ),
    (
        "damping",
        {
            "label": "阻尼系数 / damping (N·m·s·rad⁻¹)",
            "types": ALL_ACTUATOR_TYPES,
            "widget": "line",
            "parser": "float",
        },
    ),
    (
        "armature",
        {
            "label": "等效惯量 / armature (kg·m²)",
            "types": ALL_ACTUATOR_TYPES,
            "widget": "line",
            "parser": "float",
        },
    ),
    (
        "friction",
        {
            "label": "静摩擦系数 / friction (-)",
            "types": ALL_ACTUATOR_TYPES,
            "widget": "line",
            "parser": "float",
        },
    ),
    (
        "dynamic_friction",
        {
            "label": "动摩擦系数 / dynamic_friction (-)",
            "types": ALL_ACTUATOR_TYPES,
            "widget": "line",
            "parser": "float",
        },
    ),
    (
        "viscous_friction",
        {
            "label": "粘性摩擦 / viscous_friction (N·m·s·rad⁻¹)",
            "types": ALL_ACTUATOR_TYPES,
            "widget": "line",
            "parser": "float",
        },
    ),
    (
        "saturation_effort",
        {
            "label": "饱和扭矩 / saturation_effort (N·m)",
            "types": {"DCMotor"},
            "widget": "line",
            "parser": "float",
            "placeholder": "仅直流电机需要",
            "clear_on_hide": True,
        },
    ),
    (
        "min_delay",
        {
            "label": "最小延迟步数 / min_delay (steps)",
            "types": {"DelayedPDActuator", "RemotizedPDActuator"},
            "widget": "line",
            "parser": "int",
            "placeholder": "整数，单位：仿真步",
            "clear_on_hide": True,
        },
    ),
    (
        "max_delay",
        {
            "label": "最大延迟步数 / max_delay (steps)",
            "types": {"DelayedPDActuator", "RemotizedPDActuator"},
            "widget": "line",
            "parser": "int",
            "placeholder": "整数，单位：仿真步",
            "clear_on_hide": True,
        },
    ),
    (
        "joint_parameter_lookup",
        {
            "label": "关节查找表 / joint_parameter_lookup (JSON)",
            "types": {"RemotizedPDActuator"},
            "widget": "text",
            "parser": "json_list",
            "placeholder": "格式: [[角度rad, 传动比, 输出扭矩], ...]",
            "clear_on_hide": True,
        },
    ),
    (
        "extra_params",
        {
            "label": "额外参数 / extra_params (JSON，可选)",
            "types": ALL_ACTUATOR_TYPES,
            "widget": "text",
            "parser": "json_object",
            "placeholder": "例如 {\"custom\": 1.0}",
            "merge": True,
        },
    ),
]


def _sanitize_actuator_name(name: str) -> str:
    token = re.sub(r"[^0-9A-Za-z_]+", "_", name.strip())
    token = re.sub(r"_+", "_", token).strip("_")
    return token or "actuator"


def _default_actuator_base_name(actuator_type: Optional[str], joint_names: List[str]) -> str:
    if not joint_names:
        base = "actuator"
    else:
        cleaned = [re.sub(r"[^0-9A-Za-z_]+", "_", name).strip("_") or "joint" for name in joint_names[:2]]
        base = "_".join(cleaned)
        if len(joint_names) > 2:
            base = f"{base}_grp"
    if actuator_type:
        return f"{base}_{actuator_type}"
    return base


USD_JOINT_TYPES = {
    "PhysicsRevoluteJoint",
    "PhysicsPrismaticJoint",
    "PhysicsSphericalJoint",
    "PhysicsFixedJoint",
    "PhysicsDistanceJoint",
}

JOINT_TYPE_LABELS = {
    "PhysicsRevoluteJoint": "转动关节",
    "PhysicsPrismaticJoint": "移动关节",
    "PhysicsSphericalJoint": "球铰",
    "PhysicsFixedJoint": "固定关节",
    "PhysicsDistanceJoint": "距离关节",
}

JOINT_TYPES_WITH_DOF = {
    "PhysicsRevoluteJoint",
    "PhysicsPrismaticJoint",
    "PhysicsSphericalJoint",
    "PhysicsDistanceJoint",
}


class NumericLineEdit(QLineEdit):
    """Line edit that behaves like a spin box but without step buttons."""

    valueChanged = Signal(float)

    def __init__(
        self,
        minimum: float,
        maximum: float,
        value: float,
        *,
        decimals: int = 3,
        allow_float: bool = True,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self._allow_float = bool(allow_float)
        self._decimals = int(decimals)
        self._block_signal = False

        if self._allow_float:
            validator = QDoubleValidator(self)
            validator.setNotation(QDoubleValidator.StandardNotation)
            validator.setRange(float(minimum), float(maximum), self._decimals)
        else:
            validator = QIntValidator(self)
            validator.setRange(int(minimum), int(maximum))
        self.setValidator(validator)
        self.setAlignment(Qt.AlignRight)
        self.setValue(value)

        self.textChanged.connect(self._on_text_changed)

    def setRange(self, minimum: float, maximum: float) -> None:
        if self._allow_float:
            validator: QDoubleValidator = self.validator()  # type: ignore[assignment]
            validator.setRange(float(minimum), float(maximum), self._decimals)
        else:
            validator: QIntValidator = self.validator()  # type: ignore[assignment]
            validator.setRange(int(minimum), int(maximum))

    def setDecimals(self, decimals: int) -> None:
        if not self._allow_float:
            return
        self._decimals = int(decimals)
        validator: QDoubleValidator = self.validator()  # type: ignore[assignment]
        validator.setRange(validator.bottom(), validator.top(), self._decimals)
        self.setValue(self.value())

    def setSingleStep(self, _step: float) -> None:  # noqa: D401 - compatibility hook
        """Compatibility placeholder used to align with spin box API."""

    def value(self) -> float:
        text = self.text().strip()
        if not text:
            return 0 if not self._allow_float else 0.0
        try:
            if self._allow_float:
                return float(text)
            return int(round(float(text)))
        except ValueError:
            return 0.0

    def intValue(self) -> int:
        return int(round(self.value()))

    def setValue(self, value: float) -> None:
        if value is None:
            text = ""
        else:
            if self._allow_float:
                number = float(value)
                validator: QDoubleValidator = self.validator()  # type: ignore[assignment]
                bottom, top = validator.bottom(), validator.top()
                if bottom != -float("inf"):
                    number = max(number, bottom)
                if top != float("inf"):
                    number = min(number, top)
                text = f"{number:.{self._decimals}f}"
                if "." in text:
                    text = text.rstrip("0").rstrip(".")
            else:
                number = int(round(float(value)))
                validator: QIntValidator = self.validator()  # type: ignore[assignment]
                number = max(min(number, validator.top()), validator.bottom())
                text = str(number)

        self._block_signal = True
        self.setText(text)
        self._block_signal = False

    def _on_text_changed(self, _text: str) -> None:
        if self._block_signal or self.signalsBlocked():
            return
        try:
            value = self.value()
        except ValueError:
            return
        self.valueChanged.emit(float(value))

    def wheelEvent(self, event) -> None:  # noqa: D401
        """Disable wheel-based adjustments to avoid accidental changes."""

        event.ignore()


class JointMappingDialog(QDialog):
    """Dialog for establishing URDF⇄USD joint name correspondence."""

    def __init__(
        self,
        urdf_joint_names: List[str],
        usd_joint_names: List[str],
        existing: Optional[Dict[str, str]] = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("设置关节名称对应")
        self.resize(520, 560)

        self._urdf_names = list(urdf_joint_names)
        self._usd_names = list(usd_joint_names)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        info = QLabel("为每个 URDF 关节选择对应的 USD 关节名称，可手动调整或自动匹配。", self)
        info.setWordWrap(True)
        layout.addWidget(info)

        toolbar = QHBoxLayout()
        toolbar.addStretch(1)
        auto_button = QPushButton("自动匹配同名", self)
        auto_button.clicked.connect(self._auto_match)
        toolbar.addWidget(auto_button)
        layout.addLayout(toolbar)

        self._combo_container = QWidget(self)
        self._combo_form = QFormLayout(self._combo_container)
        self._combo_form.setLabelAlignment(Qt.AlignRight)
        self._combo_boxes: Dict[str, QComboBox] = {}

        for urdf_name in self._urdf_names:
            combo = QComboBox(self._combo_container)
            combo.setEditable(False)
            combo.addItem("（未映射）", "")
            for usd_name in self._usd_names:
                combo.addItem(usd_name, usd_name)
            self._combo_boxes[urdf_name] = combo
            self._combo_form.addRow(QLabel(urdf_name, self._combo_container), combo)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setWidget(self._combo_container)
        layout.addWidget(scroll, 1)

        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, self)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

        self._apply_existing_mapping(existing)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _apply_existing_mapping(self, existing: Optional[Dict[str, str]]) -> None:
        if not existing:
            self._auto_match()
            return
        for urdf_name, combo in self._combo_boxes.items():
            target = existing.get(urdf_name)
            if not target:
                continue
            index = combo.findText(target)
            if index >= 0:
                combo.setCurrentIndex(index)
            else:
                combo.setCurrentIndex(0)

    def _auto_match(self) -> None:
        usd_name_set = set(self._usd_names)
        for urdf_name, combo in self._combo_boxes.items():
            if urdf_name in usd_name_set:
                index = combo.findText(urdf_name)
                if index >= 0:
                    combo.setCurrentIndex(index)
                continue
            combo.setCurrentIndex(0)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def mapping(self) -> Dict[str, str]:
        result: Dict[str, str] = {}
        for urdf_name, combo in self._combo_boxes.items():
            usd_name = combo.currentData()
            if usd_name:
                result[urdf_name] = str(usd_name)
        return result
def _try_import_usd():
    try:
        from pxr import Usd  # type: ignore
    except ImportError:
        return None
    return Usd


def _parse_float(text: str) -> Optional[float]:
    text = text.strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        raise ValueError(f"无法解析数值: {text}")


def _parse_int(text: str) -> Optional[int]:
    text = text.strip()
    if not text:
        return None
    try:
        return int(text)
    except ValueError:
        raise ValueError(f"无法解析整数: {text}")


def _format_float(value: float) -> str:
    if value is None:
        return ""
    if math.isfinite(value):
        return f"{value:.6f}".rstrip("0").rstrip(".")
    return str(value)


class ActuatorConfigDialog(QDialog):
    """Dialog used to configure an actuator without manual naming."""

    def __init__(self, joint_names: List[str], existing: Optional[Dict] = None, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setWindowTitle("配置作动器")
        self.setMinimumWidth(520)

        self._result: Optional[Dict] = None
        self._joint_names = joint_names
        self._spec_map = {key: spec for key, spec in ACTUATOR_PARAM_SPECS}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignRight)

        self.type_combo = QComboBox(self)
        for key, label in ACTUATOR_TYPES:
            self.type_combo.addItem(label, key)
        form.addRow("作动器类型", self.type_combo)

        self.joint_list = QListWidget(self)
        self.joint_list.setSelectionMode(QAbstractItemView.MultiSelection)
        self.joint_list.setMinimumHeight(120)
        for joint in joint_names:
            self.joint_list.addItem(QListWidgetItem(joint))
        form.addRow("关联关节", self.joint_list)

        self.auto_name_label = QLabel("选择关节后自动生成", self)
        self.auto_name_label.setStyleSheet("color: #9aa;")
        form.addRow("自动名称", self.auto_name_label)

        self.param_widgets: Dict[str, QWidget] = {}
        self.param_labels: Dict[str, QLabel] = {}
        self._create_param_rows(form)

        layout.addLayout(form)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, Qt.Horizontal, self)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        self.type_combo.currentIndexChanged.connect(self._on_type_changed)
        self.joint_list.itemSelectionChanged.connect(self._update_auto_name_preview)

        if existing:
            self._load_existing(existing)
        else:
            self._on_type_changed()
            self._update_auto_name_preview()

    def _create_param_rows(self, form: QFormLayout) -> None:
        for key, spec in ACTUATOR_PARAM_SPECS:
            label = QLabel(spec["label"], self)
            if spec["widget"] == "line":
                widget = QLineEdit(self)
                if spec.get("placeholder"):
                    widget.setPlaceholderText(spec["placeholder"])
            else:
                widget = QPlainTextEdit(self)
                if spec.get("placeholder"):
                    widget.setPlaceholderText(spec["placeholder"])
                widget.setMinimumHeight(72 if key == "extra_params" else 64)
            self.param_labels[key] = label
            self.param_widgets[key] = widget
            form.addRow(label, widget)

    def _on_type_changed(self) -> None:
        self._update_param_visibility()
        self._update_auto_name_preview()

    def _update_param_visibility(self) -> None:
        type_key = self.type_combo.currentData()
        for key, spec in self._spec_map.items():
            applies = self._spec_applies(spec, type_key)
            label = self.param_labels[key]
            widget = self.param_widgets[key]
            label.setVisible(applies)
            widget.setVisible(applies)
            if not applies and spec.get("clear_on_hide"):
                if isinstance(widget, QLineEdit):
                    widget.clear()
                elif isinstance(widget, QPlainTextEdit):
                    widget.clear()

    def _spec_applies(self, spec: Dict, type_key: Optional[str]) -> bool:
        target_types = spec.get("types")
        if not target_types:
            return True
        if type_key is None:
            return False
        return type_key in target_types

    def _update_auto_name_preview(self) -> None:
        type_key = self.type_combo.currentData()
        joint_names = [item.text() for item in self.joint_list.selectedItems()]
        if not joint_names:
            self.auto_name_label.setText("选择关节后自动生成")
            self.auto_name_label.setStyleSheet("color: #9aa;")
            return
        base_name = _default_actuator_base_name(type_key, joint_names)
        preview = _sanitize_actuator_name(base_name)
        self.auto_name_label.setText(preview)
        self.auto_name_label.setStyleSheet("color: #6cb36c;")

    def _load_existing(self, existing: Dict) -> None:
        type_key = existing.get("type")
        idx = self.type_combo.findData(type_key)
        if idx >= 0:
            self.type_combo.setCurrentIndex(idx)
        joint_names = set(existing.get("joint_names", []))
        for row in range(self.joint_list.count()):
            item = self.joint_list.item(row)
            item.setSelected(item.text() in joint_names)

        params = dict(existing.get("parameters", {}))
        for key, spec in self._spec_map.items():
            widget = self.param_widgets[key]
            if spec.get("merge"):
                continue
            value = params.pop(key, None)
            self._set_widget_value(widget, spec, value)

        if "extra_params" in self.param_widgets:
            widget = self.param_widgets["extra_params"]
            if params:
                widget.setPlainText(json.dumps(params, ensure_ascii=False, indent=2))
            else:
                widget.clear()

        self._on_type_changed()

    @staticmethod
    def _set_widget_value(widget: QWidget, spec: Dict, value: Optional[object]) -> None:
        if isinstance(widget, QLineEdit):
            if value is None:
                widget.clear()
            else:
                if spec.get("parser") == "int":
                    widget.setText(str(int(value)))
                else:
                    widget.setText(_format_float(float(value)))
        elif isinstance(widget, QPlainTextEdit):
            if value is None:
                widget.clear()
            else:
                widget.setPlainText(json.dumps(value, ensure_ascii=False, indent=2))

    def accept(self) -> None:  # type: ignore[override]
        selected = [item.text() for item in self.joint_list.selectedItems()]
        if not selected:
            QMessageBox.warning(self, "缺少关联关节", "请至少选择一个关节。")
            return
        try:
            config = self._collect_config(selected)
        except ValueError as exc:
            QMessageBox.warning(self, "参数错误", str(exc))
            return
        self._result = config
        super().accept()

    def _collect_config(self, joint_names: List[str]) -> Dict:
        type_key = self.type_combo.currentData()
        base_name = _default_actuator_base_name(type_key, joint_names)
        params: Dict[str, object] = {}

        for key, spec in self._spec_map.items():
            widget = self.param_widgets[key]
            if spec.get("merge"):
                text = widget.toPlainText().strip()
                if text:
                    try:
                        value = json.loads(text)
                    except json.JSONDecodeError as exc:
                        raise ValueError(f"{spec['label']} JSON 无效: {exc}") from exc
                    if not isinstance(value, dict):
                        raise ValueError(f"{spec['label']} 需要是 JSON 对象。")
                    params.update(value)
                continue

            if not self._spec_applies(spec, type_key):
                continue

            if isinstance(widget, QLineEdit):
                text = widget.text().strip()
                if not text:
                    continue
                parser = spec.get("parser")
                if parser == "int":
                    value = _parse_int(text)
                    if value is None:
                        continue
                    params[key] = value
                else:
                    params[key] = _parse_float(text)
            elif isinstance(widget, QPlainTextEdit):
                text = widget.toPlainText().strip()
                if not text:
                    continue
                parser = spec.get("parser")
                try:
                    value = json.loads(text)
                except json.JSONDecodeError as exc:
                    raise ValueError(f"{spec['label']} JSON 无效: {exc}") from exc
                if parser == "json_list":
                    if not isinstance(value, list):
                        raise ValueError(f"{spec['label']} 需要是列表。")
                    params[key] = value
                else:
                    params[key] = value

        return {
            "name": _sanitize_actuator_name(base_name),
            "type": type_key,
            "joint_names": joint_names,
            "parameters": params,
        }

    def result_config(self) -> Optional[Dict]:
        return self._result


class SimulationConfigWidget(QWidget):
    """Widget responsible for configuring IsaacLab simulation parameters."""

    def __init__(self, status_callback: Callable[[str], None], parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._status_callback = status_callback

        self.joint_infos: List[Dict[str, object]] = []
        self.dof_joint_infos: List[Dict[str, object]] = []
        self.actuator_configs: List[Dict] = []
        self.joint_spinboxes: Dict[str, NumericLineEdit] = {}
        self.joint_defaults: Dict[str, float] = {}
        self.joint_defaults_path: Optional[str] = None
        self.urdf_joint_names: List[str] = []
        self.joint_name_mapping: Dict[str, str] = {}
        self.joint_limits: Dict[str, Tuple[Optional[float], Optional[float]]] = {}

        self._build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(10)

        title_label = QLabel("⑤ 仿真配置", self)
        title_label.setStyleSheet("font-size: 18px; font-weight: 600;")
        main_layout.addWidget(title_label)

        usd_row = QHBoxLayout()
        usd_row.addWidget(QLabel("USD 文件路径", self))
        self.usd_path_edit = QLineEdit(self)
        self.usd_path_edit.setPlaceholderText("选择机器人 USD 文件，例如 /path/to/robot.usd")
        usd_row.addWidget(self.usd_path_edit, 1)
        browse_button = QPushButton("浏览…", self)
        browse_button.clicked.connect(self._choose_usd_path)
        usd_row.addWidget(browse_button)
        load_button = QPushButton("解析关节", self)
        load_button.clicked.connect(self._load_usd_joints)
        usd_row.addWidget(load_button)
        main_layout.addLayout(usd_row)

        button_row = QHBoxLayout()
        self.load_json_button = QPushButton("加载配置 JSON", self)
        self.load_json_button.clicked.connect(self._on_load_json)
        button_row.addWidget(self.load_json_button)

        self.save_json_button = QPushButton("保存配置 JSON", self)
        self.save_json_button.clicked.connect(self._on_save_json)
        button_row.addWidget(self.save_json_button)

        button_row.addStretch(1)
        main_layout.addLayout(button_row)

        self.tab_widget = QTabWidget(self)
        main_layout.addWidget(self.tab_widget, 1)

        self._build_parameters_tab()
        self._build_modes_tab()

        self.log_output = QPlainTextEdit(self)
        self.log_output.setReadOnly(True)
        self.log_output.setMinimumHeight(120)
        self.log_output.setLineWrapMode(QPlainTextEdit.WidgetWidth)
        self.log_output.setWordWrapMode(QTextOption.WrapAtWordBoundaryOrAnywhere)
        main_layout.addWidget(self.log_output)

    def _build_parameters_tab(self) -> None:
        container = QWidget(self)
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        scroll_area = QScrollArea(container)
        scroll_area.setWidgetResizable(True)
        inner = QWidget(scroll_area)
        inner_layout = QVBoxLayout(inner)
        inner_layout.setContentsMargins(12, 12, 12, 12)
        inner_layout.setSpacing(12)

        inner_layout.addWidget(self._build_system_group())
        inner_layout.addWidget(self._build_rigid_body_group())
        inner_layout.addWidget(self._build_articulation_group())
        inner_layout.addWidget(self._build_initial_state_group())
        inner_layout.addWidget(self._build_actuator_group())
        inner_layout.addWidget(self._build_ground_group())
        inner_layout.addWidget(self._build_lighting_group())
        inner_layout.addWidget(self._build_imu_group())

        inner_layout.addStretch(1)

        scroll_area.setWidget(inner)
        layout.addWidget(scroll_area)
        self.tab_widget.addTab(container, "参数设置")

    def _build_modes_tab(self) -> None:
        container = QWidget(self)
        layout = QVBoxLayout(container)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(12)

        layout.addWidget(self._build_manual_group())
        layout.addWidget(self._build_template_group())
        layout.addStretch(1)

        self.tab_widget.addTab(container, "模式选择")

    def _build_system_group(self) -> QGroupBox:
        group = QGroupBox("系统参数", self)
        form = QFormLayout(group)

        self.description_edit = QLineEdit(group)
        self.description_edit.setPlaceholderText("仿真简介，例如 ChiwuC2 手动演示")
        form.addRow("描述", self.description_edit)

        self.num_envs_spin = self._make_int_spin(group, 1, 10000, 5)
        form.addRow("环境数量", self.num_envs_spin)

        self.control_freq_spin = self._make_double_spin(group, 0.1, 1000.0, 30.0, decimals=2)
        form.addRow("控制频率 (Hz)", self.control_freq_spin)

        return group

    def _build_rigid_body_group(self) -> QGroupBox:
        group = QGroupBox("刚体属性 (RigidBodyPropertiesCfg)", self)
        form = QFormLayout(group)

        self.disable_gravity_check = QCheckBox("禁用重力", group)
        self.disable_gravity_check.setChecked(False)
        form.addRow(self.disable_gravity_check)

        self.max_linear_spin = self._make_double_spin(group, 0.0, 10000.0, 1000.0)
        form.addRow("最大线速度", self.max_linear_spin)

        self.max_angular_spin = self._make_double_spin(group, 0.0, 10000.0, 1000.0)
        form.addRow("最大角速度", self.max_angular_spin)

        self.max_depenetration_spin = self._make_double_spin(group, 0.0, 1000.0, 1.0)
        form.addRow("最大分离速度", self.max_depenetration_spin)

        self.enable_gyro_check = QCheckBox("启用陀螺力", group)
        self.enable_gyro_check.setChecked(True)
        form.addRow(self.enable_gyro_check)

        return group

    def _build_articulation_group(self) -> QGroupBox:
        group = QGroupBox("根体属性 (ArticulationRootPropertiesCfg)", self)
        form = QFormLayout(group)

        self.self_collision_check = QCheckBox("启用自碰撞", group)
        self.self_collision_check.setChecked(False)
        form.addRow(self.self_collision_check)

        self.solver_pos_spin = self._make_int_spin(group, 0, 1024, 4)
        form.addRow("位置迭代次数", self.solver_pos_spin)

        self.solver_vel_spin = self._make_int_spin(group, 0, 1024, 0)
        form.addRow("速度迭代次数", self.solver_vel_spin)

        self.sleep_threshold_spin = self._make_double_spin(group, 0.0, 10.0, 0.005, decimals=6)
        form.addRow("休眠阈值", self.sleep_threshold_spin)

        self.stabilization_spin = self._make_double_spin(group, 0.0, 10.0, 0.001, decimals=6)
        form.addRow("稳定阈值", self.stabilization_spin)

        return group

    def _build_initial_state_group(self) -> QGroupBox:
        group = QGroupBox("初始状态 (InitialStateCfg)", self)
        layout = QVBoxLayout(group)

        pos_layout = QHBoxLayout()
        pos_layout.addWidget(QLabel("初始位置 (XYZ)", group))
        self.init_pos_spins = [self._make_double_spin(group, -100.0, 100.0, value) for value in (0.0, 0.0, 0.7)]
        for spin in self.init_pos_spins:
            pos_layout.addWidget(spin)
        layout.addLayout(pos_layout)

        rot_layout = QHBoxLayout()
        rot_layout.addWidget(QLabel("初始朝向 (四元数 wxyz)", group))
        self.init_rot_spins = [self._make_double_spin(group, -1.0, 1.0, value, decimals=4) for value in (1.0, 0.0, 0.0, 0.0)]
        for spin in self.init_rot_spins:
            rot_layout.addWidget(spin)
        layout.addLayout(rot_layout)

        self.joint_section = QGroupBox("关节初始角度", group)
        joint_layout = QVBoxLayout(self.joint_section)
        joint_layout.setContentsMargins(8, 8, 8, 8)
        joint_layout.setSpacing(6)

        control_row = QHBoxLayout()
        self.load_defaults_button = QPushButton("导入 URDF 默认角度…", self.joint_section)
        self.load_defaults_button.clicked.connect(self._choose_joint_defaults_file)
        control_row.addWidget(self.load_defaults_button)

        self.mapping_button = QPushButton("设置关节对应…", self.joint_section)
        self.mapping_button.clicked.connect(self._open_joint_mapping_dialog)
        self.mapping_button.setEnabled(False)
        control_row.addWidget(self.mapping_button)

        self.joint_mapping_label = QLabel("未建立对应", self.joint_section)
        self.joint_mapping_label.setStyleSheet("color: #9aa; font-size: 12px;")
        control_row.addWidget(self.joint_mapping_label)
        control_row.addStretch(1)
        joint_layout.addLayout(control_row)

        self.joint_form = QFormLayout()
        self.joint_form.setContentsMargins(0, 0, 0, 0)
        joint_layout.addLayout(self.joint_form)

        layout.addWidget(self.joint_section)
        self._refresh_mapping_label()

        return group

    def _build_actuator_group(self) -> QGroupBox:
        group = QGroupBox("作动器配置", self)
        layout = QVBoxLayout(group)

        self.actuator_list = QListWidget(group)
        self.actuator_list.setSelectionMode(QAbstractItemView.SingleSelection)
        layout.addWidget(self.actuator_list)

        button_row = QHBoxLayout()
        add_button = QPushButton("添加", group)
        add_button.clicked.connect(self._add_actuator)
        button_row.addWidget(add_button)

        edit_button = QPushButton("编辑", group)
        edit_button.clicked.connect(self._edit_actuator)
        button_row.addWidget(edit_button)

        remove_button = QPushButton("删除", group)
        remove_button.clicked.connect(self._remove_actuator)
        button_row.addWidget(remove_button)

        button_row.addStretch(1)
        layout.addLayout(button_row)

        info_label = QLabel("步骤提示：1) 解析 USD 获取关节 → 2) 选择作动器类型 → 3) 填写对应参数。", group)
        info_label.setWordWrap(True)
        layout.addWidget(info_label)

        return group

    def _build_ground_group(self) -> QGroupBox:
        group = QGroupBox("地面参数", self)
        form = QFormLayout(group)

        self.ground_friction_spin = self._make_double_spin(group, 0.0, 10.0, 0.6, decimals=3)
        form.addRow("静摩擦系数", self.ground_friction_spin)

        self.ground_dynamic_friction_spin = self._make_double_spin(group, 0.0, 10.0, 0.6, decimals=3)
        form.addRow("动摩擦系数", self.ground_dynamic_friction_spin)

        self.ground_restitution_spin = self._make_double_spin(group, 0.0, 1.0, 0.0, decimals=3)
        form.addRow("恢复系数", self.ground_restitution_spin)

        return group

    def _build_lighting_group(self) -> QGroupBox:
        group = QGroupBox("灯光参数", self)
        form = QFormLayout(group)

        self.light_intensity_spin = self._make_double_spin(group, 0.0, 100000.0, 3000.0, decimals=1)
        form.addRow("穹顶光强度", self.light_intensity_spin)

        color_layout = QHBoxLayout()
        self.light_color_spins = [self._make_double_spin(group, 0.0, 10.0, 0.75, decimals=3) for _ in range(3)]
        for spin in self.light_color_spins:
            color_layout.addWidget(spin)
        form.addRow(QLabel("颜色 (RGB)", group), color_layout)

        return group

    def _build_imu_group(self) -> QGroupBox:
        group = QGroupBox("IMU 传感器", self)
        form = QFormLayout(group)

        self.imu_enable_check = QCheckBox("启用 IMU 传感器", group)
        self.imu_enable_check.setChecked(True)
        form.addRow(self.imu_enable_check)

        self.imu_prim_edit = QLineEdit(group)
        self.imu_prim_edit.setPlaceholderText("例如 {ENV_REGEX_NS}/Robot/base_link")
        self.imu_prim_edit.setText("{ENV_REGEX_NS}/Robot/base_link")
        form.addRow("Prim 路径", self.imu_prim_edit)

        self.imu_update_spin = self._make_double_spin(group, 0.001, 10.0, 0.01, decimals=4)
        form.addRow("更新周期 (s)", self.imu_update_spin)

        self.imu_debug_check = QCheckBox("开启调试可视化", group)
        self.imu_debug_check.setChecked(True)
        form.addRow(self.imu_debug_check)

        offset_layout = QHBoxLayout()
        self.imu_offset_pos_spins = [self._make_double_spin(group, -10.0, 10.0, 0.0, decimals=4) for _ in range(3)]
        for spin in self.imu_offset_pos_spins:
            offset_layout.addWidget(spin)
        form.addRow(QLabel("位置偏移", group), offset_layout)

        rot_layout = QHBoxLayout()
        self.imu_offset_rot_spins = [self._make_double_spin(group, -1.0, 1.0, value, decimals=4) for value in (1.0, 0.0, 0.0, 0.0)]
        for spin in self.imu_offset_rot_spins:
            rot_layout.addWidget(spin)
        form.addRow(QLabel("旋转偏移", group), rot_layout)

        bias_layout = QHBoxLayout()
        self.imu_bias_spins = [self._make_double_spin(group, -50.0, 50.0, value, decimals=3) for value in (0.0, 0.0, 9.81)]
        for spin in self.imu_bias_spins:
            bias_layout.addWidget(spin)
        form.addRow(QLabel("重力偏置", group), bias_layout)

        return group

    def _build_manual_group(self) -> QGroupBox:
        group = QGroupBox("手动演示模式", self)
        layout = QVBoxLayout(group)

        info = QLabel("导出可直接运行的 Python 手动演示脚本（含滑块控制界面）。", group)
        info.setWordWrap(True)
        layout.addWidget(info)

        hint = QLabel("脚本会使用当前仿真配置与关节数量生成对应的 Tkinter 控制面板。", group)
        hint.setWordWrap(True)
        layout.addWidget(hint)

        path_row = QHBoxLayout()
        self.manual_path_edit = QLineEdit(group)
        self.manual_path_edit.setPlaceholderText("选择导出的 Python 文件路径")
        path_row.addWidget(self.manual_path_edit, 1)

        choose_button = QPushButton("浏览…", group)
        choose_button.clicked.connect(self._choose_manual_path)
        path_row.addWidget(choose_button)
        layout.addLayout(path_row)

        export_button = QPushButton("导出手动演示脚本", group)
        export_button.clicked.connect(self._export_manual_demo)
        layout.addWidget(export_button, alignment=Qt.AlignLeft)

        return group

    def _build_template_group(self) -> QGroupBox:
        group = QGroupBox("导出模板模式", self)
        layout = QVBoxLayout(group)

        info = QLabel("将当前参数导出为 Python 模板，便于批量自动化或命令行运行。", group)
        info.setWordWrap(True)
        layout.addWidget(info)

        path_row = QHBoxLayout()
        self.template_path_edit = QLineEdit(group)
        self.template_path_edit.setPlaceholderText("选择保存的 Python 文件路径")
        path_row.addWidget(self.template_path_edit, 1)
        choose_button = QPushButton("浏览…", group)
        choose_button.clicked.connect(self._choose_template_path)
        path_row.addWidget(choose_button)
        layout.addLayout(path_row)

        export_button = QPushButton("导出 Python 模板", group)
        export_button.clicked.connect(self._export_template)
        layout.addWidget(export_button, alignment=Qt.AlignLeft)

        return group

    # ------------------------------------------------------------------
    # Joint defaults integration
    # ------------------------------------------------------------------

    def update_joint_defaults(self, defaults: Dict[str, float], source: Optional[str] = None) -> None:
        cleaned: Dict[str, float] = {}
        for name, value in defaults.items():
            try:
                numeric = float(value)
            except (TypeError, ValueError):
                continue
            cleaned_name = str(name)
            cleaned[cleaned_name] = self._clamp_joint_value(cleaned_name, numeric)
        self.joint_defaults = cleaned
        self.joint_defaults_path = source
        self.urdf_joint_names = list(cleaned.keys())
        self._seed_mapping_from_defaults()
        self._update_mapping_button_state()
        self._refresh_mapping_label()
        if cleaned:
            source_text = source if source else "装配器"
            self._log(f"已接收关节默认角度（{len(cleaned)} 个，来源: {source_text}）")
        self._apply_joint_defaults()

    def joint_defaults_snapshot(self) -> Dict[str, float]:
        return dict(self.joint_defaults)

    def joint_defaults_source(self) -> Optional[str]:
        return self.joint_defaults_path

    def load_joint_defaults_from_file(self, path: str, *, show_dialogs: bool = True) -> bool:
        try:
            with open(path, "r", encoding="utf-8") as fp:
                data = json.load(fp)
        except Exception as exc:  # noqa: BLE001
            if show_dialogs:
                QMessageBox.warning(self, "加载失败", f"无法读取关节默认角度文件:\n{exc}")
            self._log(f"加载关节默认角度失败: {exc}")
            return False

        defaults_obj = None
        if isinstance(data, dict):
            if isinstance(data.get("joint_defaults"), dict):
                defaults_obj = data["joint_defaults"]
            elif isinstance(data.get("defaults"), dict):
                defaults_obj = data["defaults"]
            else:
                defaults_obj = {k: v for k, v in data.items() if isinstance(v, (int, float))}

        if not isinstance(defaults_obj, dict) or not defaults_obj:
            if show_dialogs:
                QMessageBox.information(self, "无有效数据", "JSON 文件中不包含关节默认角度。")
            self._log("关节默认角度文件缺少有效数据。")
            return False

        self.update_joint_defaults(defaults_obj, source=path)
        return True

    def _apply_joint_defaults(self) -> None:
        if not self.joint_defaults or not self.joint_spinboxes:
            return

        applied = 0
        usd_joint_names = set(self.joint_spinboxes.keys())

        for urdf_name, angle in self.joint_defaults.items():
            candidates: List[str] = []
            mapped = self.joint_name_mapping.get(urdf_name)
            if mapped:
                candidates.append(mapped)
            candidates.append(urdf_name)

            target_name = None
            for candidate in candidates:
                if candidate in usd_joint_names:
                    target_name = candidate
                    break
            if target_name is None:
                continue

            spin = self.joint_spinboxes[target_name]
            spin.blockSignals(True)
            spin.setValue(angle)
            spin.blockSignals(False)
            applied += 1

        if applied:
            source_text = self.joint_defaults_path or "装配器"
            self._log(f"已应用关节默认角度（{applied} 个，来源: {source_text}）")
            if self._status_callback:
                self._status_callback(f"仿真配置：应用 {applied} 个关节默认角度")
        self._refresh_mapping_label()

    def _choose_joint_defaults_file(self) -> None:
        start_dir = ""
        if self.joint_defaults_path:
            start_dir = str(Path(self.joint_defaults_path).parent)
        path, _ = QFileDialog.getOpenFileName(
            self,
            "选择关节默认角度 JSON",
            start_dir,
            "JSON 文件 (*.json);;所有文件 (*)",
        )
        if path:
            self.load_joint_defaults_from_file(path)

    def _open_joint_mapping_dialog(self) -> None:
        if not self.urdf_joint_names:
            QMessageBox.information(self, "缺少数据", "请先导入 URDF 关节默认角度。")
            return
        if not self.dof_joint_infos:
            QMessageBox.information(self, "缺少数据", "请先解析 USD 并识别可动关节。")
            return

        usd_joint_names = [joint["name"] for joint in self.dof_joint_infos]
        dialog = JointMappingDialog(
            self.urdf_joint_names,
            usd_joint_names,
            existing=self.joint_name_mapping,
            parent=self,
        )
        if dialog.exec() != QDialog.Accepted:
            return

        mapping = dialog.mapping()
        self.joint_name_mapping = mapping
        self._refresh_mapping_label()
        self._seed_mapping_from_defaults()  # refill missing identity pairs if needed
        self._update_mapping_button_state()
        mapped_count = len(mapping)
        self._log(f"已更新关节名称对应关系（{mapped_count} 个映射）")
        self._apply_joint_defaults()

    def _update_mapping_button_state(self) -> None:
        if not hasattr(self, "mapping_button"):
            return
        should_enable = bool(self.urdf_joint_names) and bool(self.dof_joint_infos)
        self.mapping_button.setEnabled(should_enable)

    def _refresh_mapping_label(self) -> None:
        if not hasattr(self, "joint_mapping_label"):
            return
        total = len(self.urdf_joint_names)
        mapped = sum(1 for name in self.urdf_joint_names if self.joint_name_mapping.get(name))
        if total == 0:
            text = "未加载 URDF 默认角度"
            color = "#9aa"
        elif mapped == 0:
            text = f"未映射 {total} 个关节"
            color = "#d08080"
        elif mapped < total:
            text = f"已映射 {mapped}/{total}"
            color = "#c9a640"
        else:
            text = f"已映射 {mapped}/{total}"
            color = "#6cb36c"
        self.joint_mapping_label.setText(text)
        self.joint_mapping_label.setStyleSheet(f"color: {color}; font-size: 12px;")

    def _seed_mapping_from_defaults(self) -> None:
        if not self.urdf_joint_names:
            return
        usd_names = {joint["name"] for joint in self.dof_joint_infos} or {joint["name"] for joint in self.joint_infos}
        if not usd_names:
            return
        for name in self.urdf_joint_names:
            if name not in self.joint_name_mapping and name in usd_names:
                self.joint_name_mapping[name] = name

    # ------------------------------------------------------------------
    # Helper widgets
    # ------------------------------------------------------------------

    @staticmethod
    def _make_double_spin(parent: QWidget, minimum: float, maximum: float, value: float, *, decimals: int = 3) -> NumericLineEdit:
        return NumericLineEdit(minimum, maximum, value, decimals=decimals, allow_float=True, parent=parent)

    @staticmethod
    def _make_int_spin(parent: QWidget, minimum: int, maximum: int, value: int) -> NumericLineEdit:
        return NumericLineEdit(minimum, maximum, value, allow_float=False, parent=parent)

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------

    def _choose_usd_path(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "选择 USD 文件", "", "USD 文件 (*.usd *.usda);;所有文件 (*)")
        if path:
            self.usd_path_edit.setText(path)

    def _log(self, message: str) -> None:
        self.log_output.appendPlainText(message)
        if self._status_callback:
            status_line = message.splitlines()[0] if message else ""
            self._status_callback(status_line)

    @staticmethod
    def _format_list_for_log(values: List[str], *, indent: str = "  ", max_chars: int = 60) -> str:
        if not values:
            return ""

        lines: List[str] = []
        current: List[str] = []
        current_len = 0

        for item in values:
            text = str(item)
            extra = len(text) + (1 if current else 0)
            if current and current_len + extra > max_chars:
                lines.append("，".join(current))
                current = [text]
                current_len = len(text)
            else:
                current.append(text)
                current_len += extra

        if current:
            lines.append("，".join(current))

        return "\n".join(f"{indent}{line}" for line in lines)

    def _load_usd_joints(self) -> None:
        usd_path = self.usd_path_edit.text().strip()
        if not usd_path:
            QMessageBox.warning(self, "缺少路径", "请先选择 USD 文件。")
            return
        path = Path(usd_path)
        if not path.is_file():
            QMessageBox.warning(self, "路径无效", f"未找到 USD 文件: {usd_path}")
            return
        usd_module = _try_import_usd()
        if usd_module is None:
            QMessageBox.warning(self, "缺少依赖", "未找到 pxr.Usd 模块，请在 IsaacLab 环境中运行。")
            self._log("加载失败：缺少 pxr 模块。")
            return
        self._log(f"正在解析 USD: {usd_path}")
        try:
            stage = usd_module.Stage.Open(str(path))
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "解析失败", f"无法打开 USD 文件:\n{exc}")
            self._log(f"解析失败: {exc}")
            return

        joints: List[Dict[str, object]] = []

        def _attr_to_float(attribute) -> Optional[float]:
            if attribute is None:
                return None
            try:
                if not attribute.HasValue():
                    return None
                value = attribute.Get()
            except Exception:  # noqa: BLE001
                return None
            try:
                return float(value)
            except (TypeError, ValueError):
                return None

        def _extract_limit(prim_obj, names: List[str], *, fallback_terms: Optional[List[str]] = None) -> Optional[float]:
            for attr_name in names:
                attr = prim_obj.GetAttribute(attr_name)
                value = _attr_to_float(attr)
                if value is not None:
                    return value
            if fallback_terms:
                for attribute in prim_obj.GetAttributes():
                    attr_name = attribute.GetName()
                    attr_lower = attr_name.lower()
                    if any(term in attr_lower for term in fallback_terms):
                        value = _attr_to_float(attribute)
                        if value is not None:
                            return value
            return None

        for prim in stage.Traverse():
            type_name = prim.GetTypeName()
            if type_name in USD_JOINT_TYPES:
                lower_limit = _extract_limit(
                    prim,
                    [
                        "physics:lowerLimit",
                        "drive:lowerLimit",
                        "physics:lower",
                        "drive:lower",
                        "physics:limits:lower",
                        "drive:limits:lower",
                        "limit:lower",
                        "limits:lower",
                        "limitLower",
                        "lowerLimit",
                    ],
                    fallback_terms=["lower", "min"],
                )
                upper_limit = _extract_limit(
                    prim,
                    [
                        "physics:upperLimit",
                        "drive:upperLimit",
                        "physics:upper",
                        "drive:upper",
                        "physics:limits:upper",
                        "drive:limits:upper",
                        "limit:upper",
                        "limits:upper",
                        "limitUpper",
                        "upperLimit",
                    ],
                    fallback_terms=["upper", "max"],
                )

                joint_info: Dict[str, object] = {
                    "name": prim.GetName(),
                    "path": prim.GetPath().pathString,
                    "type": type_name,
                }
                if lower_limit is not None:
                    joint_info["lower_limit"] = lower_limit
                if upper_limit is not None:
                    joint_info["upper_limit"] = upper_limit

                joints.append(joint_info)

        self._assign_joint_lists(joints)
        self._populate_joint_list()
        summary = "识别关节 {} 个（自由度 {} 个）".format(
            len(self.joint_infos),
            len(self.dof_joint_infos),
        )
        joint_details = self._format_list_for_log([j["name"] for j in self.joint_infos])
        if joint_details:
            summary = f"{summary}\n{joint_details}"
        self._log(summary)

        if self.dof_joint_infos:
            dof_details = self._format_list_for_log([j["name"] for j in self.dof_joint_infos])
            message = "可配置自由度："
            if dof_details:
                message = f"{message}\n{dof_details}"
            self._log(message)

    def _populate_joint_list(self) -> None:
        self.joint_spinboxes.clear()
        while self.joint_form.rowCount():
            self.joint_form.removeRow(0)
        self.actuator_list.clear()
        self.actuator_configs.clear()

        if not self.joint_infos:
            info = QLabel("尚未解析到关节。", self.joint_section)
            self.joint_form.addRow(info)
            return

        dof_names = {joint["name"] for joint in self.dof_joint_infos}
        for joint in self.joint_infos:
            spin = self._make_double_spin(self.joint_section, -10.0, 10.0, 0.0, decimals=4)
            spin.setSingleStep(0.05)
            label_text = f"{joint['name']}（{JOINT_TYPE_LABELS.get(joint['type'], joint['type'])}）"
            label = QLabel(label_text, self.joint_section)
            if joint["name"] not in dof_names:
                spin.setEnabled(False)
                spin.setToolTip("固定关节，无自由度")
            else:
                self.joint_spinboxes[joint["name"]] = spin
            self.joint_form.addRow(label, spin)

        self._apply_joint_defaults()

    def _ensure_unique_actuator_name(self, base_name: str, *, skip_index: Optional[int] = None) -> str:
        sanitized = _sanitize_actuator_name(base_name)
        if not sanitized:
            sanitized = "actuator"
        existing = {
            cfg.get("name", "")
            for idx, cfg in enumerate(self.actuator_configs)
            if skip_index is None or idx != skip_index
        }
        candidate = sanitized
        suffix = 2
        while candidate in existing:
            candidate = f"{sanitized}_{suffix}"
            suffix += 1
        return candidate

    def _add_actuator(self) -> None:
        if not self.joint_infos:
            QMessageBox.information(self, "缺少关节", "请先解析 USD 并识别关节。")
            return
        dialog = ActuatorConfigDialog(self._dof_joint_name_list(), parent=self)
        if dialog.exec() == QDialog.Accepted:
            config = dialog.result_config()
            if config:
                config["name"] = self._ensure_unique_actuator_name(config.get("name", "actuator"))
                self.actuator_configs.append(config)
                self._refresh_actuator_list()
                self._log(f"新增作动器: {config['name']}")

    def _edit_actuator(self) -> None:
        row = self.actuator_list.currentRow()
        if row < 0 or row >= len(self.actuator_configs):
            QMessageBox.information(self, "请选择作动器", "请先在列表中选择一个作动器。")
            return
        valid_joint_names = self._dof_joint_name_list()
        dialog = ActuatorConfigDialog(valid_joint_names, existing=self.actuator_configs[row], parent=self)
        if dialog.exec() == QDialog.Accepted:
            config = dialog.result_config()
            if config:
                config["name"] = self._ensure_unique_actuator_name(config.get("name", "actuator"), skip_index=row)
                self.actuator_configs[row] = config
                self._refresh_actuator_list()
                self._log(f"更新作动器: {config['name']}")

    def _remove_actuator(self) -> None:
        row = self.actuator_list.currentRow()
        if row < 0 or row >= len(self.actuator_configs):
            return
        config = self.actuator_configs.pop(row)
        self._refresh_actuator_list()
        self._log(f"已删除作动器: {config['name']}")

    def _refresh_actuator_list(self) -> None:
        self.actuator_list.clear()
        for config in self.actuator_configs:
            type_label = next((label for key, label in ACTUATOR_TYPES if key == config.get("type")), config.get("type"))
            joint_count = len(config.get("joint_names", []))
            item = QListWidgetItem(f"{config.get('name')}（{type_label}，{joint_count} 个关节）")
            self.actuator_list.addItem(item)

    def _choose_manual_path(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "导出手动演示脚本", "manual_demo.py", "Python 文件 (*.py)")
        if not path:
            return
        if not path.endswith(".py"):
            path += ".py"
        self.manual_path_edit.setText(path)

    def _export_manual_demo(self) -> None:
        joint_names: List[str] = []
        seen: set[str] = set()

        for actuator in self.actuator_configs:
            for name in actuator.get("joint_names", []):
                cleaned = str(name).strip()
                if cleaned and cleaned not in seen:
                    seen.add(cleaned)
                    joint_names.append(cleaned)

        if not joint_names:
            QMessageBox.information(
                self,
                "缺少作动器",
                "未在作动器配置中找到关节名，无法生成手动演示脚本。",
            )
            return

        config = self._collect_current_config()
        if config is None:
            return

        path_text = self.manual_path_edit.text().strip()
        if not path_text:
            QMessageBox.information(self, "缺少路径", "请先选择导出路径。")
            return

        defaults: List[float] = []
        joint_limits_payload: List[Tuple[Optional[float], Optional[float]]] = []
        for name in joint_names:
            if name in self.joint_spinboxes:
                value = float(self.joint_spinboxes[name].value())
            else:
                value = float(self.joint_defaults.get(name, 0.0))
            clamped_value = self._clamp_joint_value(name, value)
            defaults.append(clamped_value)
            joint_limits_payload.append(self.joint_limits.get(name, (None, None)))

        path = Path(path_text)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "创建目录失败", f"无法创建目录:\n{exc}")
            return

        try:
            code = self._build_manual_python(config, joint_names, defaults, joint_limits_payload)
            path.write_text(code, encoding="utf-8")
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "导出失败", f"写入文件失败:\n{exc}")
            self._log(f"导出手动演示脚本失败: {exc}")
            return

        self._log(f"手动演示脚本已导出到 {path}")
        QMessageBox.information(self, "导出成功", f"已生成手动演示脚本:\n{path}")

    def _choose_template_path(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "导出 Python 模板", "simulation_config.py", "Python 文件 (*.py)")
        if path:
            if not path.endswith(".py"):
                path += ".py"
            self.template_path_edit.setText(path)

    def _export_template(self) -> None:
        config = self._collect_current_config()
        if config is None:
            return
        path_text = self.template_path_edit.text().strip()
        if not path_text:
            QMessageBox.information(self, "缺少路径", "请先选择导出路径。")
            return
        path = Path(path_text)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "创建目录失败", f"无法创建目录:\n{exc}")
            return
        try:
            code = self._build_template_python(config)
            path.write_text(code, encoding="utf-8")
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "导出失败", f"写入文件失败:\n{exc}")
            self._log(f"导出失败: {exc}")
            return
        self._log(f"模板已导出到 {path}")
        QMessageBox.information(self, "导出成功", f"已生成模板:\n{path}")

    def _on_save_json(self) -> None:
        config = self._collect_current_config()
        if config is None:
            return
        path, _ = QFileDialog.getSaveFileName(self, "保存配置 JSON", "simulation_config.json", "JSON 文件 (*.json)")
        if not path:
            return
        if not path.endswith(".json"):
            path += ".json"
        try:
            with open(path, "w", encoding="utf-8") as fp:
                json.dump(config, fp, ensure_ascii=False, indent=2)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "保存失败", f"写入 JSON 失败:\n{exc}")
            self._log(f"保存 JSON 失败: {exc}")
            return
        self._log(f"配置已保存到 {path}")

    def _on_load_json(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "加载配置 JSON", "", "JSON 文件 (*.json)")
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as fp:
                data = json.load(fp)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "加载失败", f"读取 JSON 失败:\n{exc}")
            self._log(f"加载 JSON 失败: {exc}")
            return
        self._apply_config(data)
        self._log(f"已加载配置: {path}")

    # ------------------------------------------------------------------
    # Config persistence
    # ------------------------------------------------------------------

    def _collect_current_config(self) -> Optional[Dict]:
        usd_path = self.usd_path_edit.text().strip()
        if not usd_path:
            QMessageBox.warning(self, "缺少 USD", "请先选择 USD 文件。")
            return None

        joint_pos: Dict[str, float] = {}
        for name, spin in self.joint_spinboxes.items():
            raw_value = float(spin.value())
            clamped_value = self._clamp_joint_value(name, raw_value)
            if not math.isclose(clamped_value, raw_value, abs_tol=1e-4):
                spin.blockSignals(True)
                spin.setValue(clamped_value)
                spin.blockSignals(False)
                self._log(f"关节 {name} 超出限制，已自动调整至 {clamped_value:.6f} rad")
            joint_pos[name] = clamped_value

        initial_state = {
            "pos": [float(spin.value()) for spin in self.init_pos_spins],
            "rot": [float(spin.value()) for spin in self.init_rot_spins],
            "joint_pos": joint_pos,
        }

        config: Dict[str, object] = {
            "usd_path": usd_path,
            "system": {
                "description": self.description_edit.text().strip(),
                "num_envs": int(self.num_envs_spin.value()),
                "control_freq": float(self.control_freq_spin.value()),
            },
            "rigid_body": {
                "disable_gravity": self.disable_gravity_check.isChecked(),
                "max_linear_velocity": float(self.max_linear_spin.value()),
                "max_angular_velocity": float(self.max_angular_spin.value()),
                "max_depenetration_velocity": float(self.max_depenetration_spin.value()),
                "enable_gyroscopic_forces": self.enable_gyro_check.isChecked(),
            },
            "articulation_root": {
                "enabled_self_collisions": self.self_collision_check.isChecked(),
                "solver_position_iteration_count": int(self.solver_pos_spin.value()),
                "solver_velocity_iteration_count": int(self.solver_vel_spin.value()),
                "sleep_threshold": float(self.sleep_threshold_spin.value()),
                "stabilization_threshold": float(self.stabilization_spin.value()),
            },
            "initial_state": initial_state,
            "actuators": self.actuator_configs,
            "ground": {
                "static_friction": float(self.ground_friction_spin.value()),
                "dynamic_friction": float(self.ground_dynamic_friction_spin.value()),
                "restitution": float(self.ground_restitution_spin.value()),
            },
            "lighting": {
                "intensity": float(self.light_intensity_spin.value()),
                "color": [float(spin.value()) for spin in self.light_color_spins],
            },
            "imu": {
                "enabled": self.imu_enable_check.isChecked(),
                "prim_path": self.imu_prim_edit.text().strip(),
                "update_period": float(self.imu_update_spin.value()),
                "debug_vis": self.imu_debug_check.isChecked(),
                "offset_pos": [float(spin.value()) for spin in self.imu_offset_pos_spins],
                "offset_rot": [float(spin.value()) for spin in self.imu_offset_rot_spins],
                "gravity_bias": [float(spin.value()) for spin in self.imu_bias_spins],
            },
            "joints": self.joint_infos,
        }
        if self.joint_name_mapping:
            config["joint_name_mapping"] = dict(self.joint_name_mapping)
        if self.joint_defaults_path:
            config["joint_defaults_path"] = self.joint_defaults_path
        if self.joint_defaults:
            clamped_defaults = {
                name: self._clamp_joint_value(name, float(value))
                for name, value in self.joint_defaults.items()
            }
            config["joint_defaults_values"] = clamped_defaults
        return config

    def _apply_config(self, data: Dict) -> None:
        self.usd_path_edit.setText(data.get("usd_path", ""))
        system = data.get("system", {})
        self.description_edit.setText(system.get("description", ""))
        if "num_envs" in system:
            self.num_envs_spin.setValue(int(system["num_envs"]))
        if "control_freq" in system:
            self.control_freq_spin.setValue(float(system["control_freq"]))

        rigid = data.get("rigid_body", {})
        self.disable_gravity_check.setChecked(bool(rigid.get("disable_gravity", False)))
        if "max_linear_velocity" in rigid:
            self.max_linear_spin.setValue(float(rigid["max_linear_velocity"]))
        if "max_angular_velocity" in rigid:
            self.max_angular_spin.setValue(float(rigid["max_angular_velocity"]))
        if "max_depenetration_velocity" in rigid:
            self.max_depenetration_spin.setValue(float(rigid["max_depenetration_velocity"]))
        self.enable_gyro_check.setChecked(bool(rigid.get("enable_gyroscopic_forces", True)))

        articulation = data.get("articulation_root", {})
        self.self_collision_check.setChecked(bool(articulation.get("enabled_self_collisions", False)))
        if "solver_position_iteration_count" in articulation:
            self.solver_pos_spin.setValue(int(articulation["solver_position_iteration_count"]))
        if "solver_velocity_iteration_count" in articulation:
            self.solver_vel_spin.setValue(int(articulation["solver_velocity_iteration_count"]))
        if "sleep_threshold" in articulation:
            self.sleep_threshold_spin.setValue(float(articulation["sleep_threshold"]))
        if "stabilization_threshold" in articulation:
            self.stabilization_spin.setValue(float(articulation["stabilization_threshold"]))

        self._assign_joint_lists(data.get("joints", self.joint_infos))
        self._populate_joint_list()

        initial_state = data.get("initial_state", {})
        pos = initial_state.get("pos", [])
        for spin, value in zip(self.init_pos_spins, pos):
            spin.setValue(float(value))
        rot = initial_state.get("rot", [])
        for spin, value in zip(self.init_rot_spins, rot):
            spin.setValue(float(value))
        joint_pos = initial_state.get("joint_pos", {})
        for name, value in joint_pos.items():
            if name in self.joint_spinboxes:
                numeric = float(value)
                clamped = self._clamp_joint_value(name, numeric)
                if not math.isclose(clamped, numeric, abs_tol=1e-4):
                    self._log(f"关节 {name} 默认值超出限制，已调整至 {clamped:.6f} rad")
                self.joint_spinboxes[name].setValue(clamped)

        self.actuator_configs = data.get("actuators", [])
        self._refresh_actuator_list()

        ground = data.get("ground", {})
        if "static_friction" in ground:
            self.ground_friction_spin.setValue(float(ground["static_friction"]))
        if "dynamic_friction" in ground:
            self.ground_dynamic_friction_spin.setValue(float(ground["dynamic_friction"]))
        if "restitution" in ground:
            self.ground_restitution_spin.setValue(float(ground["restitution"]))

        lighting = data.get("lighting", {})
        if "intensity" in lighting:
            self.light_intensity_spin.setValue(float(lighting["intensity"]))
        colors = lighting.get("color", [])
        for spin, value in zip(self.light_color_spins, colors):
            spin.setValue(float(value))

        imu = data.get("imu", {})
        self.imu_enable_check.setChecked(bool(imu.get("enabled", True)))
        self.imu_prim_edit.setText(imu.get("prim_path", "{ENV_REGEX_NS}/Robot/base_link"))
        if "update_period" in imu:
            self.imu_update_spin.setValue(float(imu["update_period"]))
        self.imu_debug_check.setChecked(bool(imu.get("debug_vis", True)))
        offset_pos = imu.get("offset_pos", [])
        for spin, value in zip(self.imu_offset_pos_spins, offset_pos):
            spin.setValue(float(value))
        offset_rot = imu.get("offset_rot", [])
        for spin, value in zip(self.imu_offset_rot_spins, offset_rot):
            spin.setValue(float(value))
        bias = imu.get("gravity_bias", [])
        for spin, value in zip(self.imu_bias_spins, bias):
            spin.setValue(float(value))

        mapping = data.get("joint_name_mapping")
        if isinstance(mapping, dict):
            cleaned_mapping = {
                str(name): str(target)
                for name, target in mapping.items()
                if isinstance(name, str) and isinstance(target, str) and target
            }
            self.joint_name_mapping = cleaned_mapping
        else:
            self.joint_name_mapping = {}

        defaults_path = data.get("joint_defaults_path")
        defaults_values = data.get("joint_defaults_values")
        if isinstance(defaults_values, dict) and defaults_values:
            self.update_joint_defaults(defaults_values, source=defaults_path)
        elif isinstance(defaults_path, str) and defaults_path:
            self.load_joint_defaults_from_file(defaults_path, show_dialogs=False)
        else:
            self.urdf_joint_names = list(self.joint_defaults.keys())
            self._refresh_mapping_label()
            self._update_mapping_button_state()

    # ------------------------------------------------------------------
    # Template generation
    # ------------------------------------------------------------------

    def _build_manual_python(
        self,
        config: Dict,
        joint_names: List[str],
        defaults: List[float],
        joint_limits: List[Tuple[Optional[float], Optional[float]]],
    ) -> str:
        if not joint_names:
            raise ValueError("joint_names must not be empty when exporting the manual demo script.")

        sanitized_defaults: List[float] = []
        limits_payload: List[List[Optional[float]]] = []

        for idx, name in enumerate(joint_names):
            source_value = defaults[idx] if idx < len(defaults) else 0.0
            try:
                numeric = float(source_value)
            except (TypeError, ValueError):
                numeric = 0.0
            if not math.isfinite(numeric):
                numeric = 0.0

            clamped = self._clamp_joint_value(name, numeric)
            limits = self.joint_limits.get(name, (None, None))

            sanitized_defaults.append(clamped)
            limits_payload.append([
                limits[0] if isinstance(limits[0], (int, float)) else None,
                limits[1] if isinstance(limits[1], (int, float)) else None,
            ])

        export_config = copy.deepcopy(config)
        initial_state = export_config.setdefault("initial_state", {})
        joint_pos_cfg = initial_state.setdefault("joint_pos", {})
        for joint_name, raw_value in list(joint_pos_cfg.items()):
            try:
                numeric = float(raw_value)
            except (TypeError, ValueError):
                numeric = 0.0
            joint_pos_cfg[joint_name] = self._clamp_joint_value(joint_name, numeric)

        json_block = json.dumps(export_config, ensure_ascii=False, indent=4)
        joint_block = json.dumps(joint_names, ensure_ascii=False, indent=4)
        defaults_block = json.dumps(sanitized_defaults, ensure_ascii=False, indent=4)
        limits_block = json.dumps(limits_payload, ensure_ascii=False, indent=4)

        template = """# 自动生成的 IsaacLab 手动演示脚本
# 由 FluxhWeave Workbench 仿真配置工具导出

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from typing import List, Optional, Tuple

import torch

CONFIG_DATA = json.loads('''__JSON_BLOCK__''')
JOINT_NAMES = json.loads('''__JOINT_LIST__''')
DEFAULT_JOINT_TARGETS = json.loads('''__DEFAULT_LIST__''')
JOINT_LIMITS = json.loads('''__LIMIT_LIST__''')

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(
    description=CONFIG_DATA["system"].get("description", "FluxhWeave Manual Demo"),
)
parser.add_argument(
    "--num_envs",
    type=int,
    default=CONFIG_DATA["system"].get("num_envs", 1),
    help="Number of environments to spawn.",
)
parser.add_argument(
    "--control_freq",
    type=float,
    default=CONFIG_DATA["system"].get("control_freq", 30.0),
    help="Control frequency in Hz.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.actuators import (
    ImplicitActuatorCfg,
    IdealPDActuatorCfg,
    DCMotorCfg,
    DelayedPDActuatorCfg,
    RemotizedPDActuatorCfg,
)
from isaaclab.assets import AssetBaseCfg, ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import ImuCfg
from isaaclab.utils import configclass

try:
    import tkinter as tk
    from tkinter import ttk
    GUI_AVAILABLE = True
except Exception:  # noqa: BLE001
    GUI_AVAILABLE = False
    tk = None
    ttk = None

_ACTUATOR_CLASS = {
    "ImplicitActuator": ImplicitActuatorCfg,
    "IdealPDActuator": IdealPDActuatorCfg,
    "DCMotor": DCMotorCfg,
    "DelayedPDActuator": DelayedPDActuatorCfg,
    "RemotizedPDActuator": RemotizedPDActuatorCfg,
}


@configclass
class ManualSceneCfg(InteractiveSceneCfg):
    '''自动生成的交互式仿真场景配置'''

    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="multiply",
                restitution_combine_mode="multiply",
                static_friction=CONFIG_DATA["ground"]["static_friction"],
                dynamic_friction=CONFIG_DATA["ground"]["dynamic_friction"],
                restitution=CONFIG_DATA["ground"]["restitution"],
            ),
        ),
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(
            intensity=CONFIG_DATA["lighting"]["intensity"],
            color=tuple(CONFIG_DATA["lighting"]["color"]),
        ),
    )

    robot = None
    imu_sensor = None


def build_robot_cfg() -> ArticulationCfg:
    data = CONFIG_DATA
    actuator_cfgs = {}
    for actuator in data.get("actuators", []):
        cls = _ACTUATOR_CLASS.get(actuator.get("type"))
        if cls is None:
            raise KeyError(f"未知的作动器类型: {actuator.get('type')}")
        params = dict(actuator.get("parameters", {}))
        params["joint_names_expr"] = actuator.get("joint_names", [])
        actuator_cfgs[actuator["name"]] = cls(**params)

    robot_cfg = ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=data["usd_path"],
            rigid_props=sim_utils.RigidBodyPropertiesCfg(**data["rigid_body"]),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(**data["articulation_root"]),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=tuple(data["initial_state"]["pos"]),
            rot=tuple(data["initial_state"]["rot"]),
            joint_pos=data["initial_state"].get("joint_pos", {}),
        ),
        actuators=actuator_cfgs,
    )
    return robot_cfg


def build_scene_cfg(num_envs: int) -> ManualSceneCfg:
    scene = ManualSceneCfg(num_envs=num_envs, env_spacing=3.0)
    scene.robot = build_robot_cfg().replace(prim_path="{ENV_REGEX_NS}/Robot")
    imu_data = CONFIG_DATA.get("imu", {})
    if imu_data.get("enabled", True):
        scene.imu_sensor = ImuCfg(
            prim_path=imu_data.get("prim_path", "{ENV_REGEX_NS}/Robot/base_link"),
            offset=ImuCfg.OffsetCfg(
                pos=tuple(imu_data.get("offset_pos", (0.0, 0.0, 0.0))),
                rot=tuple(imu_data.get("offset_rot", (1.0, 0.0, 0.0, 0.0))),
            ),
            debug_vis=imu_data.get("debug_vis", True),
            update_period=imu_data.get("update_period", 0.01),
            gravity_bias=tuple(imu_data.get("gravity_bias", (0.0, 0.0, 9.81))),
        )
    return scene


class RobotController:
    '''线程安全的关节位置控制器'''

    def __init__(self, defaults: List[float], limits: List[List[Optional[float]]]) -> None:
        self._limits = limits or []
        self._lock = threading.Lock()
        self.action_scale = 1.0
        clamped_defaults = [self._clamp_value(idx, value) for idx, value in enumerate(defaults)]
        self._values = clamped_defaults
        self._defaults = list(clamped_defaults)

    def _limit_for(self, index: int) -> Tuple[Optional[float], Optional[float]]:
        if 0 <= index < len(self._limits):
            entry = self._limits[index]
            if isinstance(entry, (list, tuple)) and entry:
                lower = entry[0] if len(entry) > 0 else None
                upper = entry[1] if len(entry) > 1 else None
                lower_val = float(lower) if isinstance(lower, (int, float)) else None
                upper_val = float(upper) if isinstance(upper, (int, float)) else None
                if lower_val is not None and upper_val is not None and lower_val > upper_val:
                    lower_val, upper_val = upper_val, lower_val
                return lower_val, upper_val
        return (None, None)

    def _clamp_value(self, index: int, value: float) -> float:
        result = float(value)
        if not math.isfinite(result):
            result = 0.0
        lower, upper = self._limit_for(index)
        if lower is not None and result < lower:
            result = lower
        if upper is not None and result > upper:
            result = upper
        if upper is not None and math.isclose(result, upper, abs_tol=1e-4):
            result = upper - 1e-4
        if lower is not None and math.isclose(result, lower, abs_tol=1e-4):
            result = lower + 1e-4
        if lower is not None:
            result = max(result, lower)
        if upper is not None:
            result = min(result, upper)
        if lower is None and upper is None and abs(result) > 1e-4:
            result -= math.copysign(1e-4, result)
        return result

    def limits_for(self, index: int) -> Tuple[Optional[float], Optional[float]]:
        return self._limit_for(index)

    def get_actions(self) -> List[float]:
        with self._lock:
            return list(self._values)

    def set_action(self, index: int, value: float) -> float:
        clamped = self._clamp_value(index, value)
        with self._lock:
            if 0 <= index < len(self._values):
                self._values[index] = clamped
        return clamped

    def set_actions(self, values: List[float]) -> None:
        if len(values) != len(self._values):
            return
        clamped = [self._clamp_value(idx, value) for idx, value in enumerate(values)]
        with self._lock:
            self._values = clamped

    def reset_joint(self, index: int) -> float:
        default = self._defaults[index] if 0 <= index < len(self._defaults) else 0.0
        return self.set_action(index, default)

    def reset(self) -> None:
        self.set_actions(self._defaults)


class ManualControlWindow:
    '''基于 Tkinter 的关节滑块界面'''

    def __init__(self, controller: RobotController) -> None:
        self.controller = controller
        self.root = tk.Tk()
        self.root.title("FluxhWeave 手动演示控制台")
        self._scales = []
        self._value_vars = []
        self.status_var = tk.StringVar()
        self._updating_slider = False
        self._build_layout()

    def _build_layout(self) -> None:
        header = ttk.Label(self.root, text="手动滑块演示", font=("Arial", 16, "bold"))
        header.pack(pady=10)

        joints_frame = ttk.Frame(self.root)
        joints_frame.pack(fill="both", expand=True, padx=12, pady=8)

        column_count = min(4, max(1, len(JOINT_NAMES)))
        for idx in range(column_count):
            joints_frame.columnconfigure(idx, weight=1)

        current_actions = self.controller.get_actions()

        for index, joint_name in enumerate(JOINT_NAMES):
            frame = ttk.Frame(joints_frame, padding=6)
            frame.grid(row=index // column_count, column=index % column_count, padx=6, pady=6, sticky="nsew")

            name_label = ttk.Label(frame, text=joint_name)
            name_label.pack()

            initial_value = current_actions[index] if index < len(current_actions) else 0.0
            value_var = tk.StringVar(value=f"{initial_value:.3f}")
            value_label = ttk.Label(frame, textvariable=value_var, font=("Courier", 10))
            value_label.pack(pady=4)

            lower, upper = self.controller.limits_for(index)
            slider_min = lower if lower is not None else -3.14
            slider_max = upper if upper is not None else 3.14
            if slider_min is not None and slider_max is not None and slider_min == slider_max:
                slider_min -= 1.0
                slider_max += 1.0

            scale = tk.Scale(
                frame,
                from_=slider_max,
                to=slider_min,
                resolution=0.001,
                orient=tk.VERTICAL,
                length=180,
            )
            scale.configure(command=lambda value, idx=index, var=value_var, scale_widget=scale: self._on_slider(idx, value, var, scale_widget))
            scale.set(initial_value)
            scale.pack(fill="y", expand=True)

            reset_btn = ttk.Button(frame, text="归零", command=lambda idx=index: self._reset_joint(idx))
            reset_btn.pack(pady=4)

            self._scales.append(scale)
            self._value_vars.append(value_var)

        controls = ttk.Frame(self.root)
        controls.pack(fill="x", padx=12, pady=8)
        reset_all_btn = ttk.Button(controls, text="全部归零", command=self._reset_all)
        reset_all_btn.pack(side="left")

        self.status_var.set(self._format_status())
        status_label = ttk.Label(self.root, textvariable=self.status_var, font=("Courier", 10))
        status_label.pack(fill="x", padx=12, pady=4)

    def _format_status(self) -> str:
        actions = self.controller.get_actions()
        formatted = ", ".join(f"{value:.2f}" for value in actions)
        return f"当前关节目标: [{formatted}]"

    def _schedule_update(self) -> None:
        self.status_var.set(self._format_status())
        self.root.after(120, self._schedule_update)

    def _on_slider(self, index: int, value: str, value_var, scale_widget) -> None:
        if self._updating_slider:
            return
        try:
            numeric = float(value)
        except ValueError:
            numeric = 0.0
        actual = self.controller.set_action(index, numeric)
        value_var.set(f"{actual:.3f}")
        if abs(actual - numeric) > 1e-5 and scale_widget is not None:
            self._updating_slider = True
            try:
                scale_widget.set(actual)
            finally:
                self._updating_slider = False

    def _reset_joint(self, index: int) -> None:
        actual = self.controller.reset_joint(index)
        scale = self._scales[index]
        self._updating_slider = True
        try:
            scale.set(actual)
        finally:
            self._updating_slider = False
        self._value_vars[index].set(f"{actual:.3f}")

    def _reset_all(self) -> None:
        for idx in range(len(self._scales)):
            self._reset_joint(idx)

    def run(self) -> None:
        self._schedule_update()
        self.root.mainloop()


def run_manual_interface(controller: RobotController) -> None:
    if not GUI_AVAILABLE:
        print("[WARNING] Tkinter 不可用，无法显示滑块界面。")
        return
    window = ManualControlWindow(controller)
    window.run()


def run_simulation(sim: sim_utils.SimulationContext, scene: InteractiveScene, controller: RobotController) -> None:
    robot = scene["robot"]

    actual_joint_names = list(robot.data.joint_names)
    print(f"[INFO] USD 关节顺序: {actual_joint_names}")
    print(f"[INFO] 配置关节顺序: {JOINT_NAMES}")

    joint_index_mapping: List[int] = []
    for cfg_name in JOINT_NAMES:
        try:
            actual_idx = actual_joint_names.index(cfg_name)
            joint_index_mapping.append(actual_idx)
        except ValueError:
            print(f"[WARNING] 无法在 USD 关节中找到 {cfg_name}，后续控制将忽略该关节。")
            joint_index_mapping.append(-1)

    print(f"[INFO] 关节索引映射: {joint_index_mapping}")

    sim_dt = sim.get_physics_dt()
    control_dt = max(1.0 / max(args_cli.control_freq, 1e-6), sim_dt)
    next_control_time = 0.0
    sim_time = 0.0
    step_count = 0

    print("[INFO] 开始仿真循环，使用手动滑块控制关节。")

    while simulation_app.is_running():
        if sim_time >= next_control_time:
            actions_list = controller.get_actions()
            reordered_actions = [0.0] * len(actual_joint_names)
            for cfg_idx, actual_idx in enumerate(joint_index_mapping):
                if 0 <= actual_idx < len(actual_joint_names) and cfg_idx < len(actions_list):
                    reordered_actions[actual_idx] = actions_list[cfg_idx]

            actions_tensor = torch.tensor(
                [reordered_actions] * robot.num_instances,
                device=robot.device,
                dtype=torch.float32,
            )
            actions_tensor = actions_tensor * controller.action_scale
            robot.set_joint_position_target(actions_tensor)
            next_control_time += control_dt

        scene.write_data_to_sim()
        sim.step()
        sim_time += sim_dt
        scene.update(sim_dt)

        if step_count % 500 == 0:
            root_pos = robot.data.root_pos_w[0]
            joint_pos = robot.data.joint_pos[0]
            joint_text = ", ".join(f"{value:.3f}" for value in joint_pos)
            action_targets = controller.get_actions()
            mapped_actions = [0.0] * len(actual_joint_names)
            for cfg_idx, actual_idx in enumerate(joint_index_mapping):
                if 0 <= actual_idx < len(actual_joint_names) and cfg_idx < len(action_targets):
                    mapped_actions[actual_idx] = action_targets[cfg_idx]
            action_text = ", ".join(f"{value:.3f}" for value in mapped_actions)
            print(f"[步骤 {step_count}] 根位置: ({root_pos[0]:.3f}, {root_pos[1]:.3f}, {root_pos[2]:.3f})")
            print(f"[步骤 {step_count}] 当前关节: [{joint_text}]")
            print(f"[步骤 {step_count}] 目标角度: [{action_text}]")

        step_count += 1


def main() -> None:
    description = CONFIG_DATA["system"].get("description") or "FluxhWeave Manual Demo"
    print(f"[INFO] 启动 {description}")

    if not JOINT_NAMES:
        raise RuntimeError("JOINT_NAMES 为空，无法生成手动演示脚本。")

    defaults = list(DEFAULT_JOINT_TARGETS) if DEFAULT_JOINT_TARGETS else [0.0] * len(JOINT_NAMES)
    if len(defaults) < len(JOINT_NAMES):
        defaults.extend([0.0] * (len(JOINT_NAMES) - len(defaults)))
    elif len(defaults) > len(JOINT_NAMES):
        defaults = defaults[: len(JOINT_NAMES)]

    controller = RobotController(defaults, JOINT_LIMITS)

    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([3.0, 3.0, 2.0], [0.0, 0.0, 1.0])

    scene_cfg = build_scene_cfg(args_cli.num_envs)
    scene = InteractiveScene(scene_cfg)

    sim.reset()

    if GUI_AVAILABLE:
        threading.Thread(target=run_manual_interface, args=(controller,), daemon=True).start()
        time.sleep(0.5)
    else:
        print("[WARNING] Tkinter 未安装，使用默认关节目标运行。")

    try:
        run_simulation(sim, scene, controller)
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
"""

        return (
            template.replace("__JSON_BLOCK__", json_block)
            .replace("__JOINT_LIST__", joint_block)
            .replace("__DEFAULT_LIST__", defaults_block)
            .replace("__LIMIT_LIST__", limits_block)
        )

    def _build_template_python(self, config: Dict) -> str:
        json_block = json.dumps(config, ensure_ascii=False, indent=4)
        template = """# 自动生成的 IsaacLab 仿真配置模板
# 由 FluxhWeave Workbench 仿真配置工具导出

from __future__ import annotations

import json

import isaaclab.sim as sim_utils
from isaaclab.actuators import (
    ImplicitActuatorCfg,
    IdealPDActuatorCfg,
    DCMotorCfg,
    DelayedPDActuatorCfg,
    RemotizedPDActuatorCfg,
)
from isaaclab.assets import AssetBaseCfg, ArticulationCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ImuCfg
from isaaclab.utils import configclass

CONFIG_DATA = __JSON_BLOCK__

_ACTUATOR_CLASS = {
    "ImplicitActuator": ImplicitActuatorCfg,
    "IdealPDActuator": IdealPDActuatorCfg,
    "DCMotor": DCMotorCfg,
    "DelayedPDActuator": DelayedPDActuatorCfg,
    "RemotizedPDActuator": RemotizedPDActuatorCfg,
}


_def_ground_cfg = sim_utils.GroundPlaneCfg(
    physics_material=sim_utils.RigidBodyMaterialCfg(
        friction_combine_mode="multiply",
        restitution_combine_mode="multiply",
        static_friction=CONFIG_DATA["ground"]["static_friction"],
        dynamic_friction=CONFIG_DATA["ground"]["dynamic_friction"],
        restitution=CONFIG_DATA["ground"]["restitution"],
    ),
)


@configclass
class SimulationSceneCfg(InteractiveSceneCfg):
    \"\"\"自动生成的交互式仿真场景配置\"\"\"

    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=_def_ground_cfg,
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(
            intensity=CONFIG_DATA["lighting"]["intensity"],
            color=tuple(CONFIG_DATA["lighting"]["color"]),
        ),
    )


    robot = None  # 占位符，在 build_robot_cfg 中设定

    imu_sensor = None


def build_robot_cfg():
    data = CONFIG_DATA
    actuator_cfgs = {}
    for actuator in data["actuators"]:
        cls = _ACTUATOR_CLASS.get(actuator["type"])
        if cls is None:
            raise KeyError(f"未知的作动器类型: {actuator['type']}")
        params = dict(actuator.get("parameters", {}))
        params["joint_names_expr"] = actuator.get("joint_names", [])
        actuator_cfgs[actuator["name"]] = cls(**params)

    robot_cfg = ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=data["usd_path"],
            rigid_props=sim_utils.RigidBodyPropertiesCfg(**data["rigid_body"]),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(**data["articulation_root"]),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=tuple(data["initial_state"]["pos"]),
            rot=tuple(data["initial_state"]["rot"]),
            joint_pos=data["initial_state"].get("joint_pos", {}),
        ),
        actuators=actuator_cfgs,
    )
    return robot_cfg


def build_scene_cfg():
    scene = SimulationSceneCfg()
    scene.robot = build_robot_cfg().replace(prim_path="{ENV_REGEX_NS}/Robot")
    imu_data = CONFIG_DATA.get("imu", {})
    if imu_data.get("enabled", True):
        scene.imu_sensor = ImuCfg(
            prim_path=imu_data.get("prim_path", "{ENV_REGEX_NS}/Robot/base_link"),
            offset=ImuCfg.OffsetCfg(
                pos=tuple(imu_data.get("offset_pos", (0.0, 0.0, 0.0))),
                rot=tuple(imu_data.get("offset_rot", (1.0, 0.0, 0.0, 0.0))),
            ),
            debug_vis=imu_data.get("debug_vis", True),
            update_period=imu_data.get("update_period", 0.01),
            gravity_bias=tuple(imu_data.get("gravity_bias", (0.0, 0.0, 9.81))),
        )
    return scene


if __name__ == "__main__":
    print(json.dumps(CONFIG_DATA, ensure_ascii=False, indent=2))
"""
        return template.replace("__JSON_BLOCK__", json_block)

    # ------------------------------------------------------------------
    # Joint helpers
    # ------------------------------------------------------------------

    def _assign_joint_lists(self, joints: List[Dict[str, object]]) -> None:
        self.joint_infos = joints or []
        limits: Dict[str, Tuple[Optional[float], Optional[float]]] = {}
        for joint in self.joint_infos:
            name = str(joint.get("name", ""))
            if not name:
                continue
            lower_val = joint.get("lower_limit")
            upper_val = joint.get("upper_limit")
            lower = float(lower_val) if isinstance(lower_val, (int, float)) else None
            upper = float(upper_val) if isinstance(upper_val, (int, float)) else None
            limits[name] = (lower, upper)
        self.joint_limits = limits
        self.dof_joint_infos = [joint for joint in self.joint_infos if joint.get("type") in JOINT_TYPES_WITH_DOF]
        self._seed_mapping_from_defaults()
        self._update_mapping_button_state()
        self._refresh_mapping_label()

    def _dof_joint_name_list(self) -> List[str]:
        names = [joint["name"] for joint in self.dof_joint_infos]
        return names

    def _clamp_joint_value(self, joint_name: str, value: float, *, margin: float = 1e-4) -> float:
        lower, upper = self.joint_limits.get(joint_name, (None, None))
        if lower is not None and upper is not None and lower > upper:
            lower, upper = upper, lower

        clamped = float(value)

        if lower is not None and clamped < lower:
            clamped = lower
        if upper is not None and clamped > upper:
            clamped = upper

        if lower is not None and upper is not None and math.isclose(lower, upper, abs_tol=margin):
            return (lower + upper) / 2.0

        if upper is not None and math.isclose(clamped, upper, abs_tol=margin):
            if lower is not None and upper - lower > 2 * margin:
                clamped = min(upper - margin, upper)
            else:
                clamped = upper - margin

        if lower is not None and math.isclose(clamped, lower, abs_tol=margin):
            if upper is not None and upper - lower > 2 * margin:
                clamped = max(lower + margin, lower)
            else:
                clamped = lower + margin

        if lower is not None:
            clamped = max(clamped, lower)
        if upper is not None:
            clamped = min(clamped, upper)

        if lower is None and upper is None and abs(clamped) > margin:
            clamped -= math.copysign(margin, clamped)

        return clamped


# 便于直接预览
if __name__ == "__main__":  # pragma: no cover - 手动测试入口
    app = QApplication([])
    widget = SimulationConfigWidget(lambda text: print(f"[状态] {text}"))
    widget.show()
    app.exec()
