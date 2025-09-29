"""
URDF Kitchen Assembler V3
=========================

A complete rebuild of the assembler focused on STL driven nodes and
connector nodes that keep SDF style joint definitions until the user
explicitly converts them to URDF.

Key differences versus V2:
- Part nodes expose their authoring points directly; no generic "in" port.
- Connections are represented by dedicated connector nodes sitting between
  parent and child parts. These boxes store the SDF style data and allow
  dynamic tweaks before URDF conversion.
- Joint computation happens in two phases: SDF capture (while editing) and
  URDF projection (when the user presses the "Calu" button).
"""
from __future__ import annotations

import math
import numpy as np
import vtk
import os
import signal
import sys
import traceback
import xml.etree.ElementTree as ET
import shutil
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
import json

from Qt import QtCore, QtGui, QtWidgets
from NodeGraphQt import BaseNode, NodeGraph

from stl_metadata import extract_metadata_text
from URDF_STLViewerWidget import STLViewerWidget


JOINT_TYPE_CHOICES = [
    ('revolute', '旋转关节'),
    ('continuous', '连续旋转'),
    ('prismatic', '滑动关节'),
    ('fixed', '固定连接'),
]
JOINT_TYPE_LABEL_MAP = {value: label for value, label in JOINT_TYPE_CHOICES}


# ---------------------------------------------------------------------------
# Metadata helpers
# ---------------------------------------------------------------------------

@dataclass
class PointDefinition:
    """Structure describing a connection point embedded in an STL."""

    index: int
    name: str
    position: Tuple[float, float, float]
    axis: Tuple[float, float, float]
    link_position: Optional[Tuple[float, float, float]] = None
    link_axis: Optional[Tuple[float, float, float]] = None

    def axis_length(self) -> float:
        x, y, z = self.axis
        return math.sqrt(x * x + y * y + z * z)


@dataclass
class PartMetadata:
    """Metadata loaded from an STL file prepared by the Parts Editor."""

    link_name: str
    origin_xyz: Tuple[float, float, float]
    origin_rpy: Tuple[float, float, float]
    points: List[PointDefinition] = field(default_factory=list)
    color_rgba: Tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)


def _parse_metadata_xml(xml_text: str) -> PartMetadata:
    """Parse the embedded metadata XML into a :class:`PartMetadata`."""

    root = ET.fromstring(xml_text)
    if root.tag != 'urdf_part':
        raise ValueError('Unexpected metadata root element: %s' % root.tag)

    link_elem = root.find('link')
    link_name = link_elem.get('name', 'unnamed_link') if link_elem is not None else 'unnamed_link'

    origin_xyz = (0.0, 0.0, 0.0)
    origin_rpy = (0.0, 0.0, 0.0)
    if link_elem is not None:
        origin_elem = link_elem.find('origin')
        if origin_elem is not None:
            origin_xyz = tuple(float(v) for v in origin_elem.get('xyz', '0 0 0').split())
            origin_rpy = tuple(float(v) for v in origin_elem.get('rpy', '0 0 0').split())

    color_rgba = (1.0, 1.0, 1.0, 1.0)
    material_elem = root.find('material')
    if material_elem is not None:
        color_elem = material_elem.find('color')
        if color_elem is not None:
            rgba = color_elem.get('rgba', '1 1 1 1').split()
            if len(rgba) == 4:
                color_rgba = tuple(float(v) for v in rgba)

    points: List[PointDefinition] = []
    for idx, point_elem in enumerate(root.findall('point')):
        name = point_elem.get('name', f'point{idx + 1}')
        xyz_elem = point_elem.find('point_xyz')
        if xyz_elem is not None and xyz_elem.text:
            xyz_values = tuple(float(v) for v in xyz_elem.text.strip().split())
        else:
            xyz_values = (0.0, 0.0, 0.0)
        axis_elem = point_elem.find('joint_axis')
        axis_values = (1.0, 0.0, 0.0)
        if axis_elem is not None:
            axis_values = tuple(float(v) for v in axis_elem.get('xyz', '1 0 0').split())
        points.append(PointDefinition(idx, name, xyz_values, axis_values))

    return PartMetadata(
        link_name=link_name,
        origin_xyz=origin_xyz,
        origin_rpy=origin_rpy,
        points=points,
        color_rgba=color_rgba,
    )


# ---------------------------------------------------------------------------
# Node classes
# ---------------------------------------------------------------------------


def _rotation_matrix_from_rpy(rpy: Tuple[float, float, float]) -> np.ndarray:
    roll, pitch, yaw = (float(r) for r in rpy)
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)

    rot_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    rot_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rot_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rot_z @ rot_y @ rot_x


def matrix_from_xyz_rpy_np(xyz: Tuple[float, float, float], rpy: Tuple[float, float, float]) -> np.ndarray:
    matrix = np.identity(4)
    matrix[:3, :3] = _rotation_matrix_from_rpy(rpy)
    matrix[:3, 3] = [float(v) for v in xyz]
    return matrix


def matrix_to_rpy_np(rotation: np.ndarray) -> Tuple[float, float, float]:
    sy = math.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
    singular = sy < 1e-9
    if not singular:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw

class PartNode(BaseNode):
    """Node representing an STL part with predefined connection points."""

    __identifier__ = 'urdf_kitchen.v3'
    NODE_NAME = 'Part'

    def __init__(self):
        super().__init__()
        self.metadata: Optional[PartMetadata] = None
        self.stl_path: Optional[str] = None
        self.link_name: str = ''
        self.set_color(60, 90, 140)
        self.link_to_mesh_matrix: np.ndarray = np.identity(4)
        self.mesh_to_link_matrix: np.ndarray = np.identity(4)
        self.default_color: List[float] = [1.0, 1.0, 1.0]

    # -- Configuration ----------------------------------------------------
    def configure(self, metadata: PartMetadata, stl_path: str) -> None:
        self.metadata = metadata
        self.stl_path = stl_path
        self.stl_file = stl_path
        self.link_name = metadata.link_name or self.name()
        self.default_color = list(metadata.color_rgba[:3])
        self.node_color = list(self.default_color)

        self.link_to_mesh_matrix = matrix_from_xyz_rpy_np(metadata.origin_xyz, metadata.origin_rpy)
        self.mesh_to_link_matrix = np.linalg.inv(self.link_to_mesh_matrix)

        rotation_mesh_to_link = self.mesh_to_link_matrix[:3, :3]
        for point in self.metadata.points:
            mesh_vec = np.append(np.array(point.position, dtype=float), 1.0)
            link_vec = self.mesh_to_link_matrix @ mesh_vec
            point.link_position = tuple(link_vec[:3])

            axis_vec = np.array(point.axis, dtype=float)
            link_axis = rotation_mesh_to_link @ axis_vec
            norm = np.linalg.norm(link_axis)
            if norm > 1e-9:
                link_axis = link_axis / norm
            point.link_axis = tuple(link_axis.tolist())

        self._configure_ports()

    # -- Port helpers -----------------------------------------------------
    def _configure_ports(self) -> None:
        """Create ports for all available connection points."""
        self._remove_all_ports()
        if not self.metadata:
            return
        for point in self.metadata.points:
            parent_port = f"P:{point.name}"
            child_port = f"C:{point.name}"
            self.add_output(parent_port)
            self.add_input(child_port)

    def _remove_all_ports(self) -> None:
        for port in list(self.input_ports()):
            self.delete_input(port)
        for port in list(self.output_ports()):
            self.delete_output(port)

    def find_point_by_port(self, port_name: str) -> Optional[PointDefinition]:
        if ':' not in port_name:
            return None
        _, point_name = port_name.split(':', 1)
        if not self.metadata:
            return None
        for point in self.metadata.points:
            if point.name == point_name:
                return point
        return None


class ConnectorNode(BaseNode):
    """A small node placed between parent and child parts to capture joint data."""

    __identifier__ = 'urdf_kitchen.v3'
    NODE_NAME = 'Connector'

    def __init__(self):
        super().__init__()
        self.set_color(180, 120, 60)
        self.add_input('parent')
        self.add_output('child')

        self._original_double_click = self.view.mouseDoubleClickEvent
        self.view.mouseDoubleClickEvent = self._handle_double_click

        # Connection bookkeeping
        self.parent_node_id: Optional[int] = None
        self.parent_point: Optional[str] = None
        self.parent_axis = (0.0, 0.0, 0.0)
        self.child_node_id: Optional[int] = None
        self.child_point: Optional[str] = None
        self.child_axis = (1.0, 0.0, 0.0)

        # SDF style data captured from the endpoints
        self.parent_local_xyz = (0.0, 0.0, 0.0)
        self.parent_local_rpy = (0.0, 0.0, 0.0)
        self.child_local_xyz = (0.0, 0.0, 0.0)
        self.child_local_rpy = (0.0, 0.0, 0.0)

        # User adjustable offsets which are applied during URDF projection
        self.offset_xyz = [0.0, 0.0, 0.0]
        self.offset_rpy = [0.0, 0.0, 0.0]

        self.joint_type = 'revolute'
        self.joint_name = ''
        self.joint_angle = 0.0
        self.joint_limit_lower = -math.pi
        self.joint_limit_upper = math.pi
        self.joint_effort = 0.0
        self.joint_velocity = 0.0

    # -- Reset helpers ---------------------------------------------------
    def clear_parent_binding(self) -> None:
        self.parent_node_id = None
        self.parent_point = None
        self.parent_local_xyz = (0.0, 0.0, 0.0)
        self.parent_local_rpy = (0.0, 0.0, 0.0)
        self.parent_axis = (0.0, 0.0, 0.0)

    def clear_child_binding(self) -> None:
        self.child_node_id = None
        self.child_point = None
        self.child_local_xyz = (0.0, 0.0, 0.0)
        self.child_local_rpy = (0.0, 0.0, 0.0)
        self.child_axis = (1.0, 0.0, 0.0)

    # Access helpers
    def is_fully_connected(self) -> bool:
        return self.parent_node_id is not None and self.child_node_id is not None

    def capture_from_parent(self, point: PointDefinition) -> None:
        self.parent_point = point.name
        source_xyz = point.link_position if point.link_position is not None else point.position
        self.parent_local_xyz = tuple(source_xyz)
        self.parent_local_rpy = (0.0, 0.0, 0.0)
        source_axis = point.link_axis if point.link_axis is not None else point.axis
        self.parent_axis = tuple(source_axis)

    def capture_from_child(self, point: PointDefinition) -> None:
        self.child_point = point.name
        source_xyz = point.link_position if point.link_position is not None else point.position
        self.child_local_xyz = tuple(source_xyz)
        self.child_local_rpy = (0.0, 0.0, 0.0)
        source_axis = point.link_axis if point.link_axis is not None else point.axis
        self.child_axis = tuple(source_axis)

    # -- UI handlers -----------------------------------------------------
    def _handle_double_click(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            dialog = ConnectorEditorDialog(self)
            dialog.exec()
        if callable(self._original_double_click):
            self._original_double_click(event)


class ConnectorEditorDialog(QtWidgets.QDialog):
    """Dialog that exposes connector joint parameters."""

    def __init__(self, connector: ConnectorNode):
        super().__init__(connector.view.window())
        self.connector = connector
        self.setWindowTitle('连接器设置')
        self.setModal(True)
        self.setMinimumWidth(320)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        info = QtWidgets.QGroupBox('连接信息')
        info_layout = QtWidgets.QFormLayout(info)
        info_layout.setSpacing(4)
        info_layout.setContentsMargins(8, 8, 8, 8)

        parent_label, parent_name = self._describe_endpoint('parent')
        child_label, child_name = self._describe_endpoint('child')
        info_layout.addRow('父节点：', QtWidgets.QLabel(parent_label))
        info_layout.addRow('子节点：', QtWidgets.QLabel(child_label))
        parent_axis = ', '.join(f'{c:.4f}' for c in connector.parent_axis)
        child_axis = ', '.join(f'{c:.4f}' for c in connector.child_axis)
        info_layout.addRow('父轴：', QtWidgets.QLabel(parent_axis))
        info_layout.addRow('子轴：', QtWidgets.QLabel(child_axis))
        layout.addWidget(info)

        # Joint name/type
        joint_box = QtWidgets.QGroupBox('关节设置')
        joint_form = QtWidgets.QFormLayout(joint_box)
        joint_form.setSpacing(4)
        joint_form.setContentsMargins(8, 8, 8, 8)

        default_joint_name = connector.joint_name or f"{parent_name}_{child_name}".strip('_')
        self.joint_name_edit = QtWidgets.QLineEdit(default_joint_name)
        joint_form.addRow('关节名称：', self.joint_name_edit)

        self.joint_type_combo = QtWidgets.QComboBox()
        current_type = connector.joint_type or 'revolute'
        for value, label in JOINT_TYPE_CHOICES:
            self.joint_type_combo.addItem(label, userData=value)
        index = self.joint_type_combo.findData(current_type)
        if index != -1:
            self.joint_type_combo.setCurrentIndex(index)
        joint_form.addRow('关节类型：', self.joint_type_combo)
        layout.addWidget(joint_box)

        # Offsets
        offset_box = QtWidgets.QGroupBox('URDF 偏移量')
        offset_layout = QtWidgets.QGridLayout(offset_box)
        offset_layout.setSpacing(4)
        offset_layout.setContentsMargins(8, 8, 8, 8)

        self.offset_xyz_edits = [self._make_spin(value) for value in connector.offset_xyz]
        self.offset_rpy_edits = [self._make_angle_spin(math.degrees(value)) for value in connector.offset_rpy]

        labels = ['X', 'Y', 'Z']
        for col, label in enumerate(labels):
            offset_layout.addWidget(QtWidgets.QLabel(label), 0, col + 1)

        offset_layout.addWidget(QtWidgets.QLabel('原点偏移 (m)'), 1, 0)
        for col, spin in enumerate(self.offset_xyz_edits):
            offset_layout.addWidget(spin, 1, col + 1)

        offset_layout.addWidget(QtWidgets.QLabel('RPY 偏移 (deg)'), 2, 0)
        for col, spin in enumerate(self.offset_rpy_edits):
            offset_layout.addWidget(spin, 2, col + 1)

        layout.addWidget(offset_box)

        button_box = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

    # -- Helpers ---------------------------------------------------------
    def _make_spin(self, value: float) -> QtWidgets.QDoubleSpinBox:
        spin = QtWidgets.QDoubleSpinBox()
        spin.setDecimals(5)
        spin.setRange(-1000.0, 1000.0)
        spin.setSingleStep(0.001)
        spin.setValue(float(value))
        spin.setAlignment(QtCore.Qt.AlignRight)
        return spin

    def _make_angle_spin(self, value: float) -> QtWidgets.QDoubleSpinBox:
        spin = QtWidgets.QDoubleSpinBox()
        spin.setDecimals(3)
        spin.setRange(-3600.0, 3600.0)
        spin.setSingleStep(0.1)
        spin.setValue(float(value))
        spin.setAlignment(QtCore.Qt.AlignRight)
        spin.setSuffix(' deg')
        return spin

    def _describe_endpoint(self, kind: str) -> Tuple[str, str]:
        connector = self.connector
        graph = connector.graph if hasattr(connector, 'graph') else None
        if kind == 'parent':
            node_id = connector.parent_node_id
            point_name = connector.parent_point or ''
        else:
            node_id = connector.child_node_id
            point_name = connector.child_point or ''
        if graph and node_id:
            node = graph.get_node_by_id(node_id)
            node_name = node.name() if node else 'unknown'
        else:
            node_name = 'unassigned'
        if not point_name:
            point_name = '-'
        return f"{node_name} · {point_name}", node_name

    def accept(self) -> None:
        self.connector.joint_name = self.joint_name_edit.text().strip()
        self.connector.joint_type = self.joint_type_combo.currentData()
        self.connector.offset_xyz = [spin.value() for spin in self.offset_xyz_edits]
        self.connector.offset_rpy = [math.radians(spin.value()) for spin in self.offset_rpy_edits]
        graph = getattr(self.connector, 'graph', None)
        if graph and hasattr(graph, 'recalculate_transforms'):
            graph.recalculate_transforms()
            panel = getattr(graph, 'joint_panel', None)
            if panel and hasattr(panel, 'refresh_from_graph'):
                panel.refresh_from_graph(graph)
        super().accept()


class JointPanelV3(QtWidgets.QWidget):
    """Side panel listing connectors with live joint controls."""

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.graph: Optional[AssemblerGraphV3] = None
        self.rows: Dict[ConnectorNode, Dict[str, QtWidgets.QWidget]] = {}

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        title = QtWidgets.QLabel('关节控制')
        font = title.font()
        font.setBold(True)
        title.setFont(font)
        layout.addWidget(title)

        self.scroll = QtWidgets.QScrollArea()
        self.scroll.setWidgetResizable(True)
        layout.addWidget(self.scroll, 1)

        self.container = QtWidgets.QWidget()
        self.scroll.setWidget(self.container)
        self.container_layout = QtWidgets.QVBoxLayout(self.container)
        self.container_layout.setContentsMargins(0, 0, 0, 0)
        self.container_layout.setSpacing(6)

        refresh_btn = QtWidgets.QPushButton('刷新关节列表')
        refresh_btn.clicked.connect(self._manual_refresh)
        layout.addWidget(refresh_btn)

    # -- Refresh ---------------------------------------------------------
    def refresh_from_graph(self, graph: AssemblerGraphV3):
        self.graph = graph
        self._clear_rows()
        for connector in graph.connector_nodes:
            if not connector.is_fully_connected():
                continue
            self._add_row(connector)
        self.container_layout.addStretch(1)

    def _manual_refresh(self):
        if self.graph:
            self.refresh_from_graph(self.graph)

    def _clear_rows(self):
        while self.container_layout.count():
            item = self.container_layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()
        self.rows.clear()

    def _add_row(self, connector: ConnectorNode):
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(4)

        # Ensure newly added attributes exist for legacy graphs
        if not hasattr(connector, 'joint_limit_lower'):
            connector.joint_limit_lower = -math.pi
        if not hasattr(connector, 'joint_limit_upper'):
            connector.joint_limit_upper = math.pi
        if not hasattr(connector, 'joint_effort'):
            connector.joint_effort = 0.0
        if not hasattr(connector, 'joint_velocity'):
            connector.joint_velocity = 0.0

        parent_graph = connector.graph
        parent_name = 'base_link'
        if parent_graph and connector.parent_node_id:
            parent_node = parent_graph.get_node_by_id(connector.parent_node_id)
            if parent_node:
                parent_name = getattr(parent_node, 'link_name', parent_node.name())
        child_name = 'child'
        if connector.graph and connector.child_node_id:
            child_node = connector.graph.get_node_by_id(connector.child_node_id)
            if child_node:
                child_name = getattr(child_node, 'link_name', child_node.name())

        joint_label = JOINT_TYPE_LABEL_MAP.get(connector.joint_type, connector.joint_type)
        label = QtWidgets.QLabel(f'{parent_name} → {child_name}（{joint_label}）')
        layout.addWidget(label)

        angle_layout = QtWidgets.QHBoxLayout()
        angle_layout.setSpacing(4)
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(-1800, 1800)
        slider.setSingleStep(5)
        slider.setValue(0)
        angle_layout.addWidget(slider, 1)

        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(-100.0, 100.0)
        spin.setDecimals(4)
        spin.setSingleStep(0.01)
        spin.setValue(connector.joint_angle)
        angle_layout.addWidget(spin)

        layout.addLayout(angle_layout)

        limit_layout = QtWidgets.QGridLayout()
        limit_layout.setHorizontalSpacing(6)
        limit_layout.setVerticalSpacing(4)

        use_degrees = connector.joint_type in ('revolute', 'continuous')
        range_suffix = ' rad'
        if connector.joint_type == 'prismatic':
            range_suffix = ' m'
        elif use_degrees:
            range_suffix = ' deg'

        lower_spin = QtWidgets.QDoubleSpinBox()
        if use_degrees:
            lower_spin.setRange(-3600.0, 3600.0)
            lower_spin.setDecimals(3)
            lower_spin.setSingleStep(0.1)
        else:
            lower_spin.setRange(-1000.0, 1000.0)
            lower_spin.setDecimals(4)
            lower_spin.setSingleStep(0.01)
        lower_spin.setSuffix(range_suffix)

        upper_spin = QtWidgets.QDoubleSpinBox()
        if use_degrees:
            upper_spin.setRange(-3600.0, 3600.0)
            upper_spin.setDecimals(3)
            upper_spin.setSingleStep(0.1)
        else:
            upper_spin.setRange(-1000.0, 1000.0)
            upper_spin.setDecimals(4)
            upper_spin.setSingleStep(0.01)
        upper_spin.setSuffix(range_suffix)

        effort_spin = QtWidgets.QDoubleSpinBox()
        effort_spin.setRange(0.0, 10000.0)
        effort_spin.setDecimals(3)
        effort_spin.setSingleStep(0.1)
        effort_spin.setValue(max(0.0, float(connector.joint_effort)))

        velocity_spin = QtWidgets.QDoubleSpinBox()
        velocity_spin.setRange(0.0, 1000.0)
        velocity_spin.setDecimals(3)
        velocity_spin.setSingleStep(0.1)
        velocity_spin.setValue(max(0.0, float(connector.joint_velocity)))

        if use_degrees:
            limit_tip = '此处以度数输入，导出 URDF 时将自动转换为弧度。'
            angle_tip = '滑块以度为单位，数值会自动转换为弧度并写入 URDF。'
        elif connector.joint_type == 'prismatic':
            limit_tip = '位移上下限（米），将直接写入 URDF。'
            angle_tip = '当前关节位移（米），与下方滑块联动。'
        else:
            limit_tip = '角度上下限（弧度），将直接写入 URDF。'
            angle_tip = '当前关节角度（弧度），与下方滑块联动。'

        if connector.joint_type == 'continuous':
            limit_tip = '连续旋转关节在 URDF 中不会写入上下限。'

        lower_spin.setToolTip(limit_tip)
        upper_spin.setToolTip(limit_tip)
        slider.setToolTip(angle_tip)
        spin.setToolTip(angle_tip)
        effort_spin.setToolTip('施加在该关节上的最大力矩/力。')
        velocity_spin.setToolTip('关节的最大速度，用于 URDF 限幅。')

        if connector.joint_type == 'continuous':
            lower_spin.setEnabled(False)
            upper_spin.setEnabled(False)

        limit_layout.addWidget(QtWidgets.QLabel('下限'), 0, 0)
        limit_layout.addWidget(lower_spin, 0, 1)
        limit_layout.addWidget(QtWidgets.QLabel('上限'), 0, 2)
        limit_layout.addWidget(upper_spin, 0, 3)
        limit_layout.addWidget(QtWidgets.QLabel('力矩/力'), 1, 0)
        limit_layout.addWidget(effort_spin, 1, 1)
        limit_layout.addWidget(QtWidgets.QLabel('速度'), 1, 2)
        limit_layout.addWidget(velocity_spin, 1, 3)

        layout.addLayout(limit_layout)

        slider_scale = 10.0 if use_degrees else 1000.0

        def _display_value(value: float) -> float:
            return math.degrees(value) if use_degrees else float(value)

        def _slider_to_value(position: int) -> float:
            raw = position / slider_scale
            return math.radians(raw) if use_degrees else raw

        def _value_to_slider(value: float) -> int:
            raw = _display_value(value)
            return int(round(raw * slider_scale))

        def _set_spin_value(box: QtWidgets.QDoubleSpinBox, value: float) -> None:
            box.blockSignals(True)
            box.setValue(value)
            box.blockSignals(False)

        def _current_limits() -> Tuple[float, float]:
            lower_val = float(connector.joint_limit_lower)
            upper_val = float(connector.joint_limit_upper)
            if lower_val > upper_val:
                return upper_val, lower_val
            return lower_val, upper_val

        def _clamp_to_limits(value: float) -> float:
            lower_val, upper_val = _current_limits()
            return min(max(value, lower_val), upper_val)

        def _trigger_recalc(immediate: bool = False) -> None:
            graph = connector.graph
            if not graph:
                return
            if not immediate:
                scheduler = getattr(graph, 'schedule_recalculate_transforms', None)
                if callable(scheduler):
                    scheduler()
                    return
            graph.recalculate_transforms()

        def _update_slider_range() -> None:
            lower_val, upper_val = _current_limits()
            slider_min = _value_to_slider(lower_val)
            slider_max = _value_to_slider(upper_val)
            if slider_min == slider_max:
                delta = max(1, int(round(slider_scale)))
                slider_min -= delta
                slider_max += delta
            slider.blockSignals(True)
            slider.setRange(min(slider_min, slider_max), max(slider_min, slider_max))
            slider.blockSignals(False)

        def _sync_angle_widgets(angle: float) -> None:
            slider.blockSignals(True)
            slider.setValue(_value_to_slider(angle))
            slider.blockSignals(False)
            spin.blockSignals(True)
            spin.setValue(angle)
            spin.blockSignals(False)

        def _apply_angle(angle: float, trigger_transform: bool = True) -> None:
            clamped = _clamp_to_limits(angle)
            if not math.isclose(clamped, connector.joint_angle, rel_tol=1e-9, abs_tol=1e-9):
                connector.joint_angle = clamped
                _sync_angle_widgets(clamped)
                if trigger_transform:
                    _trigger_recalc()
            else:
                _sync_angle_widgets(clamped)

        def on_slider(val):
            target = _slider_to_value(val)
            clamped = _clamp_to_limits(target)
            if not math.isclose(target, clamped, rel_tol=1e-9, abs_tol=1e-9):
                slider_target = _value_to_slider(clamped)
                if slider.value() != slider_target:
                    slider.blockSignals(True)
                    slider.setValue(slider_target)
                    slider.blockSignals(False)
            if not math.isclose(spin.value(), clamped, rel_tol=1e-9, abs_tol=1e-9):
                spin.blockSignals(True)
                spin.setValue(clamped)
                spin.blockSignals(False)
            if not math.isclose(connector.joint_angle, clamped, rel_tol=1e-9, abs_tol=1e-9):
                connector.joint_angle = clamped
                _trigger_recalc()

        def on_spin(val):
            clamped = _clamp_to_limits(val)
            if not math.isclose(val, clamped, rel_tol=1e-9, abs_tol=1e-9):
                spin.blockSignals(True)
                spin.setValue(clamped)
                spin.blockSignals(False)
            slider_target = _value_to_slider(clamped)
            if slider.value() != slider_target:
                slider.blockSignals(True)
                slider.setValue(slider_target)
                slider.blockSignals(False)
            if not math.isclose(connector.joint_angle, clamped, rel_tol=1e-9, abs_tol=1e-9):
                connector.joint_angle = clamped
                _trigger_recalc()

        def on_lower(val):
            raw = math.radians(val) if use_degrees else float(val)
            connector.joint_limit_lower = raw
            if connector.joint_limit_upper < connector.joint_limit_lower:
                connector.joint_limit_upper = raw
                _set_spin_value(upper_spin, _display_value(raw))
            _update_slider_range()
            _apply_angle(connector.joint_angle)

        def on_upper(val):
            raw = math.radians(val) if use_degrees else float(val)
            connector.joint_limit_upper = raw
            if connector.joint_limit_upper < connector.joint_limit_lower:
                connector.joint_limit_lower = raw
                _set_spin_value(lower_spin, _display_value(raw))
            _update_slider_range()
            _apply_angle(connector.joint_angle)

        def on_effort(val):
            connector.joint_effort = float(val)

        def on_velocity(val):
            connector.joint_velocity = float(val)

        _set_spin_value(lower_spin, _display_value(connector.joint_limit_lower))
        _set_spin_value(upper_spin, _display_value(connector.joint_limit_upper))
        _update_slider_range()
        _apply_angle(connector.joint_angle, trigger_transform=False)

        slider.valueChanged.connect(on_slider)
        spin.valueChanged.connect(on_spin)

        lower_spin.valueChanged.connect(on_lower)
        upper_spin.valueChanged.connect(on_upper)
        effort_spin.valueChanged.connect(on_effort)
        velocity_spin.valueChanged.connect(on_velocity)

        if connector.joint_type == 'fixed':
            slider.setEnabled(False)
            spin.setEnabled(False)
            lower_spin.setEnabled(False)
            upper_spin.setEnabled(False)
            effort_spin.setEnabled(False)
            velocity_spin.setEnabled(False)

        self.container_layout.addWidget(widget)
        self.rows[connector] = {
            'slider': slider,
            'spin': spin,
            'lower': lower_spin,
            'upper': upper_spin,
            'effort': effort_spin,
            'velocity': velocity_spin,
        }
class BaseLinkNode(BaseNode):
    """Root node representing the URDF base link."""

    __identifier__ = 'urdf_kitchen.v3'
    NODE_NAME = 'BaseLink'

    def __init__(self):
        super().__init__()
        self.set_color(40, 120, 60)
        self.add_output('root')
        self.link_name = 'base_link'


# ---------------------------------------------------------------------------
# Graph implementation
# ---------------------------------------------------------------------------

class AssemblerGraphV3(NodeGraph):
    """Custom graph that manages part and connector nodes."""

    def __init__(self):
        super().__init__()
        self.widget.setWindowTitle('URDF 装配器 V3')
        self.base_node: Optional[BaseLinkNode] = None
        self.stl_viewer: Optional[STLViewerWidget] = None
        self.joint_panel = None
        self.preview_color_callback = None
        self._suspend_updates = False
        self._pending_recalc = False
        self._pending_notify = False
        self._recalc_timer = QtCore.QTimer()
        self._recalc_timer.setSingleShot(True)
        self._recalc_timer.timeout.connect(self._on_recalc_timeout)

        # Register node classes
        for node_cls in (BaseLinkNode, PartNode, ConnectorNode):
            self.register_node(node_cls)

        # Track connectors for quick lookup
        self.connector_nodes: List[ConnectorNode] = []
        self._loading_project = False
        debug_flag = os.environ.get('FLUXHWEAVE_DEBUG', '1')
        self.debug_logging = str(debug_flag).strip().lower() not in ('', '0', 'false', 'off', 'no')

        self.port_connected.connect(self._on_port_connected)
        self.port_disconnected.connect(self._on_port_disconnected)

        self._ensure_base_node()
        self._install_context_menu()

    # -- Node management -------------------------------------------------
    def set_debug_logging(self, enabled: bool) -> None:
        self.debug_logging = bool(enabled)

    def _debug(self, message: str) -> None:
        if self.debug_logging:
            print(f'[AssemblerGraphV3] {message}')

    def _validate_connector_links(self) -> List[str]:
        issues: List[str] = []
        for connector in list(self.connector_nodes):
            if connector not in self.all_nodes():
                continue
            parent_expected = connector.parent_node_id is not None
            child_expected = connector.child_node_id is not None
            parent_connected = False
            child_connected = False
            try:
                parent_port = connector.get_input('parent')
            except Exception:
                parent_port = None
            if parent_port:
                try:
                    parent_connected = any(parent_port.connected_ports())
                except Exception:
                    parent_connected = False
            try:
                child_port = connector.get_output('child')
            except Exception:
                child_port = None
            if child_port:
                try:
                    child_connected = any(child_port.connected_ports())
                except Exception:
                    child_connected = False
            if parent_expected and not parent_connected:
                issues.append(
                    f"Connector '{connector.name()}' expects parent node {connector.parent_node_id} but port has no connection"
                )
            if child_expected and not child_connected:
                issues.append(
                    f"Connector '{connector.name()}' expects child node {connector.child_node_id} but port has no connection"
                )
        return issues

    def debug_dump_state(self, label: str = '') -> None:
        if not self.debug_logging:
            return
        header = f'--- Graph State {label} ---' if label else '--- Graph State ---'
        print(f'[AssemblerGraphV3] {header}')
        nodes = list(self.all_nodes())
        print(f'[AssemblerGraphV3] Total nodes: {len(nodes)} (connectors: {len(self.connector_nodes)})')
        for connector in self.connector_nodes:
            if connector not in nodes:
                continue
            parent_info = 'None'
            child_info = 'None'
            parent_connected = 0
            child_connected = 0
            try:
                parent_port = connector.get_input('parent')
                if parent_port:
                    parent_connected = len(parent_port.connected_ports())
            except Exception:
                parent_port = None
            try:
                child_port = connector.get_output('child')
                if child_port:
                    child_connected = len(child_port.connected_ports())
            except Exception:
                child_port = None

            if connector.parent_node_id is not None:
                parent_info = f'id={connector.parent_node_id}, point={connector.parent_point}'
            if connector.child_node_id is not None:
                child_info = f'id={connector.child_node_id}, point={connector.child_point}'

            print(
                f"[AssemblerGraphV3]   Connector '{connector.name()}': parent({parent_info}) connections={parent_connected}; "
                f"child({child_info}) connections={child_connected}"
            )
        issues = self._validate_connector_links()
        if issues:
            print('[AssemblerGraphV3] Connector validation warnings:')
            for issue in issues:
                print(f'[AssemblerGraphV3]   WARNING: {issue}')
        else:
            print('[AssemblerGraphV3] Connector validation passed (all expected links present).')

        connection_count = 0
        for node in nodes:
            for port in node.output_ports():
                connection_count += len(port.connected_ports())
        print(f'[AssemblerGraphV3] Total output→input connections observed: {connection_count}')

    def _connect_ports(self, out_port, in_port, *, emit_signal=True) -> bool:
        if not out_port or not in_port:
            return False
        try:
            out_port.connect_to(in_port, push_undo=False, emit_signal=emit_signal)
            return True
        except Exception as exc:
            self._debug(f"_connect_ports failed: {exc}")
            return False

    def _on_recalc_timeout(self) -> None:
        if self._suspend_updates:
            self._pending_recalc = True
            return
        self.recalculate_transforms()

    def schedule_recalculate_transforms(self, delay_ms: int = 16) -> None:
        if self._suspend_updates:
            self._pending_recalc = True
            return
        delay_ms = max(0, int(delay_ms))
        self._recalc_timer.start(delay_ms)

    def _ensure_base_node(self) -> None:
        if self.base_node and self.base_node in self.all_nodes():
            return
        self.base_node = self.create_node('urdf_kitchen.v3.BaseLinkNode', name='base_link', pos=(0, 0))

    def add_part(self, stl_path: str) -> PartNode:
        metadata = self._load_metadata(stl_path)
        node_name = os.path.splitext(os.path.basename(stl_path))[0]
        part_node: PartNode = self.create_node(
            'urdf_kitchen.v3.PartNode',
            name=node_name,
            pos=(200, len(self.all_nodes()) * 120),
        )
        if isinstance(part_node, PartNode):
            part_node.configure(metadata, stl_path)
            if self.stl_viewer:
                self.stl_viewer.load_stl_for_node(part_node)
        self.recalculate_transforms()
        self._notify_structure_changed()
        return part_node

    def add_connector(self, pos: Tuple[int, int] = (0, 0)) -> ConnectorNode:
        connector: ConnectorNode = self.create_node('urdf_kitchen.v3.ConnectorNode', name='connector', pos=pos)
        self.connector_nodes.append(connector)
        self.recalculate_transforms()
        self._notify_structure_changed()
        return connector

    def _install_context_menu(self) -> None:
        try:
            self.widget.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
            self.widget.customContextMenuRequested.connect(self._show_context_menu)
        except Exception as exc:
            print(f"Warning: failed to install context menu: {exc}")

    def _show_context_menu(self, pos):
        self._maybe_select_node_at(pos)

        menu = QtWidgets.QMenu(self.widget)
        selected = list(self.selected_nodes())
        deletable = [n for n in selected if not isinstance(n, BaseLinkNode)]

        act_delete = menu.addAction('Delete Node(s)')
        act_delete.setEnabled(bool(deletable))

        action = menu.exec_(self.widget.mapToGlobal(pos))
        if action == act_delete:
            for node in deletable:
                self.remove_node(node)

    def _maybe_select_node_at(self, pos: QtCore.QPoint) -> None:
        try:
            scene_pos = self.widget.mapToScene(pos)
            items = self.widget.scene().items(scene_pos)
        except Exception:
            return

        target_node = None
        for item in items:
            candidate = getattr(item, 'node', None)
            if isinstance(candidate, BaseNode):
                target_node = candidate
                break
            candidate_port = getattr(item, 'port', None)
            if candidate_port is not None:
                try:
                    port_node = candidate_port.node()
                except Exception:
                    port_node = getattr(candidate_port, 'node', None)
                if isinstance(port_node, BaseNode):
                    target_node = port_node
                    break
            candidate = getattr(item, 'model', None)
            if hasattr(candidate, 'node') and isinstance(candidate.node, BaseNode):
                target_node = candidate.node
                break

        if target_node is None:
            # fallback: check all node bounding rectangles
            try:
                scene_point = QtCore.QPointF(scene_pos)
            except Exception:
                scene_point = scene_pos
            for node in self.all_nodes():
                try:
                    geom = node.view.sceneBoundingRect()
                except Exception:
                    continue
                if geom.contains(scene_point):
                    target_node = node
                    break

        if target_node is None:
            return

        if target_node not in self.selected_nodes():
            for node in self.selected_nodes():
                node.set_selected(False)
            target_node.set_selected(True)

    def remove_node(self, node):
        if isinstance(node, BaseLinkNode):
            print('Base link cannot be removed.')
            return

        pending_cleanup: List[ConnectorNode] = []
        if isinstance(node, ConnectorNode):
            pending_cleanup.append(node)
        else:
            for connector in list(self.connector_nodes):
                if connector.parent_node_id == node.id or connector.child_node_id == node.id:
                    pending_cleanup.append(connector)

        removed_ids = set()
        for connector in pending_cleanup:
            if connector in self.connector_nodes:
                self.connector_nodes.remove(connector)
            try:
                super().remove_node(connector)
                removed_ids.add(connector.id)
            except Exception as exc:
                print(f"Warning: unable to remove connector {connector.name()}: {exc}")

        if node in self.connector_nodes:
            self.connector_nodes.remove(node)

        try:
            if node.id not in removed_ids:
                if isinstance(node, PartNode) and self.stl_viewer:
                    self.stl_viewer.remove_stl_for_node(node)
                super().remove_node(node)
        finally:
            self.recalculate_transforms()
            self._notify_structure_changed()

    # -- Connection handling --------------------------------------------
    def _on_port_connected(self, input_port, output_port):
        parent_node = output_port.node()
        child_node = input_port.node()
        loading_project = getattr(self, '_loading_project', False)

        if isinstance(parent_node, PartNode) and isinstance(child_node, ConnectorNode):
            if loading_project and getattr(child_node, '_restored_parent_from_payload', False):
                child_node.parent_node_id = parent_node.id
                return
            self._connect_parent_to_connector(parent_node, output_port, child_node)
        elif isinstance(parent_node, ConnectorNode) and isinstance(child_node, PartNode):
            if loading_project and getattr(parent_node, '_restored_child_from_payload', False):
                parent_node.child_node_id = child_node.id
                return
            self._connect_connector_to_child(parent_node, child_node, input_port)
        elif isinstance(parent_node, BaseLinkNode) and isinstance(child_node, ConnectorNode):
            child_node.parent_node_id = parent_node.id
            child_node.parent_point = 'root'
        else:
            print('Unsupported connection type for V3 graph')

    def _on_port_disconnected(self, input_port, output_port):
        parent_node = output_port.node()
        child_node = input_port.node()

        if isinstance(parent_node, PartNode) and isinstance(child_node, ConnectorNode):
            child_node.clear_parent_binding()
        elif isinstance(parent_node, ConnectorNode) and isinstance(child_node, PartNode):
            parent_node.clear_child_binding()
        elif isinstance(parent_node, BaseLinkNode) and isinstance(child_node, ConnectorNode):
            child_node.clear_parent_binding()
        self.recalculate_transforms()
        self._notify_structure_changed()

    def _connect_parent_to_connector(self, parent: PartNode, output_port, connector: ConnectorNode):
        point = parent.find_point_by_port(output_port.name())
        if not point:
            print('Unable to determine parent point from port', output_port.name())
            return
        connector.clear_parent_binding()
        connector.parent_node_id = parent.id
        connector.capture_from_parent(point)

    def _connect_connector_to_child(self, connector: ConnectorNode, child: PartNode, input_port):
        point = child.find_point_by_port(input_port.name())
        if not point:
            print('Unable to determine child point from port', input_port.name())
            return
        connector.clear_child_binding()
        connector.child_node_id = child.id
        connector.capture_from_child(point)
        self.recalculate_transforms()
        self._notify_structure_changed()

    # -- Metadata loading ------------------------------------------------
    def _load_metadata(self, stl_path: str) -> PartMetadata:
        xml_text = extract_metadata_text(stl_path)
        if not xml_text:
            raise ValueError(f'STL file has no embedded URDF metadata: {stl_path}')
        return _parse_metadata_xml(xml_text)

    # -- Notifications ---------------------------------------------------
    def _notify_structure_changed(self):
        if self._suspend_updates:
            self._pending_notify = True
            return
        panel = getattr(self, 'joint_panel', None)
        if panel and hasattr(panel, 'refresh_from_graph'):
            panel.refresh_from_graph(self)
        callback = getattr(self, 'preview_color_callback', None)
        if callable(callback):
            callback()

    def begin_batch_update(self) -> None:
        self._suspend_updates = True
        self._pending_recalc = False
        self._pending_notify = False

    def end_batch_update(self) -> None:
        recalc_needed = self._pending_recalc
        notify_needed = self._pending_notify
        self._suspend_updates = False
        self._pending_recalc = False
        self._pending_notify = False
        if recalc_needed:
            self.recalculate_transforms()
        if notify_needed:
            self._notify_structure_changed()

    def reset_graph(self) -> None:
        for connector in list(self.connector_nodes):
            self.remove_node(connector)
        for node in list(self.all_nodes()):
            if isinstance(node, PartNode):
                if self.stl_viewer:
                    self.stl_viewer.remove_stl_for_node(node)
                self.remove_node(node)
        self.connector_nodes = [c for c in self.connector_nodes if c in self.all_nodes()]
        if self.base_node and self.base_node in self.all_nodes():
            try:
                self.base_node.set_name('base_link')
                self.base_node.link_name = 'base_link'
                self.base_node.set_pos(0, 0)
            except Exception:
                pass
        else:
            self._ensure_base_node()

    @staticmethod
    def _node_pos_list(node: BaseNode) -> List[float]:
        try:
            pos = node.pos()
            if hasattr(pos, 'x'):
                return [float(pos.x()), float(pos.y())]
            if isinstance(pos, (tuple, list)) and len(pos) >= 2:
                return [float(pos[0]), float(pos[1])]
        except Exception:
            pass
        return [0.0, 0.0]

    def serialize_project(self, base_dir: str) -> Dict[str, Any]:
        def _relative_path(path: str) -> str:
            if not path:
                return ''
            abs_path = os.path.abspath(path)
            try:
                rel_path = os.path.relpath(abs_path, base_dir)
                return rel_path
            except Exception:
                return abs_path

        nodes_data: List[Dict[str, Any]] = []
        if self.base_node and self.base_node in self.all_nodes():
            base = self.base_node
            nodes_data.append(
                {
                    'uid': str(base.id),
                    'type': 'base',
                    'name': base.name(),
                    'link_name': getattr(base, 'link_name', 'base_link'),
                    'pos': self._node_pos_list(base),
                }
            )

        for node in self.all_nodes():
            if node is self.base_node:
                continue
            if isinstance(node, PartNode):
                stl_path = getattr(node, 'stl_file', '') or ''
                nodes_data.append(
                    {
                        'uid': str(node.id),
                        'type': 'part',
                        'name': node.name(),
                        'link_name': getattr(node, 'link_name', node.name()),
                        'stl_path': _relative_path(stl_path),
                        'pos': self._node_pos_list(node),
                    }
                )
            elif isinstance(node, ConnectorNode):
                connector_payload: Dict[str, Any] = {
                    'uid': str(node.id),
                    'type': 'connector',
                    'name': node.name(),
                    'pos': self._node_pos_list(node),
                    'joint_type': node.joint_type,
                    'joint_name': node.joint_name,
                    'joint_angle': float(node.joint_angle),
                    'offset_xyz': [float(v) for v in node.offset_xyz],
                    'offset_rpy': [float(v) for v in node.offset_rpy],
                    'joint_limit_lower': float(node.joint_limit_lower),
                    'joint_limit_upper': float(node.joint_limit_upper),
                    'joint_effort': float(node.joint_effort),
                    'joint_velocity': float(node.joint_velocity),
                    'parent_local_xyz': [float(v) for v in node.parent_local_xyz],
                    'parent_local_rpy': [float(v) for v in node.parent_local_rpy],
                    'parent_axis': [float(v) for v in node.parent_axis],
                    'child_local_xyz': [float(v) for v in node.child_local_xyz],
                    'child_local_rpy': [float(v) for v in node.child_local_rpy],
                    'child_axis': [float(v) for v in node.child_axis],
                }

                parent_binding: Dict[str, Any] = {}
                if node.parent_node_id is not None:
                    parent_binding['node_uid'] = str(node.parent_node_id)
                if node.parent_point:
                    parent_binding['point'] = node.parent_point
                if parent_binding:
                    connector_payload['parent_binding'] = parent_binding

                child_binding: Dict[str, Any] = {}
                if node.child_node_id is not None:
                    child_binding['node_uid'] = str(node.child_node_id)
                if node.child_point:
                    child_binding['point'] = node.child_point
                if child_binding:
                    connector_payload['child_binding'] = child_binding

                nodes_data.append(connector_payload)

        connections: List[Dict[str, Any]] = []
        for node in self.all_nodes():
            for port in node.output_ports():
                for connected_port in port.connected_ports():
                    connections.append(
                        {
                            'from_uid': str(node.id),
                            'from_port': port.name(),
                            'to_uid': str(connected_port.node().id),
                            'to_port': connected_port.name(),
                        }
                    )

        self.debug_dump_state('serialize_project (pre-save)')

        return {
            'nodes': nodes_data,
            'connections': connections,
        }

    def deserialize_project(self, payload: Dict[str, Any], base_dir: str) -> None:
        self.begin_batch_update()
        try:
            self.reset_graph()

            node_map: Dict[str, BaseNode] = {}
            connector_configs: List[Tuple[ConnectorNode, Dict[str, Any]]] = []

            def _tuple_from_values(values, size: int) -> Tuple[float, ...]:
                result: List[float] = []
                if isinstance(values, (list, tuple)):
                    for idx in range(min(len(values), size)):
                        try:
                            result.append(float(values[idx]))
                        except Exception:
                            result.append(0.0)
                while len(result) < size:
                    result.append(0.0)
                return tuple(result)

            for node_info in payload.get('nodes', []):
                node_type = node_info.get('type')
                uid = str(node_info.get('uid')) if node_info.get('uid') is not None else None
                if not uid:
                    continue

                if node_type == 'base':
                    base = self.base_node
                    if not base:
                        self._ensure_base_node()
                        base = self.base_node
                    if base:
                        if node_info.get('name'):
                            base.set_name(node_info['name'])
                        if node_info.get('link_name'):
                            base.link_name = node_info['link_name']
                        pos = node_info.get('pos')
                        if pos and len(pos) == 2:
                            base.set_pos(float(pos[0]), float(pos[1]))
                        node_map[uid] = base
                elif node_type == 'part':
                    stl_path = node_info.get('stl_path', '') or ''
                    if stl_path and not os.path.isabs(stl_path):
                        stl_path = os.path.normpath(os.path.join(base_dir, stl_path))
                    if not stl_path or not os.path.exists(stl_path):
                        raise FileNotFoundError(f'STL file not found: {stl_path}')
                    part_node = self.add_part(stl_path)
                    if node_info.get('name'):
                        part_node.set_name(node_info['name'])
                    if node_info.get('link_name'):
                        part_node.link_name = node_info['link_name']
                    pos = node_info.get('pos')
                    if pos and len(pos) == 2:
                        part_node.set_pos(float(pos[0]), float(pos[1]))
                    node_map[uid] = part_node
                elif node_type == 'connector':
                    pos = node_info.get('pos', [0.0, 0.0])
                    base_pos = (
                        int(pos[0]) if isinstance(pos[0], (int, float)) else 0,
                        int(pos[1]) if isinstance(pos[1], (int, float)) else 0,
                    )
                    connector_node = self.add_connector(base_pos)
                    if node_info.get('name'):
                        connector_node.set_name(node_info['name'])
                    if pos and len(pos) == 2:
                        connector_node.set_pos(float(pos[0]), float(pos[1]))
                    node_map[uid] = connector_node
                    connector_configs.append((connector_node, node_info))

            for connector_node, cfg in connector_configs:
                connector_node.joint_type = cfg.get('joint_type', connector_node.joint_type)
                connector_node.joint_name = cfg.get('joint_name', connector_node.joint_name)
                if 'offset_xyz' in cfg:
                    connector_node.offset_xyz = [float(v) for v in (cfg.get('offset_xyz') or connector_node.offset_xyz)]
                if 'offset_rpy' in cfg:
                    connector_node.offset_rpy = [float(v) for v in (cfg.get('offset_rpy') or connector_node.offset_rpy)]
                connector_node.joint_limit_lower = float(cfg.get('joint_limit_lower', connector_node.joint_limit_lower))
                connector_node.joint_limit_upper = float(cfg.get('joint_limit_upper', connector_node.joint_limit_upper))
                connector_node.joint_effort = float(cfg.get('joint_effort', connector_node.joint_effort))
                connector_node.joint_velocity = float(cfg.get('joint_velocity', connector_node.joint_velocity))
                angle_val = float(cfg.get('joint_angle', connector_node.joint_angle))
                lower_val = connector_node.joint_limit_lower
                upper_val = connector_node.joint_limit_upper
                connector_node.joint_angle = min(max(angle_val, lower_val), upper_val)

                parent_binding = cfg.get('parent_binding') or {}
                parent_uid = parent_binding.get('node_uid', parent_binding.get('node_id'))
                parent_direct = any(key in cfg for key in ('parent_local_xyz', 'parent_local_rpy', 'parent_axis'))
                restored_parent = False
                if parent_uid is not None:
                    parent_node = node_map.get(str(parent_uid))
                    if parent_node:
                        connector_node.parent_node_id = parent_node.id
                        connector_node.parent_point = parent_binding.get('point')
                        if 'parent_local_xyz' in cfg:
                            connector_node.parent_local_xyz = _tuple_from_values(cfg['parent_local_xyz'], 3)
                        if 'parent_local_rpy' in cfg:
                            connector_node.parent_local_rpy = _tuple_from_values(cfg['parent_local_rpy'], 3)
                        if 'parent_axis' in cfg:
                            connector_node.parent_axis = _tuple_from_values(cfg['parent_axis'], 3)
                        if parent_direct:
                            restored_parent = True
                        elif isinstance(parent_node, PartNode) and connector_node.parent_point:
                            port_name = f'P:{connector_node.parent_point}'
                            point_def = parent_node.find_point_by_port(port_name)
                            if point_def:
                                connector_node.capture_from_parent(point_def)
                                self._debug(
                                    f"Recovered parent capture for connector '{connector_node.name()}' via point '{connector_node.parent_point}'"
                                )
                            else:
                                self._debug(
                                    f"Unable to locate parent point '{connector_node.parent_point}' on node '{parent_node.name()}' during project load"
                                )
                        else:
                            self._debug(
                                f"Connector '{connector_node.name()}' parent binding lacks direct data and cannot be recomputed (node type: {type(parent_node).__name__})"
                            )
                    else:
                        self._debug(
                            f"Connector '{connector_node.name()}' references parent uid '{parent_uid}' but node was not created."
                        )
                else:
                    self._debug(f"Connector '{connector_node.name()}' has no parent binding data.")
                connector_node._restored_parent_from_payload = bool(restored_parent)

                child_binding = cfg.get('child_binding') or {}
                child_uid = child_binding.get('node_uid', child_binding.get('node_id'))
                child_direct = any(key in cfg for key in ('child_local_xyz', 'child_local_rpy', 'child_axis'))
                restored_child = False
                if child_uid is not None:
                    child_node = node_map.get(str(child_uid))
                    if child_node:
                        connector_node.child_node_id = child_node.id
                        connector_node.child_point = child_binding.get('point')
                        if 'child_local_xyz' in cfg:
                            connector_node.child_local_xyz = _tuple_from_values(cfg['child_local_xyz'], 3)
                        if 'child_local_rpy' in cfg:
                            connector_node.child_local_rpy = _tuple_from_values(cfg['child_local_rpy'], 3)
                        if 'child_axis' in cfg:
                            connector_node.child_axis = _tuple_from_values(cfg['child_axis'], 3)
                        if child_direct:
                            restored_child = True
                        elif isinstance(child_node, PartNode) and connector_node.child_point:
                            port_name = f'C:{connector_node.child_point}'
                            point_def = child_node.find_point_by_port(port_name)
                            if point_def:
                                connector_node.capture_from_child(point_def)
                                self._debug(
                                    f"Recovered child capture for connector '{connector_node.name()}' via point '{connector_node.child_point}'"
                                )
                            else:
                                self._debug(
                                    f"Unable to locate child point '{connector_node.child_point}' on node '{child_node.name()}' during project load"
                                )
                        else:
                            self._debug(
                                f"Connector '{connector_node.name()}' child binding lacks direct data and cannot be recomputed (node type: {type(child_node).__name__})"
                            )
                    else:
                        self._debug(
                            f"Connector '{connector_node.name()}' references child uid '{child_uid}' but node was not created."
                        )
                else:
                    self._debug(f"Connector '{connector_node.name()}' has no child binding data.")
                connector_node._restored_child_from_payload = bool(restored_child)

            id_lookup: Dict[str, BaseNode] = {str(node.id): node for node in self.all_nodes()}

            def _ports_already_connected(out_port, in_port) -> bool:
                try:
                    if out_port and in_port:
                        for connected in in_port.connected_ports():
                            if connected is out_port:
                                return True
                        for connected in out_port.connected_ports():
                            if connected is in_port:
                                return True
                except Exception:
                    return False
                return False

            def _connect_direct(out_node: BaseNode, out_name: str, in_node: BaseNode, in_name: str) -> None:
                if not out_node or not in_node:
                    return
                out_port = out_node.get_output(out_name) if out_name else None
                in_port = in_node.get_input(in_name) if in_name else None
                if not out_port or not in_port:
                    return
                if _ports_already_connected(out_port, in_port):
                    return
                if not self._connect_ports(out_port, in_port, emit_signal=False):
                    print(f'Warning: failed to restore connection {out_name} -> {in_name}')

            self._loading_project = True
            try:
                for connector_node, _ in connector_configs:
                    parent_id = connector_node.parent_node_id
                    parent_node = id_lookup.get(str(parent_id)) if parent_id is not None else None
                    if parent_node:
                        if isinstance(parent_node, BaseLinkNode):
                            parent_port_name = 'root'
                        elif isinstance(parent_node, PartNode) and connector_node.parent_point:
                            parent_port_name = f'P:{connector_node.parent_point}'
                        else:
                            parent_port_name = None
                        if parent_port_name:
                            _connect_direct(parent_node, parent_port_name, connector_node, 'parent')

                    child_id = connector_node.child_node_id
                    child_node = id_lookup.get(str(child_id)) if child_id is not None else None
                    if child_node and connector_node.child_point:
                        child_port_name = f'C:{connector_node.child_point}'
                        _connect_direct(connector_node, 'child', child_node, child_port_name)

                for connector_node, _ in connector_configs:
                    if hasattr(connector_node, '_restored_parent_from_payload'):
                        delattr(connector_node, '_restored_parent_from_payload')
                    if hasattr(connector_node, '_restored_child_from_payload'):
                        delattr(connector_node, '_restored_child_from_payload')

                for connection in payload.get('connections', []):
                    from_uid = str(connection.get('from_uid')) if connection.get('from_uid') is not None else None
                    to_uid = str(connection.get('to_uid')) if connection.get('to_uid') is not None else None
                    if not from_uid or not to_uid:
                        continue
                    from_node = node_map.get(from_uid)
                    to_node = node_map.get(to_uid)
                    if not from_node or not to_node:
                        continue
                    from_port = connection.get('from_port')
                    to_port = connection.get('to_port')
                    if not from_port or not to_port:
                        continue
                    out_port = from_node.get_output(from_port)
                    in_port = to_node.get_input(to_port)
                    if out_port and in_port and not _ports_already_connected(out_port, in_port):
                        if not self._connect_ports(out_port, in_port, emit_signal=False):
                            print(f'Warning: failed to reconnect {from_port} -> {to_port}')
            finally:
                self._loading_project = False

            self.debug_dump_state('deserialize_project (after rebuild)')

            self.connector_nodes = [c for c in self.connector_nodes if c in self.all_nodes()]
            self._pending_recalc = True
            self._pending_notify = True
        finally:
            self.end_batch_update()

    # -- Transform recomputation (filled later) --------------------------
    def recalculate_transforms(self):
        if self._suspend_updates:
            self._pending_recalc = True
            return

        timer = getattr(self, '_recalc_timer', None)
        if timer and timer.isActive():
            timer.stop()

        viewer = self.stl_viewer
        if not viewer:
            return

        viewer.begin_batch_updates()
        try:
            self.connector_nodes = [c for c in self.connector_nodes if c in self.all_nodes()]

            for node in self.all_nodes():
                if isinstance(node, PartNode):
                    base_matrix = getattr(node, 'link_to_mesh_matrix', np.identity(4))
                    viewer.update_stl_transform(node, self._matrix_to_vtk(base_matrix))

            transforms: Dict[int, np.ndarray] = {}
            base = self.base_node
            if not base:
                return

            base_matrix = np.identity(4)
            transforms[base.id] = base_matrix

            queue: List[int] = [base.id]
            visited_connectors: set[ConnectorNode] = set()

            while queue:
                parent_id = queue.pop(0)
                parent_matrix = transforms.get(parent_id)
                if parent_matrix is None:
                    continue

                for connector in self.connector_nodes:
                    if connector in visited_connectors:
                        continue
                    if connector.parent_node_id != parent_id:
                        continue
                    if connector.child_node_id is None:
                        continue
                    child_node = self.get_node_by_id(connector.child_node_id)
                    if not isinstance(child_node, PartNode):
                        continue

                    visited_connectors.add(connector)
                    relative_matrix = self._compute_relative_matrix(connector)
                    child_matrix = parent_matrix @ relative_matrix
                    transforms[child_node.id] = child_matrix
                    queue.append(child_node.id)

                    final_matrix = child_matrix @ getattr(child_node, 'link_to_mesh_matrix', np.identity(4))
                    viewer.update_stl_transform(child_node, self._matrix_to_vtk(final_matrix))
        finally:
            viewer.end_batch_updates()

    # -- Math helpers ----------------------------------------------------
    @staticmethod
    def _normalize(vec) -> np.ndarray:
        arr = np.array(vec, dtype=float)
        norm = np.linalg.norm(arr)
        if norm < 1e-9:
            return np.zeros(3)
        return arr / norm

    @staticmethod
    def _rotation_between(source, target) -> np.ndarray:
        s = AssemblerGraphV3._normalize(source)
        t = AssemblerGraphV3._normalize(target)
        if np.linalg.norm(s) < 1e-9 or np.linalg.norm(t) < 1e-9:
            return np.identity(3)
        v = np.cross(s, t)
        c = np.clip(np.dot(s, t), -1.0, 1.0)
        k = np.linalg.norm(v)
        if k < 1e-9:
            if c > 0:
                return np.identity(3)
            # Opposite direction: rotate around any perpendicular axis
            axis = AssemblerGraphV3._orthogonal_axis(s)
            return AssemblerGraphV3._rotation_about(axis, math.pi)
        vx = AssemblerGraphV3._skew(v / k)
        return np.identity(3) + vx + vx @ vx * ((1 - c) / (k * k))

    @staticmethod
    def _rotation_about(axis, angle) -> np.ndarray:
        axis_n = AssemblerGraphV3._normalize(axis)
        if np.linalg.norm(axis_n) < 1e-9:
            return np.identity(3)
        x, y, z = axis_n
        c = math.cos(angle)
        s = math.sin(angle)
        C = 1 - c
        return np.array([
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ])

    @staticmethod
    def _skew(vec) -> np.ndarray:
        x, y, z = vec
        return np.array([
            [0, -z, y],
            [z, 0, -x],
            [-y, x, 0],
        ])

    @staticmethod
    def _orthogonal_axis(vec) -> np.ndarray:
        x, y, z = vec
        if abs(x) < abs(y) and abs(x) < abs(z):
            ortho = np.array([0, -z, y])
        elif abs(y) < abs(z):
            ortho = np.array([-z, 0, x])
        else:
            ortho = np.array([-y, x, 0])
        return AssemblerGraphV3._normalize(ortho)

    @staticmethod
    def _matrix_from_xyz_rpy(xyz, rpy) -> np.ndarray:
        return matrix_from_xyz_rpy_np(xyz, rpy)

    @staticmethod
    def _matrix_to_vtk(matrix: np.ndarray) -> vtk.vtkTransform:
        vtk_matrix = vtk.vtkMatrix4x4()
        for row in range(4):
            for col in range(4):
                vtk_matrix.SetElement(row, col, float(matrix[row, col]))

        transform = vtk.vtkTransform()
        transform.SetMatrix(vtk_matrix)
        return transform

    @staticmethod
    def _matrix_to_rpy(rotation: np.ndarray) -> Tuple[float, float, float]:
        return matrix_to_rpy_np(rotation)

    def _compute_relative_matrix(self, connector: ConnectorNode, angle: Optional[float] = None) -> np.ndarray:
        parent_axis = self._normalize(connector.parent_axis)
        child_axis = self._normalize(connector.child_axis)
        if np.linalg.norm(parent_axis) < 1e-9:
            parent_axis = np.array([1.0, 0.0, 0.0])
        if np.linalg.norm(child_axis) < 1e-9:
            child_axis = parent_axis

        align_rotation = self._rotation_between(child_axis, parent_axis)
        if angle is None:
            angle = connector.joint_angle
        joint_rotation = self._rotation_about(parent_axis, angle)
        base_rotation = joint_rotation @ align_rotation

        child_point = np.array(connector.child_local_xyz, dtype=float)
        parent_point = np.array(connector.parent_local_xyz, dtype=float)

        translation = parent_point - base_rotation @ child_point

        base_matrix = np.identity(4)
        base_matrix[:3, :3] = base_rotation
        base_matrix[:3, 3] = translation

        offset_matrix = self._matrix_from_xyz_rpy(connector.offset_xyz, connector.offset_rpy)
        return base_matrix @ offset_matrix

    # -- URDF generation -------------------------------------------------
    def build_urdf_text(
        self,
        mesh_path_prefix: str = 'meshes/',
        mesh_name_map: Optional[Dict[int, str]] = None,
    ) -> str:
        base = self.base_node
        if not base:
            raise ValueError('Base link node is missing')

        parts: List[PartNode] = [node for node in self.all_nodes() if isinstance(node, PartNode)]
        link_names: Dict[int, str] = {base.id: base.link_name}
        for part in parts:
            link_names[part.id] = part.link_name or part.name()

        lines: List[str] = []
        lines.append('<?xml version="1.0"?>')
        lines.append('<robot name="kitchen_robot_v3">')
        lines.append(f'  <link name="{base.link_name}"/>')

        # Link sections
        for part in parts:
            mesh_file = os.path.basename(part.stl_file) if part.stl_file else ''
            if mesh_name_map and part.id in mesh_name_map:
                mesh_file = mesh_name_map[part.id]
            mesh_ref = f'{mesh_path_prefix}{mesh_file}' if mesh_file else ''
            origin_xyz = ' '.join(f'{v:.6f}' for v in part.metadata.origin_xyz)
            origin_rpy = ' '.join(f'{v:.6f}' for v in part.metadata.origin_rpy)
            color_rgba = part.metadata.color_rgba if part.metadata else (1.0, 1.0, 1.0, 1.0)
            rgba_text = ' '.join(f'{v:.3f}' for v in color_rgba)
            lines.append(f'  <link name="{link_names[part.id]}">')
            lines.append('    <visual>')
            lines.append(f'      <origin xyz="{origin_xyz}" rpy="{origin_rpy}"/>')
            lines.append('      <geometry>')
            lines.append(f'        <mesh filename="{mesh_ref}"/>')
            lines.append('      </geometry>')
            lines.append(f'      <material name="{link_names[part.id]}_mat">')
            lines.append(f'        <color rgba="{rgba_text}"/>')
            lines.append('      </material>')
            lines.append('    </visual>')
            lines.append('    <collision>')
            lines.append(f'      <origin xyz="{origin_xyz}" rpy="{origin_rpy}"/>')
            lines.append('      <geometry>')
            lines.append(f'        <mesh filename="{mesh_ref}"/>')
            lines.append('      </geometry>')
            lines.append('    </collision>')
            lines.append('  </link>')

        # Joint sections
        queue: List[int] = [base.id]
        visited_connectors: set[ConnectorNode] = set()
        while queue:
            parent_id = queue.pop(0)
            for connector in self.connector_nodes:
                if connector in visited_connectors:
                    continue
                if connector.parent_node_id != parent_id:
                    continue
                if connector.child_node_id is None:
                    continue
                child_node = self.get_node_by_id(connector.child_node_id)
                if not isinstance(child_node, PartNode):
                    continue

                visited_connectors.add(connector)
                queue.append(child_node.id)

                zero_matrix = self._compute_relative_matrix(connector, angle=0.0)
                origin_xyz = zero_matrix[:3, 3]
                origin_rpy = self._matrix_to_rpy(zero_matrix[:3, :3])
                axis_vec = self._normalize(connector.parent_axis)
                if np.linalg.norm(axis_vec) < 1e-9:
                    axis_vec = np.array([1.0, 0.0, 0.0])

                joint_name = connector.joint_name or f"joint_{link_names.get(connector.child_node_id, 'child')}"
                lines.append(f'  <joint name="{joint_name}" type="{connector.joint_type}">')
                lines.append(f'    <parent link="{link_names.get(connector.parent_node_id, base.link_name)}"/>')
                lines.append(f'    <child link="{link_names.get(connector.child_node_id, child_node.name())}"/>')
                xyz_text = ' '.join(f'{v:.6f}' for v in origin_xyz)
                rpy_text = ' '.join(f'{v:.6f}' for v in origin_rpy)
                lines.append(f'    <origin xyz="{xyz_text}" rpy="{rpy_text}"/>')
                axis_text = ' '.join(f'{v:.6f}' for v in axis_vec)
                lines.append(f'    <axis xyz="{axis_text}"/>')
                joint_type = connector.joint_type
                lower = getattr(connector, 'joint_limit_lower', -math.pi)
                upper = getattr(connector, 'joint_limit_upper', math.pi)
                effort = max(0.0, getattr(connector, 'joint_effort', 0.0))
                velocity = max(0.0, getattr(connector, 'joint_velocity', 0.0))

                if joint_type in ('revolute', 'prismatic'):
                    limit_text = (
                        f'    <limit lower="{lower:.6f}" upper="{upper:.6f}" '
                        f'effort="{effort:.6f}" velocity="{velocity:.6f}"/>'
                    )
                    lines.append(limit_text)
                elif joint_type == 'continuous':
                    limit_text = f'    <limit effort="{effort:.6f}" velocity="{velocity:.6f}"/>'
                    lines.append(limit_text)
                lines.append('  </joint>')

        lines.append('</robot>')
        return '\n'.join(lines)


# ---------------------------------------------------------------------------
# Main window & application bootstrap
# ---------------------------------------------------------------------------


class AssemblerWindow(QtWidgets.QMainWindow):
    """Main application window that hosts the graph and control widgets."""

    partImported = QtCore.Signal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle('URDF 装配器 V3')
        self.resize(1500, 900)

        self.viewer = STLViewerWidget(self)
        self.joint_panel = JointPanelV3(self)
        self.graph = AssemblerGraphV3()
        self.project_file_path: Optional[str] = None
        graph_widget = self.graph.widget

        # Link graph with viewer for transform updates
        self.graph.stl_viewer = self.viewer
        self.graph.joint_panel = self.joint_panel

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        main_layout = QtWidgets.QHBoxLayout(central)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        # Left panel with controls
        left_panel = QtWidgets.QVBoxLayout()
        left_panel.setSpacing(6)
        main_layout.addLayout(left_panel, 0)

        import_btn = QtWidgets.QPushButton('导入 STL 部件')
        import_btn.clicked.connect(self._on_import_part)
        left_panel.addWidget(import_btn)

        connector_btn = QtWidgets.QPushButton('添加连接器')
        connector_btn.clicked.connect(self._on_add_connector)
        left_panel.addWidget(connector_btn)

        project_label = QtWidgets.QLabel('项目名称：')
        left_panel.addWidget(project_label)
        self.project_name_edit = QtWidgets.QLineEdit('my_project')
        left_panel.addWidget(self.project_name_edit)

        self.open_project_btn = QtWidgets.QPushButton('打开项目...')
        self.open_project_btn.clicked.connect(self._on_open_project)
        left_panel.addWidget(self.open_project_btn)

        self.save_project_btn = QtWidgets.QPushButton('保存项目...')
        self.save_project_btn.clicked.connect(self._on_save_project)
        left_panel.addWidget(self.save_project_btn)

        self.auto_color_checkbox = QtWidgets.QCheckBox('预览自动着色')
        self.auto_color_checkbox.setToolTip('在预览窗口中为已加载部件自动分配不同颜色。')
        self.auto_color_checkbox.stateChanged.connect(self._apply_preview_colors)
        left_panel.addWidget(self.auto_color_checkbox)

        self.calu_btn = QtWidgets.QPushButton('生成 URDF')
        self.calu_btn.clicked.connect(self._on_calu)
        left_panel.addWidget(self.calu_btn)

        self.save_btn = QtWidgets.QPushButton('保存 URDF...')
        self.save_btn.clicked.connect(self._on_save_urdf)
        left_panel.addWidget(self.save_btn)

        left_panel.addStretch(1)

        content_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        content_splitter.addWidget(graph_widget)
        content_splitter.addWidget(self.viewer)
        content_splitter.addWidget(self.joint_panel)
        content_splitter.setStretchFactor(0, 2)
        content_splitter.setStretchFactor(1, 3)
        content_splitter.setStretchFactor(2, 2)

        main_layout.addWidget(content_splitter, 1)

        self.graph.preview_color_callback = self._apply_preview_colors
        self.graph.recalculate_transforms()
        self.joint_panel.refresh_from_graph(self.graph)
        self._apply_preview_colors()

    # -- Actions ---------------------------------------------------------
    def _on_import_part(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            '选择 STL 文件',
            '',
            'STL 文件 (*.stl);;所有文件 (*)',
        )
        if not path:
            return
        try:
            self.graph.add_part(path)
        except Exception as exc:
            traceback.print_exc()
            QtWidgets.QMessageBox.critical(
                self,
                '导入失败',
                f'无法载入以下 STL：\n{path}\n\n{exc}',
            )
            return

        self.joint_panel.refresh_from_graph(self.graph)
        self._apply_preview_colors()
        self.partImported.emit(path)

    def import_part_from_path(self, path: str, show_message: bool = True) -> bool:
        if not path:
            return False
        if not os.path.exists(path):
            if show_message:
                QtWidgets.QMessageBox.warning(
                    self,
                    '导入 STL',
                    f'未找到 STL 文件：\n{path}',
                )
            return False

        try:
            self.graph.add_part(path)
        except Exception as exc:
            traceback.print_exc()
            if show_message:
                QtWidgets.QMessageBox.critical(
                    self,
                    '导入失败',
                    f'无法载入以下 STL：\n{path}\n\n{exc}',
                )
            return False

        self.joint_panel.refresh_from_graph(self.graph)
        self._apply_preview_colors()
        self.partImported.emit(path)
        return True

    def _on_add_connector(self):
        view = self.graph.widget
        try:
            # Drop near the visible center of the graph to keep the node in sight.
            center_scene = view.mapToScene(view.viewport().rect().center())
        except Exception:
            center_scene = QtCore.QPointF(0, 0)

        base_pos = None
        if self.graph.base_node is not None:
            try:
                base_pos = self.graph.base_node.pos()
            except Exception:
                base_pos = None

        if isinstance(base_pos, QtCore.QPointF):
            center_scene = QtCore.QPointF(base_pos.x() + 220, base_pos.y() + 140)
        elif isinstance(base_pos, (tuple, list)) and len(base_pos) >= 2:
            center_scene = QtCore.QPointF(float(base_pos[0]) + 220.0, float(base_pos[1]) + 140.0)

        self.graph.add_connector((int(center_scene.x()), int(center_scene.y())))

    def _on_calu(self):
        try:
            urdf_text = self.graph.build_urdf_text()
        except Exception as exc:
            traceback.print_exc()
            QtWidgets.QMessageBox.critical(
                self,
                '生成 URDF 失败',
                f'生成 URDF 失败：\n{exc}',
            )
            return

        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle('URDF 预览')
        dialog.resize(720, 640)
        layout = QtWidgets.QVBoxLayout(dialog)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        text_edit = QtWidgets.QPlainTextEdit()
        text_edit.setPlainText(urdf_text)
        text_edit.setReadOnly(True)
        font = QtGui.QFont('Courier New')
        font.setPointSize(10)
        text_edit.setFont(font)
        layout.addWidget(text_edit)

        button_box = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Close)
        btn_copy = button_box.addButton('复制', QtWidgets.QDialogButtonBox.ActionRole)
        btn_save = button_box.addButton('保存...', QtWidgets.QDialogButtonBox.ActionRole)
        button_box.rejected.connect(dialog.reject)
        layout.addWidget(button_box)

        btn_copy.clicked.connect(lambda: QtWidgets.QApplication.clipboard().setText(urdf_text))

        def save_to_file():
            path, _ = QtWidgets.QFileDialog.getSaveFileName(
                dialog,
                '保存 URDF',
                'robot.urdf',
                'URDF 文件 (*.urdf);;所有文件 (*)',
            )
            if path:
                try:
                    with open(path, 'w', encoding='utf-8') as fh:
                        fh.write(urdf_text)
                except Exception as exc:
                    QtWidgets.QMessageBox.critical(dialog, '保存失败', str(exc))

        btn_save.clicked.connect(save_to_file)

        dialog.exec()

    def _on_save_project(self):
        if not self.graph:
            return

        project_name = self._project_name()
        default_dir = os.path.dirname(self.project_file_path) if self.project_file_path else os.getcwd()
        default_filename = f'{project_name}.json'
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            '保存项目',
            os.path.join(default_dir, default_filename),
            'URDF 装配项目 (*.json);;所有文件 (*)',
        )
        if not path:
            return

        base_dir = os.path.dirname(path)
        try:
            graph_payload = self.graph.serialize_project(base_dir)
        except Exception as exc:
            traceback.print_exc()
            QtWidgets.QMessageBox.critical(self, '保存项目', f'序列化项目失败：\n{exc}')
            return

        payload = {
            'version': 1,
            'project_name': project_name,
            'auto_color': bool(self.auto_color_checkbox.isChecked()),
            'graph': graph_payload,
        }

        try:
            with open(path, 'w', encoding='utf-8') as fh:
                json.dump(payload, fh, indent=2)
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, '保存项目', f'写入项目失败：\n{exc}')
            return

        self.project_file_path = path
        QtWidgets.QMessageBox.information(self, '保存项目', f'项目已保存至：\n{path}')

    def load_project_from_path(self, path: str, show_messages: bool = True) -> bool:
        if not path:
            return False
        if not os.path.exists(path):
            if show_messages:
                QtWidgets.QMessageBox.warning(
                    self,
                    '打开项目',
                    f'未找到项目文件：\n{path}',
                )
            return False

        try:
            with open(path, 'r', encoding='utf-8') as fh:
                payload = json.load(fh)
        except Exception as exc:
            if show_messages:
                QtWidgets.QMessageBox.critical(self, '打开项目', f'读取项目失败：\n{exc}')
            return False

        graph_payload = payload.get('graph')
        if not isinstance(graph_payload, dict):
            if show_messages:
                QtWidgets.QMessageBox.critical(self, '打开项目', '项目文件缺少图形数据。')
            return False

        base_dir = os.path.dirname(path) or os.getcwd()
        try:
            self.graph.deserialize_project(graph_payload, base_dir)
        except FileNotFoundError as exc:
            if show_messages:
                QtWidgets.QMessageBox.critical(self, '打开项目', f'缺少资源：\n{exc}')
            return False
        except Exception as exc:
            traceback.print_exc()
            if show_messages:
                QtWidgets.QMessageBox.critical(self, '打开项目', f'加载项目失败：\n{exc}')
            return False

        project_name = payload.get('project_name') or 'my_project'
        self.project_name_edit.setText(project_name)
        auto_color_value = payload.get('auto_color')
        if auto_color_value is not None:
            self.auto_color_checkbox.setChecked(bool(auto_color_value))
        else:
            self._apply_preview_colors()

        self.project_file_path = path
        self.joint_panel.refresh_from_graph(self.graph)
        self.graph.debug_dump_state('after load_project_from_path')

        if show_messages:
            QtWidgets.QMessageBox.information(self, '打开项目', f'项目已加载：\n{path}')
        return True

    def _on_open_project(self):
        default_dir = os.path.dirname(self.project_file_path) if self.project_file_path else os.getcwd()
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            '打开项目',
            default_dir,
            'URDF 装配项目 (*.json);;所有文件 (*)',
        )
        if not path:
            return
        self.load_project_from_path(path)

    def _project_name(self) -> str:
        text = ''
        if hasattr(self, 'project_name_edit'):
            text = self.project_name_edit.text()
        text = (text or '').strip()
        if not text:
            return 'my_project'
        sanitized = ''.join(ch if ch.isalnum() or ch in ('-', '_') else '_' for ch in text)
        sanitized = sanitized.strip('_')
        return sanitized or 'my_project'

    def _on_save_urdf(self):
        if not self.graph:
            return

        parts = [node for node in self.graph.all_nodes() if isinstance(node, PartNode)]
        if not parts:
            QtWidgets.QMessageBox.warning(self, '保存 URDF', '没有可导出的部件。')
            return

        project_name = self._project_name()

        base_dir = QtWidgets.QFileDialog.getExistingDirectory(
            self,
            '选择导出目录',
            '',
        )
        if not base_dir:
            return

        project_dir = os.path.join(base_dir, project_name)
        urdf_dir = os.path.join(project_dir, 'urdf')

        try:
            os.makedirs(project_dir, exist_ok=True)
            os.makedirs(urdf_dir, exist_ok=True)
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, '保存 URDF', f'创建导出目录失败：\n{exc}')
            return

        used_names: set[str] = set()
        mesh_name_map: Dict[int, str] = {}
        copy_errors: list[str] = []
        for part in parts:
            stl_path = getattr(part, 'stl_file', None)
            if not stl_path or not os.path.isfile(stl_path):
                continue
            base_name = os.path.basename(stl_path)
            name_root, ext = os.path.splitext(base_name)
            candidate = base_name
            counter = 1
            while candidate in used_names:
                candidate = f'{name_root}_{counter}{ext}'
                counter += 1
            used_names.add(candidate)
            dest_path = os.path.join(project_dir, candidate)
            mesh_name_map[part.id] = candidate
            try:
                if os.path.abspath(stl_path) != os.path.abspath(dest_path):
                    shutil.copy2(stl_path, dest_path)
            except Exception as exc:
                copy_errors.append(f'{base_name}: {exc}')

        try:
            urdf_text = self.graph.build_urdf_text(mesh_path_prefix='../', mesh_name_map=mesh_name_map)
        except Exception as exc:
            traceback.print_exc()
            QtWidgets.QMessageBox.critical(self, '保存 URDF', f'无法生成 URDF：\n{exc}')
            return

        urdf_path = os.path.join(urdf_dir, f'{project_name}.urdf')
        try:
            with open(urdf_path, 'w', encoding='utf-8') as fh:
                fh.write(urdf_text)
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, '保存 URDF', f'写入 URDF 失败：\n{exc}')
            return

        message = f'URDF 已保存至：\n{urdf_path}'
        if copy_errors:
            details = '\n'.join(copy_errors)
            message += f'\n\n但部分网格文件复制失败：\n{details}'
            QtWidgets.QMessageBox.warning(self, '保存 URDF', message)
        else:
            QtWidgets.QMessageBox.information(self, '保存 URDF', message)

    # -- Preview helpers -------------------------------------------------
    @staticmethod
    def _auto_color_palette() -> List[Tuple[float, float, float]]:
        return [
            (0.882, 0.341, 0.341),
            (0.341, 0.631, 0.882),
            (0.451, 0.780, 0.400),
            (0.952, 0.686, 0.329),
            (0.682, 0.482, 0.871),
            (0.322, 0.725, 0.749),
            (0.941, 0.522, 0.639),
            (0.400, 0.702, 0.427),
            (0.902, 0.800, 0.353),
            (0.447, 0.447, 0.749),
        ]

    def _apply_preview_colors(self, *_):
        if not hasattr(self, 'graph') or not hasattr(self.graph, 'all_nodes'):
            return

        parts = [node for node in self.graph.all_nodes() if isinstance(node, PartNode)]
        if not parts:
            return

        use_auto = self.auto_color_checkbox.isChecked() if hasattr(self, 'auto_color_checkbox') else False
        palette = self._auto_color_palette()

        parts_sorted = sorted(parts, key=lambda n: n.name())
        for idx, part in enumerate(parts_sorted):
            if use_auto:
                color = palette[idx % len(palette)]
                part.node_color = [float(c) for c in color]
            else:
                base_color = getattr(part, 'default_color', None)
                if base_color is not None:
                    part.node_color = list(base_color)

            if hasattr(self, 'viewer') and self.viewer is not None:
                self.viewer.apply_color_to_node(part)


def apply_dark_theme(app: QtWidgets.QApplication) -> None:
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.Window, QtGui.QColor(40, 40, 45))
    palette.setColor(QtGui.QPalette.WindowText, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Base, QtGui.QColor(28, 28, 32))
    palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor(40, 40, 45))
    palette.setColor(QtGui.QPalette.Text, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Button, QtGui.QColor(60, 60, 65))
    palette.setColor(QtGui.QPalette.ButtonText, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor(90, 140, 255))
    palette.setColor(QtGui.QPalette.HighlightedText, QtCore.Qt.black)
    app.setPalette(palette)


def main():
    app = QtWidgets.QApplication(sys.argv)
    apply_dark_theme(app)

    window = AssemblerWindow()
    window.show()

    def signal_handler(sig, frame):
        app.quit()

    signal.signal(signal.SIGINT, signal_handler)

    timer = QtCore.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
