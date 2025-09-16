"""
File Name: urdf_kitchen_Assembler.py
Description: A Python script to assembling files configured with urdf_kitchen_PartsEditor.py into a URDF file.

Author      : Ninagawa123
Created On  : Nov 24, 2024
Version     : 0.0.2
License     : MIT License
URL         : https://github.com/Ninagawa123/URDF_kitchen_beta
Copyright (c) 2024 Ninagawa123

pip install numpy
pip install PySide6
pip install vtk
pip install NodeGraphQt
"""

import sys
import signal
import traceback
from Qt import QtWidgets, QtCore, QtGui
from NodeGraphQt import NodeGraph, BaseNode
import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from PySide6.QtWidgets import QFileDialog
from PySide6.QtCore import QPointF, QRegularExpression
from PySide6.QtGui import QDoubleValidator, QRegularExpressionValidator, QPalette, QColor
import os
import xml.etree.ElementTree as ET
import base64
import shutil
import datetime


def apply_dark_theme(app):
    dark_palette = QPalette()
    dark_palette.setColor(QPalette.Window, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.WindowText, QColor(255, 255, 255))
    dark_palette.setColor(QPalette.Base, QColor(42, 42, 42))
    dark_palette.setColor(QPalette.AlternateBase, QColor(66, 66, 66))
    dark_palette.setColor(QPalette.ToolTipBase, QColor(255, 255, 255))
    dark_palette.setColor(QPalette.ToolTipText, QColor(255, 255, 255))
    dark_palette.setColor(QPalette.Text, QColor(255, 255, 255))
    dark_palette.setColor(QPalette.Button, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.ButtonText, QColor(255, 255, 255))
    dark_palette.setColor(QPalette.BrightText, QColor(255, 0, 0))
    dark_palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    dark_palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
    app.setPalette(dark_palette)


class BaseLinkNode(BaseNode):
    """Base link node class"""
    __identifier__ = 'insilico.nodes'
    NODE_NAME = 'BaseLinkNode'

    def __init__(self):
        super(BaseLinkNode, self).__init__()
        self.add_output('out')

        self.volume_value = 0.0  # 追加
        self.mass_value = 0.0

        self.inertia = {
            'ixx': 0.0, 'ixy': 0.0, 'ixz': 0.0,
            'iyy': 0.0, 'iyz': 0.0, 'izz': 0.0
        }
        self.points = [{
            'name': 'base_link_point1',
            'type': 'fixed',
            'xyz': [0.0, 0.0, 0.0],
            'axis': 3  # 固定
        }]
        self.cumulative_coords = [{
            'point_index': 0,
            'xyz': [0.0, 0.0, 0.0]
        }]

        self.stl_file = None

        # 色信息
        self.node_color = [1.0, 1.0, 1.0]

        # 默认整体旋转轴 (向后兼容)
        self.rotation_axis = 0
        # link origin / rpy (URDF 用)
        self.origin_xyz = [0.0, 0.0, 0.0]
        self.origin_rpy = [0.0, 0.0, 0.0]

    def add_input(self, name='', **kwargs):
        # 入力ポートの追加を禁止
        print("Base Link node cannot have input ports")
        return None

    def add_output(self, name='out_1', **kwargs):
        # 出力ポートが既に存在する場合は追加しない
        if not self.has_output(name):
            return super(BaseLinkNode, self).add_output(name, **kwargs)
        return None

    def remove_output(self, port=None):
        # 出力ポートの削除を禁止
        print("Cannot remove output port from Base Link node")
        return None

    def has_output(self, name):
        """指定した名前の出力ポートが存在するかチェック"""
        return name in [p.name() for p in self.output_ports()]


class FooNode(BaseNode):
    """General purpose node class"""
    __identifier__ = 'insilico.nodes'
    NODE_NAME = 'FooNode'

    def __init__(self):
        super(FooNode, self).__init__()
        self.add_input('in', color=(180, 80, 0))

        self.output_count = 0
        self.volume_value = 0.0
        self.mass_value = 0.0
        self.inertia = {
            'ixx': 0.0, 'ixy': 0.0, 'ixz': 0.0,
            'iyy': 0.0, 'iyz': 0.0, 'izz': 0.0
        }
        self.points = []  # 每个点 dict: name, type, xyz, axis
        self.cumulative_coords = []
        self.stl_file = None
        self.node_color = [1.0, 1.0, 1.0]
        self.rotation_axis = 0  # 旧总轴
        self.origin_xyz = [0.0, 0.0, 0.0]
        self.origin_rpy = [0.0, 0.0, 0.0]

        self._add_output()
        self.set_port_deletion_allowed(True)
        self._original_double_click = self.view.mouseDoubleClickEvent
        self.view.mouseDoubleClickEvent = self.node_double_clicked

    def _add_output(self, name=''):
        if self.output_count < 8:
            self.output_count += 1
            port_name = f'out_{self.output_count}'
            super(FooNode, self).add_output(port_name)
            self.points.append({
                'name': f'point_{self.output_count}',
                'type': 'fixed',
                'xyz': [0.0, 0.0, 0.0],
                'axis': 0  # 默认X
            })
            self.cumulative_coords.append({
                'point_index': self.output_count - 1,
                'xyz': [0.0, 0.0, 0.0]
            })
            print(f"新增输出端口 '{port_name}' (初始坐标0,0,0, axis=X)")
            return port_name

    def remove_output(self):
        """出力ポートの削除（修正版）"""
        if self.output_count > 1:
            port_name = f'out_{self.output_count}'
            output_port = self.get_output(port_name)
            if output_port:
                try:
                    # 接続されているポートを処理
                    for connected_port in output_port.connected_ports():
                        try:
                            print(
                                f"Disconnecting {port_name} from {connected_port.node().name()}.{connected_port.name()}")
                            # NodeGraphQtの標準メソッドを使用
                            self.graph.disconnect_node(self.id, port_name,
                                                       connected_port.node().id, connected_port.name())
                        except Exception as e:
                            print(f"Error during disconnection: {str(e)}")

                    # 対応するポイントデータを削除
                    if len(self.points) >= self.output_count:
                        self.points.pop()
                        print(f"Removed point data for port {port_name}")

                    # 累積座標を削除
                    if len(self.cumulative_coords) >= self.output_count:
                        self.cumulative_coords.pop()
                        print(f"Removed cumulative coordinates for port {port_name}")

                    # ポートの削除
                    self.delete_output(output_port)
                    self.output_count -= 1
                    print(f"Removed port {port_name}")

                    # ビューの更新
                    self.view.update()

                except Exception as e:
                    print(f"Error removing port and associated data: {str(e)}")
                    traceback.print_exc()
            else:
                print(f"Output port {port_name} not found")
        else:
            print("Cannot remove the last output port")

    def node_double_clicked(self, event):
        print(f"Node {self.name()} double-clicked!")
        if hasattr(self.graph, 'show_inspector'):
            try:
                # グラフのビューを正しく取得
                graph_view = self.graph.viewer()  # NodeGraphQtではviewer()メソッドを使用

                # シーン座標をビュー座標に変換
                scene_pos = event.scenePos()
                view_pos = graph_view.mapFromScene(scene_pos)
                screen_pos = graph_view.mapToGlobal(view_pos)

                print(f"Double click at screen coordinates: ({screen_pos.x()}, {screen_pos.y()})")
                self.graph.show_inspector(self, screen_pos)

            except Exception as e:
                print(f"Error getting mouse position: {str(e)}")
                traceback.print_exc()
                # フォールバック：位置指定なしでインスペクタを表示
                self.graph.show_inspector(self)
        else:
            print("Error: graph does not have show_inspector method")


class InspectorWindow(QtWidgets.QWidget):

    def __init__(self, parent=None, stl_viewer=None):
        super(InspectorWindow, self).__init__(parent)
        self.setWindowTitle("Node Inspector")
        self.setMinimumWidth(400)
        self.setMinimumHeight(600)

        self.setWindowFlags(self.windowFlags() |
                            QtCore.Qt.WindowStaysOnTopHint)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)

        self.current_node = None
        self.stl_viewer = stl_viewer
        self.port_widgets = []

        # UI初始化
        self.setup_ui()

        # キーボードフォーカスを受け取れるように設定
        self.setFocusPolicy(QtCore.Qt.StrongFocus)

    def setup_ui(self):
        """UI初始化 (中文)"""
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setSpacing(6)
        main_layout.setContentsMargins(8,4,8,4)
        scroll_area = QtWidgets.QScrollArea(); scroll_area.setWidgetResizable(True)
        scroll_content = QtWidgets.QWidget(); content_layout = QtWidgets.QVBoxLayout(scroll_content)
        content_layout.setSpacing(12)

        # 节点名称
        name_layout = QtWidgets.QHBoxLayout()
        name_layout.addWidget(QtWidgets.QLabel("节点名称:"))
        self.name_edit = QtWidgets.QLineEdit(); self.name_edit.editingFinished.connect(self.update_node_name)
        name_layout.addWidget(self.name_edit)
        content_layout.addLayout(name_layout)

        # 原点 / 姿态
        origin_group = QtWidgets.QGroupBox("URDF 原点 / 姿态 (xyz / rpy)")
        og = QtWidgets.QGridLayout(origin_group)
        self.origin_edits = [QtWidgets.QLineEdit("0.0") for _ in range(3)]
        self.rpy_edits = [QtWidgets.QLineEdit("0.0") for _ in range(3)]
        labels_o = ['x','y','z']; labels_r = ['r','p','y']
        for i,l in enumerate(labels_o):
            og.addWidget(QtWidgets.QLabel(l),0,i*2)
            og.addWidget(self.origin_edits[i],0,i*2+1)
        for i,l in enumerate(labels_r):
            og.addWidget(QtWidgets.QLabel(l),1,i*2)
            og.addWidget(self.rpy_edits[i],1,i*2+1)
        self.btn_apply_origin = QtWidgets.QPushButton("应用原点/姿态")
        self.btn_apply_origin.clicked.connect(self.apply_origin_rpy)
        og.addWidget(self.btn_apply_origin,2,0,1,6)
        content_layout.addWidget(origin_group)

        # 体积 / 质量
        physics_layout = QtWidgets.QGridLayout()
        physics_layout.addWidget(QtWidgets.QLabel("体积:"),0,0)
        self.volume_input = QtWidgets.QLineEdit(); self.volume_input.setReadOnly(True)
        physics_layout.addWidget(self.volume_input,0,1)
        physics_layout.addWidget(QtWidgets.QLabel("质量:"),1,0)
        self.mass_input = QtWidgets.QLineEdit(); physics_layout.addWidget(self.mass_input,1,1)
        content_layout.addLayout(physics_layout)

        # 颜色
        color_layout = QtWidgets.QHBoxLayout(); color_layout.addWidget(QtWidgets.QLabel("颜色(RGB):"))
        self.color_sample = QtWidgets.QLabel(); self.color_sample.setFixedSize(24,24)
        self.color_sample.setStyleSheet("background:#fff;border:1px solid #333;")
        self.color_inputs = []
        for lab in ['R','G','B']:
            color_layout.addWidget(QtWidgets.QLabel(lab))
            le = QtWidgets.QLineEdit("1.0"); le.setFixedWidth(50)
            self.color_inputs.append(le)
            color_layout.addWidget(le)
        btn_pick = QtWidgets.QPushButton("拾取"); btn_pick.clicked.connect(self.show_color_picker)
        btn_apply_color = QtWidgets.QPushButton("应用"); btn_apply_color.clicked.connect(self.apply_color_to_stl)
        color_layout.addWidget(self.color_sample); color_layout.addWidget(btn_pick); color_layout.addWidget(btn_apply_color)
        content_layout.addLayout(color_layout)

        # 端口 (点) 列表区域
        self.ports_container = QtWidgets.QVBoxLayout()
        ports_box = QtWidgets.QGroupBox("连接点(名称 / 坐标 / 轴 / 操作)")
        ports_box.setLayout(self.ports_container)
        content_layout.addWidget(ports_box)

        scroll_area.setWidget(scroll_content)
        main_layout.addWidget(scroll_area)
        self.port_widgets = []

    # 新增: 节点重命名
    def update_node_name(self):
        if not self.current_node:
            return
        new_name = self.name_edit.text().strip()
        if not new_name:
            return
        try:
            if hasattr(self.current_node, 'set_name'):
                # NodeGraphQt BaseNode API
                self.current_node.set_name(new_name)
            else:
                # 回退属性 (不推荐, 仅防御性)
                self.current_node._name = new_name  # noqa
            print(f"节点重命名 -> {new_name}")
        except Exception as e:
            print(f"重命名失败: {e}")

    # 新增: 更新颜色块显示
    def update_color_sample(self):
        try:
            r = float(self.color_inputs[0].text())
            g = float(self.color_inputs[1].text())
            b = float(self.color_inputs[2].text())
            r = max(0,min(1,r)); g=max(0,min(1,g)); b=max(0,min(1,b))
            self.color_sample.setStyleSheet(f"background: rgb({int(r*255)},{int(g*255)},{int(b*255)}); border:1px solid #333;")
        except Exception:
            pass

    # 新增: 颜色拾取
    def show_color_picker(self):
        try:
            from PySide6.QtWidgets import QColorDialog
            c = QColorDialog.getColor(parent=self)
            if c.isValid():
                self.color_inputs[0].setText(f"{c.redF():.3f}")
                self.color_inputs[1].setText(f"{c.greenF():.3f}")
                self.color_inputs[2].setText(f"{c.blueF():.3f}")
                self.update_color_sample()
                self.apply_color_to_stl()
        except Exception as e:
            print(f"颜色拾取失败: {e}")

    # 新增: 应用颜色到节点与预览
    def apply_color_to_stl(self):
        if not self.current_node:
            return
        try:
            r = float(self.color_inputs[0].text())
            g = float(self.color_inputs[1].text())
            b = float(self.color_inputs[2].text())
            self.current_node.node_color = [max(0,min(1,r)), max(0,min(1,g)), max(0,min(1,b))]
            self.update_color_sample()
            if self.stl_viewer:
                self.stl_viewer.update_node_color(self.current_node)
        except ValueError:
            print("颜色输入错误")

    # 覆盖 update_info 追加 origin/rpy, 端口轴
    def update_info(self, node):
        self.current_node = node
        try:
            self.name_edit.setText(node.name())
            if hasattr(node,'volume_value'): self.volume_input.setText(f"{node.volume_value:.6f}")
            if hasattr(node,'mass_value'): self.mass_input.setText(f"{node.mass_value:.6f}")
            # origin/rpy
            if not hasattr(node,'origin_xyz'): node.origin_xyz = [0.0,0.0,0.0]
            if not hasattr(node,'origin_rpy'): node.origin_rpy = [0.0,0.0,0.0]
            for i,v in enumerate(node.origin_xyz): self.origin_edits[i].setText(f"{v:.6f}")
            for i,v in enumerate(node.origin_rpy): self.rpy_edits[i].setText(f"{v:.6f}")
            # 颜色
            if hasattr(node,'node_color'):
                for i,v in enumerate(node.node_color[:3]):
                    if i < len(self.color_inputs): self.color_inputs[i].setText(f"{v:.3f}")
                self.update_color_sample()
            # points 显示
            self.update_output_ports(node)
        except Exception as e:
            print(f"Inspector update error: {e}")

    def apply_origin_rpy(self):
        if not self.current_node: return
        try:
            self.current_node.origin_xyz = [float(le.text()) for le in self.origin_edits]
            self.current_node.origin_rpy = [float(le.text()) for le in self.rpy_edits]
            print(f"Updated origin/rpy: {self.current_node.origin_xyz} / {self.current_node.origin_rpy}")
        except ValueError:
            print("Origin/RPY 输入非法")

    def create_port_widget(self, index, point):
        layout = QtWidgets.QHBoxLayout(); layout.setContentsMargins(2,2,2,2)
        cb_enable = QtWidgets.QCheckBox(); cb_enable.setChecked(True)
        name_edit = QtWidgets.QLineEdit(point.get('name',f'point_{index+1}')); name_edit.setFixedWidth(80)
        x_edit = QtWidgets.QLineEdit(f"{point['xyz'][0]:.6f}"); x_edit.setFixedWidth(70)
        y_edit = QtWidgets.QLineEdit(f"{point['xyz'][1]:.6f}"); y_edit.setFixedWidth(70)
        z_edit = QtWidgets.QLineEdit(f"{point['xyz'][2]:.6f}"); z_edit.setFixedWidth(70)
        axis_combo = QtWidgets.QComboBox(); axis_combo.addItems(['X','Y','Z','固定'])
        axis_combo.setCurrentIndex(point.get('axis',0))
        btn_set = QtWidgets.QPushButton("Set"); btn_reset = QtWidgets.QPushButton("Reset")
        btn_test = QtWidgets.QPushButton("Test"); btn_test.setCheckable(True)

        def apply_changes():
            try:
                point['name'] = name_edit.text() or f'point_{index+1}'
                point['xyz'][0] = float(x_edit.text()); point['xyz'][1] = float(y_edit.text()); point['xyz'][2] = float(z_edit.text())
                point['axis'] = axis_combo.currentIndex()
                print(f"Point {index+1} updated: {point}")
                if self.stl_viewer:
                    self.stl_viewer.vtkWidget.GetRenderWindow().Render()
            except ValueError:
                print("坐标输入错误")

        def reset_point():
            point['xyz'] = [0.0,0.0,0.0]
            x_edit.setText("0.0"); y_edit.setText("0.0"); z_edit.setText("0.0")
            apply_changes()

        def toggle_test(on):
            if not self.stl_viewer: return
            self.stl_viewer.toggle_point_rotation(self.current_node, index, on)

        btn_set.clicked.connect(apply_changes)
        btn_reset.clicked.connect(reset_point)
        btn_test.toggled.connect(toggle_test)
        for w in [cb_enable, QtWidgets.QLabel(f"#{index+1}"), name_edit, x_edit, y_edit, z_edit, axis_combo, btn_set, btn_reset, btn_test]:
            layout.addWidget(w)
        layout.addStretch()
        wrap = QtWidgets.QWidget(); wrap.setLayout(layout)
        return wrap

    def update_output_ports(self, node):
        # 清理
        for w in self.port_widgets:
            w.setParent(None)
        self.port_widgets.clear()
        if not hasattr(node,'points'): return
        for i,pt in enumerate(node.points):
            # 补充缺失 axis
            if 'axis' not in pt: pt['axis']=0
            w = self.create_port_widget(i, pt)
            self.ports_container.addWidget(w)
            self.port_widgets.append(w)


class STLViewerWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(STLViewerWidget, self).__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        self.vtkWidget = QVTKRenderWindowInteractor(self)
        layout.addWidget(self.vtkWidget)
        self.renderer = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.renderer)
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()
        self.iren.Initialize()
        self.renderer.SetBackground(0.08,0.08,0.1)
        self.axes_actor = vtk.vtkAxesActor(); self.axes_actor.SetTotalLength(0.1,0.1,0.1)
        self.renderer.AddActor(self.axes_actor)
        self.center_actor = None
        self.stl_actors = {}   # node -> vtkActor
        self.transforms = {}   # node -> vtkTransform (base transform without test rotation)
        self.point_rotation_timers = {}  # (node,index)->QTimer
        self.point_rotation_angles = {}

    def add_or_update_node_stl(self, node):
        if not hasattr(node,'stl_file') or not node.stl_file or not os.path.exists(node.stl_file):
            return
        # remove existing
        if node in self.stl_actors:
            self.renderer.RemoveActor(self.stl_actors[node])
        reader = vtk.vtkSTLReader(); reader.SetFileName(node.stl_file); reader.Update()
        mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(reader.GetOutputPort())
        actor = vtk.vtkActor(); actor.SetMapper(mapper)
        color = getattr(node,'node_color',[1,1,1])
        actor.GetProperty().SetColor(*color)
        self.renderer.AddActor(actor)
        self.stl_actors[node]=actor
        tf = vtk.vtkTransform(); tf.Identity(); self.transforms[node]=tf
        actor.SetUserTransform(tf)
        self._update_center_marker()
        self.vtkWidget.GetRenderWindow().Render()

    def update_node_color(self, node):
        if node in self.stl_actors:
            c = getattr(node,'node_color',[1,1,1])
            self.stl_actors[node].GetProperty().SetColor(*c)
            self.vtkWidget.GetRenderWindow().Render()

    def _update_center_marker(self):
        # 计算全部模型包围盒中心
        if self.center_actor:
            self.renderer.RemoveActor(self.center_actor)
            self.center_actor=None
        if not self.stl_actors:
            self.vtkWidget.GetRenderWindow().Render(); return
        b_min = [1e9,1e9,1e9]; b_max=[-1e9,-1e9,-1e9]
        for act in self.stl_actors.values():
            b = act.GetBounds()
            b_min[0]=min(b_min[0],b[0]); b_max[0]=max(b_max[0],b[1])
            b_min[1]=min(b_min[1],b[2]); b_max[1]=max(b_max[1],b[3])
            b_min[2]=min(b_min[2],b[4]); b_max[2]=max(b_max[2],b[5])
        center = [(b_min[i]+b_max[i])/2.0 for i in range(3)]
        sphere = vtk.vtkSphereSource(); sphere.SetRadius(0.005)
        mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(sphere.GetOutputPort())
        self.center_actor = vtk.vtkActor(); self.center_actor.SetMapper(mapper)
        self.center_actor.SetPosition(*center)
        self.center_actor.GetProperty().SetColor(1,1,0)
        self.renderer.AddActor(self.center_actor)

    def reset_camera(self):
        self.renderer.ResetCamera()
        cam = self.renderer.GetActiveCamera(); cam.ParallelProjectionOn()
        self.renderer.ResetCameraClippingRange()
        self.vtkWidget.GetRenderWindow().Render()

    def toggle_point_rotation(self, node, point_index, on):
        if on:
            key = (node, point_index)
            timer = QtCore.QTimer(); timer.setInterval(40)
            timer.timeout.connect(lambda k=key: self._tick_point_rotation(k))
            self.point_rotation_timers[key]=timer
            self.point_rotation_angles[key]=0
            timer.start()
        else:
            key = (node, point_index)
            if key in self.point_rotation_timers:
                self.point_rotation_timers[key].stop(); del self.point_rotation_timers[key]
                if node in self.stl_actors:
                    self.stl_actors[node].SetUserTransform(self.transforms.get(node))
                    self.vtkWidget.GetRenderWindow().Render()

    def _tick_point_rotation(self, key):
        node, idx = key
        if node not in self.stl_actors or node not in self.transforms: return
        pt_list = getattr(node,'points',[])
        if idx >= len(pt_list): return
        pt = pt_list[idx]
        if 'axis' not in pt: pt['axis']=0
        self.point_rotation_angles[key] = self.point_rotation_angles.get(key,0)+5
        angle = self.point_rotation_angles[key]
        base_tf = vtk.vtkTransform(); base_tf.DeepCopy(self.transforms[node])
        px,py,pz = pt['xyz'] if 'xyz' in pt else (0,0,0)
        base_tf.Translate(px,py,pz)
        if pt['axis']==0: base_tf.RotateX(angle)
        elif pt['axis']==1: base_tf.RotateY(angle)
        elif pt['axis']==2: base_tf.RotateZ(angle)
        base_tf.Translate(-px,-py,-pz)
        self.stl_actors[node].SetUserTransform(base_tf)
        self.vtkWidget.GetRenderWindow().Render()


class AssemblerWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("URDF Kitchen - 组装器 (Assembler) v0.0.3")
        self.resize(1600, 900)
        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        h = QtWidgets.QHBoxLayout(central)

        # 左侧控制面板
        left_panel = QtWidgets.QVBoxLayout(); h.addLayout(left_panel,2)

        # 机器人目录选择
        dir_layout = QtWidgets.QHBoxLayout()
        self.btn_choose_dir = QtWidgets.QPushButton("选择 *_description 目录")
        self.btn_choose_dir.clicked.connect(self.choose_description_dir)
        dir_layout.addWidget(self.btn_choose_dir)
        self.lbl_dir = QtWidgets.QLabel("未选择目录")
        dir_layout.addWidget(self.lbl_dir)
        left_panel.addLayout(dir_layout)

        # 操作按钮
        btn_layout = QtWidgets.QGridLayout()
        self.btn_load_parts = QtWidgets.QPushButton("批量加载零件(XML+STL)")
        self.btn_load_parts.clicked.connect(self.batch_load_parts)
        btn_layout.addWidget(self.btn_load_parts,0,0)
        self.btn_save_proj = QtWidgets.QPushButton("保存项目")
        self.btn_save_proj.clicked.connect(self.save_project)
        btn_layout.addWidget(self.btn_save_proj,0,1)
        self.btn_load_proj = QtWidgets.QPushButton("加载项目")
        self.btn_load_proj.clicked.connect(self.load_project)
        btn_layout.addWidget(self.btn_load_proj,0,2)
        self.btn_export_urdf = QtWidgets.QPushButton("导出 URDF")
        self.btn_export_urdf.clicked.connect(self.export_urdf)
        btn_layout.addWidget(self.btn_export_urdf,1,0)
        self.btn_reset_cam = QtWidgets.QPushButton("重置相机")
        self.btn_reset_cam.clicked.connect(lambda: self.viewer.reset_camera())
        btn_layout.addWidget(self.btn_reset_cam,1,1)
        left_panel.addLayout(btn_layout)

        # 状态输出
        self.txt_log = QtWidgets.QTextEdit(); self.txt_log.setReadOnly(True)
        left_panel.addWidget(QtWidgets.QLabel("日志:"))
        left_panel.addWidget(self.txt_log,1)

        # 中部: 节点图 + 检查器
        mid_layout = QtWidgets.QVBoxLayout(); h.addLayout(mid_layout,3)
        self.graph_view = NodeGraph()
        self.graph_view.register_node(BaseLinkNode)
        self.graph_view.register_node(FooNode)
        self._setup_graph_signals()
        self.graph_widget = self.graph_view.widget
        mid_layout.addWidget(self.graph_widget,4)

        # 右侧: STL 预览 + Inspector
        right_layout = QtWidgets.QVBoxLayout(); h.addLayout(right_layout,4)
        self.viewer = STLViewerWidget()
        right_layout.addWidget(self.viewer,6)
        self.inspector = InspectorWindow(stl_viewer=self.viewer)
        right_layout.addWidget(self.inspector,4)

        # 初始 base_link
        self.base_node = self.graph_view.create_node('insilico.nodes.BaseLinkNode', name='base_link', pos=(0,0))

        self.description_dir = None
        self.meshes_dir = None
        self.robot_name = 'robot'

    # ---------------- Graph 事件 ----------------
    def _setup_graph_signals(self):
        self.graph_view.node_double_clicked.connect(self._on_node_double_clicked)

    def _on_node_double_clicked(self, node):
        self.show_inspector(node)

    def show_inspector(self, node, pos=None):
        self.inspector.update_info(node)
        self.viewer.add_or_update_node_stl(node)

    # ---------------- 目录 & 加载 ----------------
    def choose_description_dir(self):
        d = QFileDialog.getExistingDirectory(self, "选择 *_description 目录")
        if not d: return
        self.description_dir = d
        self.lbl_dir.setText(os.path.basename(d))
        name = os.path.basename(d)
        if name.endswith('_description'): self.robot_name = name[:-12]
        meshes_path = os.path.join(d,'meshes')
        if os.path.isdir(meshes_path):
            self.meshes_dir = meshes_path
        self.log(f"已选择目录: {d}, robot_name={self.robot_name}")

    def batch_load_parts(self):
        if not self.meshes_dir:
            self.log("请先选择 *_description 目录 (需包含 meshes)")
            return
        # 遍历 meshes 下 *.xml
        for fn in os.listdir(self.meshes_dir):
            if not fn.lower().endswith('.xml'): continue
            xml_path = os.path.join(self.meshes_dir, fn)
            try:
                self._load_single_part(xml_path)
            except Exception as e:
                self.log(f"加载失败 {fn}: {e}")
        self.viewer.reset_camera()

    def _load_single_part(self, xml_path):
        tree = ET.parse(xml_path); root = tree.getroot()
        link_elem = root.find('link');
        if link_elem is None: return
        link_name = link_elem.get('name','part')
        # 若已存在节点则复用
        existing = [n for n in self.graph_view.all_nodes() if n.name()==link_name]
        if existing:
            node = existing[0]
        else:
            node = self.graph_view.create_node('insilico.nodes.FooNode', name=link_name, pos=(200, 80+len(self.graph_view.all_nodes())*40))
        # origin
        origin_elem = link_elem.find('origin')
        if origin_elem is not None:
            try:
                node.origin_xyz = [float(v) for v in origin_elem.get('xyz','0 0 0').split()[:3]]
                node.origin_rpy = [float(v) for v in origin_elem.get('rpy','0 0 0').split()[:3]]
            except: pass
        # inertial
        inertial = link_elem.find('inertial')
        if inertial is not None:
            mass_elem = inertial.find('mass')
            inertia_elem = inertial.find('inertia')
            if mass_elem is not None:
                try: node.mass_value = float(mass_elem.get('value','0'))
                except: pass
            if inertia_elem is not None:
                for k in ['ixx','ixy','ixz','iyy','iyz','izz']:
                    try: node.inertia[k]=float(inertia_elem.get(k,'0'))
                    except: node.inertia[k]=0.0
        # material color
        mat = root.find('material/color')
        if mat is not None:
            try:
                rgba = [float(x) for x in mat.get('rgba','1 1 1 1').split()]
                node.node_color = rgba[:3]
            except: pass
        # points
        pts = root.findall('point')
        node.points = []
        node.output_count = 0
        for i,pt in enumerate(pts):
            if i==0 and not isinstance(node, BaseLinkNode):
                # 已有一个 output, 重置它
                node.points.append({'name':pt.get('name',f'point_{i+1}'),'type':pt.get('type','fixed'),'xyz':[0,0,0],'axis':3})
            else:
                if isinstance(node, FooNode) and i>0: node._add_output()
            xyz_elem = pt.find('point_xyz'); axis_elem = pt.find('joint_axis')
            xyz=[0,0,0]
            if xyz_elem is not None and xyz_elem.text:
                try: xyz=[float(v) for v in xyz_elem.text.strip().split()[:3]]
                except: pass
            axis_index=3
            if axis_elem is not None:
                vec = [float(v) for v in axis_elem.get('xyz','0 0 0').split()]
                if vec==[1,0,0]: axis_index=0
                elif vec==[0,1,0]: axis_index=1
                elif vec==[0,0,1]: axis_index=2
            # 覆盖对应点
            if i < len(node.points):
                node.points[i]['name']=pt.get('name',f'point_{i+1}')
                node.points[i]['xyz']=xyz
                node.points[i]['axis']=axis_index
            else:
                node.points.append({'name':pt.get('name',f'point_{i+1}'),'type':'fixed','xyz':xyz,'axis':axis_index})
        # stl 文件(与 xml 同名 .stl)
        stem = os.path.splitext(os.path.basename(xml_path))[0]
        stl_candidate = os.path.join(self.meshes_dir, f"{stem}.stl")
        if os.path.exists(stl_candidate): node.stl_file = stl_candidate
        self.viewer.add_or_update_node_stl(node)
        self.log(f"加载零件: {link_name}")

    # ---------------- 项目保存/加载 ----------------
    def save_project(self):
        if not self.description_dir: self.log("未选择目录"); return
        path,_ = QFileDialog.getSaveFileName(self,"保存项目", os.path.join(self.description_dir,'assembly.ukproj'),"URDF Kitchen 项目 (*.ukproj)")
        if not path: return
        data = {'nodes':[], 'connections':[], 'robot_name':self.robot_name}
        for n in self.graph_view.all_nodes():
            d = {
                'cls': n.__class__.__name__,
                'name': n.name(),
                'pos': n.pos(),
                'stl': getattr(n,'stl_file',None),
                'origin_xyz': getattr(n,'origin_xyz',[0,0,0]),
                'origin_rpy': getattr(n,'origin_rpy',[0,0,0]),
                'mass': getattr(n,'mass_value',0.0),
                'inertia': getattr(n,'inertia',{}),
                'color': getattr(n,'node_color',[1,1,1]),
                'points': getattr(n,'points',[])
            }
            data['nodes'].append(d)
        # connections
        for n in self.graph_view.all_nodes():
            for p in n.output_ports():
                for cp in p.connected_ports():
                    data['connections'].append({'out_node':n.name(),'out_port':p.name(),'in_node':cp.node().name(),'in_port':cp.name()})
        import json
        with open(path,'w',encoding='utf-8') as f: json.dump(data,f,ensure_ascii=False,indent=2)
        self.log(f"项目已保存: {path}")

    def load_project(self):
        path,_ = QFileDialog.getOpenFileName(self,"加载项目","","URDF Kitchen 项目 (*.ukproj)")
        if not path: return
        import json
        with open(path,'r',encoding='utf-8') as f: data=json.load(f)
        self.robot_name = data.get('robot_name','robot')
        # 清空图
        for n in list(self.graph_view.all_nodes()): self.graph_view.delete_node(n)
        name_map = {}
        for nd in data.get('nodes',[]):
            cls = nd.get('cls','FooNode')
            if cls == 'BaseLinkNode': node = self.graph_view.create_node('insilico.nodes.BaseLinkNode', name=nd['name'], pos=tuple(nd.get('pos',(0,0))))
            else: node = self.graph_view.create_node('insilico.nodes.FooNode', name=nd['name'], pos=tuple(nd.get('pos',(0,0))))
            node.stl_file = nd.get('stl')
            node.origin_xyz = nd.get('origin_xyz',[0,0,0])
            node.origin_rpy = nd.get('origin_rpy',[0,0,0])
            node.mass_value = nd.get('mass',0.0)
            node.inertia = nd.get('inertia',{})
            node.node_color = nd.get('color',[1,1,1])
            node.points = nd.get('points',[])
            # 输出端口数量匹配
            if isinstance(node, FooNode):
                needed = max(1,len(node.points))
                while node.output_count < needed:
                    node._add_output()
            name_map[node.name()]=node
            self.viewer.add_or_update_node_stl(node)
        # connections
        for c in data.get('connections',[]):
            on = name_map.get(c['out_node']); inn = name_map.get(c['in_node'])
            if on and inn:
                try:
                    self.graph_view.connect_ports(on.output_port(c['out_port']), inn.input_port(c['in_port']))
                except Exception as e:
                    self.log(f"连接失败: {c} {e}")
        self.log(f"项目已加载: {path}")

    # ---------------- URDF 导出 ----------------
    def export_urdf(self):
        if not self.description_dir:
            self.log("未选择描述目录")
            return
        urdf_dir = os.path.join(self.description_dir,'urdf')
        os.makedirs(urdf_dir, exist_ok=True)
        path,_ = QFileDialog.getSaveFileName(self,"导出 URDF", os.path.join(urdf_dir, f"{self.robot_name}.urdf"),"URDF (*.urdf)")
        if not path: return
        try:
            with open(path,'w',encoding='utf-8') as f:
                f.write(f"<?xml version='1.0'?>\n")
                f.write(f"<robot name='{self.robot_name}'>\n")
                # 材质集合
                materials=set()
                for n in self.graph_view.all_nodes():
                    self._write_link(f,n,materials)
                # joints
                for n in self.graph_view.all_nodes():
                    for p in n.output_ports():
                        for cp in p.connected_ports():
                            self._write_joint(f,n,cp.node())
                f.write("</robot>\n")
            self.log(f"URDF 已导出: {path}")
        except Exception as e:
            self.log(f"导出失败: {e}")

    def _write_joint(self, file, parent_node, child_node):
        """重写: 使用父节点对应point的axis决定joint"""
        try:
            origin_xyz = [0,0,0]
            axis_mode = 3  # 默认固定
            for port in parent_node.output_ports():
                for connected_port in port.connected_ports():
                    if connected_port.node() == child_node:
                        # 解析端口索引
                        pname = port.name()
                        idx = 0
                        if '_' in pname:
                            part = pname.split('_')[-1]
                            if part.isdigit(): idx = int(part)-1
                        if hasattr(parent_node,'points') and idx < len(parent_node.points):
                            pt = parent_node.points[idx]
                            origin_xyz = pt.get('xyz',[0,0,0])
                            axis_mode = pt.get('axis',3)
                        break
            joint_name = f"{parent_node.name()}_to_{child_node.name()}"
            if axis_mode == 3:
                file.write(f'  <joint name="{joint_name}" type="fixed">\n')
                file.write(f'    <origin xyz="{origin_xyz[0]} {origin_xyz[1]} {origin_xyz[2]}" rpy="0 0 0"/>\n')
                file.write(f'    <parent link="{parent_node.name()}"/>\n')
                file.write(f'    <child link="{child_node.name()}"/>\n')
                file.write('  </joint>\n')
            else:
                axis_vec = {0:'1 0 0',1:'0 1 0',2:'0 0 1'}[axis_mode]
                file.write(f'  <joint name="{joint_name}" type="revolute">\n')
                file.write(f'    <origin xyz="{origin_xyz[0]} {origin_xyz[1]} {origin_xyz[2]}" rpy="0 0 0"/>\n')
                file.write(f'    <axis xyz="{axis_vec}"/>\n')
                file.write(f'    <parent link="{parent_node.name()}"/>\n')
                file.write(f'    <child link="{child_node.name()}"/>\n')
                file.write('    <limit lower="-3.14159" upper="3.14159" effort="0" velocity="0"/>\n')
                file.write('  </joint>\n')
        except Exception as e:
            self.log(f"写joint出错: {e}")

    def _write_link(self, file, node, materials):
        try:
            file.write(f'  <link name="{node.name()}">\n')
            ox,oy,oz = getattr(node,'origin_xyz',[0,0,0])
            rr,rp,ry = getattr(node,'origin_rpy',[0,0,0])
            if hasattr(node,'mass_value') and hasattr(node,'inertia'):
                file.write('    <inertial>\n')
                file.write(f'      <origin xyz="{ox} {oy} {oz}" rpy="{rr} {rp} {ry}"/>\n')
                file.write(f'      <mass value="{node.mass_value:.6f}"/>\n')
                file.write('      <inertia')
                for k,v in node.inertia.items(): file.write(f' {k}="{v:.6f}"')
                file.write('/>\n')
                file.write('    </inertial>\n')
            if hasattr(node,'stl_file') and node.stl_file:
                mesh_dir_name = "meshes"
                if self.meshes_dir:
                    dir_name = os.path.basename(self.meshes_dir)
                    if dir_name.startswith('mesh'): mesh_dir_name = dir_name
                stl_filename = os.path.basename(node.stl_file)
                package_path = f"package://{self.robot_name}_description/{mesh_dir_name}/{stl_filename}"
                file.write('    <visual>\n')
                file.write(f'      <origin xyz="{ox} {oy} {oz}" rpy="{rr} {rp} {ry}"/>\n')
                file.write('      <geometry>\n')
                file.write(f'        <mesh filename="{package_path}"/>\n')
                file.write('      </geometry>\n')
                if hasattr(node,'node_color'):
                    hex_color = '#{:02x}{:02x}{:02x}'.format(int(node.node_color[0]*255),int(node.node_color[1]*255),int(node.node_color[2]*255))
                    file.write(f'      <material name="{hex_color}"/>\n')
                file.write('    </visual>\n')
                file.write('    <collision>\n')
                file.write(f'      <origin xyz="{ox} {oy} {oz}" rpy="{rr} {rp} {ry}"/>\n')
                file.write('      <geometry>\n')
                file.write(f'        <mesh filename="{package_path}"/>\n')
                file.write('      </geometry>\n')
                file.write('    </collision>\n')
            file.write('  </link>\n')
        except Exception as e:
            self.log(f"写link出错: {e}")

    # ---------------- 实用 ----------------
    def log(self, msg):
        self.txt_log.append(msg)
        print(msg)


def main():
    app = QtWidgets.QApplication(sys.argv)
    apply_dark_theme(app)
    wnd = AssemblerWindow(); wnd.show()
    signal.signal(signal.SIGINT, lambda *a: app.quit())
    timer = QtCore.QTimer(); timer.start(500); timer.timeout.connect(lambda: None)
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
