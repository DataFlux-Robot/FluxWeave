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
import math
from functools import partial
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
from URDF_CustomNodeGraph import CustomNodeGraph
from URDF_STLViewerWidget import STLViewerWidget
from URDF_BASIC1 import BaseLinkNode


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




class JointControlPanel(QtWidgets.QWidget):
    slider_precision = 10  # 0.1 degree resolution

    def __init__(self, graph, parent=None):
        super().__init__(parent)
        self.graph = graph
        self.rows: dict = {}
        self._block_updates = False
        self._host_splitter: QtWidgets.QSplitter | None = None
        self._splitter_index: int | None = None
        self._saved_size: int = 320

        self.setMinimumWidth(340)
        size_policy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        self.setSizePolicy(size_policy)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        title = QtWidgets.QLabel("关节运动测试及上下限度指定")
        title.setWordWrap(True)
        title.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop)
        layout.addWidget(title)

        self.scroll_area = QtWidgets.QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        layout.addWidget(self.scroll_area, 1)

        self.container = QtWidgets.QWidget()
        self.scroll_area.setWidget(self.container)
        self.container_layout = QtWidgets.QVBoxLayout(self.container)
        self.container_layout.setContentsMargins(0, 0, 0, 0)
        self.container_layout.setSpacing(6)

        self.refresh_button = QtWidgets.QPushButton("刷新列表")
        self.refresh_button.clicked.connect(self.refresh_joint_list)
        layout.addWidget(self.refresh_button)

        self.refresh_joint_list()

    def set_graph(self, graph):
        self.graph = graph
        self.refresh_joint_list()

    def set_host_splitter(self, splitter: QtWidgets.QSplitter, index: int):
        self._host_splitter = splitter
        self._splitter_index = index
        splitter.setCollapsible(index, True)

    def refresh_joint_list(self):
        if self._block_updates:
            return
        self._block_updates = True
        try:
            self._clear_rows()

            joint_nodes = []
            connected_nodes = set()
            if self.graph is not None:
                base_node = self.graph.get_node_by_name('base_link') if hasattr(self.graph, 'get_node_by_name') else None
                if base_node is None:
                    self._set_active(False)
                    return
                stack = [base_node]
                visited = {base_node}
                while stack:
                    current = stack.pop()
                    for port in current.output_ports():
                        for connected_port in port.connected_ports():
                            child_node = connected_port.node()
                            if child_node not in visited:
                                visited.add(child_node)
                                stack.append(child_node)
                                connected_nodes.add(child_node)
                for node in self.graph.all_nodes():
                    if isinstance(node, BaseLinkNode):
                        continue
                    if getattr(node, 'massless_decoration', False):
                        continue
                    if connected_nodes and node not in connected_nodes:
                        continue
                    if not node.input_ports():
                        continue
                    if not node.input_ports()[0].connected_ports():
                        continue
                    joint_nodes.append(node)

            if not joint_nodes:
                self._set_active(False)
                return

            joint_nodes.sort(key=lambda n: n.name())
            for node in joint_nodes:
                self._add_joint_row(node)

            self.container_layout.addStretch()
            self._set_active(True)
        finally:
            self._block_updates = False

    def _clear_rows(self):
        while self.container_layout.count():
            item = self.container_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
        self.rows.clear()

    def _joint_label(self, node):
        parent_name = "base_link"
        if node.input_ports():
            connections = node.input_ports()[0].connected_ports()
            if connections:
                parent_name = connections[0].node().name()
        axis_vec = None
        if parent_name and hasattr(node, 'graph'):
            parent_node = node.graph.get_node_by_name(parent_name)
            if parent_node and parent_node.output_ports():
                for port_idx, port in enumerate(parent_node.output_ports()):
                    for connected_port in port.connected_ports():
                        if connected_port.node() == node:
                            if hasattr(parent_node, 'points') and port_idx < len(parent_node.points):
                                axis_vec = parent_node.points[port_idx].get('axis')
                            break
        axis_hint = ""
        axis_index = getattr(node, 'rotation_axis', 3)
        if axis_vec is not None:
            try:
                axis_hint = f" axis({axis_vec[0]:.1f},{axis_vec[1]:.1f},{axis_vec[2]:.1f})"
            except (TypeError, ValueError, IndexError):
                axis_hint = ""
        elif axis_index == 0:
            axis_hint = " axis X"
        elif axis_index == 1:
            axis_hint = " axis Y"
        elif axis_index == 2:
            axis_hint = " axis Z"
        else:
            axis_hint = " 固定"
        return f"{parent_name} → {node.name()}{axis_hint}"

    def _add_joint_row(self, node):
        row_widget = QtWidgets.QWidget()
        row_layout = QtWidgets.QGridLayout(row_widget)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setHorizontalSpacing(6)
        row_layout.setVerticalSpacing(4)

        row_layout.setColumnStretch(0, 0)
        row_layout.setColumnStretch(1, 0)
        row_layout.setColumnStretch(2, 1)
        row_layout.setColumnStretch(3, 0)

        name_label = QtWidgets.QLabel(self._joint_label(node))
        name_label.setStyleSheet("font-weight: bold;")
        name_label.setWordWrap(True)
        row_layout.addWidget(name_label, 0, 0, 2, 1)

        lower_spin = QtWidgets.QDoubleSpinBox()
        lower_spin.setDecimals(2)
        lower_spin.setSuffix("°")
        lower_spin.setSingleStep(1.0)
        lower_spin.setRange(-720.0, 720.0)
        lower_spin.setValue(math.degrees(getattr(node, 'joint_limit_lower', -math.pi)))
        row_layout.addWidget(lower_spin, 0, 1)

        slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        slider.setTickPosition(QtWidgets.QSlider.TickPosition.NoTicks)
        slider.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        row_layout.addWidget(slider, 0, 2, 2, 1)

        upper_spin = QtWidgets.QDoubleSpinBox()
        upper_spin.setDecimals(2)
        upper_spin.setSuffix("°")
        upper_spin.setSingleStep(1.0)
        upper_spin.setRange(-720.0, 720.0)
        upper_spin.setValue(math.degrees(getattr(node, 'joint_limit_upper', math.pi)))
        row_layout.addWidget(upper_spin, 0, 3)

        value_label = QtWidgets.QLabel(self._format_angle_label(getattr(node, 'joint_position', 0.0)))
        value_label.setAlignment(QtCore.Qt.AlignCenter)
        row_layout.addWidget(value_label, 1, 1, 1, 3)

        self.container_layout.addWidget(row_widget)

        row_data = {
            'widget': row_widget,
            'label': name_label,
            'lower': lower_spin,
            'upper': upper_spin,
            'slider': slider,
            'value_label': value_label,
        }
        self.rows[node] = row_data

        lower_spin.valueChanged.connect(partial(self._on_lower_changed, node))
        upper_spin.valueChanged.connect(partial(self._on_upper_changed, node))
        slider.valueChanged.connect(partial(self._on_slider_changed, node))

        self._configure_slider(node, trigger_recalc=False)

    def _configure_slider(self, node, trigger_recalc=True):
        if node not in self.rows:
            return
        row = self.rows[node]
        slider: QtWidgets.QSlider = row['slider']
        lower_spin: QtWidgets.QDoubleSpinBox = row['lower']
        upper_spin: QtWidgets.QDoubleSpinBox = row['upper']

        lower = lower_spin.value()
        upper = upper_spin.value()
        if lower > upper:
            lower, upper = upper, lower
            lower_spin.blockSignals(True)
            upper_spin.blockSignals(True)
            lower_spin.setValue(lower)
            upper_spin.setValue(upper)
            lower_spin.blockSignals(False)
            upper_spin.blockSignals(False)

        node.joint_limit_lower = math.radians(lower)
        node.joint_limit_upper = math.radians(upper)

        slider_min = int(round(lower * self.slider_precision))
        slider_max = int(round(upper * self.slider_precision))

        slider.blockSignals(True)
        slider.setMinimum(slider_min)
        slider.setMaximum(slider_max if slider_max != slider_min else slider_min)
        slider.blockSignals(False)

        delta = abs(upper - lower)
        is_rotatable = getattr(node, 'rotation_axis', 3) != 3 and delta >= 1e-3
        slider.setEnabled(is_rotatable)

        current_deg = math.degrees(getattr(node, 'joint_position', 0.0))
        clamped_deg = max(min(current_deg, upper), lower)
        position_changed = abs(clamped_deg - current_deg) > 1e-6
        if position_changed:
            node.joint_position = math.radians(clamped_deg)

        slider_value = int(round(clamped_deg * self.slider_precision))
        slider.blockSignals(True)
        slider.setValue(slider_value)
        slider.blockSignals(False)

        row['value_label'].setText(self._format_angle_label(node.joint_position))

        if trigger_recalc and position_changed and self.graph is not None:
            self.graph.recalculate_all_positions()
            if getattr(self.graph, 'stl_viewer', None):
                self.graph.stl_viewer.vtkWidget.GetRenderWindow().Render()

    def _on_lower_changed(self, node, value):
        if self._block_updates:
            return
        self._block_updates = True
        node.joint_limit_lower = math.radians(float(value))
        self._configure_slider(node)
        self._block_updates = False

    def _on_upper_changed(self, node, value):
        if self._block_updates:
            return
        self._block_updates = True
        node.joint_limit_upper = math.radians(float(value))
        self._configure_slider(node)
        self._block_updates = False

    def _on_slider_changed(self, node, slider_value):
        if self._block_updates:
            return
        if node not in self.rows:
            return
        position_deg = slider_value / self.slider_precision
        lower_deg = math.degrees(getattr(node, 'joint_limit_lower', -math.pi))
        upper_deg = math.degrees(getattr(node, 'joint_limit_upper', math.pi))
        position_deg = max(min(position_deg, upper_deg), lower_deg)
        node.joint_position = math.radians(position_deg)
        row = self.rows[node]
        row['value_label'].setText(self._format_angle_label(node.joint_position))
        if self.graph is not None:
            self.graph.recalculate_all_positions()
            if getattr(self.graph, 'stl_viewer', None):
                self.graph.stl_viewer.vtkWidget.GetRenderWindow().Render()

    @staticmethod
    def _format_angle_label(radians):
        degrees = math.degrees(radians)
        return f"当前: {degrees:.1f}° ({radians:.3f} rad)"

    def _set_active(self, active: bool):
        if self._host_splitter is None or self._splitter_index is None:
            self.setVisible(active)
            return

        splitter = self._host_splitter
        index = self._splitter_index
        sizes = splitter.sizes()

        if active:
            if not self.isVisible():
                self.setVisible(True)
                if self._saved_size <= 0:
                    self._saved_size = max(320, self.minimumWidth())
                if index < len(sizes):
                    sizes[index] = self._saved_size
                    total = sum(sizes)
                    if total == 0:
                        total = 1
                    splitter.setSizes(sizes)
        else:
            if self.isVisible():
                if index < len(sizes):
                    self._saved_size = sizes[index]
                    sizes[index] = 0
                    splitter.setSizes(sizes)
                self.setVisible(False)

# ユーティリティ関数
def signal_handler(signum, frame):
    print("\nCtrl+C pressed. Closing all windows and exiting...")
    shutdown()

def load_project(graph, joint_panel=None):
    """プロジェクトを読み込み"""
    try:
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            None,
            "Load Project",
            "",
            "XML Files (*.xml)"
        )

        if not file_path:
            print("Load cancelled")
            return

        project_base_dir = os.path.dirname(file_path)
        print(f"Project base directory: {project_base_dir}")

        # XMLファイルを解析
        tree = ET.parse(file_path)
        root = tree.getroot()

        # meshesディレクトリのパスを取得
        meshes_dir = None
        meshes_dir_elem = root.find("meshes_dir")
        if meshes_dir_elem is not None and meshes_dir_elem.text:
            meshes_dir = os.path.normpath(os.path.join(project_base_dir, meshes_dir_elem.text))
            if not os.path.exists(meshes_dir):
                # meshesディレクトリが見つからない場合、ユーザーに選択を求める
                msg = QtWidgets.QMessageBox()
                msg.setIcon(QtWidgets.QMessageBox.Question)
                msg.setText("Meshes directory not found")
                msg.setInformativeText("Would you like to select the meshes directory location?")
                msg.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
                msg.setDefaultButton(QtWidgets.QMessageBox.Yes)

                if msg.exec() == QtWidgets.QMessageBox.Yes:
                    meshes_dir = QtWidgets.QFileDialog.getExistingDirectory(
                        None,
                        "Select Meshes Directory",
                        project_base_dir,
                        QtWidgets.QFileDialog.ShowDirsOnly
                    )
                    if not meshes_dir:
                        print("Meshes directory selection cancelled")
                        meshes_dir = None
                    else:
                        print(f"Selected meshes directory: {meshes_dir}")

        # 現在のグラフをクリア
        graph.clear_graph()

        # プロジェクトファイルを読み込み
        success = graph.load_project(file_path)

        if success:
            print("Project loaded, resolving STL paths...")
            # STLファイルのパスを解決
            for node in graph.all_nodes():
                if hasattr(node, 'stl_file') and node.stl_file:
                    try:
                        stl_path = node.stl_file
                        if not os.path.isabs(stl_path):
                            # まずmeshesディレクトリからの相対パスを試す
                            if meshes_dir:
                                abs_stl_path = os.path.normpath(os.path.join(meshes_dir, stl_path))
                                if os.path.exists(abs_stl_path):
                                    node.stl_file = abs_stl_path
                                    print(f"Found STL file in meshes dir for node {node.name()}: {abs_stl_path}")
                                    if graph.stl_viewer:
                                        graph.stl_viewer.load_stl_for_node(node)
                                    continue

                            # プロジェクトディレクトリからの相対パスを試す
                            abs_stl_path = os.path.normpath(os.path.join(project_base_dir, stl_path))
                            if os.path.exists(abs_stl_path):
                                node.stl_file = abs_stl_path
                                print(f"Found STL file in project dir for node {node.name()}: {abs_stl_path}")
                                if graph.stl_viewer:
                                    graph.stl_viewer.load_stl_for_node(node)
                            else:
                                print(f"Warning: Could not find STL file for node {node.name()}: {stl_path}")
                        else:
                            if os.path.exists(stl_path):
                                print(f"Using absolute STL path for node {node.name()}: {stl_path}")
                                if graph.stl_viewer:
                                    graph.stl_viewer.load_stl_for_node(node)
                            else:
                                print(f"Warning: STL file not found: {stl_path}")

                    except Exception as e:
                        print(f"Error resolving STL path for node {node.name()}: {str(e)}")
                        traceback.print_exc()

            print(f"Project loaded successfully from: {file_path}")

            # 位置を再計算
            graph.recalculate_all_positions()

            # 3Dビューをリセット
            if graph.stl_viewer:
                QtCore.QTimer.singleShot(500, lambda: graph.stl_viewer.reset_view_to_fit())

            if joint_panel is not None:
                joint_panel.refresh_joint_list()

        else:
            print("Failed to load project")

    except Exception as e:
        print(f"Error loading project: {str(e)}")
        traceback.print_exc()

def delete_selected_node(graph):
    selected_nodes = graph.selected_nodes()
    if selected_nodes:
        for node in selected_nodes:
            # BaseLinkNodeは削除不可
            if isinstance(node, BaseLinkNode):
                print("Cannot delete Base Link node")
                continue
            graph.remove_node(node)
        print(f"Deleted {len(selected_nodes)} node(s)")
    else:
        print("No node selected for deletion")
        for connected_port in port.connected_ports():
            child_node = connected_port.node()
            self.print_node_hierarchy(child_node, level + 1)

def cleanup_and_exit():
    """アプリケーションのクリーンアップと終了"""
    try:
        # グラフのクリーンアップ
        if 'graph' in globals():
            try:
                graph.cleanup()
            except Exception as e:
                print(f"Error cleaning up graph: {str(e)}")

        # STLビューアのクリーンアップ
        if 'stl_viewer' in globals():
            try:
                stl_viewer.cleanup()
            except Exception as e:
                print(f"Error cleaning up STL viewer: {str(e)}")

        # その他のリソースのクリーンアップ
        for window in QtWidgets.QApplication.topLevelWidgets():
            try:
                window.close()
            except Exception as e:
                print(f"Error closing window: {str(e)}")

    except Exception as e:
        print(f"Error during cleanup: {str(e)}")
    finally:
        # アプリケーションの終了
        if QtWidgets.QApplication.instance():
            QtWidgets.QApplication.instance().quit()

def signal_handler(signum, frame):
    """Ctrl+Cシグナルのハンドラ"""
    print("\nCtrl+C detected, closing application...")
    try:
        # アプリケーションのクリーンアップと終了
        if QtWidgets.QApplication.instance():
            # 全てのウィンドウを閉じる
            for window in QtWidgets.QApplication.topLevelWidgets():
                try:
                    window.close()
                except:
                    pass

            # アプリケーションの終了
            QtWidgets.QApplication.instance().quit()
    except Exception as e:
        print(f"Error during application shutdown: {str(e)}")
    finally:
        # 強制終了
        sys.exit(0)

def center_window_top_left(window):
    """ウィンドウを画面の左上に配置"""
    screen = QtWidgets.QApplication.primaryScreen().geometry()
    window.move(0, 0)


if __name__ == '__main__':
    try:
        # Ctrl+Cシグナルハンドラの設定
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        app = QtWidgets.QApplication(sys.argv)
        apply_dark_theme(app)

        # アプリケーション終了時のクリーンアップ設定
        app.aboutToQuit.connect(cleanup_and_exit)

        timer = QtCore.QTimer()
        timer.start(500)
        timer.timeout.connect(lambda: None)

        # メインウィンドウの作成
        main_window = QtWidgets.QMainWindow()
        main_window.setWindowTitle("URDF Kitchen - Assembler - v0.0.1")
        main_window.resize(1200, 600)

        # セントラルウィジェットの設定
        central_widget = QtWidgets.QWidget()
        main_layout = QtWidgets.QHBoxLayout(central_widget)

        # STLビューアとグラフの設定（先に作成）
        stl_viewer = STLViewerWidget(central_widget)
        graph = CustomNodeGraph(stl_viewer)
        graph.setup_custom_view()

        # 右側ジョイントパネル
        joint_panel = JointControlPanel(graph)
        graph.set_joint_change_callback(joint_panel.refresh_joint_list)

        # base_linkノードの作成
        base_node = graph.create_base_link()

        # 左パネルの設定
        left_panel = QtWidgets.QWidget()
        left_panel.setMinimumWidth(150)
        left_panel.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        left_layout = QtWidgets.QVBoxLayout(left_panel)

        # 名前入力フィールドの設定
        name_label = QtWidgets.QLabel("Name:")
        left_layout.addWidget(name_label)
        name_input = QtWidgets.QLineEdit("robot_x")
        name_input.setFixedWidth(120)
        name_input.setStyleSheet("QLineEdit { padding-left: 3px; padding-top: 0px; padding-bottom: 0px; }")
        left_layout.addWidget(name_input)


        # 名前入力フィールドとグラフを接続
        # 在urdf_kitchen_Assembler_V2.py中，将部件BODY_UP.stl的连接点和存在多个连接点的BODY_DOWN.stl部件的任意连接点进行连接建立关节生成urdf过程中，其暴露出如下问题：
        #
        #
        # 我们已经知道，在urdf中定义定义两个link间的关系需要确定如下信息：
        # 1,parent link名称
        # 2,child link名称
        # 3,origin xyz
        # 4,originrpy
        # 5,axis xyz
        #
        # urdf_kitchen_Assembler_V2.py虽然通过部件间连线很好解决了“parent link名称”和“child link名称”的确定，但是其存在如下问题
        # 1, origin xyz定义过程中，虽然会索引parent link中的连接点作为origin xyz，但是child link和parent link的连接点的对应位置却很随意，没有图形预览中却没有实现两点的完全重合
        # 2, originrpy目前没有明确的定义逻辑，是否需要在child link的连接点以及parent link的连接点间的连接线上进行明确定义并进行实时预览
        # 3, axis xyz进行旋转过程中，没有明确保证其一定旋转过程中的原点一定在child link的连接点以及parent link的连接点而不是仅仅是stl的默认坐标原点
        #
        # 同时urdf_kitchen_Assembler_V2.py中还存在如下架构问题：
        # 1,5, BODY_UP的in是什么，感觉多余，我感觉感觉所有基于关节的接驳点（包括和base_link的接驳点）应该都在urdf_kitchen_PartsEditor_V2.py中进行指定且尽可能等价以体现机器人部件组间类似lego的自由组合，而不是自动生成in这种，当然这样会涉及到更为复杂的urdf内部转换（使得parent和child间的相对位置关系更为复杂），但是这正是当前开发项目的魅力和核心价值所在
        # 6,删除urdf_kitchen_Assembler_V2.py中的“Add Node（因为目前操作核心是STL文件）”和“recalc Position（这个被自动/主动进行了）”（graphが定義された後に接続）
        name_input.textChanged.connect(graph.update_robot_name)




        # ボタンの作成と設定
        buttons = {
            "--spacer1--": None,
            "Add STL(s)": None,  # 逐个添加 STL 作为节点
            "--spacer2--": None,
            "Load Project": None,
            "Save Project": None,
            "--spacer3--": None,
            "Preview URDF": None,
            "Export URDF": None,
            "Export for Unity": None,
            "--spacer4--": None,
            "open urdf-loaders": None
        }

        for button_text in buttons.keys():
            if button_text.startswith("--spacer"):
                # スペーサーの追加
                spacer = QtWidgets.QWidget()
                spacer.setFixedHeight(1)  # スペースの高さを1ピクセルに設定
                left_layout.addWidget(spacer)
            else:
                # 通常のボタンの追加
                button = QtWidgets.QPushButton(button_text)
                button.setFixedWidth(120)
                left_layout.addWidget(button)
                buttons[button_text] = button

        left_layout.addStretch()

        # ボタンのコネクション設定
        buttons["Add STL(s)"].clicked.connect(graph.add_parts_from_stl)
        buttons["Save Project"].clicked.connect(graph.save_project)
        buttons["Load Project"].clicked.connect(lambda: load_project(graph, joint_panel))
        buttons["Preview URDF"].clicked.connect(graph.preview_urdf)
        buttons["Export URDF"].clicked.connect(lambda: graph.export_urdf())
        buttons["Export for Unity"].clicked.connect(graph.export_for_unity)
        buttons["open urdf-loaders"].clicked.connect(
            lambda: QtGui.QDesktopServices.openUrl(
                QtCore.QUrl(
                    "https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/")
            )
        )









        # # ボタンの作成と設定
        # buttons = {
        #     "--spacer1--": None,  # スペーサー用のダミーキー
        #     "Import Parts": None,
        #     "--spacer2--": None,  # スペーサー用のダミーキー
        #     "Add Node": None,
        #     "Delete Node": None,
        #     "Recalc Positions": None,
        #     "--spacer3--": None,  # スペーサー用のダミーキー
        #     "Load Project": None,
        #     "Save Project": None,
        #     "--spacer4--": None,  # スペーサー用のダミーキー
        #     "Export URDF": None,
        # }


        # for button_text in buttons.keys():
        #     if button_text.startswith("--spacer"):
        #         # スペーサーの追加
        #         spacer = QtWidgets.QWidget()
        #         spacer.setFixedHeight(1)  # スペースの高さを20ピクセルに設定
        #         left_layout.addWidget(spacer)
        #     else:
        #         # 通常のボタンの追加
        #         button = QtWidgets.QPushButton(button_text)
        #         button.setFixedWidth(120)
        #         left_layout.addWidget(button)
        #         buttons[button_text] = button

        # left_layout.addStretch()

        # # ボタンのコネクション設定
        # buttons["Add Node"].clicked.connect(
        #     lambda: graph.create_node(
        #         'insilico.nodes.FooNode',
        #         name=f'Node_{len(graph.all_nodes())}',
        #         pos=QtCore.QPointF(0, 0)
        #     )
        # )
        # buttons["Delete Node"].clicked.connect(
        #     lambda: delete_selected_node(graph))
        # buttons["Import Parts"].clicked.connect(graph.import_xmls_from_folder)
        # buttons["Export URDF"].clicked.connect(lambda: graph.export_urdf())
        # buttons["Save Project"].clicked.connect(graph.save_project)  # lambdaを使わない直接接続
        # buttons["Load Project"].clicked.connect(lambda: load_project(graph))
        # buttons["Recalc Positions"].clicked.connect(graph.recalculate_all_positions)

        # スプリッターの設定
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.addWidget(graph.widget)
        splitter.addWidget(stl_viewer)
        splitter.setSizes([800, 400])

        right_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        right_splitter.addWidget(splitter)
        right_splitter.addWidget(joint_panel)
        right_splitter.setStretchFactor(0, 3)
        right_splitter.setStretchFactor(1, 1)

        joint_panel.set_host_splitter(right_splitter, 1)
        joint_panel.refresh_joint_list()

        top_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        top_splitter.addWidget(left_panel)
        top_splitter.addWidget(right_splitter)
        top_splitter.setStretchFactor(0, 0)
        top_splitter.setStretchFactor(1, 1)
        top_splitter.setCollapsible(0, False)
        top_splitter.setCollapsible(1, False)

        # メインレイアウトの設定
        main_layout.addWidget(top_splitter)

        # セントラルウィジェットの設定
        main_window.setCentralWidget(central_widget)

        # グラフに名前入力フィールドを関連付け
        graph.name_input = name_input

        # ウィンドウを画面の左上に配置して表示
        center_window_top_left(main_window)
        main_window.show()

        print("Application started. Double-click on a node to open the inspector.")
        print("Right-click a node to access deletion options.")
        print("Use 'Save' and 'Load' buttons to save and load your project.")
        print("Press Ctrl+C in the terminal to close all windows and exit.")

        # タイマーの設定（シグナル処理のため）
        timer = QtCore.QTimer()
        timer.start(500)
        timer.timeout.connect(lambda: None)

        # アプリケーションの実行
        sys.exit(app.exec() if hasattr(app, 'exec') else app.exec_())

    except Exception as e:
        print(f"An error occurred: {str(e)}")
        print("Traceback:")
        print(traceback.format_exc())
        cleanup_and_exit()
        sys.exit(1)
