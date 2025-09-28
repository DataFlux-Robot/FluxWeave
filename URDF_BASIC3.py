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
from stl_metadata import extract_metadata_text



class InspectorWindow(QtWidgets.QWidget):

    def __init__(self, parent=None, stl_viewer=None):
        super(InspectorWindow, self).__init__(parent)
        self.setWindowTitle("节点检查器")
        self.setMinimumWidth(400)
        self.setMinimumHeight(600)

        self.setWindowFlags(self.windowFlags() |
                            QtCore.Qt.WindowStaysOnTopHint)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)

        self.current_node = None
        self.stl_viewer = stl_viewer
        self.port_widgets = []

        # 初始化界面组件
        self.setup_ui()

        # 允许窗口获取键盘焦点
        self.setFocusPolicy(QtCore.Qt.StrongFocus)

    def setup_ui(self):
        """初始化界面布局"""
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setSpacing(10)  # 控制整体间距
        main_layout.setContentsMargins(10, 5, 10, 5)  # 调整上下边距

        # 配置滚动区域
        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)

        # 滚动区域内部内容
        scroll_content = QtWidgets.QWidget()
        content_layout = QtWidgets.QVBoxLayout(scroll_content)
        content_layout.setSpacing(30)  # 控制各分区间距
        content_layout.setContentsMargins(5, 5, 5, 5)  # 减小内部留白

        # 节点名称区域（横向排列）
        name_layout = QtWidgets.QHBoxLayout()
        name_layout.addWidget(QtWidgets.QLabel("节点名称："))
        self.name_edit = QtWidgets.QLineEdit()
        self.name_edit.setPlaceholderText("输入节点名称")
        self.name_edit.editingFinished.connect(self.update_node_name)
        name_layout.addWidget(self.name_edit)

        content_layout.addLayout(name_layout)
        content_layout.addSpacing(5)  # 插入额外留白

        # 物理属性区域
        physics_layout = QtWidgets.QGridLayout()
        physics_layout.addWidget(QtWidgets.QLabel("体积："), 0, 0)
        self.volume_input = QtWidgets.QLineEdit()
        self.volume_input.setReadOnly(True)
        physics_layout.addWidget(self.volume_input, 0, 1)

        physics_layout.addWidget(QtWidgets.QLabel("质量："), 1, 0)
        self.mass_input = QtWidgets.QLineEdit()
        self.mass_input.setValidator(QtGui.QDoubleValidator())
        physics_layout.addWidget(self.mass_input, 1, 1)
        content_layout.addLayout(physics_layout)

        # 旋转轴选择区域
        rotation_layout = QtWidgets.QHBoxLayout()
        rotation_layout.addWidget(QtWidgets.QLabel("旋转轴：   "))
        self.axis_group = QtWidgets.QButtonGroup(self)
        for i, axis in enumerate(['X（滚转）', 'Y（俯仰）', 'Z（偏航）', '固定']):
            radio = QtWidgets.QRadioButton(axis)
            self.axis_group.addButton(radio, i)  # 依次对应 0~3（3 表示固定）
            rotation_layout.addWidget(radio)
        content_layout.addLayout(rotation_layout)

        # 旋转测试按钮区域
        rotation_test_layout = QtWidgets.QHBoxLayout()
        rotation_test_layout.addStretch()  # 左侧留出弹性空间
        self.rotation_test_button = QtWidgets.QPushButton("旋转测试")
        self.rotation_test_button.setFixedWidth(120)  # 统一按钮宽度
        self.rotation_test_button.pressed.connect(self.start_rotation_test)
        self.rotation_test_button.released.connect(self.stop_rotation_test)
        rotation_test_layout.addWidget(self.rotation_test_button)
        content_layout.addLayout(rotation_test_layout)

        # 质量忽略（装饰用途）选项
        massless_layout = QtWidgets.QHBoxLayout()
        self.massless_checkbox = QtWidgets.QCheckBox("仅作装饰（不计质量）")
        self.massless_checkbox.setChecked(False)  # 默认关闭
        massless_layout.addWidget(self.massless_checkbox)
        content_layout.addLayout(massless_layout)

        # 绑定复选框状态变更回调
        self.massless_checkbox.stateChanged.connect(self.update_massless_decoration)

        # 颜色设置区域
        color_layout = QtWidgets.QHBoxLayout()
        color_layout.addWidget(QtWidgets.QLabel("颜色："))

        # 颜色预览块
        self.color_sample = QtWidgets.QLabel()
        self.color_sample.setFixedSize(20, 20)
        self.color_sample.setStyleSheet(
        "background-color: rgb(255,255,255); border: 1px solid black;")
        color_layout.addWidget(self.color_sample)

        # RGB 数值输入
        color_layout.addWidget(QtWidgets.QLabel("   R："))
        self.color_inputs = []
        for label in ['', 'G：', 'B：']:
            if label:
                color_layout.addWidget(QtWidgets.QLabel(label))
            color_input = QtWidgets.QLineEdit("1.0")
            color_input.setFixedWidth(50)
            color_input.setValidator(QtGui.QDoubleValidator(0.0, 1.0, 3))
            self.color_inputs.append(color_input)
            color_layout.addWidget(color_input)

        color_layout.addStretch()  # 占满右侧空间
        content_layout.addLayout(color_layout)

        # 取色按钮
        pick_button = QtWidgets.QPushButton("取色")
        pick_button.clicked.connect(self.show_color_picker)
        pick_button.setFixedWidth(40)
        color_layout.addWidget(pick_button)

        # 应用颜色按钮
        apply_button = QtWidgets.QPushButton("应用")
        apply_button.clicked.connect(self.apply_color_to_stl)
        apply_button.setFixedWidth(40)
        color_layout.addWidget(apply_button)

        color_layout.addStretch()
        content_layout.addLayout(color_layout)

        # 输出端口区域
        ports_layout = QtWidgets.QVBoxLayout()
        self.ports_layout = QtWidgets.QVBoxLayout()  # 存放动态端口条目
        ports_layout.addLayout(self.ports_layout)

        # 端口数值应用按钮
        set_button_layout = QtWidgets.QHBoxLayout()
        set_button_layout.addStretch()
        set_button = QtWidgets.QPushButton("应用端口坐标")
        set_button.clicked.connect(self.apply_port_values)
        set_button_layout.addWidget(set_button)
        ports_layout.addLayout(set_button_layout)
        content_layout.addLayout(ports_layout)

        # 初始化端口控件列表
        self.port_widgets = []

        # 点位管理区域
        point_layout = QtWidgets.QHBoxLayout()
        point_layout.addWidget(QtWidgets.QLabel("点位控制："))
        self.add_point_btn = QtWidgets.QPushButton("[+] 新增")
        self.remove_point_btn = QtWidgets.QPushButton("[-] 移除")
        point_layout.addWidget(self.add_point_btn)
        point_layout.addWidget(self.remove_point_btn)
        self.add_point_btn.clicked.connect(self.add_point)
        self.remove_point_btn.clicked.connect(self.remove_point)
        content_layout.addLayout(point_layout)

        # File Controls: 只保留 STL 载入（XML 已不需要）
        file_layout = QtWidgets.QHBoxLayout()
        self.load_stl_btn = QtWidgets.QPushButton("加载 STL")
        file_layout.addWidget(self.load_stl_btn)
        self.load_stl_btn.clicked.connect(self.load_stl)
        content_layout.addLayout(file_layout)

        # 将内容放入滚动区域
        scroll_area.setWidget(scroll_content)
        main_layout.addWidget(scroll_area)

        # 为各布局统一设置间距
        name_layout.setSpacing(2)
        physics_layout.setSpacing(2)
        rotation_layout.setSpacing(2)
        color_layout.setSpacing(2)
        ports_layout.setSpacing(2)
        point_layout.setSpacing(2)
        file_layout.setSpacing(2)

        # 调整网格布局的边距
        physics_layout.setVerticalSpacing(2)
        physics_layout.setHorizontalSpacing(2)

        for line_edit in self.findChildren(QtWidgets.QLineEdit):
            line_edit.setStyleSheet("QLineEdit { padding-left: 2px; padding-top: 0px; padding-bottom: 0px; }")

    def setup_validators(self):
        """为数值输入框配置校验器"""
        try:
            # 质量输入框限定为非负
            mass_validator = QtGui.QDoubleValidator()
            mass_validator.setBottom(0.0)  # 禁止负值
            self.mass_input.setValidator(mass_validator)

            # 体积输入框限定为非负
            volume_validator = QtGui.QDoubleValidator()
            volume_validator.setBottom(0.0)  # 禁止负值
            self.volume_input.setValidator(volume_validator)

            # RGB 输入限定在 0~1，保留三位小数
            rgb_validator = QtGui.QDoubleValidator(0.0, 1.0, 3)
            for color_input in self.color_inputs:
                color_input.setValidator(rgb_validator)

            # 输出端口坐标使用通用浮点校验
            coord_validator = QtGui.QDoubleValidator()
            for port_widget in self.port_widgets:
                for input_field in port_widget.findChildren(QtWidgets.QLineEdit):
                    input_field.setValidator(coord_validator)

            print("输入校验器配置完成")

        except Exception as e:
            print(f"配置输入校验器时出错: {str(e)}")
            import traceback
            traceback.print_exc()

    def apply_color_to_stl(self):
        """選択された色をSTLモデルとカラーサンプルに適用"""
        if not self.current_node:
            print("No node selected")
            return

        try:
            # RGB値の取得（0-1の範囲）
            rgb_values = [float(input.text()) for input in self.color_inputs]

            # 値の範囲チェック
            rgb_values = [max(0.0, min(1.0, value)) for value in rgb_values]

            # ノードの色情報を更新
            self.current_node.node_color = rgb_values

            # カラーサンプルチップを必ず更新
            rgb_display = [int(v * 255) for v in rgb_values]
            self.color_sample.setStyleSheet(
                f"background-color: rgb({rgb_display[0]},{rgb_display[1]},{rgb_display[2]}); "
                f"border: 1px solid black;"
            )

            # STLモデルの色を変更
            if self.stl_viewer and hasattr(self.stl_viewer, 'stl_actors'):
                if self.current_node in self.stl_viewer.stl_actors:
                    actor = self.stl_viewer.stl_actors[self.current_node]
                    actor.GetProperty().SetColor(*rgb_values)
                    self.stl_viewer.vtkWidget.GetRenderWindow().Render()
                    print(f"Applied color: RGB({rgb_values[0]:.3f}, {rgb_values[1]:.3f}, {rgb_values[2]:.3f})")
                else:
                    print("No STL model found for this node")

        except ValueError as e:
            print(f"Error: Invalid color value - {str(e)}")
        except Exception as e:
            print(f"Error applying color: {str(e)}")
            traceback.print_exc()

    def update_color_sample(self):
        """カラーサンプルの表示を更新"""
        try:
            rgb_values = [min(255, max(0, int(float(input.text()) * 255)))
                        for input in self.color_inputs]
            self.color_sample.setStyleSheet(
                f"background-color: rgb({rgb_values[0]},{rgb_values[1]},{rgb_values[2]}); "
                f"border: 1px solid black;"
            )

            if self.current_node:
                self.current_node.node_color = [float(input.text()) for input in self.color_inputs]

        except ValueError as e:
            print(f"Error updating color sample: {str(e)}")
            traceback.print_exc()

    def update_port_coordinate(self, port_index, coord_index, value):
        """ポート座標の更新"""
        try:
            if self.current_node and hasattr(self.current_node, 'points'):
                if 0 <= port_index < len(self.current_node.points):
                    try:
                        new_value = float(value)
                        self.current_node.points[port_index]['xyz'][coord_index] = new_value
                        print(
                            f"Updated port {port_index+1} coordinate {coord_index} to {new_value}")
                    except ValueError:
                        print("Invalid coordinate value")
        except Exception as e:
            print(f"Error updating coordinate: {str(e)}")

    def update_info(self, node):
        """ノード情報の更新"""
        self.current_node = node

        try:
            # Node Name
            self.name_edit.setText(node.name())

            # Volume & Mass
            if hasattr(node, 'volume_value'):
                self.volume_input.setText(f"{node.volume_value:.6f}")
                print(f"Volume set to: {node.volume_value}")

            if hasattr(node, 'mass_value'):
                self.mass_input.setText(f"{node.mass_value:.6f}")
                print(f"Mass set to: {node.mass_value}")

            # Rotation Axis - nodeのrotation_axis属性を確認して設定
            if hasattr(node, 'rotation_axis'):
                axis_button = self.axis_group.button(node.rotation_axis)
                if axis_button:
                    axis_button.setChecked(True)
                    print(f"Rotation axis set to: {node.rotation_axis}")
            else:
                # デフォルトでX軸を選択
                node.rotation_axis = 0
                if self.axis_group.button(0):
                    self.axis_group.button(0).setChecked(True)
                    print("Default rotation axis set to X (0)")

            # Massless Decoration の状態を設定
            if hasattr(node, 'massless_decoration'):
                self.massless_checkbox.setChecked(node.massless_decoration)
                print(f"Massless decoration set to: {node.massless_decoration}")
            else:
                node.massless_decoration = False
                self.massless_checkbox.setChecked(False)
                print("Default massless decoration set to False")

            # Color settings - nodeのnode_color属性を確認して設定
            if hasattr(node, 'node_color') and node.node_color:
                print(f"Setting color: {node.node_color}")
                for i, value in enumerate(node.node_color[:3]):
                    self.color_inputs[i].setText(f"{value:.3f}")

                # カラーサンプルチップの更新
                rgb_display = [int(v * 255) for v in node.node_color[:3]]
                self.color_sample.setStyleSheet(
                    f"background-color: rgb({rgb_display[0]},{rgb_display[1]},{rgb_display[2]}); "
                    f"border: 1px solid black;"
                )
                # STLモデルにも色を適用
                self.apply_color_to_stl()
            else:
                # デフォルトの色を設定（白）
                node.node_color = [1.0, 1.0, 1.0]
                for color_input in self.color_inputs:
                    color_input.setText("1.000")
                self.color_sample.setStyleSheet(
                    "background-color: rgb(255,255,255); border: 1px solid black;"
                )
                print("Default color set to white")

            # 回転軸の選択を更新するためのシグナルを接続
            for button in self.axis_group.buttons():
                button.clicked.connect(lambda checked, btn=button: self.update_rotation_axis(btn))

            # Output Ports
            self.update_output_ports(node)

            # ラジオボタンのイベントハンドラを設定
            self.axis_group.buttonClicked.connect(self.on_axis_selection_changed)

            # バリデータの設定
            self.setup_validators()

            print(f"Inspector window updated for node: {node.name()}")

        except Exception as e:
            print(f"Error updating inspector info: {str(e)}")
            traceback.print_exc()

    def update_rotation_axis(self, button):
        """回転軸の選択が変更されたときの処理"""
        if self.current_node:
            self.current_node.rotation_axis = self.axis_group.id(button)
            print(f"Updated rotation axis to: {self.current_node.rotation_axis}")

    def on_axis_selection_changed(self, button):
        """回転軸の選択が変更されたときのイベントハンドラ"""
        if self.current_node:
            # 現在のノードの変換情報を保存
            if self.stl_viewer and self.current_node in self.stl_viewer.transforms:
                current_transform = self.stl_viewer.transforms[self.current_node]
                current_position = current_transform.GetPosition()
            else:
                current_position = [0, 0, 0]

            # 回転軸の更新
            axis_id = self.axis_group.id(button)
            self.current_node.rotation_axis = axis_id

            # 軸のタイプを判定して表示
            axis_types = ['X (Roll)', 'Y (Pitch)', 'Z (Yaw)', 'Fixed']
            if 0 <= axis_id < len(axis_types):
                print(f"Rotation axis changed to: {axis_types[axis_id]}")
            else:
                print(f"Invalid rotation axis ID: {axis_id}")

            # STLモデルの更新
            if self.stl_viewer:
                # 変換の更新
                if self.current_node in self.stl_viewer.transforms:
                    transform = self.stl_viewer.transforms[self.current_node]
                    transform.Identity()  # 変換をリセット
                    transform.Translate(*current_position)  # 元の位置を維持

                    # 回転軸に基づいて現在の角度を設定（必要な場合）
                    if hasattr(self.current_node, 'current_rotation'):
                        angle = self.current_node.current_rotation
                        if axis_id == 0:      # X軸
                            transform.RotateX(angle)
                        elif axis_id == 1:    # Y軸
                            transform.RotateY(angle)
                        elif axis_id == 2:    # Z軸
                            transform.RotateZ(angle)

                    # 変換を適用
                    if self.current_node in self.stl_viewer.stl_actors:
                        self.stl_viewer.stl_actors[self.current_node].SetUserTransform(transform)
                        self.stl_viewer.vtkWidget.GetRenderWindow().Render()
                        print(f"Updated transform for node {self.current_node.name()} at position {current_position}")

    def show_color_picker(self):
        """カラーピッカーを表示"""
        try:
            current_color = QtGui.QColor(
                *[min(255, max(0, int(float(input.text()) * 255)))
                for input in self.color_inputs]
            )
        except ValueError:
            current_color = QtGui.QColor(255, 255, 255)

        color = QtWidgets.QColorDialog.getColor(
            initial=current_color,
            parent=self,
            options=QtWidgets.QColorDialog.DontUseNativeDialog
        )

        if color.isValid():
            # RGB値を0-1の範囲に変換してセット
            rgb_values = [color.red() / 255.0, color.green() / 255.0, color.blue() / 255.0]

            # 入力フィールドを更新
            for i, value in enumerate(rgb_values):
                self.color_inputs[i].setText(f"{value:.3f}")

            # カラーサンプルチップを直接更新
            self.color_sample.setStyleSheet(
                f"background-color: rgb({color.red()},{color.green()},{color.blue()}); "
                f"border: 1px solid black;"
            )

            # ノードの色情報を更新
            if self.current_node:
                self.current_node.node_color = rgb_values

            # STLモデルに色を適用
            self.apply_color_to_stl()

            print(f"Color picker: Selected RGB({rgb_values[0]:.3f}, {rgb_values[1]:.3f}, {rgb_values[2]:.3f})")

    def update_node_name(self):
        """ノード名の更新"""
        if self.current_node:
            new_name = self.name_edit.text()
            old_name = self.current_node.name()
            if new_name != old_name:
                self.current_node.set_name(new_name)
                print(f"Node name updated from '{old_name}' to '{new_name}'")

    def add_point(self):
        """ポイントの追加"""
        if self.current_node and hasattr(self.current_node, '_add_output'):
            new_port_name = self.current_node._add_output()
            if new_port_name:
                self.update_info(self.current_node)
                print(f"Added new port: {new_port_name}")

    def remove_point(self):
        """ポイントの削除"""
        if self.current_node and hasattr(self.current_node, 'remove_output'):
            self.current_node.remove_output()
            self.update_info(self.current_node)
            print("Removed last port")

    def load_stl(self):
        """STLファイルの読み込み"""
        if self.current_node:
            file_name, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, "Open STL File", "", "STL Files (*.stl)")
            if file_name:
                self.current_node.stl_file = file_name
                if self.stl_viewer:
                    self.stl_viewer.load_stl_for_node(self.current_node)
                # Try apply embedded metadata if present in STL
                try:
                    metadata_text = extract_metadata_text(file_name)
                    if metadata_text:
                        root = ET.fromstring(metadata_text)
                        if root.tag == 'urdf_part':
                            self._apply_urdf_part_root(root)
                            print("Applied embedded metadata from STL")
                except Exception as e:
                    print(f"Warning: Failed to apply embedded metadata: {e}")

    def closeEvent(self, event):
        """ウィンドウが閉じられるときのイベントを処理"""
        try:
            # 全てのウィジェットを明示的に削除
            for widget in self.findChildren(QtWidgets.QWidget):
                if widget is not self:
                    widget.setParent(None)
                    widget.deleteLater()

            # 参照のクリア
            self.current_node = None
            self.stl_viewer = None
            self.port_widgets.clear()

            # イベントを受け入れ
            event.accept()

        except Exception as e:
            print(f"Error in closeEvent: {str(e)}")
            event.accept()

    def load_xml(self):
        """XMLファイルの読み込み"""
        if not self.current_node:
            print("No node selected")
            return

        file_name, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open XML File", "", "XML Files (*.xml)")

        if not file_name:
            return

        try:
            tree = ET.parse(file_name)
            root = tree.getroot()

            if root.tag != 'urdf_part':
                print("Invalid XML format: Root element should be 'urdf_part'")
                return

            print("Loading XML file...")

            # リンク名の取得と設定
            link_elem = root.find('link')
            if link_elem is not None:
                link_name = link_elem.get('name')
                if link_name:
                    self.current_node.set_name(link_name)
                    self.name_edit.setText(link_name)
                    print(f"Set link name: {link_name}")

                # 物理プロパティの設定
                inertial_elem = link_elem.find('inertial')
                if inertial_elem is not None:
                    # ボリュームの設定
                    volume_elem = inertial_elem.find('volume')
                    if volume_elem is not None:
                        volume = float(volume_elem.get('value', '0.0'))
                        self.current_node.volume_value = volume
                        self.volume_input.setText(f"{volume:.6f}")
                        print(f"Set volume: {volume}")

                    # 質量の設定
                    mass_elem = inertial_elem.find('mass')
                    if mass_elem is not None:
                        mass = float(mass_elem.get('value', '0.0'))
                        self.current_node.mass_value = mass
                        self.mass_input.setText(f"{mass:.6f}")
                        print(f"Set mass: {mass}")

                    # 慣性モーメントの設定
                    inertia_elem = inertial_elem.find('inertia')
                    if inertia_elem is not None:
                        self.current_node.inertia = {
                            'ixx': float(inertia_elem.get('ixx', '0')),
                            'ixy': float(inertia_elem.get('ixy', '0')),
                            'ixz': float(inertia_elem.get('ixz', '0')),
                            'iyy': float(inertia_elem.get('iyy', '0')),
                            'iyz': float(inertia_elem.get('iyz', '0')),
                            'izz': float(inertia_elem.get('izz', '0'))
                        }
                        print("Set inertia tensor")

            # 色情報の処理
            material_elem = root.find('.//material/color')
            if material_elem is not None:
                rgba = material_elem.get('rgba', '1.0 1.0 1.0 1.0').split()
                rgb_values = [float(x) for x in rgba[:3]]  # RGBのみ使用
                self.current_node.node_color = rgb_values
                # 色の設定をUIに反映
                for i, value in enumerate(rgb_values):
                    self.color_inputs[i].setText(f"{value}")
                self.update_color_sample()
                # STLモデルに色を適用
                self.apply_color_to_stl()
                print(f"Set color: RGB({rgb_values[0]:.3f}, {rgb_values[1]:.3f}, {rgb_values[2]:.3f})")

            # 回転軸の処理
            joint_elem = root.find('joint')
            if joint_elem is not None:
                # jointのtype属性を確認
                joint_type = joint_elem.get('type', '')
                if joint_type == 'fixed':
                    self.current_node.rotation_axis = 3  # 3をFixedとして使用
                    if self.axis_group.button(3):  # Fixed用のボタンが存在する場合
                        self.axis_group.button(3).setChecked(True)
                    print("Set rotation axis to Fixed")
                else:
                    # 回転軸の処理
                    axis_elem = joint_elem.find('axis')
                    if axis_elem is not None:
                        axis_xyz = axis_elem.get('xyz', '1 0 0').split()
                        axis_values = [float(x) for x in axis_xyz]
                        if axis_values[2] == 1:  # Z軸
                            self.current_node.rotation_axis = 2
                            self.axis_group.button(2).setChecked(True)
                            print("Set rotation axis to Z")
                        elif axis_values[1] == 1:  # Y軸
                            self.current_node.rotation_axis = 1
                            self.axis_group.button(1).setChecked(True)
                            print("Set rotation axis to Y")
                        else:  # X軸（デフォルト）
                            self.current_node.rotation_axis = 0
                            self.axis_group.button(0).setChecked(True)
                            print("Set rotation axis to X")
                        print(f"Set rotation axis from xyz: {axis_xyz}")

            # ポイントの処理
            points = root.findall('point')
            num_points = len(points)
            print(f"Found {num_points} points in XML")

            # 現在のポート数と必要なポート数を比較
            current_ports = len(self.current_node.output_ports())
            print(f"Current ports: {current_ports}, Required points: {num_points}")

            # ポート数を調整
            if isinstance(self.current_node, FooNode):
                while current_ports < num_points:
                    self.current_node._add_output()
                    current_ports += 1
                    print(f"Added new port, total now: {current_ports}")

                while current_ports > num_points:
                    self.current_node.remove_output()
                    current_ports -= 1
                    print(f"Removed port, total now: {current_ports}")

                # ポイントデータの更新
                self.current_node.points = []
                for point_elem in points:
                    point_name = point_elem.get('name')
                    point_type = point_elem.get('type')
                    point_xyz_elem = point_elem.find('point_xyz')

                    if point_xyz_elem is not None and point_xyz_elem.text:
                        xyz_values = [float(x) for x in point_xyz_elem.text.strip().split()]
                        self.current_node.points.append({
                            'name': point_name,
                            'type': point_type,
                            'xyz': xyz_values
                        })
                        print(f"Added point {point_name}: {xyz_values}")

                # 累積座標の更新
                self.current_node.cumulative_coords = []
                for i in range(len(self.current_node.points)):
                    self.current_node.cumulative_coords.append({
                        'point_index': i,
                        'xyz': [0.0, 0.0, 0.0]
                    })

                # output_countを更新
                self.current_node.output_count = len(self.current_node.points)
                print(f"Updated output_count to: {self.current_node.output_count}")

            # UI更新
            self.update_info(self.current_node)
            print(f"XML file loaded: {file_name}")

        except Exception as e:
            print(f"Error loading XML: {str(e)}")
            import traceback
            traceback.print_exc()

    def load_xml_with_stl(self):
        """XMLファイルとそれに対応するSTLファイルを読み込む"""
        if not self.current_node:
            print("No node selected")
            return

        # XMLファイルの選択
        xml_file, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open XML File", "", "XML Files (*.xml)")

        if not xml_file:
            return

        try:
            # 対応するSTLファイルのパスを生成
            xml_dir = os.path.dirname(xml_file)
            xml_name = os.path.splitext(os.path.basename(xml_file))[0]
            stl_path = os.path.join(xml_dir, f"{xml_name}.stl")

            # まずXMLファイルを読み込む
            tree = ET.parse(xml_file)
            root = tree.getroot()

            if root.tag != 'urdf_part':
                print("Invalid XML format: Root element should be 'urdf_part'")
                return

            # リンク情報の処理
            link_elem = root.find('link')
            if link_elem is not None:
                link_name = link_elem.get('name')
                if link_name:
                    self.current_node.set_name(link_name)
                    self.name_edit.setText(link_name)

                # 質量の設定
                mass_elem = link_elem.find('mass')
                if mass_elem is not None:
                    mass = float(mass_elem.get('value', '0.0'))
                    self.current_node.mass_value = mass
                    self.mass_input.setText(f"{mass:.6f}")

                # ボリュームの設定
                volume_elem = root.find('.//volume')
                if volume_elem is not None:
                    volume = float(volume_elem.get('value', '0.0'))
                    self.current_node.volume_value = volume
                    self.volume_input.setText(f"{volume:.6f}")

                # 慣性モーメントの設定
                inertia_elem = link_elem.find('inertia')
                if inertia_elem is not None:
                    self.current_node.inertia = {
                        'ixx': float(inertia_elem.get('ixx', '0')),
                        'ixy': float(inertia_elem.get('ixy', '0')),
                        'ixz': float(inertia_elem.get('ixz', '0')),
                        'iyy': float(inertia_elem.get('iyy', '0')),
                        'iyz': float(inertia_elem.get('iyz', '0')),
                        'izz': float(inertia_elem.get('izz', '0'))
                    }

            # 色情報の処理
            material_elem = root.find('.//material/color')
            if material_elem is not None:
                rgba = material_elem.get('rgba', '1.0 1.0 1.0 1.0').split()
                rgb_values = [float(x) for x in rgba[:3]]  # RGBのみ使用
                self.current_node.node_color = rgb_values
                # 色の設定をUIに反映
                for i, value in enumerate(rgb_values):
                    self.color_inputs[i].setText(f"{value}")
                self.update_color_sample()
                print(f"Set color: RGB({rgb_values[0]:.3f}, {rgb_values[1]:.3f}, {rgb_values[2]:.3f})")

            # 回転軸の処理
            joint_elem = root.find('.//joint/axis')
            if joint_elem is not None:
                axis_xyz = joint_elem.get('xyz', '1 0 0').split()
                axis_values = [float(x) for x in axis_xyz]
                if axis_values[2] == 1:  # Z軸
                    self.current_node.rotation_axis = 2
                    self.axis_group.button(2).setChecked(True)
                elif axis_values[1] == 1:  # Y軸
                    self.current_node.rotation_axis = 1
                    self.axis_group.button(1).setChecked(True)
                else:  # X軸（デフォルト）
                    self.current_node.rotation_axis = 0
                    self.axis_group.button(0).setChecked(True)
                print(f"Set rotation axis: {self.current_node.rotation_axis} from xyz: {axis_xyz}")

            # ポイントの処理
            points = root.findall('point')
            num_points = len(points)
            print(f"Found {num_points} points")

            # 現在のポート数と必要なポート数を比較
            current_ports = len(self.current_node.points)
            if num_points > current_ports:
                # 不足しているポートを追加
                ports_to_add = num_points - current_ports
                for _ in range(ports_to_add):
                    self.add_point()
            elif num_points < current_ports:
                # 余分なポートを削除
                ports_to_remove = current_ports - num_points
                for _ in range(ports_to_remove):
                    self.remove_point()

            # ポイントデータの更新
            self.current_node.points = []
            for point_elem in points:
                point_name = point_elem.get('name')
                point_type = point_elem.get('type')
                point_xyz_elem = point_elem.find('point_xyz')

                if point_xyz_elem is not None and point_xyz_elem.text:
                    xyz_values = [float(x) for x in point_xyz_elem.text.strip().split()]
                    self.current_node.points.append({
                        'name': point_name,
                        'type': point_type,
                        'xyz': xyz_values
                    })
                    print(f"Added point {point_name}: {xyz_values}")

            # STLファイルの処理
            if os.path.exists(stl_path):
                print(f"Found corresponding STL file: {stl_path}")
                self.current_node.stl_file = stl_path
                if self.stl_viewer:
                    self.stl_viewer.load_stl_for_node(self.current_node)
                    # STLモデルに色を適用
                    self.apply_color_to_stl()
            else:
                print(f"Warning: STL file not found: {stl_path}")
                msg_box = QtWidgets.QMessageBox()
                msg_box.setIcon(QtWidgets.QMessageBox.Warning)
                msg_box.setWindowTitle("STL File Not Found")
                msg_box.setText("STL file not found in the same directory.")
                msg_box.setInformativeText("Would you like to select the STL file manually?")
                msg_box.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
                msg_box.setDefaultButton(QtWidgets.QMessageBox.Yes)

                if msg_box.exec() == QtWidgets.QMessageBox.Yes:
                    stl_file, _ = QtWidgets.QFileDialog.getOpenFileName(
                        self, "Select STL File", xml_dir, "STL Files (*.stl)")
                    if stl_file:
                        self.current_node.stl_file = stl_file
                        if self.stl_viewer:
                            self.stl_viewer.load_stl_for_node(self.current_node)
                            # STLモデルに色を適用
                            self.apply_color_to_stl()
                        print(f"Manually selected STL file: {stl_file}")
                    else:
                        print("STL file selection cancelled")
                else:
                    print("STL file loading skipped")

            # UI更新
            self.update_info(self.current_node)
            print(f"XML file loaded: {xml_file}")

        except Exception as e:
            print(f"Error loading XML with STL: {str(e)}")
            import traceback
            traceback.print_exc()

    def apply_port_values(self):
        """Output Portsの値を適用"""
        if not self.current_node:
            print("No node selected")
            return

        try:
            # ポートウィジェットから値を取得して適用
            for i, port_widget in enumerate(self.port_widgets):
                # ポートの座標入力フィールドを検索
                coord_inputs = []
                for child in port_widget.findChildren(QtWidgets.QLineEdit):
                    coord_inputs.append(child)

                # 座標入力フィールドが3つ（X,Y,Z）あることを確認
                if len(coord_inputs) >= 3:
                    try:
                        # 座標値を取得
                        x = float(coord_inputs[0].text())
                        y = float(coord_inputs[1].text())
                        z = float(coord_inputs[2].text())

                        # ノードのポイントデータを更新
                        if hasattr(self.current_node, 'points') and i < len(self.current_node.points):
                            self.current_node.points[i]['xyz'] = [x, y, z]
                            print(
                                f"Updated point {i+1} coordinates to: ({x:.6f}, {y:.6f}, {z:.6f})")

                            # 累積座標も更新
                            if hasattr(self.current_node, 'cumulative_coords') and i < len(self.current_node.cumulative_coords):
                                if isinstance(self.current_node, BaseLinkNode):
                                    self.current_node.cumulative_coords[i]['xyz'] = [
                                        x, y, z]
                                else:
                                    # base_link以外のノードの場合は相対座標を保持
                                    self.current_node.cumulative_coords[i]['xyz'] = [
                                        0.0, 0.0, 0.0]

                    except ValueError:
                        print(f"Invalid numerical input for point {i+1}")
                        continue

            # ノードの位置を再計算（必要な場合）
            if hasattr(self.current_node, 'graph') and self.current_node.graph:
                self.current_node.graph.recalculate_all_positions()
                print("Node positions recalculated")

            # STLビューアの更新
            if self.stl_viewer:
                self.stl_viewer.vtkWidget.GetRenderWindow().Render()
                print("3D view updated")

        except Exception as e:
            print(f"Error applying port values: {str(e)}")
            import traceback
            traceback.print_exc()

    def create_port_widget(self, port_number, x=0.0, y=0.0, z=0.0):
        """Output Port用のウィジェットを作成"""
        port_layout = QtWidgets.QHBoxLayout()  # GridLayoutからHBoxLayoutに変更
        port_layout.setSpacing(5)
        port_layout.setContentsMargins(0, 1, 0, 1)

        # ポート名称 (PartsEditor で設定された名称を優先)
        display_name = ''
        if hasattr(self.current_node, 'points') and 0 <= port_number-1 < len(self.current_node.points):
            display_name = self.current_node.points[port_number-1].get('name') or ''
        if not display_name:
            node_ports = self.current_node.output_ports() if hasattr(self.current_node, 'output_ports') else []
            if 0 <= port_number-1 < len(node_ports):
                port_obj = node_ports[port_number-1]
                if hasattr(port_obj, 'name'):
                    display_name = port_obj.name()
        if not display_name:
            display_name = f"out_{port_number}"
        port_name = QtWidgets.QLabel(display_name)
        port_name.setFixedWidth(120)
        port_layout.addWidget(port_name)

        # 座標入力のペアを作成
        coords = []
        for label, value in [('X:', x), ('Y:', y), ('Z:', z)]:
            # 各座標のペアをHBoxLayoutで作成
            coord_pair = QtWidgets.QHBoxLayout()
            coord_pair.setSpacing(2)

            # ラベル
            coord_label = QtWidgets.QLabel(label)
            coord_label.setFixedWidth(15)
            coord_pair.addWidget(coord_label)

            # 入力フィールド
            coord_input = QtWidgets.QLineEdit(f"{value:.6f}")
            coord_input.setFixedWidth(70)
            coord_input.setFixedHeight(20)
            coord_input.setStyleSheet("QLineEdit { padding-left: 2px; padding-top: 0px; padding-bottom: 0px; }")
            coord_input.setValidator(QtGui.QDoubleValidator())
            coord_input.textChanged.connect(
                lambda text, idx=port_number-1, coord=len(coords):
                self.update_port_coordinate(idx, coord, text))
            coord_pair.addWidget(coord_input)
            coords.append(coord_input)

            # ペアをメインレイアウトに追加
            port_layout.addLayout(coord_pair)

            # ペア間にスペースを追加
            if label != 'Z:':  # 最後のペア以外の後にスペースを追加
                port_layout.addSpacing(15)

        # 右側の余白
        port_layout.addStretch()

        # ウィジェットをラップ
        port_widget = QtWidgets.QWidget()
        port_widget.setFixedHeight(25)
        port_widget.setLayout(port_layout)
        return port_widget, coords

    def update_output_ports(self, node):
        """Output Portsセクションを更新"""
        # 既存のポートウィジェットをクリア
        for widget in self.port_widgets:
            self.ports_layout.removeWidget(widget)
            widget.setParent(None)
            widget.deleteLater()
        self.port_widgets.clear()

        # ノードの各ポートに対してウィジェットを作成
        if hasattr(node, 'points'):
            for i, point in enumerate(node.points):
                port_widget, _ = self.create_port_widget(
                    i + 1,
                    point['xyz'][0],
                    point['xyz'][1],
                    point['xyz'][2]
                )
                self.ports_layout.addWidget(port_widget)
                self.port_widgets.append(port_widget)

    def apply_color_to_stl(self):
        """選択された色をSTLモデルに適用"""
        if not self.current_node:
            return

        try:
            rgb_values = [float(input.text()) for input in self.color_inputs]
            rgb_values = [max(0.0, min(1.0, value)) for value in rgb_values]

            self.current_node.node_color = rgb_values

            if self.stl_viewer and hasattr(self.stl_viewer, 'stl_actors'):
                if self.current_node in self.stl_viewer.stl_actors:
                    actor = self.stl_viewer.stl_actors[self.current_node]
                    actor.GetProperty().SetColor(*rgb_values)
                    self.stl_viewer.vtkWidget.GetRenderWindow().Render()
        except ValueError as e:
            print(f"Error: Invalid color value - {str(e)}")

    def update_color_sample(self):
        """カラーサンプルの表示を更新"""
        try:
            rgb_values = [min(255, max(0, int(float(input.text()) * 255)))
                        for input in self.color_inputs]
            self.color_sample.setStyleSheet(
                f"background-color: rgb({rgb_values[0]},{rgb_values[1]},{rgb_values[2]}); "
                f"border: 1px solid black;"
            )
        except ValueError:
            pass

    def update_massless_decoration(self, state):
        """Massless Decorationの状態を更新"""
        if self.current_node:
            self.current_node.massless_decoration = bool(state)
            print(f"Set massless_decoration to {bool(state)} for node: {self.current_node.name()}")

    def moveEvent(self, event):
        """ウィンドウ移動イベントの処理"""
        super(InspectorWindow, self).moveEvent(event)
        # グラフオブジェクトが存在し、last_inspector_positionを保存可能な場合
        if hasattr(self, 'graph') and self.graph:
            self.graph.last_inspector_position = self.pos()

    def keyPressEvent(self, event):
        """キープレスイベントの処理"""
        # ESCキーが押されたかどうかを確認
        if event.key() == QtCore.Qt.Key.Key_Escape:
            self.close()
        else:
            # 他のキーイベントは通常通り処理
            super(InspectorWindow, self).keyPressEvent(event)

    def start_rotation_test(self):
        """回転テスト開始"""
        if self.current_node and self.stl_viewer:
            # 現在の変換を保存
            self.stl_viewer.store_current_transform(self.current_node)
            # 回転開始
            self.stl_viewer.start_rotation_test(self.current_node)

    def stop_rotation_test(self):
        """回転テスト終了"""
        if self.current_node and self.stl_viewer:
            # 回転停止と元の角度に戻す
            self.stl_viewer.stop_rotation_test(self.current_node)

    def _apply_urdf_part_root(self, root: ET.Element):
        """将 <urdf_part> 元素应用到当前节点与 UI。"""
        if not self.current_node:
            return
        try:
            # 名称
            link_elem = root.find('link')
            if link_elem is not None:
                link_name = link_elem.get('name')
                if link_name:
                    self.current_node.set_name(link_name)
                    self.name_edit.setText(link_name)

            # 物性
            if link_elem is not None:
                inertial_elem = link_elem.find('inertial')
                if inertial_elem is not None:
                    volume_elem = inertial_elem.find('volume')
                    if volume_elem is not None and volume_elem.get('value'):
                        try:
                            volume = float(volume_elem.get('value'))
                            self.current_node.volume_value = volume
                            self.volume_input.setText(f"{volume:.6f}")
                        except ValueError:
                            pass
                    mass_elem = inertial_elem.find('mass')
                    if mass_elem is not None and mass_elem.get('value'):
                        try:
                            mass = float(mass_elem.get('value'))
                            self.current_node.mass_value = mass
                            self.mass_input.setText(f"{mass:.6f}")
                        except ValueError:
                            pass
                    inertia_elem = inertial_elem.find('inertia')
                    if inertia_elem is not None:
                        try:
                            self.current_node.inertia = {
                                'ixx': float(inertia_elem.get('ixx', '0')),
                                'ixy': float(inertia_elem.get('ixy', '0')),
                                'ixz': float(inertia_elem.get('ixz', '0')),
                                'iyy': float(inertia_elem.get('iyy', '0')),
                                'iyz': float(inertia_elem.get('iyz', '0')),
                                'izz': float(inertia_elem.get('izz', '0'))
                            }
                        except ValueError:
                            pass

            # 颜色
            material_elem = root.find('.//material/color')
            if material_elem is not None and material_elem.get('rgba'):
                try:
                    rgba = material_elem.get('rgba').split()
                    rgb = [float(x) for x in rgba[:3]]
                    self.current_node.node_color = rgb
                    for i, v in enumerate(rgb):
                        self.color_inputs[i].setText(f"{v}")
                    self.update_color_sample()
                    self.apply_color_to_stl()
                except ValueError:
                    pass

            # 回転軸 / joint axis
            joint_elem = root.find('joint')
            if joint_elem is not None:
                jtype = joint_elem.get('type', '')
                if jtype == 'fixed':
                    self.current_node.rotation_axis = 3
                    if self.axis_group.button(3):
                        self.axis_group.button(3).setChecked(True)
                else:
                    axis_elem = joint_elem.find('axis')
                    if axis_elem is not None and axis_elem.get('xyz'):
                        try:
                            ax = [float(x) for x in axis_elem.get('xyz').split()]
                            if ax == [0.0, 0.0, 1.0]:
                                self.current_node.rotation_axis = 2
                                self.axis_group.button(2).setChecked(True)
                            elif ax == [0.0, 1.0, 0.0]:
                                self.current_node.rotation_axis = 1
                                self.axis_group.button(1).setChecked(True)
                            else:
                                self.current_node.rotation_axis = 0
                                self.axis_group.button(0).setChecked(True)
                        except ValueError:
                            pass

            # 点与端口
            points = root.findall('point')
            num_points = len(points)
            from URDF_BASIC2 import FooNode  # 避免循环导入
            if isinstance(self.current_node, FooNode):
                current_ports = len(self.current_node.output_ports())
                while current_ports < num_points:
                    self.current_node._add_output()
                    current_ports += 1
                while current_ports > num_points:
                    self.current_node.remove_output()
                    current_ports -= 1
                self.current_node.points = []
                for pt in points:
                    pt_name = pt.get('name')
                    pt_type = pt.get('type')
                    xyz_elem = pt.find('point_xyz')
                    if xyz_elem is not None and xyz_elem.text:
                        try:
                            xyz = [float(x) for x in xyz_elem.text.split()]
                        except ValueError:
                            xyz = [0.0, 0.0, 0.0]
                    else:
                        xyz = [0.0, 0.0, 0.0]
                    self.current_node.points.append({'name': pt_name, 'type': pt_type, 'xyz': xyz})
        except Exception as e:
            print(f"Error applying urdf_part root: {e}")

    def _calculate_base_inertia_tensor(self, poly_data, mass, center_of_mass, is_mirrored=False):
        """
        基本的な慣性テンソル計算のための共通実装。
        InspectorWindowクラスのメソッド。

        Args:
            poly_data: vtkPolyData オブジェクト
            mass: float 質量
            center_of_mass: list[float] 重心座標 [x, y, z]
            is_mirrored: bool ミラーリングモードかどうか

        Returns:
            numpy.ndarray: 3x3 慣性テンソル行列
        """
        # 体積を計算
        mass_properties = vtk.vtkMassProperties()
        mass_properties.SetInputData(poly_data)
        mass_properties.Update()
        total_volume = mass_properties.GetVolume()

        # 実際の質量から密度を逆算
        density = mass / total_volume
        print(f"Calculated density: {density:.6f} from mass: {mass:.6f} and volume: {total_volume:.6f}")

        # 慣性テンソルの初期化
        inertia_tensor = np.zeros((3, 3))
        num_cells = poly_data.GetNumberOfCells()
        print(f"Processing {num_cells} triangles for inertia tensor calculation...")

        for i in range(num_cells):
            cell = poly_data.GetCell(i)
            if cell.GetCellType() == vtk.VTK_TRIANGLE:
                # 三角形の頂点を取得（重心を原点とした座標系で）
                points = [np.array(cell.GetPoints().GetPoint(j)) - np.array(center_of_mass) for j in range(3)]

                # ミラーリングモードの場合、Y座標を反転
                if is_mirrored:
                    points = [[p[0], -p[1], p[2]] for p in points]

                # 三角形の面積と法線ベクトルを計算
                v1 = np.array(points[1]) - np.array(points[0])
                v2 = np.array(points[2]) - np.array(points[0])
                normal = np.cross(v1, v2)
                area = 0.5 * np.linalg.norm(normal)

                if area < 1e-10:  # 極小の三角形は無視
                    continue

                # 三角形の重心を計算
                tri_centroid = np.mean(points, axis=0)

                # 三角形の局所的な慣性テンソルを計算
                covariance = np.zeros((3, 3))
                for p in points:
                    r_squared = np.sum(p * p)
                    for a in range(3):
                        for b in range(3):
                            if a == b:
                                # 対角成分
                                covariance[a, a] += (r_squared - p[a] * p[a]) * area / 12.0
                            else:
                                # 非対角成分（オフセット項）
                                covariance[a, b] -= (p[a] * p[b]) * area / 12.0

                # 平行軸の定理を適用
                r_squared = np.sum(tri_centroid * tri_centroid)
                parallel_axis_term = np.zeros((3, 3))
                for a in range(3):
                    for b in range(3):
                        if a == b:
                            parallel_axis_term[a, a] = r_squared * area
                        else:
                            parallel_axis_term[a, b] = tri_centroid[a] * tri_centroid[b] * area

                # 局所的な慣性テンソルと平行軸の項を合成
                local_inertia = covariance + parallel_axis_term

                # 全体の慣性テンソルに加算
                inertia_tensor += local_inertia

        # 密度を考慮して最終的な慣性テンソルを計算
        inertia_tensor *= density

        # 数値誤差の処理
        threshold = 1e-10
        inertia_tensor[np.abs(inertia_tensor) < threshold] = 0.0

        # 対称性の確認と強制
        inertia_tensor = 0.5 * (inertia_tensor + inertia_tensor.T)

        # 対角成分が正であることを確認
        for i in range(3):
            if inertia_tensor[i, i] <= 0:
                print(f"Warning: Non-positive diagonal element detected at position ({i},{i})")
                inertia_tensor[i, i] = abs(inertia_tensor[i, i])

        return inertia_tensor

    def calculate_inertia_tensor(self):
        """
        通常モデルの慣性テンソルを計算。
        InspectorWindowクラスのメソッド。
        """
        if not self.current_node or not hasattr(self.current_node, 'stl_file'):
            print("No STL model is loaded.")
            return None

        try:
            # STLデータを取得
            if self.stl_viewer and self.current_node in self.stl_viewer.stl_actors:
                actor = self.stl_viewer.stl_actors[self.current_node]
                poly_data = actor.GetMapper().GetInput()
            else:
                print("No STL actor found for current node")
                return None

            # 体積と質量を取得
            mass_properties = vtk.vtkMassProperties()
            mass_properties.SetInputData(poly_data)
            mass_properties.Update()
            volume = mass_properties.GetVolume()
            density = float(self.density_input.text())
            mass = volume * density

            # 重心を取得
            com_filter = vtk.vtkCenterOfMass()
            com_filter.SetInputData(poly_data)
            com_filter.SetUseScalarsAsWeights(False)
            com_filter.Update()
            center_of_mass = np.array(com_filter.GetCenter())

            print("\nCalculating inertia tensor for normal model...")
            print(f"Volume: {volume:.6f}, Mass: {mass:.6f}")
            print(f"Center of Mass: {center_of_mass}")

            # 慣性テンソルを計算
            inertia_tensor = self._calculate_base_inertia_tensor(
                poly_data, mass, center_of_mass, is_mirrored=False)

            # URDFフォーマットに変換してUIを更新
            urdf_inertia = self.format_inertia_for_urdf(inertia_tensor)
            if hasattr(self, 'inertia_tensor_input'):
                self.inertia_tensor_input.setText(urdf_inertia)
                print("\nInertia tensor has been updated in UI")
            else:
                print("Warning: inertia_tensor_input not found")

            return inertia_tensor

        except Exception as e:
            print(f"Error calculating inertia tensor: {str(e)}")
            traceback.print_exc()
            return None
