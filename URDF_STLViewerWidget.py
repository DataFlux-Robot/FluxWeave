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

from URDF_BASIC1 import BaseLinkNode
from stl_metadata import prepare_stl_for_read

class STLViewerWidget(QtWidgets.QWidget):

    def __init__(self, parent=None):
        super(STLViewerWidget, self).__init__(parent)
        self.stl_actors = {}
        self.transforms = {}
        self.base_connected_node = None
        self.text_actors = []

        layout = QtWidgets.QVBoxLayout(self)
        self.vtkWidget = QVTKRenderWindowInteractor(self)
        layout.addWidget(self.vtkWidget)

        self.renderer = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.renderer)
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()

        # 设置交互样式
        style = vtk.vtkInteractorStyleTrackballCamera()
        self.iren.SetInteractorStyle(style)

        # 按钮与滑块的布局
        button_layout = QtWidgets.QVBoxLayout()  # 使用垂直布局

        # “重置角度”按钮
        self.reset_button = QtWidgets.QPushButton("重置角度")
        self.reset_button.clicked.connect(self.reset_camera)
        button_layout.addWidget(self.reset_button)

        # 背景色滑块布局
        bg_layout = QtWidgets.QHBoxLayout()
        bg_label = QtWidgets.QLabel("背景颜色：")
        bg_layout.addWidget(bg_label)

        self.bg_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.bg_slider.setMinimum(-100)  # 黑色
        self.bg_slider.setMaximum(100)   # 白色
        self.bg_slider.setValue(-80)      # 默认值偏暗
        self.bg_slider.valueChanged.connect(self.update_background)
        bg_layout.addWidget(self.bg_slider)

        button_layout.addLayout(bg_layout)
        layout.addLayout(button_layout)

        self.setup_camera()
        self.coordinate_axes_actor = self.create_coordinate_axes()
        self.renderer.AddActor(self.coordinate_axes_actor)

        self.rotation_timer = QtCore.QTimer()
        self.rotation_timer.timeout.connect(self.update_rotation)
        self.rotating_node = None
        self.original_transforms = {}
        self.current_angle = 0

        # 光照设置
        # 主光源：来自前上方45度
        light1 = vtk.vtkLight()
        light1.SetPosition(0.5, 0.5, 1.0)
        light1.SetIntensity(0.7)
        light1.SetLightTypeToSceneLight()

        # 左后方的辅助光
        light2 = vtk.vtkLight()
        light2.SetPosition(-1.0, -0.5, 0.2)
        light2.SetIntensity(0.7)
        light2.SetLightTypeToSceneLight()

        # 右后方的辅助光
        light3 = vtk.vtkLight()
        light3.SetPosition(0.3, -1.0, 0.2)
        light3.SetIntensity(0.7)
        light3.SetLightTypeToSceneLight()

        # 正面的辅助光
        light4 = vtk.vtkLight()
        light4.SetPosition(1.0, 0.0, 0.3)
        light4.SetIntensity(0.3)
        light4.SetLightTypeToSceneLight()

        # 平衡的环境光
        self.renderer.SetAmbient(0.7, 0.7, 0.7)
        self.renderer.LightFollowCameraOff()
        self.renderer.AddLight(light1)
        self.renderer.AddLight(light2)
        self.renderer.AddLight(light3)
        self.renderer.AddLight(light4)

        # 设置初始背景色（偏暗的灰色）
        initial_bg = (-80 + 100) / 200.0  # 将-80的滑块值映射到0-1范围
        self.renderer.SetBackground(initial_bg, initial_bg, initial_bg)

        self.iren.Initialize()

    def store_current_transform(self, node):
        """保存节点当前的变换矩阵"""
        if node in self.transforms:
            current_transform = vtk.vtkTransform()
            current_transform.DeepCopy(self.transforms[node])
            self.original_transforms[node] = current_transform

    def start_rotation_test(self, node):
        """开始对节点执行旋转测试"""
        if node in self.stl_actors:
            self.rotating_node = node
            self.current_angle = 0
            self.rotation_timer.start(16)  # 約60FPS

    def stop_rotation_test(self, node):
        """结束节点的旋转测试"""
        self.rotation_timer.stop()

        # 必须恢复原始颜色与变换
        if self.rotating_node in self.stl_actors:
            # 恢复原始颜色（确保执行）
            if hasattr(self.rotating_node, 'node_color'):
                self.stl_actors[self.rotating_node].GetProperty().SetColor(*self.rotating_node.node_color)

            # 恢复原始变换
            if self.rotating_node in self.original_transforms:
                self.transforms[self.rotating_node].DeepCopy(self.original_transforms[self.rotating_node])
                self.stl_actors[self.rotating_node].SetUserTransform(self.transforms[self.rotating_node])
                del self.original_transforms[self.rotating_node]

            self.vtkWidget.GetRenderWindow().Render()

        self.rotating_node = None

    def update_rotation(self):
        """更新旋转测试的动画帧"""
        if self.rotating_node and self.rotating_node in self.stl_actors:
            node = self.rotating_node
            transform = self.transforms[node]

            # 记录当前位置
            position = transform.GetPosition()

            # 检查是否为固定模式
            is_fixed = hasattr(node, 'rotation_axis') and node.rotation_axis == 3

            # 固定模式下仅执行闪烁
            if is_fixed:
                # 闪烁效果（约每400ms切换）
                # 假设帧率为60FPS，每24帧约等于400ms
                is_red = (self.current_angle // 24) % 2 == 0
                if is_red:
                    self.stl_actors[node].GetProperty().SetColor(1.0, 0.0, 0.0)  # 红色
                else:
                    self.stl_actors[node].GetProperty().SetColor(1.0, 1.0, 1.0)  # 白色
            else:
                # 常规旋转处理
                transform.Identity()  # 重置变换
                transform.Translate(position)  # 保持原始位置

                # 按照旋转轴更新角度
                self.current_angle += 5  # 每帧增加的角度
                if hasattr(node, 'rotation_axis'):
                    if node.rotation_axis == 0:    # X轴
                        transform.RotateX(self.current_angle)
                    elif node.rotation_axis == 1:  # Y轴
                        transform.RotateY(self.current_angle)
                    elif node.rotation_axis == 2:  # Z轴
                        transform.RotateZ(self.current_angle)

                self.stl_actors[node].SetUserTransform(transform)

            self.vtkWidget.GetRenderWindow().Render()
            self.current_angle += 1

    def reset_camera(self):
        """重置相机并确保所有STL模型进入视野"""
        if not self.renderer.GetActors().GetNumberOfItems():
            self.setup_camera()
            return

        # 计算所有角色的综合包围盒
        bounds = [float('inf'), float('-inf'),
                float('inf'), float('-inf'),
                float('inf'), float('-inf')]

        actors = self.renderer.GetActors()
        actors.InitTraversal()
        actor = actors.GetNextActor()
        while actor:
            actor_bounds = actor.GetBounds()
            # 更新 X 轴范围
            bounds[0] = min(bounds[0], actor_bounds[0])
            bounds[1] = max(bounds[1], actor_bounds[1])
            # 更新 Y 轴范围
            bounds[2] = min(bounds[2], actor_bounds[2])
            bounds[3] = max(bounds[3], actor_bounds[3])
            # 更新 Z 轴范围
            bounds[4] = min(bounds[4], actor_bounds[4])
            bounds[5] = max(bounds[5], actor_bounds[5])
            actor = actors.GetNextActor()

        # 计算包围盒中心
        center = [(bounds[1] + bounds[0]) / 2,
                (bounds[3] + bounds[2]) / 2,
                (bounds[5] + bounds[4]) / 2]

        # 计算包围盒对角线长度
        diagonal = ((bounds[1] - bounds[0]) ** 2 +
                (bounds[3] - bounds[2]) ** 2 +
                (bounds[5] - bounds[4]) ** 2) ** 0.5

        camera = self.renderer.GetActiveCamera()
        camera.ParallelProjectionOn()

        # 设置相机位置（位于对角线长度处）
        distance = diagonal
        camera.SetPosition(center[0] + distance, center[1], center[2])
        camera.SetFocalPoint(center[0], center[1], center[2])
        camera.SetViewUp(0, 0, 1)

        # 调整正交缩放以确保模型完整显示
        camera.SetParallelScale(diagonal * 0.5)

        # 更新裁剪范围
        self.renderer.ResetCameraClippingRange()
        self.vtkWidget.GetRenderWindow().Render()

        print("相机已重置，所有 STL 模型均已适配视图")

    def reset_view_to_fit(self):
        """重新调整视角以完整显示全部STL模型"""
        self.reset_camera()
        self.vtkWidget.GetRenderWindow().Render()

    def create_coordinate_axes(self):
        """创建包含线段与文本的三轴坐标指示"""
        base_assembly = vtk.vtkAssembly()
        length = 0.1
        text_offset = 0.02

        # 线段部分的生成逻辑保持不变
        for i, (color, _) in enumerate([
            ((1,0,0), "X"),
            ((0,1,0), "Y"),
            ((0,0,1), "Z")
        ]):
            line = vtk.vtkLineSource()
            line.SetPoint1(0, 0, 0)
            end_point = [0, 0, 0]
            end_point[i] = length
            line.SetPoint2(*end_point)

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(line.GetOutputPort())

            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(*color)
            actor.GetProperty().SetLineWidth(2)

            base_assembly.AddPart(actor)

        # 文本部分改为使用 vtkBillboardTextActor3D
        for i, (color, label) in enumerate([
            ((1,0,0), "X"),
            ((0,1,0), "Y"),
            ((0,0,1), "Z")
        ]):
            text_position = [0, 0, 0]
            text_position[i] = length + text_offset

            text_actor = vtk.vtkBillboardTextActor3D()  # 由 vtkTextActor3D 改为此类型
            text_actor.SetInput(label)
            text_actor.SetPosition(*text_position)
            text_actor.GetTextProperty().SetColor(*color)
            text_actor.GetTextProperty().SetFontSize(12)
            text_actor.GetTextProperty().SetJustificationToCentered()
            text_actor.GetTextProperty().SetVerticalJustificationToCentered()
            text_actor.SetScale(0.02)  # 使用统一比例系数

            self.renderer.AddActor(text_actor)
            if not hasattr(self, 'text_actors'):
                self.text_actors = []
            self.text_actors.append(text_actor)

        return base_assembly

    def update_coordinate_axes(self, position):
        """根据给定位置更新坐标轴与文本"""
        # 更新线段位置
        transform = vtk.vtkTransform()
        transform.Identity()
        transform.Translate(position[0], position[1], position[2])
        self.coordinate_axes_actor.SetUserTransform(transform)

        # 更新文本位置
        if hasattr(self, 'text_actors'):
            for i, text_actor in enumerate(self.text_actors):
                original_pos = list(text_actor.GetPosition())
                text_actor.SetPosition(
                    original_pos[0] + position[0],
                    original_pos[1] + position[1],
                    original_pos[2] + position[2]
                )

        self.vtkWidget.GetRenderWindow().Render()

    def update_stl_transform(self, node, transform: vtk.vtkTransform):
        """更新指定节点的STL模型姿态"""
        if isinstance(node, BaseLinkNode):
            return

        if node in self.stl_actors and node in self.transforms:
            print(f"正在更新节点 {node.name()} 的变换")
            current_transform = self.transforms[node]
            current_transform.DeepCopy(transform)
            self.stl_actors[node].SetUserTransform(current_transform)

            world_pos = current_transform.TransformPoint(0.0, 0.0, 0.0)

            if hasattr(node, 'graph'):
                base_node = node.graph.get_node_by_name('base_link')
                if base_node:
                    for port in base_node.output_ports():
                        for connected_port in port.connected_ports():
                            if connected_port.node() == node:
                                self.base_connected_node = node
                                self.update_coordinate_axes(world_pos)
                                break

            self.vtkWidget.GetRenderWindow().Render()
        else:
            print(f"警告：未找到节点 {node.name()} 对应的 STL 渲染或变换")

    def reset_stl_transform(self, node):
        """重置节点STL模型的变换"""
        # base_link 节点直接跳过
        if isinstance(node, BaseLinkNode):
            return

        if node in self.transforms:
            print(f"正在重置节点 {node.name()} 的变换")
            transform = self.transforms[node]
            transform.Identity()

            self.stl_actors[node].SetUserTransform(transform)

            # 如有需要重置坐标轴
            if node == self.base_connected_node:
                self.update_coordinate_axes([0, 0, 0])
                self.base_connected_node = None

            self.vtkWidget.GetRenderWindow().Render()
        else:
            # 仅针对非 base_link 节点提示警告
            if not isinstance(node, BaseLinkNode):
                print(f"警告：未找到节点 {node.name()} 的变换数据")

    def load_stl_for_node(self, node):
        """为节点载入STL文件并同步颜色"""
        # base_link 节点直接跳过
        if isinstance(node, BaseLinkNode):
            return

        if node.stl_file:
            print(f"正在为节点 {node.name()} 加载 STL 文件：{node.stl_file}")
            clean_path, cleanup = prepare_stl_for_read(node.stl_file)
            try:
                reader = vtk.vtkSTLReader()
                reader.SetFileName(clean_path)
                reader.Update()
            finally:
                cleanup()

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(reader.GetOutputPort())

            actor = vtk.vtkActor()
            actor.SetMapper(mapper)

            transform = vtk.vtkTransform()
            transform.Identity()
            actor.SetUserTransform(transform)

            # 移除已有的渲染对象
            if node in self.stl_actors:
                self.renderer.RemoveActor(self.stl_actors[node])

            self.stl_actors[node] = actor
            self.transforms[node] = transform
            self.renderer.AddActor(actor)

            # 同步节点的颜色
            self.apply_color_to_node(node)

            self.reset_camera()
            self.vtkWidget.GetRenderWindow().Render()
            print(f"STL 文件加载并渲染完成：{node.stl_file}")

    def apply_color_to_node(self, node):
        """为节点的STL模型应用颜色"""
        if node in self.stl_actors:
            # 若无颜色信息则使用默认白色
            if not hasattr(node, 'node_color') or node.node_color is None:
                node.node_color = [1.0, 1.0, 1.0]  # 默认白色

            # 应用颜色到渲染对象
            actor = self.stl_actors[node]
            actor.GetProperty().SetColor(*node.node_color)
            print(f"已将颜色应用到节点 {node.name()}：RGB({node.node_color[0]:.3f}, {node.node_color[1]:.3f}, {node.node_color[2]:.3f})")
            self.vtkWidget.GetRenderWindow().Render()

    def remove_stl_for_node(self, node):
        """移除节点对应的STL模型"""
        if node in self.stl_actors:
            self.renderer.RemoveActor(self.stl_actors[node])
            del self.stl_actors[node]
            if node in self.transforms:
                del self.transforms[node]

            # 如有需要重置坐标轴
            if node == self.base_connected_node:
                self.update_coordinate_axes([0, 0, 0])
                self.base_connected_node = None

            self.vtkWidget.GetRenderWindow().Render()
            print(f"已移除节点 {node.name()} 的 STL 模型")

    def setup_camera(self):
        """初始化相机参数"""
        camera = self.renderer.GetActiveCamera()
        camera.ParallelProjectionOn()
        camera.SetPosition(1, 0, 0)
        camera.SetFocalPoint(0, 0, 0)
        camera.SetViewUp(0, 0, 1)

    def cleanup(self):
        """清理STL查看器相关资源"""
        # 释放 VTK 相关对象
        if hasattr(self, 'renderer'):
            if self.renderer:
                # 删除渲染角色
                for actor in self.renderer.GetActors():
                    self.renderer.RemoveActor(actor)

                # 删除文本渲染对象
                for actor in self.text_actors:
                    self.renderer.RemoveActor(actor)
                self.text_actors.clear()

        # 结束交互器
        if hasattr(self, 'iren'):
            if self.iren:
                self.iren.TerminateApp()

        # 清理渲染窗口
        if hasattr(self, 'vtkWidget'):
            if self.vtkWidget:
                self.vtkWidget.close()

        # 清空引用数据
        self.stl_actors.clear()
        self.transforms.clear()

    def __del__(self):
        """在析构时执行资源清理"""
        self.cleanup()

    def update_rotation_axis(self, node, axis_id):
        """更新节点的旋转轴设置"""
        try:
            print(f"正在将节点 {node.name()} 的旋转轴更新为 {axis_id}")

            if node in self.stl_actors and node in self.transforms:
                transform = self.transforms[node]
                actor = self.stl_actors[node]

                # 保存当前位置
                current_position = list(actor.GetPosition())

                # 重置变换
                transform.Identity()

                # 重新应用位移
                transform.Translate(*current_position)

                # 可根据新的旋转轴在此追加旋转逻辑

                # 应用变换
                actor.SetUserTransform(transform)

                # 刷新视图
                self.vtkWidget.GetRenderWindow().Render()
                print(f"节点 {node.name()} 的旋转轴已更新")
            else:
                print(f"未找到节点 {node.name()} 对应的 STL 渲染或变换")

        except Exception as e:
            print(f"更新旋转轴时出错: {str(e)}")
            traceback.print_exc()

    def update_background(self, value):
        """根据滑块数值调整背景灰度"""
        # 将取值范围 -100~100 转换为 0~1
        normalized_value = (value + 100) / 200.0
        self.renderer.SetBackground(normalized_value, normalized_value, normalized_value)
        self.vtkWidget.GetRenderWindow().Render()

    def open_urdf_loader_website(self):
        """打开 URDF Loaders 网站"""
        url = QtCore.QUrl(
            "https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/")
        QtGui.QDesktopServices.openUrl(url)
