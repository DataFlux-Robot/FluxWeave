import sys
import signal
import traceback
import math
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



class FooNode(BaseNode):
    """通用部件节点类"""
    __identifier__ = 'insilico.nodes'
    NODE_NAME = 'FooNode'

    def __init__(self):
        super(FooNode, self).__init__()
        self.add_input('in', color=(180, 80, 0))

        self.output_count = 0
        self.volume_value = 0.0  # 追加
        self.mass_value = 0.0

        # 初始化节点关联的数据属性
        self.mass_value = 0.0
        self.inertia = {
            'ixx': 0.0, 'ixy': 0.0, 'ixz': 0.0,
            'iyy': 0.0, 'iyz': 0.0, 'izz': 0.0
        }
        self.points = []
        self.cumulative_coords = []
        self.stl_file = None

        # 记录颜色信息
        self.node_color = [1.0, 1.0, 1.0]  # RGB 初始值（白色）

        # 记录默认旋转轴
        self.rotation_axis = 0  # 0 表示 X 轴，1 表示 Y 轴，2 表示 Z 轴

        # 关节角度限制与当前角度
        self.joint_limit_lower = -math.pi
        self.joint_limit_upper = math.pi
        self.joint_position = 0.0

        # 初始化输出端口
        self._add_output()

        self.set_port_deletion_allowed(True)
        self._original_double_click = self.view.mouseDoubleClickEvent
        self.view.mouseDoubleClickEvent = self.node_double_clicked

    def _add_output(self, name=''):
        if self.output_count < 8:  # 最多允许 8 个端口
            self.output_count += 1
            port_name = name or f'point_{self.output_count}'
            super(FooNode, self).add_output(port_name)

            # 初始化点位数据
            if not hasattr(self, 'points'):
                self.points = []

            # 新增的点位默认坐标为 [0, 0, 0]
            self.points.append({
                'name': port_name,
                'type': 'fixed',
                'xyz': [0.0, 0.0, 0.0],
                'axis': [1.0, 0.0, 0.0]
            })

            # 初始化累积坐标
            if not hasattr(self, 'cumulative_coords'):
                self.cumulative_coords = []

            self.cumulative_coords.append({
                'point_index': self.output_count - 1,
                'xyz': [0.0, 0.0, 0.0]
            })

            print(f"已新增输出端口 {port_name}，默认坐标为零")
            return port_name

    def remove_output(self):
        """移除输出端口（增强版）"""
        if self.output_count > 1:
            output_ports = self.output_ports()
            output_port = output_ports[-1] if output_ports else None
            port_name = output_port.name() if output_port and hasattr(output_port, 'name') else f'out_{self.output_count}'
            if output_port:
                try:
                    # 处理已连接的端口
                    for connected_port in output_port.connected_ports():
                        try:
                            print(f"正在断开 {port_name} 与 {connected_port.node().name()}.{connected_port.name()} 的连接")
                            # 使用 NodeGraphQt 的标准断开方法
                            self.graph.disconnect_node(self.id, port_name,
                                                     connected_port.node().id, connected_port.name())
                        except Exception as e:
                            print(f"断开连接时出错: {str(e)}")

                    # 删除对应的点位数据
                    if len(self.points) >= self.output_count:
                        self.points.pop()
                        print(f"已移除端口 {port_name} 的点位数据")

                    # 删除对应的累积坐标
                    if len(self.cumulative_coords) >= self.output_count:
                        self.cumulative_coords.pop()
                        print(f"已移除端口 {port_name} 的累积坐标")

                    # 删除端口本身
                    self.delete_output(output_port)
                    self.output_count -= 1
                    print(f"已删除端口 {port_name}")

                    # 刷新视图
                    self.view.update()

                except Exception as e:
                    print(f"移除端口及相关数据失败: {str(e)}")
                    traceback.print_exc()
            else:
                print(f"未找到输出端口 {port_name}")
        else:
            print("无法删除最后一个输出端口")

    def node_double_clicked(self, event):
        print(f"节点 {self.name()} 被双击")
        if hasattr(self.graph, 'show_inspector'):
            try:
                # 正确获取图形视图对象
                graph_view = self.graph.viewer()  # NodeGraphQt 中使用 viewer() 方法

                # 将场景坐标转换为视图坐标
                scene_pos = event.scenePos()
                view_pos = graph_view.mapFromScene(scene_pos)
                screen_pos = graph_view.mapToGlobal(view_pos)

                print(f"双击位置的屏幕坐标为 ({screen_pos.x()}, {screen_pos.y()})")
                self.graph.show_inspector(self, screen_pos)

            except Exception as e:
                print(f"获取鼠标位置失败: {str(e)}")
                traceback.print_exc()
                # 兜底逻辑：不指定位置直接显示属性面板
                self.graph.show_inspector(self)
        else:
            print("错误：图形对象未提供 show_inspector 方法")
