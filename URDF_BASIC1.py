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



class BaseLinkNode(BaseNode):
    """基础链接节点类"""
    __identifier__ = 'insilico.nodes'
    NODE_NAME = 'BaseLinkNode'

    def __init__(self):
        super(BaseLinkNode, self).__init__()
        self.add_output('out')

        self.volume_value = 0.0  # 新增字段
        self.mass_value = 0.0

        self.inertia = {
            'ixx': 0.0, 'ixy': 0.0, 'ixz': 0.0,
            'iyy': 0.0, 'iyz': 0.0, 'izz': 0.0
        }
        self.points = [{
            'name': 'base_link_point1',
            'type': 'fixed',
            'xyz': [0.0, 0.0, 0.0],
            'axis': [0.0, 0.0, 0.0]
        }]
        self.cumulative_coords = [{
            'point_index': 0,
            'xyz': [0.0, 0.0, 0.0]
        }]

        self.stl_file = None

        # 记录颜色信息
        self.node_color = [1.0, 1.0, 1.0]  # RGB 初始值（白色）

        # 记录默认旋转轴方向
        self.rotation_axis = 0  # 0 表示 X 轴，1 表示 Y 轴，2 表示 Z 轴

        # 关节限位与当前角度（基座固定）
        self.joint_limit_lower = 0.0
        self.joint_limit_upper = 0.0
        self.joint_position = 0.0

    def add_input(self, name='', **kwargs):
        # 禁止为基座添加输入端口
        print("基础链接节点无法添加输入端口")
        return None

    def add_output(self, name='out_1', **kwargs):
        # 输出端口已存在时不再新增
        if not self.has_output(name):
            return super(BaseLinkNode, self).add_output(name, **kwargs)
        return None

    def remove_output(self, port=None):
        # 禁止删除基座的输出端口
        print("基础链接节点无法删除输出端口")
        return None

    def has_output(self, name):
        """检查指定名称的输出端口是否存在"""
        return name in [p.name() for p in self.output_ports()]
