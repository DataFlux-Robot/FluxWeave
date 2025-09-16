"""
File Name: urdf_kitchen_PartsEditor_V2.py
Description: 零部件连接点/物性/关节轴/颜色配置编辑器 (URDF Kitchen Parts Editor)

主要新增/修改功能:
1. 全面中文化 UI
2. 每个 Point(最多8个) 的表格式管理: 启用勾选, 自定义名称, X/Y/Z, 关节轴(X/Y/Z/固定), 旋转测试, Set/Reset
3. URDF 原点与姿态(origin_x/y/z, rota_r/p/y) 直接输入 (视觉/惯性统一)
4. 惯性两种模式 Tab:
   - 均匀密度模式: 输入密度, 自动计算质量 + 惯性张量
   - 自定义模式: 用户直接输入 mass, ixx, ixy, ixz, iyy, iyz, izz
5. 颜色选择/应用保留 (RGB 0~1)
6. XML 扩展:
   <point> 内新增 <joint_axis xyz="1 0 0"/> 以及 <name> (若名称自定义)
   <link> 下增加 <origin xyz="..." rpy="..."/>
   <inertial> 中: 均匀密度模式保存 density 属性 density="..."; 自定义模式直接写 mass+inertia
   根保留 <material> 等。兼容旧格式(若无 joint_axis / origin / inertia mode 字段则按默认处理)
7. 旋转测试可针对当前选中的点的轴进行局部测试 (简单实现: 围绕点平移后旋转再还原)

注意: 该文件保持与原 V2 版本部分结构不同, 做了较大重构。
"""

import sys
import signal
import math
import os
import numpy as np
import traceback
import xml.etree.ElementTree as ET

import vtk
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

from PySide6.QtWidgets import (
    QApplication, QFileDialog, QMainWindow, QVBoxLayout, QWidget,
    QPushButton, QHBoxLayout, QCheckBox, QLineEdit, QLabel, QGridLayout,
    QTextEdit, QRadioButton, QColorDialog, QTableWidget, QTableWidgetItem,
    QComboBox, QHeaderView, QMessageBox, QTabWidget, QGroupBox
)
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QColor, QPalette


MAX_POINTS = 8
AXIS_LABELS = ["X", "Y", "Z", "固定"]  # rotation axis options
AXIS_VEC = {
    0: (1, 0, 0),
    1: (0, 1, 0),
    2: (0, 0, 1),
    3: (0, 0, 0)  # fixed
}


def apply_dark_theme(app: QApplication):
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(48, 48, 52))
    palette.setColor(QPalette.WindowText, QColor(235, 235, 235))
    palette.setColor(QPalette.Base, QColor(58, 58, 62))
    palette.setColor(QPalette.AlternateBase, QColor(48, 48, 52))
    palette.setColor(QPalette.ToolTipBase, QColor(255, 255, 255))
    palette.setColor(QPalette.ToolTipText, QColor(0, 0, 0))
    palette.setColor(QPalette.Text, QColor(235, 235, 235))
    palette.setColor(QPalette.Button, QColor(70, 70, 74))
    palette.setColor(QPalette.ButtonText, QColor(235, 235, 235))
    palette.setColor(QPalette.Highlight, QColor(90, 140, 255))
    palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
    app.setPalette(palette)

    app.setStyleSheet(
        """
        QPushButton { padding:4px 8px; background:#4F5A64; border:1px solid #3A4248; border-radius:3px; }
        QPushButton:hover { background:#596672; }
        QPushButton:pressed { background:#3F474E; }
        QLineEdit, QTextEdit { background:#3A3F44; border:1px solid #4A525A; border-radius:3px; padding:2px; }
        QLabel { color:#E0E0E0; }
        QTableWidget { background:#2E3236; gridline-color:#555; }
        QHeaderView::section { background:#40464C; color:#E0E0E0; padding:2px; border:1px solid #444; }
        QCheckBox { color:#E0E0E0; }
        QComboBox { background:#3A3F44; border:1px solid #4A525A; }
        QTabBar::tab { background:#3A434A; padding:6px 12px; border:1px solid #4A525A; }
        QTabBar::tab:selected { background:#4A5660; }
        QGroupBox { border:1px solid #4A525A; margin-top:14px; }
        QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding:0 6px; }
        """
    )


class CustomInteractorStyle(vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self, parent=None):
        super().__init__()
        self.parent = parent
        self.AddObserver("KeyPressEvent", self.on_key_press)
        self.AddObserver("CharEvent", self.on_char_event)

    def on_char_event(self, obj, evt):
        key = self.GetInteractor().GetKeySym()
        if key.lower() == 'r' and self.parent:
            self.parent.reset_camera()
        elif key.lower() == 't' and self.parent:
            self.parent.toggle_wireframe()
        elif key.lower() in ['a','d','w','s','q','e'] and self.parent:
            self.parent.quick_rotate_90(key.lower())
        else:
            self.OnChar()

    def on_key_press(self, obj, evt):
        key = self.GetInteractor().GetKeySym()
        shift = self.GetInteractor().GetShiftKey()
        ctrl = self.GetInteractor().GetControlKey()
        if key in ["Up","Down","Left","Right"] and self.parent:
            step = 0.01
            if shift and ctrl:
                step = 0.0001
            elif shift:
                step = 0.001
            direction_vec = None
            screen_right, screen_up = self.parent.get_screen_move_vectors()
            if key == 'Up':
                direction_vec = screen_up * step
            elif key == 'Down':
                direction_vec = -screen_up * step
            elif key == 'Left':
                direction_vec = -screen_right * step
            elif key == 'Right':
                direction_vec = screen_right * step
            if direction_vec is not None:
                self.parent.nudge_selected_points(direction_vec)
        self.OnKeyPress()


class PartsEditor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("URDF Kitchen - 部件编辑器 v0.0.3")
        self.resize(1400, 800)

        # 数据结构
        self.stl_file_path: str | None = None
        self.point_enabled = [False]*MAX_POINTS
        self.point_names = [f"point{i+1}" for i in range(MAX_POINTS)]
        self.point_coords = [[0.0,0.0,0.0] for _ in range(MAX_POINTS)]
        self.point_axis = [0]*MAX_POINTS  # 0:X 1:Y 2:Z 3:固定

        self.active_rotation_tests = {}  # point_index -> {'angle':..,'timer':QTimer}

        # 惯性模式: 0 = 均匀密度  1 = 自定义
        self.inertia_mode = 0
        self.density_value = 1.0
        self.manual_mass = 0.0
        self.manual_inertia = dict(ixx=0, ixy=0, ixz=0, iyy=0, iyz=0, izz=0)

        # URDF 原点与姿态
        self.origin = [0.0,0.0,0.0]
        self.rpy = [0.0,0.0,0.0]

        # 颜色
        self.color = [1.0,1.0,1.0]

        # VTK
        self.renderer = vtk.vtkRenderer()
        self.vtk_widget = QVTKRenderWindowInteractor(self)
        self.render_window = self.vtk_widget.GetRenderWindow()
        self.render_window.AddRenderer(self.renderer)
        self.iren = self.render_window.GetInteractor()
        style = CustomInteractorStyle(self)
        self.iren.SetInteractorStyle(style)

        self.stl_actor = None
        self.point_actors = [None]*MAX_POINTS
        self.rotation_center_actor = None

        self._build_ui()
        self._build_vtk_scene()

        self.iren.Initialize()
        self.reset_camera()
        self._add_hud_text()

        # 动画计时器(整体旋转测试备用)
        self.global_rotation_timer = QTimer()
        self.global_rotation_timer.timeout.connect(self._tick_global_rotation)
        self.global_rotation_angle = 0

    # ------------- UI 构建 -------------
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        # 左侧面板
        left_panel = QVBoxLayout()
        main_layout.addLayout(left_panel, 3)
        main_layout.addWidget(self.vtk_widget, 7)

        # 文件操作
        file_layout = QHBoxLayout()
        self.btn_load_stl = QPushButton("加载 STL")
        self.btn_load_stl.clicked.connect(self.load_stl)
        self.btn_load_xml = QPushButton("加载 XML")
        self.btn_load_xml.clicked.connect(self.load_xml)
        self.btn_load_stl_xml = QPushButton("加载 STL+XML")
        self.btn_load_stl_xml.clicked.connect(self.load_stl_xml)
        file_layout.addWidget(self.btn_load_stl)
        file_layout.addWidget(self.btn_load_xml)
        file_layout.addWidget(self.btn_load_stl_xml)
        left_panel.addLayout(file_layout)

        # 颜色
        color_layout = QHBoxLayout()
        color_layout.addWidget(QLabel("颜色(R,G,B 0~1):"))
        self.le_cr = QLineEdit("1.0")
        self.le_cg = QLineEdit("1.0")
        self.le_cb = QLineEdit("1.0")
        for le in [self.le_cr,self.le_cg,self.le_cb]:
            le.setFixedWidth(55)
            le.editingFinished.connect(self.apply_color)
        self.btn_pick_color = QPushButton("拾取")
        self.btn_pick_color.clicked.connect(self.pick_color)
        color_layout.addWidget(self.le_cr)
        color_layout.addWidget(self.le_cg)
        color_layout.addWidget(self.le_cb)
        color_layout.addWidget(self.btn_pick_color)
        left_panel.addLayout(color_layout)

        # 原点 / 姿态 输入
        origin_group = QGroupBox("URDF 原点/姿态 (link 内 origin xyz 与 rpy)")
        og_layout = QGridLayout(origin_group)
        self.le_ox = QLineEdit("0.0"); self.le_oy = QLineEdit("0.0"); self.le_oz = QLineEdit("0.0")
        self.le_rr = QLineEdit("0.0"); self.le_rp = QLineEdit("0.0"); self.le_ry = QLineEdit("0.0")
        og_layout.addWidget(QLabel("origin_x"),0,0); og_layout.addWidget(self.le_ox,0,1)
        og_layout.addWidget(QLabel("origin_y"),0,2); og_layout.addWidget(self.le_oy,0,3)
        og_layout.addWidget(QLabel("origin_z"),0,4); og_layout.addWidget(self.le_oz,0,5)
        og_layout.addWidget(QLabel("rota_r"),1,0); og_layout.addWidget(self.le_rr,1,1)
        og_layout.addWidget(QLabel("rota_p"),1,2); og_layout.addWidget(self.le_rp,1,3)
        og_layout.addWidget(QLabel("rota_y"),1,4); og_layout.addWidget(self.le_ry,1,5)
        self.btn_apply_origin = QPushButton("应用原点/姿态")
        self.btn_apply_origin.clicked.connect(self.update_origin_rpy_from_inputs)
        og_layout.addWidget(self.btn_apply_origin,2,0,1,6)
        left_panel.addWidget(origin_group)

        # 惯性 Tab
        self.inertia_tabs = QTabWidget()
        # Tab 0: 均匀密度
        tab_uniform = QWidget(); u_layout = QGridLayout(tab_uniform)
        self.le_density = QLineEdit("1.0")
        self.lbl_volume = QLabel("体积: -")
        self.lbl_mass_auto = QLabel("质量(自动): -")
        self.te_inertia_auto = QTextEdit(); self.te_inertia_auto.setFixedHeight(70)
        self.btn_calc_density = QPushButton("基于密度计算")
        self.btn_calc_density.clicked.connect(self.calc_uniform_inertia)
        u_layout.addWidget(QLabel("密度(kg/m^3):"),0,0); u_layout.addWidget(self.le_density,0,1)
        u_layout.addWidget(self.btn_calc_density,0,2)
        u_layout.addWidget(self.lbl_volume,1,0,1,3)
        u_layout.addWidget(self.lbl_mass_auto,2,0,1,3)
        u_layout.addWidget(QLabel("惯性张量(自动 URDF 格式)"),3,0,1,3)
        u_layout.addWidget(self.te_inertia_auto,4,0,1,3)
        # Tab 1: 自定义
        tab_custom = QWidget(); c_layout = QGridLayout(tab_custom)
        self.le_mass_manual = QLineEdit("0.0")
        self.le_ixx = QLineEdit("0.0"); self.le_ixy = QLineEdit("0.0"); self.le_ixz = QLineEdit("0.0")
        self.le_iyy = QLineEdit("0.0"); self.le_iyz = QLineEdit("0.0"); self.le_izz = QLineEdit("0.0")
        c_layout.addWidget(QLabel("质量 mass"),0,0); c_layout.addWidget(self.le_mass_manual,0,1)
        c_layout.addWidget(QLabel("ixx"),1,0); c_layout.addWidget(self.le_ixx,1,1)
        c_layout.addWidget(QLabel("ixy"),1,2); c_layout.addWidget(self.le_ixy,1,3)
        c_layout.addWidget(QLabel("ixz"),1,4); c_layout.addWidget(self.le_ixz,1,5)
        c_layout.addWidget(QLabel("iyy"),2,0); c_layout.addWidget(self.le_iyy,2,1)
        c_layout.addWidget(QLabel("iyz"),2,2); c_layout.addWidget(self.le_iyz,2,3)
        c_layout.addWidget(QLabel("izz"),2,4); c_layout.addWidget(self.le_izz,2,5)
        self.btn_apply_manual_inertia = QPushButton("应用自定义惯性")
        self.btn_apply_manual_inertia.clicked.connect(self.apply_manual_inertia)
        c_layout.addWidget(self.btn_apply_manual_inertia,3,0,1,6)
        self.inertia_tabs.addTab(tab_uniform, "均匀密度模式")
        self.inertia_tabs.addTab(tab_custom, "自定义惯性模式")
        self.inertia_tabs.currentChanged.connect(self.on_inertia_tab_changed)
        left_panel.addWidget(self.inertia_tabs)

        # Point 表格
        self.points_table = QTableWidget(MAX_POINTS, 9)
        self.points_table.setHorizontalHeaderLabels([
            "启用","名称","X","Y","Z","轴","Set","Reset","旋转测试"
        ])
        self.points_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.points_table.setColumnWidth(0,50)
        for row in range(MAX_POINTS):
            # 启用
            chk = QCheckBox()
            chk.stateChanged.connect(lambda st, r=row: self.toggle_point(r, st))
            self.points_table.setCellWidget(row,0,chk)
            # 名称
            le_name = QLineEdit(self.point_names[row])
            le_name.editingFinished.connect(lambda r=row, w=le_name: self.rename_point(r,w))
            self.points_table.setCellWidget(row,1,le_name)
            # X/Y/Z
            for col_idx,(col_label) in enumerate(['X','Y','Z']):
                le = QLineEdit("0.0")
                le.editingFinished.connect(lambda r=row,c=col_idx,w=le: self.edit_point_coord(r,c,w))
                self.points_table.setCellWidget(row,2+col_idx,le)
            # 轴
            cb_axis = QComboBox(); cb_axis.addItems(AXIS_LABELS)
            cb_axis.currentIndexChanged.connect(lambda idx, r=row: self.change_point_axis(r, idx))
            self.points_table.setCellWidget(row,5,cb_axis)
            # Set
            btn_set = QPushButton("Set")
            btn_set.clicked.connect(lambda _, r=row: self.apply_point_row(r))
            self.points_table.setCellWidget(row,6,btn_set)
            # Reset
            btn_reset = QPushButton("Reset")
            btn_reset.clicked.connect(lambda _, r=row: self.reset_point_row(r))
            self.points_table.setCellWidget(row,7,btn_reset)
            # Test
            btn_test = QPushButton("Test")
            btn_test.setCheckable(True)
            btn_test.toggled.connect(lambda on, r=row: self.toggle_point_rotation_test(r,on))
            self.points_table.setCellWidget(row,8,btn_test)
        left_panel.addWidget(QLabel("连接点 (每行: 勾选启用→命名→坐标→轴→操作)"))
        left_panel.addWidget(self.points_table)

        # 操作按钮
        action_layout = QHBoxLayout()
        self.btn_export_xml = QPushButton("导出 XML")
        self.btn_export_xml.clicked.connect(self.export_xml)
        self.btn_export_mirror = QPushButton("镜像导出 (y) + XML")
        self.btn_export_mirror.clicked.connect(self.export_mirror)
        self.btn_save_shifted = QPushButton("以 Point1 为原点保存STL")
        self.btn_save_shifted.clicked.connect(self.export_shifted_origin)
        action_layout.addWidget(self.btn_export_xml)
        action_layout.addWidget(self.btn_export_mirror)
        action_layout.addWidget(self.btn_save_shifted)
        left_panel.addLayout(action_layout)

        # 状态显示
        self.lbl_status = QLabel("状态: 就绪")
        left_panel.addWidget(self.lbl_status)

    # ------------- VTK 场景 -------------
    def _build_vtk_scene(self):
        self.renderer.SetBackground(0.07,0.07,0.09)
        self._add_world_axes()

    def _add_world_axes(self):
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(0.1,0.1,0.1)
        self.renderer.AddActor(axes)

    def _add_hud_text(self):
        text = vtk.vtkTextActor()
        text.SetInput("快捷键: R重置视图  T线框  A/D/W/S/Q/E 90度旋转  方向键移动选中点 (Shift/ Ctrl微调)")
        text.GetTextProperty().SetFontSize(14)
        text.GetTextProperty().SetColor(0.6,0.85,1.0)
        text.SetPosition(10,10)
        self.renderer.AddActor2D(text)

    # ------------- 颜色 -------------
    def pick_color(self):
        c = QColorDialog.getColor(QColor.fromRgbF(*self.color), self, "选择颜色")
        if c.isValid():
            self.color = [c.redF(), c.greenF(), c.blueF()]
            self.le_cr.setText(f"{self.color[0]:.3f}")
            self.le_cg.setText(f"{self.color[1]:.3f}")
            self.le_cb.setText(f"{self.color[2]:.3f}")
            self.apply_color()

    def apply_color(self):
        try:
            r = float(self.le_cr.text()); g=float(self.le_cg.text()); b=float(self.le_cb.text())
            self.color=[max(0,min(1,r)), max(0,min(1,g)), max(0,min(1,b))]
            if self.stl_actor:
                self.stl_actor.GetProperty().SetColor(*self.color)
                self.render_window.Render()
        except ValueError:
            pass

    # ------------- 点操作 -------------
    def toggle_point(self, row, state):
        enabled = state == Qt.CheckState.Checked.value
        self.point_enabled[row] = enabled
        if enabled:
            self._ensure_point_actor(row)
        else:
            if self.point_actors[row]:
                self.renderer.RemoveActor(self.point_actors[row])
                self.point_actors[row]=None
        self.render_window.Render()

    def rename_point(self, row, widget):
        name = widget.text().strip()
        if not name:
            name = f"point{row+1}"
            widget.setText(name)
        self.point_names[row]=name

    def edit_point_coord(self, row, coord_index, widget):
        try:
            v = float(widget.text())
            self.point_coords[row][coord_index]=v
            if self.point_actors[row]:
                self.point_actors[row].SetPosition(*self.point_coords[row])
                self.render_window.Render()
        except ValueError:
            pass

    def change_point_axis(self, row, axis_idx):
        self.point_axis[row]=axis_idx

    def apply_point_row(self, row):
        # 直接读取表格并更新内部 + 更新 actor
        for c in range(3):
            w:QLineEdit = self.points_table.cellWidget(row,2+c)
            try:
                self.point_coords[row][c]=float(w.text())
            except ValueError:
                pass
        self._ensure_point_actor(row)
        if self.point_actors[row]:
            self.point_actors[row].SetPosition(*self.point_coords[row])
        self.render_window.Render()

    def reset_point_row(self, row):
        self.point_coords[row]=[0.0,0.0,0.0]
        for c in range(3):
            self.points_table.cellWidget(row,2+c).setText("0.0")
        if self.point_actors[row]:
            self.point_actors[row].SetPosition(0,0,0)
        self.render_window.Render()

    def toggle_point_rotation_test(self, row, on):
        if on:
            # 启动该点局部旋转测试
            timer = QTimer()
            timer.timeout.connect(lambda r=row: self._tick_point_rotation(r))
            self.active_rotation_tests[row] = dict(angle=0,timer=timer)
            timer.start(30)
        else:
            if row in self.active_rotation_tests:
                self.active_rotation_tests[row]['timer'].stop()
                del self.active_rotation_tests[row]
                # 复位模型
                if self.stl_actor:
                    self.stl_actor.SetUserTransform(None)
                    self.render_window.Render()

    def _tick_point_rotation(self, row):
        if not self.stl_actor:
            return
        data = self.active_rotation_tests.get(row)
        if not data:
            return
        data['angle'] += 5
        axis_idx = self.point_axis[row]
        ax = AXIS_VEC[axis_idx]
        # 旋转绕该点的坐标
        transform = vtk.vtkTransform()
        px,py,pz = self.point_coords[row]
        transform.PostMultiply()
        transform.Translate(px,py,pz)
        if ax==(1,0,0):
            transform.RotateX(data['angle'])
        elif ax==(0,1,0):
            transform.RotateY(data['angle'])
        elif ax==(0,0,1):
            transform.RotateZ(data['angle'])
        transform.Translate(-px,-py,-pz)
        self.stl_actor.SetUserTransform(transform)
        self.render_window.Render()

    def nudge_selected_points(self, vec):
        # 没有多选概念: 对所有启用点移动(更简单), 可改为仅第一启用
        for i,en in enumerate(self.point_enabled):
            if en:
                self.point_coords[i][0]+=vec[0]
                self.point_coords[i][1]+=vec[1]
                self.point_coords[i][2]+=vec[2]
                for c in range(3):
                    self.points_table.cellWidget(i,2+c).setText(f"{self.point_coords[i][c]:.6f}")
                if self.point_actors[i]:
                    self.point_actors[i].SetPosition(*self.point_coords[i])
        self.render_window.Render()

    def _ensure_point_actor(self,row):
        if not self.point_enabled[row]:
            return
        if self.point_actors[row] is None:
            assembly = vtk.vtkAssembly()
            radius = self._point_visual_radius()
            # 球
            sphere = vtk.vtkSphereSource(); sphere.SetRadius(radius)
            mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(sphere.GetOutputPort())
            actor = vtk.vtkActor(); actor.SetMapper(mapper); actor.GetProperty().SetColor(1,0,1)
            assembly.AddPart(actor)
            # 轴线
            for axis,(r,g,b) in enumerate([(1,0,0),(0,1,0),(0,0,1)]):
                line = vtk.vtkLineSource()
                if axis==0:
                    line.SetPoint1(-radius*4,0,0); line.SetPoint2(radius*4,0,0)
                elif axis==1:
                    line.SetPoint1(0,-radius*4,0); line.SetPoint2(0,radius*4,0)
                else:
                    line.SetPoint1(0,0,-radius*4); line.SetPoint2(0,0,radius*4)
                lmap = vtk.vtkPolyDataMapper(); lmap.SetInputConnection(line.GetOutputPort())
                la = vtk.vtkActor(); la.SetMapper(lmap); la.GetProperty().SetColor(r,g,b); la.GetProperty().SetLineWidth(2)
                assembly.AddPart(la)
            assembly.SetPosition(*self.point_coords[row])
            self.renderer.AddActor(assembly)
            self.point_actors[row]=assembly

    def _point_visual_radius(self):
        cam = self.renderer.GetActiveCamera()
        ps = cam.GetParallelScale() if cam else 1.0
        return ps*0.02

    # ------------- 相机 / 旋转 -------------
    def reset_camera(self):
        cam = self.renderer.GetActiveCamera()
        cam.ParallelProjectionOn()
        cam.SetPosition(1.0,0.0,0.0)
        cam.SetFocalPoint(0,0,0)
        cam.SetViewUp(0,0,1)
        cam.SetParallelScale(0.3)
        self.renderer.ResetCameraClippingRange()
        self.render_window.Render()

    def quick_rotate_90(self,key):
        cam = self.renderer.GetActiveCamera()
        if key == 'a':
            cam.Azimuth(90)
        elif key == 'd':
            cam.Azimuth(-90)
        elif key == 'w':
            cam.Elevation(90)
        elif key == 's':
            cam.Elevation(-90)
        elif key == 'q':
            cam.Roll(90)
        elif key == 'e':
            cam.Roll(-90)
        self.renderer.ResetCameraClippingRange()
        self.render_window.Render()

    def toggle_wireframe(self):
        if not self.stl_actor: return
        prop = self.stl_actor.GetProperty()
        if prop.GetRepresentation()==vtk.VTK_SURFACE:
            prop.SetRepresentationToWireframe()
        else:
            prop.SetRepresentationToSurface()
        self.render_window.Render()

    def get_screen_move_vectors(self):
        cam = self.renderer.GetActiveCamera()
        fp = np.array(cam.GetFocalPoint()); pos = np.array(cam.GetPosition())
        view = fp - pos; view = view/np.linalg.norm(view)
        up = np.array(cam.GetViewUp()); up = up/np.linalg.norm(up)
        right = np.cross(view, up); right = right/np.linalg.norm(right)
        return right, up

    # ------------- 惯性相关 -------------
    def on_inertia_tab_changed(self, idx):
        self.inertia_mode = idx

    def calc_uniform_inertia(self):
        if not self.stl_actor:
            QMessageBox.information(self,"提示","请先加载 STL")
            return
        try:
            density = float(self.le_density.text())
        except ValueError:
            QMessageBox.warning(self,"输入错误","密度请输入数字")
            return
        poly = self.stl_actor.GetMapper().GetInput()
        mp = vtk.vtkMassProperties(); mp.SetInputData(poly); mp.Update()
        volume = mp.GetVolume()
        mass = volume * density
        self.lbl_volume.setText(f"体积: {volume:.6f} m^3")
        self.lbl_mass_auto.setText(f"质量(自动): {mass:.6f} kg")
        # 计算简化惯性张量 (近似：遍历三角形法，与原代码类似)
        inertia = self._compute_mesh_inertia(poly, mass)
        urdf_inertia = self._format_inertia_urdf(inertia)
        self.te_inertia_auto.setText(urdf_inertia)
        self.density_value = density
        self.manual_mass = mass  # 若用户后续切换方便
        self.manual_inertia = {
            'ixx': inertia[0,0], 'ixy': inertia[0,1], 'ixz': inertia[0,2],
            'iyy': inertia[1,1], 'iyz': inertia[1,2], 'izz': inertia[2,2]
        }

    def apply_manual_inertia(self):
        try:
            self.manual_mass = float(self.le_mass_manual.text())
            self.manual_inertia = {
                'ixx': float(self.le_ixx.text()),
                'ixy': float(self.le_ixy.text()),
                'ixz': float(self.le_ixz.text()),
                'iyy': float(self.le_iyy.text()),
                'iyz': float(self.le_iyz.text()),
                'izz': float(self.le_izz.text()),
            }
            QMessageBox.information(self,"已应用","自定义惯性参数已记录，将在导出时写入 XML。")
        except ValueError:
            QMessageBox.warning(self,"输入错误","自定义惯性请输入数字")

    def _compute_mesh_inertia(self, poly_data, mass):
        # 简化版本(面积加权) – 不严格体积分解，只供预览使用
        inertia = np.zeros((3,3))
        com_filter = vtk.vtkCenterOfMass(); com_filter.SetInputData(poly_data); com_filter.SetUseScalarsAsWeights(False); com_filter.Update()
        com = np.array(com_filter.GetCenter())
        total_area = 0.0
        for i in range(poly_data.GetNumberOfCells()):
            cell = poly_data.GetCell(i)
            if cell.GetCellType()!=vtk.VTK_TRIANGLE:
                continue
            pts = cell.GetPoints()
            p = [np.array(pts.GetPoint(j)) - com for j in range(3)]
            v1 = p[1]-p[0]; v2 = p[2]-p[0]
            area = 0.5*np.linalg.norm(np.cross(v1,v2))
            if area < 1e-12: continue
            centroid = (p[0]+p[1]+p[2])/3.0
            r2 = np.sum(centroid*centroid)
            # 使用质点近似(面积权重)
            inertia[0,0] += area*(centroid[1]**2 + centroid[2]**2)
            inertia[1,1] += area*(centroid[0]**2 + centroid[2]**2)
            inertia[2,2] += area*(centroid[0]**2 + centroid[1]**2)
            inertia[0,1] -= area*centroid[0]*centroid[1]
            inertia[0,2] -= area*centroid[0]*centroid[2]
            inertia[1,2] -= area*centroid[1]*centroid[2]
            total_area += area
        if total_area > 0:
            inertia *= (mass / total_area)
        # 对称化
        inertia[1,0]=inertia[0,1]; inertia[2,0]=inertia[0,2]; inertia[2,1]=inertia[1,2]
        return inertia

    def _format_inertia_urdf(self, I):
        return (f"<inertia ixx=\"{I[0,0]:.8f}\" ixy=\"{I[0,1]:.8f}\" ixz=\"{I[0,2]:.8f}\" "
                f"iyy=\"{I[1,1]:.8f}\" iyz=\"{I[1,2]:.8f}\" izz=\"{I[2,2]:.8f}\"/>")

    # ------------- 原点/姿态 -------------
    def update_origin_rpy_from_inputs(self):
        try:
            self.origin = [float(self.le_ox.text()), float(self.le_oy.text()), float(self.le_oz.text())]
            self.rpy = [float(self.le_rr.text()), float(self.le_rp.text()), float(self.le_ry.text())]
            QMessageBox.information(self,"已更新","原点与姿态参数已记录，将在导出 XML 时写入。")
        except ValueError:
            QMessageBox.warning(self,"输入错误","原点/姿态请输入数字")

    # ------------- STL 加载 -------------
    def load_stl(self):
        path,_ = QFileDialog.getOpenFileName(self,"选择 STL 文件","","STL Files (*.stl)")
        if not path: return
        self._load_stl_only(path)

    def load_xml(self):
        path,_ = QFileDialog.getOpenFileName(self,"选择 XML 文件","","XML Files (*.xml)")
        if not path: return
        self._load_xml_only(path)

    def load_stl_xml(self):
        stl_path,_ = QFileDialog.getOpenFileName(self,"选择 STL 文件","","STL Files (*.stl)")
        if not stl_path: return
        self._load_stl_only(stl_path)
        xml_path = os.path.splitext(stl_path)[0]+'.xml'
        if os.path.exists(xml_path):
            self._load_xml_only(xml_path)
        else:
            QMessageBox.information(self,"提示",f"未找到对应 XML: {xml_path}")

    def _load_stl_only(self,path):
        # 清除旧 STL
        if self.stl_actor:
            self.renderer.RemoveActor(self.stl_actor)
        reader = vtk.vtkSTLReader(); reader.SetFileName(path); reader.Update()
        mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(reader.GetOutputPort())
        self.stl_actor = vtk.vtkActor(); self.stl_actor.SetMapper(mapper)
        self.stl_actor.GetProperty().SetColor(*self.color)
        self.renderer.AddActor(self.stl_actor)
        self.stl_file_path = path
        self._reset_all_points_visual()
        self.calc_uniform_inertia()  # 若在均匀密度 Tab 自动计算
        self.fit_model()
        self.lbl_status.setText(f"已加载 STL: {os.path.basename(path)}")

    def fit_model(self):
        if not self.stl_actor: return
        b = self.stl_actor.GetBounds()
        diag = math.sqrt((b[1]-b[0])**2 + (b[3]-b[2])**2 + (b[5]-b[4])**2)
        cam = self.renderer.GetActiveCamera()
        cam.SetParallelScale(diag*0.4)
        self.renderer.ResetCamera()
        self.renderer.ResetCameraClippingRange()
        self.render_window.Render()

    def _reset_all_points_visual(self):
        for i,act in enumerate(self.point_actors):
            if act:
                self.renderer.RemoveActor(act)
        self.point_actors=[None]*MAX_POINTS
        # 保留点数据 (不清空坐标), 仅重新生成时再添加

    # ------------- XML 加载 -------------
    def _load_xml_only(self, path):
        try:
            tree = ET.parse(path)
            root = tree.getroot()
            link = root.find('link')
            if link is not None:
                # origin
                origin_elem = link.find('origin')
                if origin_elem is not None and origin_elem.get('xyz'):
                    ox,oy,oz = map(float, origin_elem.get('xyz').split())
                    self.origin=[ox,oy,oz]
                    self.le_ox.setText(str(ox)); self.le_oy.setText(str(oy)); self.le_oz.setText(str(oz))
                if origin_elem is not None and origin_elem.get('rpy'):
                    rr,rp,ry = map(float, origin_elem.get('rpy').split())
                    self.rpy=[rr,rp,ry]
                    self.le_rr.setText(str(rr)); self.le_rp.setText(str(rp)); self.le_ry.setText(str(ry))
                # inertial
                inertial = link.find('inertial')
                if inertial is not None:
                    mass_elem = inertial.find('mass')
                    inertia_elem = inertial.find('inertia')
                    if mass_elem is not None and inertia_elem is not None:
                        # 自定义模式
                        try:
                            self.manual_mass = float(mass_elem.get('value','0'))
                            self.le_mass_manual.setText(f"{self.manual_mass:.6f}")
                            for k in ['ixx','ixy','ixz','iyy','iyz','izz']:
                                v = float(inertia_elem.get(k,'0'))
                                getattr(self, f"le_{k}").setText(f"{v:.6f}")
                                self.manual_inertia[k]=v
                            self.inertia_tabs.setCurrentIndex(1)
                        except ValueError:
                            pass
                    # 若密度模式有保存 density 属性
                    d_attr = inertial.get('density')
                    if d_attr:
                        try:
                            self.le_density.setText(str(float(d_attr)))
                            self.inertia_tabs.setCurrentIndex(0)
                        except ValueError:
                            pass
            # points
            for row in range(MAX_POINTS):
                # reset enable
                self.points_table.cellWidget(row,0).setChecked(False)
                for c in range(3):
                    self.points_table.cellWidget(row,2+c).setText("0.0")
            i=0
            for pt_elem in root.findall('point'):
                if i>=MAX_POINTS: break
                name = pt_elem.get('name', f'point{i+1}')
                xyz_elem = pt_elem.find('point_xyz')
                axis_elem = pt_elem.find('joint_axis')
                self.points_table.cellWidget(i,0).setChecked(True)
                self.points_table.cellWidget(i,1).setText(name)
                if xyz_elem is not None and xyz_elem.text:
                    try:
                        x,y,z = map(float, xyz_elem.text.strip().split())
                        self.point_coords[i]=[x,y,z]
                        for c,val in enumerate([x,y,z]):
                            self.points_table.cellWidget(i,2+c).setText(f"{val:.6f}")
                    except ValueError:
                        pass
                if axis_elem is not None:
                    xyz_axis = axis_elem.get('xyz','1 0 0').split()
                    try:
                        ax = list(map(float, xyz_axis))
                        if ax==[1.0,0,0]: idx=0
                        elif ax==[0,1.0,0]: idx=1
                        elif ax==[0,0,1.0]: idx=2
                        else: idx=3
                        self.points_table.cellWidget(i,5).setCurrentIndex(idx)
                        self.point_axis[i]=idx
                    except Exception:
                        pass
                i+=1
            self.lbl_status.setText(f"已加载 XML: {os.path.basename(path)}")
        except Exception as e:
            traceback.print_exc()
            QMessageBox.critical(self,"错误",f"解析 XML 失败: {e}")

    # ------------- 导出 XML -------------
    def export_xml(self):
        if not self.stl_file_path:
            QMessageBox.information(self,"提示","请先加载 STL")
            return
        stl_dir = os.path.dirname(self.stl_file_path)
        stl_name = os.path.splitext(os.path.basename(self.stl_file_path))[0]
        xml_path, _ = QFileDialog.getSaveFileName(self,"保存 XML",os.path.join(stl_dir,f"{stl_name}.xml"),"XML Files (*.xml)")
        if not xml_path: return
        self._write_xml(xml_path, stl_name, mirror_mode=False)

    def export_mirror(self):
        if not self.stl_file_path:
            QMessageBox.information(self,"提示","请先加载 STL")
            return
        # y 轴镜像 (l_ / r_ 命名切换)
        base_dir = os.path.dirname(self.stl_file_path)
        name = os.path.splitext(os.path.basename(self.stl_file_path))[0]
        if name.startswith('l_'):
            mname = 'r_'+name[2:]
        elif name.startswith('L_'):
            mname = 'R_'+name[2:]
        elif name.startswith('r_'):
            mname = 'l_'+name[2:]
        elif name.startswith('R_'):
            mname = 'L_'+name[2:]
        else:
            mname = 'mirrored_'+name
        mirrored_stl = os.path.join(base_dir, f"{mname}.stl")
        # 生成镜像 STL
        try:
            reader = vtk.vtkSTLReader(); reader.SetFileName(self.stl_file_path); reader.Update()
            tf = vtk.vtkTransform(); tf.Scale(1,-1,1)
            tfilter = vtk.vtkTransformPolyDataFilter(); tfilter.SetInputConnection(reader.GetOutputPort()); tfilter.SetTransform(tf); tfilter.Update()
            normals = vtk.vtkPolyDataNormals(); normals.SetInputConnection(tfilter.GetOutputPort()); normals.ConsistencyOn(); normals.AutoOrientNormalsOn(); normals.Update()
            writer = vtk.vtkSTLWriter(); writer.SetFileName(mirrored_stl); writer.SetInputData(normals.GetOutput()); writer.Write()
        except Exception as e:
            QMessageBox.critical(self,"错误",f"镜像 STL 失败: {e}")
            return
        # 写 XML
        mirrored_xml = os.path.join(base_dir, f"{mname}.xml")
        self._write_xml(mirrored_xml, mname, mirror_mode=True)
        QMessageBox.information(self,"完成",f"已导出镜像 STL 与 XML:\n{mirrored_stl}\n{mirrored_xml}")

    def _write_xml(self, path, link_name, mirror_mode=False):
        root = ET.Element('urdf_part')
        material = ET.SubElement(root,'material', name=self._color_hex())
        ET.SubElement(material,'color', rgba=f"{self.color[0]:.6f} {self.color[1]:.6f} {self.color[2]:.6f} 1.0")

        link = ET.SubElement(root,'link', name=link_name)
        # origin (visual) 与 inertial 统一使用用户输入
        ET.SubElement(link,'origin', xyz=f"{self.origin[0]:.6f} {self.origin[1]:.6f} {self.origin[2]:.6f}", rpy=f"{self.rpy[0]:.6f} {self.rpy[1]:.6f} {self.rpy[2]:.6f}")

        inertial = ET.SubElement(link,'inertial')
        ET.SubElement(inertial,'origin', xyz=f"{self.origin[0]:.6f} {self.origin[1]:.6f} {self.origin[2]:.6f}")
        if self.inertia_mode == 0:
            # 均匀密度
            # 若未计算则尝试计算
            if self.te_inertia_auto.toPlainText().strip()=="":
                self.calc_uniform_inertia()
            # 查找质量与惯性
            try:
                poly = self.stl_actor.GetMapper().GetInput()
                mp = vtk.vtkMassProperties(); mp.SetInputData(poly); mp.Update(); volume=mp.GetVolume()
                density=float(self.le_density.text()); mass=volume*density
            except Exception:
                density=1.0; mass=0.0
            inertial.set('density', f"{density:.6f}")
            ET.SubElement(inertial,'mass', value=f"{mass:.6f}")
            # 惯性张量文本
            # 解析 te_inertia_auto 内容 (已为合法 <inertia .../>)
            try:
                inertia_fragment = ET.fromstring(self.te_inertia_auto.toPlainText().strip())
                inertial.append(inertia_fragment)
            except Exception:
                pass
        else:
            # 自定义
            ET.SubElement(inertial,'mass', value=f"{self.manual_mass:.6f}")
            inertia_elem = ET.SubElement(inertial,'inertia')
            for k,v in self.manual_inertia.items():
                inertia_elem.set(k,f"{v:.8f}")

        # Points
        for i in range(MAX_POINTS):
            if not self.point_enabled[i]:
                continue
            pt = ET.SubElement(root,'point', name=self.point_names[i], type='fixed')
            # 镜像时 Y 取反
            x,y,z = self.point_coords[i]
            if mirror_mode:
                y = -y
            ET.SubElement(pt,'point_xyz').text = f"{x:.6f} {y:.6f} {z:.6f}"
            # joint axis
            ax_vec = AXIS_VEC[self.point_axis[i]]
            ET.SubElement(pt,'joint_axis', xyz=f"{ax_vec[0]} {ax_vec[1]} {ax_vec[2]}")

        tree = ET.ElementTree(root)
        try:
            tree.write(path, encoding='utf-8', xml_declaration=True)
            self.lbl_status.setText(f"已导出 XML: {os.path.basename(path)}")
        except Exception as e:
            QMessageBox.critical(self,"错误",f"写入 XML 失败: {e}")

    def _color_hex(self):
        return '#{:02x}{:02x}{:02x}'.format(int(self.color[0]*255), int(self.color[1]*255), int(self.color[2]*255))

    # ------------- Point1 重新定位导出 STL -------------
    def export_shifted_origin(self):
        if not self.stl_actor or not self.stl_file_path:
            QMessageBox.information(self,"提示","请先加载 STL")
            return
        # 以第一个启用点(默认 point1)为新原点
        idx = None
        for i,en in enumerate(self.point_enabled):
            if en:
                idx=i; break
        if idx is None:
            QMessageBox.information(self,"提示","请至少启用一个点")
            return
        px,py,pz = self.point_coords[idx]
        save_path,_ = QFileDialog.getSaveFileName(self,"保存平移后 STL","","STL Files (*.stl)")
        if not save_path: return
        try:
            poly = self.stl_actor.GetMapper().GetInput()
            tf = vtk.vtkTransform(); tf.Translate(-px,-py,-pz)
            tfilter = vtk.vtkTransformPolyDataFilter(); tfilter.SetInputData(poly); tfilter.SetTransform(tf); tfilter.Update()
            writer = vtk.vtkSTLWriter(); writer.SetFileName(save_path); writer.SetInputData(tfilter.GetOutput()); writer.Write()
            QMessageBox.information(self,"完成",f"已保存: {save_path}")
        except Exception as e:
            QMessageBox.critical(self,"错误",f"保存失败: {e}")

    # ------------- 旋转动画备用 -------------
    def _tick_global_rotation(self):
        if not self.stl_actor: return
        self.global_rotation_angle += 2
        tf = vtk.vtkTransform(); tf.RotateY(self.global_rotation_angle)
        self.stl_actor.SetUserTransform(tf)
        self.render_window.Render()

    # ------------ 退出 ------------
    def closeEvent(self, event):
        try:
            for data in self.active_rotation_tests.values():
                data['timer'].stop()
        except Exception:
            pass
        event.accept()


def signal_handler(sig, frame):
    QApplication.instance().quit()


def main():
    app = QApplication(sys.argv)
    apply_dark_theme(app)
    wnd = PartsEditor()
    wnd.show()
    timer = QTimer(); timer.start(500); timer.timeout.connect(lambda: None)
    signal.signal(signal.SIGINT, signal_handler)
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
