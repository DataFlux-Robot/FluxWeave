import os
import sys
import signal
from typing import Optional

import vtk
import numpy as np

from PySide6.QtWidgets import (
    QApplication, QFileDialog, QMainWindow, QVBoxLayout, QWidget,
    QPushButton, QHBoxLayout, QLineEdit, QLabel, QTextEdit, QSpinBox, QComboBox
)
from PySide6.QtCore import QTimer, Qt, Signal
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

from stl_metadata import prepare_stl_for_read


class CustomInteractorStyle(vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self, parent=None):
        super(CustomInteractorStyle, self).__init__()
        self.parent = parent
        self.AddObserver("CharEvent", self.on_char_event)
        self.AddObserver("KeyPressEvent", self.on_key_press)

    def on_char_event(self, obj, event):
        key = self.GetInteractor().GetKeySym().lower()
        if key == "t":
            self.parent.log("[T] 线框显示切换")
            self.toggle_wireframe()
        elif key == "r":
            self.parent.log("[R] 重置相机")
            if self.parent:
                self.parent.reset_camera()
        elif key == "a":
            if self.parent: self.parent.rotate_camera(90, 'yaw')
        elif key == "d":
            if self.parent: self.parent.rotate_camera(-90, 'yaw')
        elif key == "w":
            if self.parent: self.parent.rotate_camera(-90, 'pitch')
        elif key == "s":
            if self.parent: self.parent.rotate_camera(90, 'pitch')
        elif key == "q":
            if self.parent: self.parent.rotate_camera(90, 'roll')
        elif key == "e":
            if self.parent: self.parent.rotate_camera(-90, 'roll')
        else:
            self.OnChar()

    def on_key_press(self, obj, event):
        self.OnKeyPress()

    def toggle_wireframe(self):
        renderer = self.GetInteractor().GetRenderWindow().GetRenderers().GetFirstRenderer()
        if not renderer: return
        actors = renderer.GetActors(); actors.InitTraversal()
        actor = actors.GetNextItem()
        while actor:
            if not actor.GetUserTransform():
                prop = actor.GetProperty()
                if prop.GetRepresentation()==vtk.VTK_SURFACE:
                    prop.SetRepresentationToWireframe()
                else:
                    prop.SetRepresentationToSurface()
            actor = actors.GetNextItem()
        self.GetInteractor().GetRenderWindow().Render()


class MainWindow(QMainWindow):
    stlLoaded = Signal(str)
    stlExported = Signal(str)
    stlSaved = Signal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("URDF Kitchen - STL 原点与坐标调整工具 v0.0.4 拓展版")
        self.setGeometry(100, 100, 800, 900)
        self.camera_rotation = [0, 0, 0]
        self.absolute_origin = [0, 0, 0]
        self.initial_camera_position = [10, 0, 0]
        self.initial_camera_focal_point = [0, 0, 0]
        self.initial_camera_view_up = [0, 0, 1]

        self.current_stl_path: Optional[str] = None
        self.last_exported_path: Optional[str] = None

        self.original_polydata = None
        self.rotation_axis_actor = None

        # 新增：STL坐标系和原点调整相关属性
        self.stl_origin_axes_actors = []  # STL原点处的坐标系显示
        self.origin_offset = [0.0, 0.0, 0.0]  # 原点偏移量
        self.preview_stl_actor = None  # 预览用的STL actor
        self.original_stl_center = [0, 0, 0]  # STL几何中心

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        self.set_ui_style()

        self.file_name_label = QLabel("文件: 未加载")
        main_layout.addWidget(self.file_name_label)

        button_layout = QHBoxLayout()
        self.load_button = QPushButton("加载 STL")
        self.load_button.clicked.connect(self.load_stl_file)
        button_layout.addWidget(self.load_button)

        main_layout.addLayout(button_layout)

        # 新功能：旋转控制
        rotate_layout = QHBoxLayout()
        rotate_layout.addWidget(QLabel("旋转轴:"))
        self.axis_selector = QComboBox()
        self.axis_selector.addItems(["X", "Y", "Z"])
        self.axis_selector.currentIndexChanged.connect(self.show_rotation_axis)
        rotate_layout.addWidget(self.axis_selector)

        rotate_layout.addWidget(QLabel("角度:"))
        self.angle_selector = QSpinBox()
        self.angle_selector.setRange(-360, 360)
        self.angle_selector.setValue(90)
        rotate_layout.addWidget(self.angle_selector)

        # 预览按钮
        preview_btn = QPushButton("预览旋转")
        preview_btn.clicked.connect(self.preview_rotate_stl)
        rotate_layout.addWidget(preview_btn)

        reset_preview_btn = QPushButton("重置预览")
        reset_preview_btn.clicked.connect(self.reset_preview)
        rotate_layout.addWidget(reset_preview_btn)

        rotate_btn = QPushButton("执行旋转并保存")
        rotate_btn.clicked.connect(self.rotate_stl_and_save)
        rotate_layout.addWidget(rotate_btn)

        main_layout.addLayout(rotate_layout)

        # 新功能：原点调整控制
        origin_layout = QVBoxLayout()
        origin_label = QLabel("STL 原点调整:")
        origin_layout.addWidget(origin_label)

        self.origin_unit = 'm'
        unit_layout = QHBoxLayout()
        unit_layout.addWidget(QLabel("偏移单位:"))
        self.origin_unit_combo = QComboBox()
        self.origin_unit_combo.addItems(["米 (m)", "毫米 (mm)"])
        self.origin_unit_combo.currentIndexChanged.connect(self.on_origin_unit_changed)
        unit_layout.addWidget(self.origin_unit_combo)
        unit_layout.addStretch(1)
        origin_layout.addLayout(unit_layout)

        origin_input_layout = QHBoxLayout()
        origin_input_layout.addWidget(QLabel("相对偏移 X:"))
        self.origin_x_input = QLineEdit(self._format_origin_value(0.0))
        origin_input_layout.addWidget(self.origin_x_input)

        origin_input_layout.addWidget(QLabel("Y:"))
        self.origin_y_input = QLineEdit(self._format_origin_value(0.0))
        origin_input_layout.addWidget(self.origin_y_input)

        origin_input_layout.addWidget(QLabel("Z:"))
        self.origin_z_input = QLineEdit(self._format_origin_value(0.0))
        origin_input_layout.addWidget(self.origin_z_input)

        origin_layout.addLayout(origin_input_layout)

        origin_btn_layout = QHBoxLayout()
        preview_origin_btn = QPushButton("预览原点调整")
        preview_origin_btn.clicked.connect(self.preview_origin_adjustment)
        origin_btn_layout.addWidget(preview_origin_btn)

        reset_origin_preview_btn = QPushButton("重置原点预览")
        reset_origin_preview_btn.clicked.connect(self.reset_origin_preview)
        origin_btn_layout.addWidget(reset_origin_preview_btn)

        apply_origin_btn = QPushButton("应用原点调整并保存STL")
        apply_origin_btn.clicked.connect(self.apply_origin_adjustment)
        origin_btn_layout.addWidget(apply_origin_btn)

        origin_layout.addLayout(origin_btn_layout)
        main_layout.addLayout(origin_layout)

        self.vtk_widget = QVTKRenderWindowInteractor(central_widget)
        main_layout.addWidget(self.vtk_widget)

        self.volume_label = QLabel("体积 (m^3): 0.000000")
        main_layout.addWidget(self.volume_label)

        self.log_edit = QTextEdit(); self.log_edit.setReadOnly(True)
        main_layout.addWidget(QLabel("日志:"))
        main_layout.addWidget(self.log_edit,1)

        self.setup_vtk()

        self.model_bounds = None
        self.stl_actor = None
        self.current_rotation = 0
        self.stl_center = list(self.absolute_origin)

        self.rotation_types = {'yaw': 0, 'pitch': 1, 'roll': 2}

        self.setup_camera()
        self.axes_widget = self.add_axes_widget()
        self.add_axes()

        self.add_instruction_text()

        self.render_window.Render()
        self.render_window_interactor.Initialize()

    def log(self, msg):
        self.log_edit.append(msg)
        print(msg)

    def set_ui_style(self):
        self.setStyleSheet("""
            QMainWindow { background-color: #2b2b2b; }
            QLabel { color: #ffffff; }
            QPushButton { background-color:#4b5660; color:#ffffff; border:1px solid #5a656f; border-radius:4px; padding:4px 8px; }
            QPushButton:hover { background-color:#566370; }
            QPushButton:pressed { background-color:#3a434a; }
            QLineEdit, QComboBox { background-color:#3a3f44; color:#ffffff; border:1px solid #4d565e; border-radius:3px; padding:2px; }
            QTextEdit { background:#262a2e; color:#d0d0d0; border:1px solid #444; }
            QCheckBox { color:#ffffff; }
        """)

    def _origin_unit_scale(self) -> float:
        return 1.0 if self.origin_unit == 'm' else 0.001

    def _format_origin_value(self, meters: float) -> str:
        scale = self._origin_unit_scale()
        value = meters / scale if scale else meters
        text = f"{value:.6f}"
        if "." in text:
            text = text.rstrip("0").rstrip(".")
        if text == "-0":
            text = "0"
        return text

    def _parse_origin_input(self, text: str) -> float:
        return float(text) * self._origin_unit_scale()

    def on_origin_unit_changed(self, index: int) -> None:
        old_scale = self._origin_unit_scale()
        new_unit = 'm' if index == 0 else 'mm'
        if new_unit == self.origin_unit:
            return

        values_m = []
        for widget in (self.origin_x_input, self.origin_y_input, self.origin_z_input):
            try:
                values_m.append(float(widget.text()) * old_scale)
            except ValueError:
                values_m.append(0.0)

        self.origin_unit = new_unit
        for widget, value in zip((self.origin_x_input, self.origin_y_input, self.origin_z_input), values_m):
            widget.blockSignals(True)
            widget.setText(self._format_origin_value(value))
            widget.blockSignals(False)

    def setup_vtk(self):
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.18, 0.18, 0.22)
        self.render_window = self.vtk_widget.GetRenderWindow()
        self.render_window.AddRenderer(self.renderer)
        self.render_window_interactor = self.render_window.GetInteractor()
        style = CustomInteractorStyle(self)
        self.render_window_interactor.SetInteractorStyle(style)

    def setup_camera(self):
        camera = self.renderer.GetActiveCamera()
        camera.SetPosition(self.absolute_origin[0] + self.initial_camera_position[0],
                           self.absolute_origin[1] + self.initial_camera_position[1],
                           self.absolute_origin[2] + self.initial_camera_position[2])
        camera.SetFocalPoint(*self.absolute_origin)
        camera.SetViewUp(*self.initial_camera_view_up)
        camera.SetParallelScale(5)
        camera.ParallelProjectionOn()
        self.renderer.ResetCameraClippingRange()

    def reset_camera(self):
        camera = self.renderer.GetActiveCamera()
        camera.SetPosition(10,0,0)
        camera.SetFocalPoint(0,0,0)
        camera.SetViewUp(0,0,1)
        camera.ParallelProjectionOn()
        self.renderer.ResetCameraClippingRange()
        self.render_window.Render()
        self.log("相机已重置 (+X 正前, +Z 向上, +Y 向右)")

    def rotate_camera(self, angle, kind):
        cam = self.renderer.GetActiveCamera()
        if kind=='yaw': cam.Azimuth(angle)
        elif kind=='pitch': cam.Elevation(angle)
        elif kind=='roll': cam.Roll(angle)
        self.renderer.ResetCameraClippingRange()
        self.render_window.Render()

    def load_stl_file(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "打开 STL", "", "STL Files (*.stl)")
        if not file_path:
            return
        self.load_stl_from_path(file_path)

    def load_stl_from_path(self, file_path: str) -> bool:
        if not file_path:
            return False
        if not os.path.exists(file_path):
            self.log(f"文件不存在: {file_path}")
            return False
        self.show_stl(file_path)
        self.reset_camera()
        self.current_stl_path = file_path
        self.stlLoaded.emit(file_path)
        return True

    def show_stl(self, file_path):
        if hasattr(self,'stl_actor') and self.stl_actor:
            self.renderer.RemoveActor(self.stl_actor)
        self.renderer.Clear()
        self.axes_widget = self.add_axes_widget()
        clean_path, cleanup = prepare_stl_for_read(file_path)
        try:
            reader = vtk.vtkSTLReader()
            reader.SetFileName(clean_path)
            reader.Update()
        finally:
            cleanup()
        clean = vtk.vtkCleanPolyData(); clean.SetInputConnection(reader.GetOutputPort()); clean.Update()
        tri = vtk.vtkTriangleFilter(); tri.SetInputConnection(clean.GetOutputPort()); tri.Update()
        self.original_polydata = tri.GetOutput()
        mapper = vtk.vtkPolyDataMapper(); mapper.SetInputData(self.original_polydata)
        self.stl_actor = vtk.vtkActor(); self.stl_actor.SetMapper(mapper)
        self.renderer.AddActor(self.stl_actor)
        mass_properties = vtk.vtkMassProperties(); mass_properties.SetInputConnection(tri.GetOutputPort()); mass_properties.Update()
        volume = mass_properties.GetVolume()
        self.volume_label.setText(f"体积 (m^3): {volume:.6f}")
        self.fit_camera_to_model()
        self.show_absolute_origin()
        # 新增：在STL原点显示大坐标系
        self.show_stl_origin_axes()
        self.file_name_label.setText(f"文件: {file_path}")
        self.current_stl_path = file_path
        self.log(f"已加载: {file_path}")
        self.show_rotation_axis()

    def fit_camera_to_model(self):
        if not self.stl_actor: return
        b = self.stl_actor.GetBounds()
        size = max(b[1]-b[0], b[3]-b[2], b[5]-b[4]) * 1.2
        cam = self.renderer.GetActiveCamera()
        center=[(b[0]+b[1])/2,(b[2]+b[3])/2,(b[4]+b[5])/2]
        cam.SetFocalPoint(*center)
        cam.SetPosition(center[0]+size, center[1], center[2])
        cam.SetViewUp(0,0,1)
        cam.SetParallelScale(size/2)
        self.renderer.ResetCameraClippingRange()

    def show_absolute_origin(self):
        sphere = vtk.vtkSphereSource(); sphere.SetCenter(0,0,0); sphere.SetRadius(0.0005)
        mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(sphere.GetOutputPort())
        actor = vtk.vtkActor(); actor.SetMapper(mapper); actor.GetProperty().SetColor(1,1,0)
        self.renderer.AddActor(actor)

    def add_instruction_text(self):
        text_actor_top = vtk.vtkTextActor(); text_actor_top.SetInput(
            "快捷键: \nR=重置相机  T=线框\nA/D=左右 Yaw  W/S=上下 Pitch\nQ/E=滚转 Roll"
        )
        tp = text_actor_top.GetTextProperty(); tp.SetFontSize(14); tp.SetColor(0.9,0.9,0.9)
        text_actor_top.GetPositionCoordinate().SetCoordinateSystemToNormalizedViewport(); text_actor_top.SetPosition(0.02,0.96)
        self.renderer.AddActor(text_actor_top)

    def add_axes(self):
        axis_len = 5
        colors=[(1,0,0),(0,1,0),(0,0,1)]
        for i,c in enumerate(colors):
            for d in [1,-1]:
                line=vtk.vtkLineSource(); p2=[0,0,0]; p2[i]=axis_len*d
                line.SetPoint1(0,0,0); line.SetPoint2(*p2)
                mapper=vtk.vtkPolyDataMapper(); mapper.SetInputConnection(line.GetOutputPort())
                act=vtk.vtkActor(); act.SetMapper(mapper); act.GetProperty().SetColor(*c); act.GetProperty().SetLineWidth(2)
                self.renderer.AddActor(act)

    def add_axes_widget(self):
        axes = vtk.vtkAxesActor(); axes.SetTotalLength(0.3,0.3,0.3)
        widget = vtk.vtkOrientationMarkerWidget(); widget.SetOrientationMarker(axes)
        widget.SetInteractor(self.vtk_widget.GetRenderWindow().GetInteractor())
        widget.SetViewport(0.78,0.78,0.99,0.99)
        widget.EnabledOn(); widget.InteractiveOff(); return widget

    def show_rotation_axis(self):
        if self.rotation_axis_actor:
            self.renderer.RemoveActor(self.rotation_axis_actor)
        axis_name = self.axis_selector.currentText()
        line = vtk.vtkLineSource()
        if axis_name == 'X':
            line.SetPoint1(-1000,0,0)
            line.SetPoint2(1000,0,0)
            color = (1,0,0)
        elif axis_name == 'Y':
            line.SetPoint1(0,-1000,0)
            line.SetPoint2(0,1000,0)
            color = (0,1,0)
        else:
            line.SetPoint1(0,0,-1000)
            line.SetPoint2(0,0,1000)
            color = (0,0,1)
        mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(line.GetOutputPort())
        actor = vtk.vtkActor(); actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color); actor.GetProperty().SetLineWidth(3)
        self.rotation_axis_actor = actor
        self.renderer.AddActor(actor)
        self.render_window.Render()

    def export_stl_with_new_origin(self):
        if not self.stl_actor or self.original_polydata is None:
            self.log("请先加载 STL")
            return
        try:
            offset_x = self._parse_origin_input(self.origin_x_input.text())
            offset_y = self._parse_origin_input(self.origin_y_input.text())
            offset_z = self._parse_origin_input(self.origin_z_input.text())
        except ValueError:
            self.log("输入的偏移值无效，请检查输入")
            return

        file_path,_ = QFileDialog.getSaveFileName(self,"保存平移后 STL","","STL Files (*.stl)")
        if not file_path: return
        try:
            poly = self.original_polydata
            needs_transform = any(abs(v) > 1e-12 for v in (offset_x, offset_y, offset_z))
            if needs_transform:
                tf = vtk.vtkTransform()
                tf.Translate(-offset_x, -offset_y, -offset_z)
                tfilter = vtk.vtkTransformPolyDataFilter()
                tfilter.SetInputData(poly)
                tfilter.SetTransform(tf)
                tfilter.Update()
                export_data = tfilter.GetOutput()
            else:
                export_data = poly

            writer = vtk.vtkSTLWriter()
            writer.SetFileName(file_path)
            writer.SetInputData(export_data)
            writer.Write()
            self.log(f"已导出: {file_path}")
            self.last_exported_path = file_path
            self.stlExported.emit(file_path)
        except Exception as e:
            self.log(f"导出失败: {e}")

    def handle_set_front_as_x(self):
        self.transform_stl_to_camera_view()
        self.reset_camera()
        self.log("已根据当前视角重映射坐标 (屏幕前 = +X, 右 = +Y, 上 = +Z)")

    def transform_stl_to_camera_view(self):
        if not self.stl_actor:
            self.log("未加载 STL")
            return
        cam = self.renderer.GetActiveCamera()
        cam_pos = np.array(cam.GetPosition()); focal = np.array(cam.GetFocalPoint())
        view_dir = cam_pos - focal; x_axis = view_dir/np.linalg.norm(view_dir)
        z_axis = np.array(cam.GetViewUp()); z_axis = z_axis/np.linalg.norm(z_axis)
        y_axis = np.cross(z_axis, x_axis); y_axis = y_axis/np.linalg.norm(y_axis)
        poly = self.stl_actor.GetMapper().GetInput(); pts = poly.GetPoints(); n=pts.GetNumberOfPoints()
        new_pts = vtk.vtkPoints()
        for i in range(n):
            p = np.array(pts.GetPoint(i))
            nx = np.dot(p, x_axis); ny = np.dot(p, y_axis); nz = np.dot(p, z_axis)
            new_pts.InsertNextPoint(nx,ny,nz)
        new_poly = vtk.vtkPolyData(); new_poly.SetPoints(new_pts); new_poly.SetPolys(poly.GetPolys())
        mapper = vtk.vtkPolyDataMapper(); mapper.SetInputData(new_poly)
        self.stl_actor.SetMapper(mapper)
        self.render_window.Render()

    def preview_rotate_stl(self):
        if not self.original_polydata:
            self.log("请先加载STL文件")
            return
        axis_name = self.axis_selector.currentText()
        angle = float(self.angle_selector.value())
        axis_map = {'X': (1,0,0), 'Y': (0,1,0), 'Z': (0,0,1)}
        axis_vec = axis_map[axis_name]
        tf = vtk.vtkTransform()
        tf.RotateWXYZ(angle, *axis_vec)
        tfilter = vtk.vtkTransformPolyDataFilter()
        tfilter.SetInputData(self.original_polydata)
        tfilter.SetTransform(tf)
        tfilter.Update()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(tfilter.GetOutput())
        self.stl_actor.SetMapper(mapper)
        self.render_window.Render()
        self.log(f"已预览旋转 {angle}° 绕 {axis_name} 轴")

    def reset_preview(self):
        if not self.original_polydata:
            return
        mapper = vtk.vtkPolyDataMapper(); mapper.SetInputData(self.original_polydata)
        self.stl_actor.SetMapper(mapper)
        self.render_window.Render()
        self.log("已重置预览到原始模型")

    def rotate_stl_and_save(self):
        if not self.stl_actor or self.original_polydata is None:
            self.log("请先加载 STL")
            return
        if not self.current_stl_path:
            self.log("未找到当前 STL 文件路径，无法保存")
            return
        if not os.path.exists(self.current_stl_path):
            self.log(f"当前 STL 文件不存在: {self.current_stl_path}")
            return
        axis_name = self.axis_selector.currentText()
        angle = float(self.angle_selector.value())
        axis_map = {'X': (1,0,0), 'Y': (0,1,0), 'Z': (0,0,1)}
        axis_vec = axis_map[axis_name]
        try:
            poly = self.original_polydata
            tf = vtk.vtkTransform()
            tf.RotateWXYZ(angle, *axis_vec)
            tfilter = vtk.vtkTransformPolyDataFilter()
            tfilter.SetInputData(poly)
            tfilter.SetTransform(tf)
            tfilter.Update()

            rotated = vtk.vtkPolyData()
            rotated.DeepCopy(tfilter.GetOutput())

            writer = vtk.vtkSTLWriter()
            writer.SetFileName(self.current_stl_path)
            writer.SetInputData(rotated)
            writer.Write()

            self.original_polydata = rotated
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(self.original_polydata)
            self.stl_actor.SetMapper(mapper)
            self.render_window.Render()

            self.log(f"已旋转并保存 STL: {self.current_stl_path}")
            self.last_exported_path = self.current_stl_path
            self.stlSaved.emit(self.current_stl_path)
        except Exception as e:
            self.log(f"旋转保存失败: {e}")

    def show_stl_origin_axes(self):
        """在STL原点位置显示大坐标系"""
        # 清除之前的STL原点坐标系
        for actor in self.stl_origin_axes_actors:
            self.renderer.RemoveActor(actor)
        self.stl_origin_axes_actors.clear()

        if not self.stl_actor:
            return

        # 计算坐标轴长度，基于模型大小
        bounds = self.stl_actor.GetBounds()
        axis_length = max(bounds[1]-bounds[0], bounds[3]-bounds[2], bounds[5]-bounds[4]) * 0.3

        # 创建三个坐标轴 (X=红色, Y=绿色, Z=蓝色)
        colors = [(1,0,0), (0,1,0), (0,0,1)]
        axis_names = ['X', 'Y', 'Z']

        for i, (color, name) in enumerate(zip(colors, axis_names)):
            # 创建轴线
            line = vtk.vtkLineSource()
            start_point = [0, 0, 0]
            end_point = [0, 0, 0]
            end_point[i] = axis_length
            line.SetPoint1(*start_point)
            line.SetPoint2(*end_point)

            # 创建mapper和actor
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(line.GetOutputPort())
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(*color)
            actor.GetProperty().SetLineWidth(4)

            self.renderer.AddActor(actor)
            self.stl_origin_axes_actors.append(actor)

            # 添加轴标签
            text = vtk.vtkVectorText()
            text.SetText(name)
            text_mapper = vtk.vtkPolyDataMapper()
            text_mapper.SetInputConnection(text.GetOutputPort())
            text_actor = vtk.vtkActor()
            text_actor.SetMapper(text_mapper)
            text_actor.GetProperty().SetColor(*color)
            text_actor.SetScale(axis_length * 0.1)

            # 设置文字位置
            label_pos = [0, 0, 0]
            label_pos[i] = axis_length * 1.1
            text_actor.SetPosition(*label_pos)

            self.renderer.AddActor(text_actor)
            self.stl_origin_axes_actors.append(text_actor)

        self.render_window.Render()
        self.log("已在STL原点显示坐标系")

    def preview_origin_adjustment(self):
        """预览原点调整效果"""
        if not self.stl_actor:
            self.log("请先加载STL文件")
            return

        try:
            # 获取用户输入的偏移值（统一转换为米）
            offset_x = self._parse_origin_input(self.origin_x_input.text())
            offset_y = self._parse_origin_input(self.origin_y_input.text())
            offset_z = self._parse_origin_input(self.origin_z_input.text())

            # 保存偏移值
            self.origin_offset = [offset_x, offset_y, offset_z]

            # 创建变换：平移STL模型
            transform = vtk.vtkTransform()
            transform.Translate(-offset_x, -offset_y, -offset_z)

            # 应用变换到原始数据
            transform_filter = vtk.vtkTransformPolyDataFilter()
            transform_filter.SetInputData(self.original_polydata)
            transform_filter.SetTransform(transform)
            transform_filter.Update()

            # 更新显示
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(transform_filter.GetOutput())
            self.stl_actor.SetMapper(mapper)

            # 更新STL原点坐标系显示（现在应该在新的原点位置）
            self.show_stl_origin_axes()

            self.render_window.Render()
            scale = self._origin_unit_scale()
            unit_label = "m" if self.origin_unit == 'm' else 'mm'
            display_offsets = [offset_x / scale, offset_y / scale, offset_z / scale]
            self.log(
                f"已预览原点调整: 偏移 ({display_offsets[0]:.3f}, {display_offsets[1]:.3f}, {display_offsets[2]:.3f}) {unit_label}"
            )

        except ValueError:
            self.log("输入的偏移值无效，请检查输入")

    def reset_origin_preview(self):
        """重置原点调整预览"""
        if not self.original_polydata:
            return

        # 恢复到原始STL显示
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.original_polydata)
        self.stl_actor.SetMapper(mapper)

        # 重置偏移值
        self.origin_offset = [0.0, 0.0, 0.0]
        self.origin_x_input.setText(self._format_origin_value(0.0))
        self.origin_y_input.setText(self._format_origin_value(0.0))
        self.origin_z_input.setText(self._format_origin_value(0.0))

        # 更新STL原点坐标系显示
        self.show_stl_origin_axes()

        self.render_window.Render()
        self.log("已重置原点调整预览")

    def apply_origin_adjustment(self):
        """应用原点调整到原始数据"""
        if not self.stl_actor or self.original_polydata is None:
            self.log("请先加载STL文件")
            return
        if not self.current_stl_path:
            self.log("未找到当前 STL 文件路径，无法保存")
            return
        if not os.path.exists(self.current_stl_path):
            self.log(f"当前 STL 文件不存在: {self.current_stl_path}")
            return

        try:
            offset_x = self._parse_origin_input(self.origin_x_input.text())
            offset_y = self._parse_origin_input(self.origin_y_input.text())
            offset_z = self._parse_origin_input(self.origin_z_input.text())

            # 创建变换
            transform = vtk.vtkTransform()
            transform.Translate(-offset_x, -offset_y, -offset_z)

            # 应用变换到原始数据
            transform_filter = vtk.vtkTransformPolyDataFilter()
            transform_filter.SetInputData(self.original_polydata)
            transform_filter.SetTransform(transform)
            transform_filter.Update()

            # 更新原始数据
            rotated = vtk.vtkPolyData()
            rotated.DeepCopy(transform_filter.GetOutput())
            self.original_polydata = rotated

            # 更新显示
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(self.original_polydata)
            self.stl_actor.SetMapper(mapper)

            # 写回当前 STL 文件
            writer = vtk.vtkSTLWriter()
            writer.SetFileName(self.current_stl_path)
            writer.SetInputData(self.original_polydata)
            writer.Write()

            # 重置输入框
            self.origin_x_input.setText(self._format_origin_value(0.0))
            self.origin_y_input.setText(self._format_origin_value(0.0))
            self.origin_z_input.setText(self._format_origin_value(0.0))
            self.origin_offset = [0.0, 0.0, 0.0]

            # 更新STL原点坐标系显示
            self.show_stl_origin_axes()

            self.render_window.Render()
            scale = self._origin_unit_scale()
            unit_label = "m" if self.origin_unit == 'm' else 'mm'
            display_offsets = [offset_x / scale, offset_y / scale, offset_z / scale]
            self.log(
                f"已应用原点调整: 偏移 ({display_offsets[0]:.3f}, {display_offsets[1]:.3f}, {display_offsets[2]:.3f}) {unit_label}"
            )
            self.last_exported_path = self.current_stl_path
            self.stlSaved.emit(self.current_stl_path)

        except ValueError:
            self.log("输入的偏移值无效，请检查输入")
        except Exception as e:
            self.log(f"应用原点调整失败: {e}")

    def handle_close(self, event):
        self.vtk_widget.GetRenderWindow().Finalize(); self.vtk_widget.close(); event.accept()


def signal_handler(sig, frame):
    QApplication.instance().quit()


def main():
    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal_handler)
    wnd = MainWindow(); wnd.show()
    timer = QTimer(); timer.start(500); timer.timeout.connect(lambda: None)
    try:
        sys.exit(app.exec())
    except SystemExit:
        pass


if __name__ == "__main__":
    main()
