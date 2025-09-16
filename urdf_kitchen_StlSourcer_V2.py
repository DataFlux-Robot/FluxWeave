import sys
import signal
import vtk
import numpy as np

from PySide6.QtWidgets import (
    QApplication, QFileDialog, QMainWindow, QVBoxLayout, QWidget,
    QPushButton, QHBoxLayout, QCheckBox, QLineEdit, QLabel, QGridLayout, QTextEdit, QSpinBox, QComboBox
)
from PySide6.QtCore import QTimer, Qt
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor


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
        key = self.GetInteractor().GetKeySym()
        shift_pressed = self.GetInteractor().GetShiftKey()
        ctrl_pressed = self.GetInteractor().GetControlKey()

        step = 0.01  # 1cm
        if shift_pressed and ctrl_pressed:
            step = 0.0001  # 0.1mm
        elif shift_pressed:
            step = 0.001  # 1mm

        if self.parent:
            _, _, screen_right, screen_up = self.parent.get_screen_axes()
            for i, checkbox in enumerate(self.parent.point_checkboxes):
                if checkbox.isChecked():
                    if key == "Up":
                        self.parent.move_point_screen(i, screen_up, step)
                    elif key == "Down":
                        self.parent.move_point_screen(i, screen_up, -step)
                    elif key == "Left":
                        self.parent.move_point_screen(i, screen_right, -step)
                    elif key == "Right":
                        self.parent.move_point_screen(i, screen_right, step)
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
    def __init__(self):
        super().__init__()
        self.setWindowTitle("URDF Kitchen - STL 原点与坐标调整工具 v0.0.4 拓展版")
        self.setGeometry(100, 100, 800, 900)
        self.camera_rotation = [0, 0, 0]
        self.absolute_origin = [0, 0, 0]
        self.initial_camera_position = [10, 0, 0]
        self.initial_camera_focal_point = [0, 0, 0]
        self.initial_camera_view_up = [0, 0, 1]

        self.num_points = 1
        self.point_coords = [list(self.absolute_origin) for _ in range(self.num_points)]
        self.point_actors = [None] * self.num_points
        self.point_checkboxes = []
        self.point_inputs = []

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

        self.export_stl_button = QPushButton("导出原点平移后 STL")
        self.export_stl_button.clicked.connect(self.export_stl_with_new_origin)
        button_layout.addWidget(self.export_stl_button)

        self.set_front_btn = QPushButton("当前视角设为 X 前向")
        self.set_front_btn.clicked.connect(self.handle_set_front_as_x)
        button_layout.addWidget(self.set_front_btn)

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

        origin_input_layout = QHBoxLayout()
        origin_input_layout.addWidget(QLabel("相对偏移 X:"))
        self.origin_x_input = QLineEdit("0.0")
        origin_input_layout.addWidget(self.origin_x_input)

        origin_input_layout.addWidget(QLabel("Y:"))
        self.origin_y_input = QLineEdit("0.0")
        origin_input_layout.addWidget(self.origin_y_input)

        origin_input_layout.addWidget(QLabel("Z:"))
        self.origin_z_input = QLineEdit("0.0")
        origin_input_layout.addWidget(self.origin_z_input)

        origin_layout.addLayout(origin_input_layout)

        origin_btn_layout = QHBoxLayout()
        preview_origin_btn = QPushButton("预览原点调整")
        preview_origin_btn.clicked.connect(self.preview_origin_adjustment)
        origin_btn_layout.addWidget(preview_origin_btn)

        reset_origin_preview_btn = QPushButton("重置原点预览")
        reset_origin_preview_btn.clicked.connect(self.reset_origin_preview)
        origin_btn_layout.addWidget(reset_origin_preview_btn)

        apply_origin_btn = QPushButton("应用原点调整")
        apply_origin_btn.clicked.connect(self.apply_origin_adjustment)
        origin_btn_layout.addWidget(apply_origin_btn)

        origin_layout.addLayout(origin_btn_layout)
        main_layout.addLayout(origin_layout)

        self.vtk_widget = QVTKRenderWindowInteractor(central_widget)
        main_layout.addWidget(self.vtk_widget)

        self.volume_label = QLabel("体积 (m^3): 0.000000")
        main_layout.addWidget(self.volume_label)

        self.setup_points_ui(main_layout)

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

        for i in range(self.num_points):
            self.show_point(i)

        self.add_instruction_text()

        self.render_window.Render()
        self.render_window_interactor.Initialize()
        self.vtk_widget.GetRenderWindow().AddObserver("ModifiedEvent", self.update_all_points_size)

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

    def setup_points_ui(self, layout):
        grid = QGridLayout()
        for i in range(self.num_points):
            checkbox = QCheckBox(f"点 {i+1}")
            checkbox.setChecked(True)
            self.point_checkboxes.append(checkbox)
            grid.addWidget(checkbox, i, 0)
            inputs = []
            for j, axis in enumerate(['X', 'Y', 'Z']):
                grid.addWidget(QLabel(axis+':'), i, j*2+1)
                le = QLineEdit(f"{self.point_coords[i][j]}")
                inputs.append(le)
                grid.addWidget(le, i, j*2+2)
            self.point_inputs.append(inputs)
        layout.addLayout(grid)

        btn_row = QHBoxLayout()
        set_btn = QPushButton("设置标记")
        set_btn.clicked.connect(self.handle_set_reset)
        btn_row.addWidget(set_btn)
        reset_btn = QPushButton("重置标记")
        reset_btn.clicked.connect(self.handle_set_reset)
        btn_row.addWidget(reset_btn)
        layout.addLayout(btn_row)

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

    def set_point(self, index):
        try:
            vals = [float(self.point_inputs[index][j].text()) for j in range(3)]
            self.point_coords[index] = vals
            if self.point_checkboxes[index].isChecked():
                self.show_point(index)
            else:
                self.update_point_display(index)
            self.log(f"点{index+1} 已设置: {vals}")
        except ValueError:
            self.log(f"点{index+1} 输入有误")

    def handle_set_reset(self):
        sender = self.sender().text()
        if sender == "设置标记":
            for i, cb in enumerate(self.point_checkboxes):
                if cb.isChecked(): self.set_point(i)
        else:
            for i, cb in enumerate(self.point_checkboxes):
                if cb.isChecked(): self.reset_point_to_origin(i)
            self.update_all_points_size()
        self.render_window.Render()

    def reset_point_to_origin(self, index):
        self.point_coords[index] = list(self.absolute_origin)
        self.update_point_display(index)
        self.log(f"点{index+1} 已重置为原点")

    def move_point_screen(self, index, direction, step):
        mv = direction * step
        self.point_coords[index] = [self.point_coords[index][0]+mv[0],
                                    self.point_coords[index][1]+mv[1],
                                    self.point_coords[index][2]+mv[2]]
        self.update_point_display(index)

    def update_point_display(self, index):
        if self.point_actors[index]:
            self.point_actors[index].SetPosition(self.point_coords[index])
        for j,v in enumerate(self.point_coords[index]):
            self.point_inputs[index][j].setText(f"{v:.6f}")
        self.render_window.Render()

    def show_point(self, index):
        if self.point_actors[index] is None:
            assembly = vtk.vtkAssembly()
            self.create_point_coordinate(assembly, [0,0,0])
            assembly.SetPosition(self.point_coords[index])
            self.renderer.AddActor(assembly)
            self.point_actors[index]=assembly
        else:
            self.point_actors[index].VisibilityOn()
        self.update_point_display(index)

    def create_point_coordinate(self, assembly, coords):
        radius = self.calculate_sphere_radius()
        sp = vtk.vtkSphereSource(); sp.SetRadius(radius)
        mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(sp.GetOutputPort())
        act = vtk.vtkActor(); act.SetMapper(mapper); act.GetProperty().SetColor(1,0,1)
        assembly.AddPart(act)
        axis_colors=[(1,0,0),(0,1,0),(0,0,1)]
        for i,(r,g,b) in enumerate(axis_colors):
            line=vtk.vtkLineSource()
            if i==0: line.SetPoint1(-radius*6,0,0); line.SetPoint2(radius*6,0,0)
            if i==1: line.SetPoint1(0,-radius*6,0); line.SetPoint2(0,radius*6,0)
            if i==2: line.SetPoint1(0,0,-radius*6); line.SetPoint2(0,0,radius*6)
            lmap=vtk.vtkPolyDataMapper(); lmap.SetInputConnection(line.GetOutputPort())
            la=vtk.vtkActor(); la.SetMapper(lmap); la.GetProperty().SetColor(r,g,b); la.GetProperty().SetLineWidth(2)
            assembly.AddPart(la)

    def calculate_sphere_radius(self):
        cam = self.renderer.GetActiveCamera()
        ps = cam.GetParallelScale() if cam else 1.0
        return ps * 0.02

    def update_all_points_size(self, *a):
        for i,act in enumerate(self.point_actors):
            if act:
                self.renderer.RemoveActor(act)
                self.point_actors[i]=None
                self.show_point(i)
        self.render_window.Render()

    def rotate_camera(self, angle, kind):
        cam = self.renderer.GetActiveCamera()
        if kind=='yaw': cam.Azimuth(angle)
        elif kind=='pitch': cam.Elevation(angle)
        elif kind=='roll': cam.Roll(angle)
        self.renderer.ResetCameraClippingRange()
        self.render_window.Render()

    def load_stl_file(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "打开 STL", "", "STL Files (*.stl)")
        if not file_path: return
        self.show_stl(file_path)
        self.reset_camera()
        self.update_all_points_size()

    def show_stl(self, file_path):
        if hasattr(self,'stl_actor') and self.stl_actor:
            self.renderer.RemoveActor(self.stl_actor)
        self.renderer.Clear()
        self.axes_widget = self.add_axes_widget()
        reader = vtk.vtkSTLReader(); reader.SetFileName(file_path); reader.Update()
        clean = vtk.vtkCleanPolyData(); clean.SetInputConnection(reader.GetOutputPort()); clean.Update()
        tri = vtk.vtkTriangleFilter(); tri.SetInputConnection(clean.GetOutputPort()); tri.Update()
        self.original_polydata = tri.GetOutput()
        mapper = vtk.vtkPolyDataMapper(); mapper.SetInputData(self.original_polydata)
        self.stl_actor = vtk.vtkActor(); self.stl_actor.SetMapper(mapper)
        self.renderer.AddActor(self.stl_actor)
        mass_properties = vtk.vtkMassProperties(); mass_properties.SetInputConnection(tri.GetOutputPort()); mass_properties.Update()
        volume = mass_properties.GetVolume()
        self.volume_label.setText(f"体积 (m^3): {volume:.6f}")
        for i in range(self.num_points): self.point_coords[i]=[0,0,0]
        self.fit_camera_to_model()
        self.show_absolute_origin()
        # 新增：在STL原点显示大坐标系
        self.show_stl_origin_axes()
        self.file_name_label.setText(f"文件: {file_path}")
        self.update_all_points_size()
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
            "快捷键: \nR=重置相机  T=线框\nA/D=左右 Yaw  W/S=上下 Pitch\nQ/E=滚转 Roll\n箭头=移动标记 (Shift/Ctrl 微调)"
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

    def get_screen_axes(self):
        cam = self.renderer.GetActiveCamera()
        view_up = np.array(cam.GetViewUp()); forward = np.array(cam.GetDirectionOfProjection())
        right = np.cross(forward, view_up)
        return 'x','y', right/np.linalg.norm(right), view_up/np.linalg.norm(view_up)

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
        if not self.stl_actor:
            self.log("请先加载 STL")
            return
        origin_index = None
        for i, cb in enumerate(self.point_checkboxes):
            if cb.isChecked(): origin_index=i; break
        if origin_index is None:
            self.log("请勾选一个点作为新原点")
            return
        origin_point = self.point_coords[origin_index]
        file_path,_ = QFileDialog.getSaveFileName(self,"保存平移后 STL","","STL Files (*.stl)")
        if not file_path: return
        try:
            poly = self.stl_actor.GetMapper().GetInput()
            tf = vtk.vtkTransform(); tf.Translate(-origin_point[0],-origin_point[1],-origin_point[2])
            tfilter = vtk.vtkTransformPolyDataFilter(); tfilter.SetInputData(poly); tfilter.SetTransform(tf); tfilter.Update()
            writer = vtk.vtkSTLWriter(); writer.SetFileName(file_path); writer.SetInputData(tfilter.GetOutput()); writer.Write()
            self.log(f"已导出: {file_path}")
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
        if not self.stl_actor:
            self.log("请先加载 STL")
            return
        axis_name = self.axis_selector.currentText()
        angle = float(self.angle_selector.value())
        axis_map = {'X': (1,0,0), 'Y': (0,1,0), 'Z': (0,0,1)}
        axis_vec = axis_map[axis_name]
        file_path,_ = QFileDialog.getSaveFileName(self,"保存旋转后的 STL","","STL Files (*.stl)")
        if not file_path: return
        try:
            poly = self.original_polydata
            tf = vtk.vtkTransform()
            tf.RotateWXYZ(angle, *axis_vec)
            tfilter = vtk.vtkTransformPolyDataFilter(); tfilter.SetInputData(poly); tfilter.SetTransform(tf); tfilter.Update()
            writer = vtk.vtkSTLWriter(); writer.SetFileName(file_path); writer.SetInputData(tfilter.GetOutput()); writer.Write()
            self.log(f"已旋转并保存 STL 到: {file_path}")
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
            # 获取用户输入的偏移值
            offset_x = float(self.origin_x_input.text())
            offset_y = float(self.origin_y_input.text())
            offset_z = float(self.origin_z_input.text())

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
            self.log(f"已预览原点调整: 偏移 ({offset_x:.3f}, {offset_y:.3f}, {offset_z:.3f})")

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
        self.origin_x_input.setText("0.0")
        self.origin_y_input.setText("0.0")
        self.origin_z_input.setText("0.0")

        # 更新STL原点坐标系显示
        self.show_stl_origin_axes()

        self.render_window.Render()
        self.log("已重置原点调整预览")

    def apply_origin_adjustment(self):
        """应用原点调整到原始数据"""
        if not self.stl_actor:
            self.log("请先加载STL文件")
            return

        try:
            offset_x = float(self.origin_x_input.text())
            offset_y = float(self.origin_y_input.text())
            offset_z = float(self.origin_z_input.text())

            # 创建变换
            transform = vtk.vtkTransform()
            transform.Translate(-offset_x, -offset_y, -offset_z)

            # 应用变换到原始数据
            transform_filter = vtk.vtkTransformPolyDataFilter()
            transform_filter.SetInputData(self.original_polydata)
            transform_filter.SetTransform(transform)
            transform_filter.Update()

            # 更新原始数据
            self.original_polydata = transform_filter.GetOutput()

            # 更新显示
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(self.original_polydata)
            self.stl_actor.SetMapper(mapper)

            # 重置输入框
            self.origin_x_input.setText("0.0")
            self.origin_y_input.setText("0.0")
            self.origin_z_input.setText("0.0")
            self.origin_offset = [0.0, 0.0, 0.0]

            # 更新STL原点坐标系显示
            self.show_stl_origin_axes()

            self.render_window.Render()
            self.log(f"已应用原点调整: 偏移 ({offset_x:.3f}, {offset_y:.3f}, {offset_z:.3f})")

        except ValueError:
            self.log("输入的偏移值无效，请检查输入")

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
