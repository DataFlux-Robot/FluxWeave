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
from URDF_BASIC4 import PartsEditor





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
        QGroupBox { bord上面的修改方案存在如下问题
1,对于urdf_kitchen_PartsEditor_V2.py使用过程，设定point2等点进行旋转测试过程中，发现演示画面的模型没有绕着point2处的指定轴进行旋转（虽然现在也进行那个了旋转，但是并没有在指定点处）er:1px solid #4A525A; margin-top:14px; }
        QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding:0 6px; }
        """
    )




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
