"""
FluxhWeave Workbench
====================

Unified application that stitches together the STL sourcer, parts editor
and assembler into a single multi-step workflow.
"""

from __future__ import annotations

import os
import shlex
import signal
import sys
from pathlib import Path
from typing import Callable, Dict, Optional

from PySide6.QtCore import QProcess, Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QFileDialog,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from FluxhWeave_Assembler_V3 import AssemblerWindow, apply_dark_theme as apply_assembler_theme
from FluxhWeave_StlSourcer_V2 import MainWindow as StlSourcerWindow
from URDF_BASIC4 import PartsEditor
from stl_metadata import extract_metadata_text
from FluxhWeave_SimulationConfig import SimulationConfigWidget


class UsdExportWidget(QWidget):
    """Widget providing USD export via IsaacLab tooling."""

    def __init__(self, status_callback: Callable[[str], None], parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._status_callback = status_callback

        self._error_reported = False
        self._canceled = False

        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self._consume_output)
        self.process.finished.connect(self._on_process_finished)
        self.process.errorOccurred.connect(self._on_process_error)

        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        title_label = QLabel("拓展功能 1：导出 USD", self)
        title_label.setStyleSheet("font-size: 18px; font-weight: 600;")
        layout.addWidget(title_label)

        hint_label = QLabel(
            "选择环境、脚本与输入输出路径后，一键调用 IsaacLab 转换工具。",
            self,
        )
        hint_label.setWordWrap(True)
        layout.addWidget(hint_label)

        self.env_line = self._add_text_row(
            layout,
            "Conda 环境名称:",
            "输入已安装 IsaacLab 的 Conda 环境名称",
        )

        self.script_line = self._add_path_row(
            layout,
            "convert_urdf.py 路径:",
            "选择 IsaacLab 的 convert_urdf.py 脚本",
            self._choose_script_path,
        )

        self.urdf_line = self._add_path_row(
            layout,
            "URDF 文件:",
            "选择需要转换的 URDF 文件",
            self._choose_urdf_path,
        )

        self.usd_line = self._add_path_row(
            layout,
            "USD 输出路径:",
            "选择或输入 USD 文件保存路径",
            self._choose_usd_path,
        )

        button_row = QHBoxLayout()
        button_row.setSpacing(8)
        button_row.addStretch(1)

        self.run_button = QPushButton("执行导出", self)
        self.run_button.clicked.connect(self._run_conversion)
        button_row.addWidget(self.run_button)

        self.cancel_button = QPushButton("取消", self)
        self.cancel_button.setEnabled(False)
        self.cancel_button.clicked.connect(self._cancel_conversion)
        button_row.addWidget(self.cancel_button)

        layout.addLayout(button_row)

        self.output_edit = QPlainTextEdit(self)
        self.output_edit.setReadOnly(True)
        self.output_edit.setMinimumHeight(220)
        layout.addWidget(self.output_edit, 1)

    def _add_path_row(
        self,
        layout: QVBoxLayout,
        label_text: str,
        placeholder: str,
        browse_handler: Callable[[], None],
    ) -> QLineEdit:
        row = QHBoxLayout()
        row.setSpacing(8)

        label = QLabel(label_text, self)
        label.setMinimumWidth(150)
        row.addWidget(label)

        line_edit = QLineEdit(self)
        line_edit.setPlaceholderText(placeholder)
        row.addWidget(line_edit, 1)

        browse_button = QPushButton("浏览…", self)
        browse_button.clicked.connect(browse_handler)
        row.addWidget(browse_button)

        layout.addLayout(row)
        return line_edit

    def _add_text_row(
        self,
        layout: QVBoxLayout,
        label_text: str,
        placeholder: str,
    ) -> QLineEdit:
        row = QHBoxLayout()
        row.setSpacing(8)

        label = QLabel(label_text, self)
        label.setMinimumWidth(150)
        row.addWidget(label)

        line_edit = QLineEdit(self)
        line_edit.setPlaceholderText(placeholder)
        row.addWidget(line_edit, 1)

        layout.addLayout(row)
        return line_edit

    def _choose_script_path(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "选择 convert_urdf.py",
            "",
            "Python Files (*.py);;All Files (*)",
        )
        if path:
            self.script_line.setText(path)

    def _choose_urdf_path(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "选择 URDF 文件",
            "",
            "URDF Files (*.urdf);;All Files (*)",
        )
        if path:
            self.urdf_line.setText(path)

    def _choose_usd_path(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self,
            "选择 USD 输出文件",
            "",
            "USD Files (*.usd);;All Files (*)",
        )
        if not path:
            return
        if not path.lower().endswith(".usd"):
            path = f"{path}.usd"
        self.usd_line.setText(path)

    def _run_conversion(self) -> None:
        if self.process.state() != QProcess.NotRunning:
            QMessageBox.warning(self, "正在执行", "当前已有任务在执行中，请稍候。")
            return

        self._error_reported = False
        self._canceled = False

        env_name = self.env_line.text().strip()
        script_path = Path(self.script_line.text()).expanduser()
        urdf_path = Path(self.urdf_line.text()).expanduser()
        usd_path = Path(self.usd_line.text()).expanduser()

        if not env_name:
            QMessageBox.warning(self, "信息缺失", "请输入 Conda 环境名称。")
            return

        if not script_path.is_file():
            QMessageBox.warning(self, "路径无效", "请选择正确的 convert_urdf.py 脚本。")
            return

        if not urdf_path.is_file():
            QMessageBox.warning(self, "路径无效", "请选择正确的 URDF 文件。")
            return

        if not usd_path.parent.exists():
            QMessageBox.warning(self, "路径无效", "USD 输出目录不存在，请先创建。")
            return

        isaaclab_root = None
        for parent in script_path.parents:
            if (parent / "isaaclab.sh").exists():
                isaaclab_root = parent
                break

        if isaaclab_root is None:
            QMessageBox.warning(
                self,
                "缺少启动脚本",
                "未在脚本路径的上级目录中找到 isaaclab.sh，请确认脚本来源无误。",
            )
            return

        try:
            relative_script = script_path.relative_to(isaaclab_root)
        except ValueError:
            QMessageBox.warning(
                self,
                "路径无效",
                "convert_urdf.py 不在 isaaclab.sh 所在目录下，无法定位脚本。",
            )
            return

        command = (
            "eval \"$(conda shell.bash hook)\" && "
            f"conda activate {shlex.quote(env_name)} && "
            f"cd {shlex.quote(str(isaaclab_root))} && "
            f"./isaaclab.sh -p {shlex.quote(str(relative_script))} "
            f"{shlex.quote(str(urdf_path))} {shlex.quote(str(usd_path))}"
        )

        self.output_edit.clear()
        self.output_edit.appendPlainText(
            "执行命令:\n" + command + "\n\n输出信息:\n"
        )

        self.run_button.setEnabled(False)
        self.cancel_button.setEnabled(True)
        self._status_callback("USD 导出：任务已开始")

        self.process.start("/bin/bash", ["-lc", command])

    def _cancel_conversion(self) -> None:
        if self.process.state() != QProcess.NotRunning:
            self._canceled = True
            self._status_callback("USD 导出：正在取消")
            self.process.kill()
            self.process.waitForFinished(2000)

    def _consume_output(self) -> None:
        data = self.process.readAllStandardOutput().data()
        if not data:
            return
        text = data.decode(errors="replace")
        self.output_edit.appendPlainText(text)

    def _on_process_finished(self, exit_code: int, exit_status: QProcess.ExitStatus) -> None:
        self.run_button.setEnabled(True)
        self.cancel_button.setEnabled(False)

        if self._canceled:
            self._status_callback("USD 导出：已取消")
            self._canceled = False
            return

        if exit_status == QProcess.CrashExit:
            self._status_callback("USD 导出：进程异常终止")
            QMessageBox.warning(self, "执行失败", "进程在执行过程中异常终止。")
            return

        if exit_code == 0:
            self._status_callback("USD 导出：完成")
            QMessageBox.information(self, "完成", "成功导出 USD 文件。")
        else:
            if self._error_reported:
                self._error_reported = False
                return
            self._status_callback(f"USD 导出：失败 (代码 {exit_code})")
            QMessageBox.warning(self, "执行失败", "命令执行失败，请查看输出日志。")

    def _on_process_error(self, _error: QProcess.ProcessError) -> None:
        self.run_button.setEnabled(True)
        self.cancel_button.setEnabled(False)
        if self._error_reported:
            return
        self._error_reported = True
        self._status_callback("USD 导出：启动失败")
        QMessageBox.warning(self, "执行失败", "无法启动命令，请检查路径配置。")


class WorkbenchWindow(QMainWindow):
    """Main container for the end-to-end FluxhWeave pipeline."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("FluxhWeave Workbench - STL → URDF Pipeline")
        self.resize(1680, 960)

        self.source_candidate_path: Optional[str] = None
        self.parts_current_path: Optional[str] = None
        self.metadata_output_path: Optional[str] = None
        self.assembler_last_import: Optional[str] = None
        self.joint_defaults_data: Dict[str, float] = {}
        self.joint_defaults_path: Optional[str] = None
        self._simulation_defaults_applied = False

        central = QWidget(self)
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(12)

        self.tab_widget = QTabWidget(self)
        self.tab_widget.currentChanged.connect(self._on_tab_changed)
        main_layout.addWidget(self.tab_widget, 1)

        self._init_stl_tab()
        self._init_parts_tab()
        self._init_assembler_tab()
        self._init_extension_tab()
        self._init_simulation_tab()

        self.status_label = QLabel("准备就绪", self)
        self.status_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.status_label.setStyleSheet("font-size: 12px; color: #d0d0d0;")
        main_layout.addWidget(self.status_label)

        self._connect_stage_signals()

    # ------------------------------------------------------------------
    # UI construction helpers
    # ------------------------------------------------------------------

    def _init_stl_tab(self) -> None:
        container = QWidget(self)
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.stl_window = StlSourcerWindow()
        self._embed_window(self.stl_window, container, layout)

        self.tab_widget.addTab(container, "① STL 预处理")

    def _init_parts_tab(self) -> None:
        container = QWidget(self)
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.parts_editor = PartsEditor()
        self._embed_window(self.parts_editor, container, layout)

        self.tab_widget.addTab(container, "② 部件元数据")

    def _init_assembler_tab(self) -> None:
        container = QWidget(self)
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.assembler = AssemblerWindow()
        self.assembler.jointDefaultsSaved.connect(self._on_joint_defaults_saved)
        self._embed_window(self.assembler, container, layout)

        self.tab_widget.addTab(container, "③ URDF 装配")

    def _init_extension_tab(self) -> None:
        container = QWidget(self)
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.extension_widget = UsdExportWidget(self._set_status, container)
        layout.addWidget(self.extension_widget)

        self.tab_widget.addTab(container, "④ 拓展功能")

    def _init_simulation_tab(self) -> None:
        container = QWidget(self)
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.simulation_widget = SimulationConfigWidget(self._set_status, container)
        layout.addWidget(self.simulation_widget)

        self.tab_widget.addTab(container, "⑤ 仿真配置")
        self.simulation_tab_index = self.tab_widget.indexOf(container)

    @staticmethod
    def _embed_window(window: QMainWindow, container: QWidget, layout: QVBoxLayout) -> None:
        window.setParent(container)
        window.setWindowFlags(Qt.Widget)
        layout.addWidget(window)
        window.show()

    # ------------------------------------------------------------------
    # Signal connections
    # ------------------------------------------------------------------

    def _connect_stage_signals(self) -> None:
        self.stl_window.stlLoaded.connect(self._on_stl_loaded)
        self.stl_window.stlExported.connect(self._on_stl_exported)
        if hasattr(self.stl_window, 'stlSaved'):
            self.stl_window.stlSaved.connect(self._on_stl_exported)

        self.parts_editor.stlLoaded.connect(self._on_parts_loaded)
        self.parts_editor.metadataEmbedded.connect(self._on_metadata_embedded)
        self.parts_editor.xmlExported.connect(self._on_metadata_xml_exported)

        self.assembler.partImported.connect(self._on_part_imported)

    def _on_tab_changed(self, index: int) -> None:
        if getattr(self, "simulation_tab_index", None) is None:
            return
        if index == self.simulation_tab_index:
            self._prepare_simulation_tab()

    def _prepare_simulation_tab(self) -> None:
        announce = not self._simulation_defaults_applied
        if self.joint_defaults_data:
            self.simulation_widget.update_joint_defaults(
                self.joint_defaults_data,
                source=self.joint_defaults_path,
            )
            self._simulation_defaults_applied = True
            if announce and self.joint_defaults_path:
                basename = os.path.basename(self.joint_defaults_path)
                self._set_status(f"仿真配置：已应用关节默认角度 {basename}")
        else:
            candidate_paths = []
            if self.joint_defaults_path:
                candidate_paths.append(self.joint_defaults_path)
            default_candidate = os.path.join(os.getcwd(), "joint_defaults.json")
            if default_candidate not in candidate_paths:
                candidate_paths.append(default_candidate)

            for path in candidate_paths:
                if self._load_joint_defaults_from_path(path, announce=announce):
                    break

        current_defaults = self.simulation_widget.joint_defaults_snapshot()
        if current_defaults:
            self.joint_defaults_data = current_defaults
            source = self.simulation_widget.joint_defaults_source()
            if source:
                self.joint_defaults_path = source
            self._simulation_defaults_applied = True

    def _load_joint_defaults_from_path(self, path: str, *, announce: bool = True) -> bool:
        if not path or not os.path.isfile(path):
            return False
        loaded = self.simulation_widget.load_joint_defaults_from_file(path, show_dialogs=False)
        if not loaded:
            return False
        self.joint_defaults_path = path
        self.joint_defaults_data = self.simulation_widget.joint_defaults_snapshot()
        self._simulation_defaults_applied = True
        if announce:
            basename = os.path.basename(path)
            self._set_status(f"仿真配置：已载入关节默认角度 {basename}")
        return True

    # ------------------------------------------------------------------
    # Stage actions
    # ------------------------------------------------------------------

    def _browse_for_stl(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "选择 STL 文件",
            "",
            "STL Files (*.stl);;All Files (*)",
        )
        if not path:
            return
        if self.stl_window.load_stl_from_path(path):
            self.tab_widget.setCurrentIndex(0)
            self._set_status(f"已加载 STL: {os.path.basename(path)}")

    def _send_to_parts_editor(self) -> None:
        candidate = self.stl_window.last_exported_path or self.stl_window.current_stl_path
        if not candidate:
            QMessageBox.information(self, "提示", "请先在步骤一加载或导出 STL 文件。")
            return
        if self.parts_editor.load_stl_path(candidate):
            self.tab_widget.setCurrentIndex(1)
            self._set_status(f"已载入部件编辑器: {os.path.basename(candidate)}")

    def _send_to_assembler(self) -> None:
        path = getattr(self.parts_editor, 'stl_file_path', None)
        if not path:
            QMessageBox.information(self, "提示", "请先在步骤二加载需要装配的 STL 文件。")
            return
        if not os.path.exists(path):
            QMessageBox.warning(self, "提示", f"当前 STL 文件不存在:\n{path}")
            return
        if not extract_metadata_text(path):
            QMessageBox.warning(
                self,
                "缺少元数据",
                "当前 STL 未嵌入部件配置，请在步骤二选择“保存时嵌入到STL”后再次导出。",
            )
            return
        if self.assembler.import_part_from_path(path):
            self.tab_widget.setCurrentIndex(2)
            self._set_status(f"已导入装配器: {os.path.basename(path)}")

    def load_project_file(self, path: str, show_dialogs: bool = True) -> bool:
        if not path:
            return False
        absolute_path = os.path.abspath(path)
        if not os.path.exists(absolute_path):
            if show_dialogs:
                QMessageBox.warning(self, "提示", f"未找到项目文件:\n{absolute_path}")
            return False
        loaded = self.assembler.load_project_from_path(absolute_path, show_messages=show_dialogs)
        if loaded:
            self.tab_widget.setCurrentIndex(2)
            self._set_status(f"已载入装配项目: {os.path.basename(absolute_path)}")
        return loaded

    # ------------------------------------------------------------------
    # Signal handlers
    # ------------------------------------------------------------------

    def _on_stl_loaded(self, path: str) -> None:
        self.source_candidate_path = path
        self._set_status(f"步骤一：已加载 {os.path.basename(path)}")

    def _on_stl_exported(self, path: str) -> None:
        self.source_candidate_path = path
        self._set_status(f"步骤一：已导出 {os.path.basename(path)}")

    def _on_parts_loaded(self, path: str) -> None:
        self.parts_current_path = path
        self._set_status(f"步骤二：已加载 {os.path.basename(path)}")

    def _on_metadata_embedded(self, path: str) -> None:
        self.metadata_output_path = path
        self._set_status(f"元数据已嵌入: {os.path.basename(path)}")

    def _on_metadata_xml_exported(self, path: str) -> None:
        self.metadata_output_path = path
        self._set_status(f"已导出 XML 配置: {os.path.basename(path)}")

    def _on_part_imported(self, path: str) -> None:
        self.assembler_last_import = path
        self._set_status(f"装配器新增部件: {os.path.basename(path)}")

    def _on_joint_defaults_saved(self, path: str, defaults: Dict[str, float]) -> None:
        self.joint_defaults_data = dict(defaults)
        self.joint_defaults_path = path
        if not self._load_joint_defaults_from_path(path, announce=False):
            self.simulation_widget.update_joint_defaults(self.joint_defaults_data, source=path)
            self._simulation_defaults_applied = True
        basename = os.path.basename(path)
        self._set_status(f"关节默认角度已保存：{basename}")

    def _set_status(self, message: str) -> None:
        self.status_label.setText(message)


def main() -> None:
    app = QApplication(sys.argv)
    apply_assembler_theme(app)

    window = WorkbenchWindow()
    window.show()

    timer = QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    def _signal_handler(sig, frame):
        app.quit()

    signal.signal(signal.SIGINT, _signal_handler)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
