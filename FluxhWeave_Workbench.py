"""
FluxhWeave Workbench
====================

Unified application that stitches together the STL sourcer, parts editor
and assembler into a single multi-step workflow.
"""

from __future__ import annotations

import os
import signal
import sys
from typing import Optional

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QFileDialog,
    QLabel,
    QMainWindow,
    QMessageBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from FluxhWeave_Assembler_V3 import AssemblerWindow, apply_dark_theme as apply_assembler_theme
from FluxhWeave_StlSourcer_V2 import MainWindow as StlSourcerWindow
from URDF_BASIC4 import PartsEditor
from stl_metadata import extract_metadata_text


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

        central = QWidget(self)
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(12)

        self.tab_widget = QTabWidget(self)
        main_layout.addWidget(self.tab_widget, 1)

        self._init_stl_tab()
        self._init_parts_tab()
        self._init_assembler_tab()

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
        self._embed_window(self.assembler, container, layout)

        self.tab_widget.addTab(container, "③ URDF 装配")

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
