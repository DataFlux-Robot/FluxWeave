import sys
import signal
import traceback
from Qt import QtWidgets, QtCore, QtGui
from NodeGraphQt import NodeGraph, BaseNode
from NodeGraphQt.errors import NodeRegistrationError
import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from PySide6.QtWidgets import QFileDialog
from PySide6.QtCore import QPointF, QRegularExpression
from PySide6.QtGui import QDoubleValidator, QRegularExpressionValidator, QPalette, QColor
import os
import xml.etree.ElementTree as ET
import base64
import datetime
import io
import math
import posixpath
import tempfile
import zipfile

import numpy as np
from URDF_BASIC1 import BaseLinkNode
from URDF_BASIC2 import FooNode
from URDF_BASIC3 import InspectorWindow
from stl_metadata import extract_metadata_text, copy_without_metadata


class ConnectionConfigDialog(QtWidgets.QDialog):
    """Small helper dialog to configure how two points should be connected."""

    def __init__(
        self,
        parent_name: str,
        child_name: str,
        parent_point_label: str,
        parent_axis: list[float] | None,
        child_points: list[dict],
        default_point: int,
        default_sign: int,
        parent: QtWidgets.QWidget | None = None,
    ):
        super().__init__(parent)
        self.setWindowTitle("Configure Connection")
        self.setModal(True)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        info_label = QtWidgets.QLabel(
            f"{parent_name}.{parent_point_label}\n→ {child_name}"
        )
        info_label.setWordWrap(True)
        layout.addWidget(info_label)

        # Child point selection
        point_box = QtWidgets.QGroupBox("Child Point")
        point_layout = QtWidgets.QVBoxLayout(point_box)
        self.point_combo = QtWidgets.QComboBox()
        for idx, point in enumerate(child_points):
            name = point.get('name') or f"point_{idx + 1}"
            axis = point.get('axis') or []
            axis_txt = f" axis=({axis[0]:.2f},{axis[1]:.2f},{axis[2]:.2f})" if len(axis) >= 3 else ""
            self.point_combo.addItem(f"{name}{axis_txt}", idx)
        if 0 <= default_point < self.point_combo.count():
            self.point_combo.setCurrentIndex(default_point)
        point_layout.addWidget(self.point_combo)
        layout.addWidget(point_box)

        # Axis direction selection (only relevant if parent axis has direction)
        self.sign_group = QtWidgets.QButtonGroup(self)
        sign_box = QtWidgets.QGroupBox("Axis Direction")
        sign_layout = QtWidgets.QHBoxLayout(sign_box)
        plus_btn = QtWidgets.QRadioButton("Align (+)")
        minus_btn = QtWidgets.QRadioButton("Flip (-)")
        self.sign_group.addButton(plus_btn, 1)
        self.sign_group.addButton(minus_btn, -1)
        sign_layout.addWidget(plus_btn)
        sign_layout.addWidget(minus_btn)
        layout.addWidget(sign_box)

        magnitude = 0.0
        if parent_axis:
            try:
                magnitude = math.sqrt(sum(float(v) * float(v) for v in parent_axis[:3]))
            except Exception:
                magnitude = 0.0

        if magnitude < 1e-8:
            sign_box.setEnabled(False)
            plus_btn.setChecked(True)
        else:
            if default_sign >= 0:
                plus_btn.setChecked(True)
            else:
                minus_btn.setChecked(True)

        if parent_axis:
            axis_label = QtWidgets.QLabel(
                "Parent axis: (" + ", ".join(f"{float(v):.2f}" for v in parent_axis[:3]) + ")"
            )
            axis_label.setWordWrap(True)
            layout.addWidget(axis_label)

        button_box = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

    def selected_point_index(self) -> int:
        return self.point_combo.currentData()

    def selected_sign(self) -> int:
        button = self.sign_group.checkedId()
        return button if button in (1, -1) else 1


class CustomNodeGraph(NodeGraph):
    def __init__(self, stl_viewer):
        super(CustomNodeGraph, self).__init__()
        self.stl_viewer = stl_viewer
        self.robot_name = "robot_x"
        self.project_dir = None
        self.meshes_dir = None
        self.last_save_dir = None
        self.last_export_dir = None
        self._joint_change_callback = None
        self.connection_settings: dict[tuple[int, int], dict] = {}

        # ポート接続/切断のシグナルを接続
        self.port_connected.connect(self.on_port_connected)
        self.port_disconnected.connect(self.on_port_disconnected)

        # ノードタイプの登録
        for node_cls in (BaseLinkNode, FooNode):
            try:
                self.register_node(node_cls)
                print(f"Registered node type: {node_cls.NODE_NAME}")
            except NodeRegistrationError:
                print(f"Node type {node_cls.type_} already registered, skipping.")
            except Exception as e:
                print(f"Error registering node type {node_cls}: {e}")
                traceback.print_exc()

        # 他の初期化コード...
        self._cleanup_handlers = []
        self._cached_positions = {}
        self._selection_cache = set()

        # 選択関連の変数を初期化
        self._selection_start = None
        self._is_selecting = False

        # ビューの設定
        self._view = self.widget

        # ラバーバンドの作成
        self._rubber_band = QtWidgets.QRubberBand(
            QtWidgets.QRubberBand.Shape.Rectangle,
            self._view
        )

        # オリジナルのイベントハンドラを保存
        self._original_handlers = {
            'press': self._view.mousePressEvent,
            'move': self._view.mouseMoveEvent,
            'release': self._view.mouseReleaseEvent
        }

        # 新しいイベントハンドラを設定
        self._view.mousePressEvent = self.custom_mouse_press
        self._view.mouseMoveEvent = self.custom_mouse_move
        self._view.mouseReleaseEvent = self.custom_mouse_release

        # インスペクタウィンドウの初期化
        self.inspector_window = InspectorWindow(stl_viewer=self.stl_viewer)

        # 右クリックメニュー（ノードの削除など）
        try:
            self.widget.setContextMenuPolicy(QtCore.Qt.ContextMenuPolicy.CustomContextMenu)
            self.widget.customContextMenuRequested.connect(self._show_context_menu)
        except Exception as e:
            print(f"Warning: failed to install context menu: {e}")

    def _show_context_menu(self, pos):
        """グラフビューの右クリックメニュー"""
        try:
            menu = QtWidgets.QMenu(self.widget)
            # Delete Node
            deletable = [n for n in self.selected_nodes() if not isinstance(n, BaseLinkNode)]
            act_del = menu.addAction("Delete Node")
            act_del.setEnabled(len(deletable) > 0)
            action = menu.exec_(self.widget.mapToGlobal(pos))
            if action == act_del:
                for n in deletable:
                    self.remove_node(n)
        except Exception as e:
            print(f"Error in context menu: {e}")

    def custom_mouse_press(self, event):
        """カスタムマウスプレスイベントハンドラ"""
        try:
            # 左ボタンの処理
            if event.button() == QtCore.Qt.MouseButton.LeftButton:
                self._selection_start = event.position().toPoint()
                self._is_selecting = True

                # ラバーバンドの設定
                if self._rubber_band:
                    rect = QtCore.QRect(self._selection_start, QtCore.QSize())
                    self._rubber_band.setGeometry(rect)
                    self._rubber_band.show()

                # Ctrlキーが押されていない場合は選択をクリア
                if not event.modifiers() & QtCore.Qt.KeyboardModifier.ControlModifier:
                    for node in self.selected_nodes():
                        node.set_selected(False)

            # オリジナルのイベントハンドラを呼び出し
            self._original_handlers['press'](event)

        except Exception as e:
            print(f"Error in mouse press: {str(e)}")

    def custom_mouse_move(self, event):
        """カスタムマウス移動イベントハンドラ"""
        try:
            if self._is_selecting and self._selection_start:
                current_pos = event.position().toPoint()
                rect = QtCore.QRect(self._selection_start,
                                    current_pos).normalized()
                if self._rubber_band:
                    self._rubber_band.setGeometry(rect)

            # オリジナルのイベントハンドラを呼び出し
            self._original_handlers['move'](event)

        except Exception as e:
            print(f"Error in mouse move: {str(e)}")

    def custom_mouse_release(self, event):
        """カスタムマウスリリースイベントハンドラ"""
        try:
            if event.button() == QtCore.Qt.MouseButton.LeftButton and self._is_selecting:
                if self._rubber_band and self._selection_start:
                    # 選択範囲の処理
                    rect = self._rubber_band.geometry()
                    scene_rect = self._view.mapToScene(rect).boundingRect()

                    # 範囲内のノードを選択
                    for node in self.all_nodes():
                        node_pos = node.pos()
                        if isinstance(node_pos, (list, tuple)):
                            node_point = QtCore.QPointF(
                                node_pos[0], node_pos[1])
                        else:
                            node_point = node_pos

                        if scene_rect.contains(node_point):
                            node.set_selected(True)

                    # ラバーバンドを隠す
                    self._rubber_band.hide()

                # 選択状態をリセット
                self._selection_start = None
                self._is_selecting = False

            # オリジナルのイベントハンドラを呼び出し
            self._original_handlers['release'](event)

        except Exception as e:
            print(f"Error in mouse release: {str(e)}")

    def cleanup(self):
        """リソースのクリーンアップ"""
        try:
            print("Starting cleanup process...")

            # イベントハンドラの復元
            if hasattr(self, '_view') and self._view:
                if hasattr(self, '_original_handlers'):
                    self._view.mousePressEvent = self._original_handlers['press']
                    self._view.mouseMoveEvent = self._original_handlers['move']
                    self._view.mouseReleaseEvent = self._original_handlers['release']
                    print("Restored original event handlers")

            # ラバーバンドのクリーンアップ
            try:
                if hasattr(self, '_rubber_band') and self._rubber_band and not self._rubber_band.isHidden():
                    self._rubber_band.hide()
                    self._rubber_band.setParent(None)
                    self._rubber_band.deleteLater()
                    self._rubber_band = None
                    print("Cleaned up rubber band")
            except Exception as e:
                print(f"Warning: Rubber band cleanup - {str(e)}")

            # ノードのクリーンアップ
            for node in self.all_nodes():
                try:
                    # STLデータのクリーンアップ
                    if self.stl_viewer:
                        self.stl_viewer.remove_stl_for_node(node)
                    # ノードの削除
                    self.remove_node(node)
                except Exception as e:
                    print(f"Error cleaning up node: {str(e)}")

            # インスペクタウィンドウのクリーンアップ
            if hasattr(self, 'inspector_window') and self.inspector_window:
                try:
                    self.inspector_window.close()
                    self.inspector_window.deleteLater()
                    self.inspector_window = None
                    print("Cleaned up inspector window")
                except Exception as e:
                    print(f"Error cleaning up inspector window: {str(e)}")

            # キャッシュのクリア
            try:
                self._cached_positions.clear()
                self._selection_cache.clear()
                if hasattr(self, '_cleanup_handlers'):
                    self._cleanup_handlers.clear()
                print("Cleared caches")
            except Exception as e:
                print(f"Error clearing caches: {str(e)}")

            print("Cleanup process completed")

        except Exception as e:
            print(f"Error during cleanup: {str(e)}")

    def __del__(self):
        """デストラクタでクリーンアップを実行"""
        self.cleanup()

    def remove_node(self, node):
        """ノード削除時のメモリリーク対策"""
        # キャッシュからノード関連データを削除
        if node in self._cached_positions:
            del self._cached_positions[node]
        self._selection_cache.discard(node)

        # ポート接続の解除
        for port in node.input_ports():
            for connected_port in port.connected_ports():
                self.disconnect_ports(port, connected_port)

        for port in node.output_ports():
            for connected_port in port.connected_ports():
                self.disconnect_ports(port, connected_port)

        # STLデータのクリーンアップ
        if self.stl_viewer:
            self.stl_viewer.remove_stl_for_node(node)

        super(CustomNodeGraph, self).remove_node(node)
        self._notify_joint_structure_changed()

    def optimize_node_positions(self):
        """ノード位置の計算を最適化"""
        # 位置計算のキャッシュを活用
        for node in self.all_nodes():
            if node not in self._cached_positions:
                pos = self.calculate_node_position(node)
                self._cached_positions[node] = pos
            node.set_pos(*self._cached_positions[node])

    def setup_custom_view(self):
        """ビューのイベントハンドラをカスタマイズ"""
        # オリジナルのイベントハンドラを保存
        self._view.mousePressEvent_original = self._view.mousePressEvent
        self._view.mouseMoveEvent_original = self._view.mouseMoveEvent
        self._view.mouseReleaseEvent_original = self._view.mouseReleaseEvent

        # 新しいイベントハンドラを設定
        self._view.mousePressEvent = lambda event: self._view_mouse_press(event)
        self._view.mouseMoveEvent = lambda event: self._view_mouse_move(event)
        self._view.mouseReleaseEvent = lambda event: self._view_mouse_release(event)

    def eventFilter(self, obj, event):
        """イベントフィルターでマウスイベントを処理"""
        if obj is self._view:
            if event.type() == QtCore.QEvent.Type.MouseButtonPress:
                return self._handle_mouse_press(event)
            elif event.type() == QtCore.QEvent.Type.MouseMove:
                return self._handle_mouse_move(event)
            elif event.type() == QtCore.QEvent.Type.MouseButtonRelease:
                return self._handle_mouse_release(event)

        return super(CustomNodeGraph, self).eventFilter(obj, event)

    def _handle_mouse_press(self, event):
        """マウスプレスイベントの処理"""
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self._selection_start = event.position().toPoint()
            self._is_selecting = True

            # 選択範囲の設定
            if self._rubber_band:
                rect = QtCore.QRect(self._selection_start, QtCore.QSize())
                self._rubber_band.setGeometry(rect)
                self._rubber_band.show()

            # Ctrlキーが押されていない場合は既存の選択をクリア
            if not event.modifiers() & QtCore.Qt.KeyboardModifier.ControlModifier:
                for node in self.selected_nodes():
                    node.set_selected(False)

        return False  # イベントを伝播させる

    def _handle_mouse_move(self, event):
        """マウス移動イベントの処理"""
        if self._is_selecting and self._selection_start is not None and self._rubber_band:
            current_pos = event.position().toPoint()
            rect = QtCore.QRect(self._selection_start,
                                current_pos).normalized()
            self._rubber_band.setGeometry(rect)

        return False  # イベントを伝播させる

    def _handle_mouse_release(self, event):
        """マウスリリースイベントの処理"""
        if (event.button() == QtCore.Qt.MouseButton.LeftButton and
                self._is_selecting and self._rubber_band):
            try:
                # 選択範囲の取得
                rect = self._rubber_band.geometry()
                scene_rect = self._view.mapToScene(rect).boundingRect()

                # 範囲内のノードを選択
                for node in self.all_nodes():
                    node_pos = node.pos()
                    if isinstance(node_pos, (list, tuple)):
                        node_point = QtCore.QPointF(node_pos[0], node_pos[1])
                    else:
                        node_point = node_pos

                    if scene_rect.contains(node_point):
                        node.set_selected(True)

                # ラバーバンドを隠す
                self._rubber_band.hide()

            except Exception as e:
                print(f"Error in mouse release: {str(e)}")
            finally:
                # 状態をリセット
                self._selection_start = None
                self._is_selecting = False

        return False  # イベントを伝播させる

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            # ビューの座標系でマウス位置を取得
            view = self.scene().views()[0]
            self._selection_start = view.mapFromGlobal(event.globalPos())

            # Ctrlキーが押されていない場合は既存の選択をクリア
            if not event.modifiers() & QtCore.Qt.ControlModifier:
                for node in self.selected_nodes():
                    node.set_selected(False)

            # ラバーバンドの開始位置を設定
            self._rubber_band.setGeometry(QtCore.QRect(self._selection_start, QtCore.QSize()))
            self._rubber_band.show()

        super(CustomNodeGraph, self).mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._selection_start is not None:
            # ビューの座標系で現在位置を取得
            view = self.scene().views()[0]
            current_pos = view.mapFromGlobal(event.globalPos())

            # ラバーバンドの領域を更新
            rect = QtCore.QRect(self._selection_start, current_pos).normalized()
            self._rubber_band.setGeometry(rect)

        super(CustomNodeGraph, self).mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton and self._selection_start is not None:
            # ビューの座標系でラバーバンドの領域を取得
            view = self.scene().views()[0]
            rubber_band_rect = self._rubber_band.geometry()
            scene_rect = view.mapToScene(rubber_band_rect).boundingRect()

            # 範囲内のノードを選択
            for node in self.all_nodes():
                node_center = QtCore.QPointF(node.pos()[0], node.pos()[1])
                if scene_rect.contains(node_center):
                    node.set_selected(True)

            # ラバーバンドをクリア
            self._rubber_band.hide()
            self._selection_start = None

        super(CustomNodeGraph, self).mouseReleaseEvent(event)

    def _view_mouse_press(self, event):
        """ビューのマウスプレスイベント"""
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self._selection_start = event.position().toPoint()
            self._is_selecting = True

            # 選択範囲の設定
            if self._rubber_band:
                rect = QtCore.QRect(self._selection_start, QtCore.QSize())
                self._rubber_band.setGeometry(rect)
                self._rubber_band.show()

            # Ctrlキーが押されていない場合は既存の選択をクリア
            if not event.modifiers() & QtCore.Qt.KeyboardModifier.ControlModifier:
                for node in self.selected_nodes():
                    node.set_selected(False)

        # 元のイベントハンドラを呼び出し
        if hasattr(self._view, 'mousePressEvent_original'):
            self._view.mousePressEvent_original(event)

    def _view_mouse_move(self, event):
        """ビューのマウス移動イベント"""
        if self._is_selecting and self._selection_start is not None and self._rubber_band:
            current_pos = event.position().toPoint()
            rect = QtCore.QRect(self._selection_start,
                                current_pos).normalized()
            self._rubber_band.setGeometry(rect)

        # 元のイベントハンドラを呼び出し
        if hasattr(self._view, 'mouseMoveEvent_original'):
            self._view.mouseMoveEvent_original(event)

    def _view_mouse_release(self, event):
        """ビューのマウスリリースイベント"""
        if (event.button() == QtCore.Qt.MouseButton.LeftButton and
                self._is_selecting and self._rubber_band):
            try:
                # 選択範囲の取得
                rect = self._rubber_band.geometry()
                scene_rect = self._view.mapToScene(rect).boundingRect()

                # 範囲内のノードを選択
                for node in self.all_nodes():
                    node_pos = node.pos()
                    if isinstance(node_pos, (list, tuple)):
                        node_point = QtCore.QPointF(node_pos[0], node_pos[1])
                    else:
                        node_point = node_pos

                    if scene_rect.contains(node_point):
                        node.set_selected(True)

                # ラバーバンドを隠す
                self._rubber_band.hide()

            except Exception as e:
                print(f"Error in mouse release: {str(e)}")
            finally:
                # 状態をリセット
                self._selection_start = None
                self._is_selecting = False

        # 元のイベントハンドラを呼び出し
        if hasattr(self._view, 'mouseReleaseEvent_original'):
            self._view.mouseReleaseEvent_original(event)

    def create_base_link(self):
        """初期のbase_linkノードを作成"""
        try:
            node_type = f"{BaseLinkNode.__identifier__}.{BaseLinkNode.NODE_NAME}"
            base_node = self.create_node(node_type)
            base_node.set_name('base_link')
            base_node.set_pos(20, 20)
            print("Base Link node created successfully")
            return base_node
        except Exception as e:
            print(f"Error creating base link node: {str(e)}")
            import traceback
            traceback.print_exc()
            raise

    def register_nodes(self, node_classes):
        """複数のノードクラスを一度に登録"""
        for node_class in node_classes:
            self.register_node(node_class)
            print(f"Registered node type: {node_class.__identifier__}")

    # ------------------------------------------------------------------
    # Connection helpers
    def _output_port_index(self, node, port) -> int | None:
        try:
            outputs = list(node.output_ports())
            return outputs.index(port)
        except (ValueError, AttributeError):
            return None

    @staticmethod
    def _vector_dot(a: list[float] | None, b: list[float] | None) -> float:
        if not a or not b:
            return 0.0
        try:
            return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1]) + float(a[2]) * float(b[2])
        except (IndexError, TypeError, ValueError):
            return 0.0

    @staticmethod
    def _normalize_vector(vec: list[float] | None) -> list[float] | None:
        if not vec:
            return None
        try:
            mag = math.sqrt(sum(float(v) * float(v) for v in vec[:3]))
        except (TypeError, ValueError):
            return None
        if mag < 1e-8:
            return None
        return [float(vec[0]) / mag, float(vec[1]) / mag, float(vec[2]) / mag]

    def _label_input_port(self, node, port, label: str):
        try:
            port_view = port.view
            port_view.name = label
            port_view.display_name = True
            text_item = node.view.get_input_text_item(port_view)
            if text_item:
                text_item.setPlainText(label)
        except Exception:
            pass

    def _reset_input_port_label(self, node, port):
        if port is None:
            return
        default_label = 'in'
        self._label_input_port(node, port, default_label)

    def _prepare_child_points(self, child_node) -> list[dict]:
        points = getattr(child_node, 'points', None) or []
        if not points:
            points = [{
                'name': 'origin',
                'type': 'fixed',
                'xyz': [0.0, 0.0, 0.0],
                'axis': [1.0, 0.0, 0.0],
            }]
            child_node.points = points
        return points

    def _prompt_connection_config(
        self,
        parent_node,
        parent_port_idx: int,
        input_port,
        child_node,
    ) -> dict | None:
        child_points = self._prepare_child_points(child_node)

        parent_point = None
        parent_axis = None
        if hasattr(parent_node, 'points') and parent_port_idx is not None:
            points = parent_node.points
            if 0 <= parent_port_idx < len(points):
                parent_point = points[parent_port_idx]
                parent_axis = parent_point.get('axis')

        default_point = 0
        default_sign = 1

        previous = getattr(child_node, 'parent_joint_config', None)
        if previous:
            if previous.get('parent_node_name') == parent_node.name():
                default_point = int(previous.get('child_point_index', 0))
                default_sign = int(previous.get('axis_sign', 1))

        child_axis = None
        if child_points and 0 <= default_point < len(child_points):
            child_axis = child_points[default_point].get('axis')

        if parent_axis and child_axis:
            dot = self._vector_dot(parent_axis, child_axis)
            if dot < 0:
                default_sign = -1

        parent_point_name = parent_point.get('name') if parent_point else input_port.name()

        dialog = ConnectionConfigDialog(
            parent_node.name(),
            child_node.name(),
            parent_point_name or f"point_{parent_port_idx + 1 if parent_port_idx is not None else 1}",
            parent_axis,
            child_points,
            default_point,
            default_sign,
            parent=self.widget.window() if self.widget else None,
        )

        if dialog.exec() != QtWidgets.QDialog.Accepted:
            return None

        selected_point = dialog.selected_point_index()
        if selected_point is None:
            selected_point = default_point
        if not isinstance(selected_point, int):
            try:
                selected_point = int(selected_point)
            except Exception:
                selected_point = default_point
        if not (0 <= selected_point < len(child_points)):
            selected_point = max(0, min(len(child_points) - 1, default_point))

        config = {
            'parent_node_name': parent_node.name(),
            'parent_point_index': parent_port_idx,
            'parent_point_name': parent_point_name,
            'child_point_index': selected_point,
            'child_point_name': child_points[selected_point].get('name')
            if 0 <= selected_point < len(child_points)
            else None,
            'axis_sign': dialog.selected_sign(),
            'input_port_name': input_port.name(),
        }
        return config

    def _store_connection_config(self, parent_node, child_node, input_port, config: dict):
        to_delete = [key for key in self.connection_settings.keys() if key[1] == id(child_node)]
        for key in to_delete:
            del self.connection_settings[key]

        child_node.parent_joint_config = dict(config)
        key = (id(parent_node), id(child_node))
        self.connection_settings[key] = dict(config)

        label = config.get('child_point_name')
        if not label:
            label = f"point_{config.get('child_point_index', 0) + 1}"
        self._label_input_port(child_node, input_port, label)

    def on_port_connected(self, input_port, output_port):
        """ポートが接続された時の処理"""
        print(f"**Connecting port: {output_port.name()}")

        # 接続情報の出力
        parent_node = output_port.node()
        child_node = input_port.node()
        print(f"Parent node: {parent_node.name()}, Child node: {child_node.name()}")

        try:
            parent_port_idx = self._output_port_index(parent_node, output_port)
            config = self._prompt_connection_config(
                parent_node,
                parent_port_idx,
                input_port,
                child_node,
            )

            if config is None:
                print("Connection cancelled by user")
                QtCore.QTimer.singleShot(
                    0,
                    lambda: self.disconnect_ports(output_port, input_port)
                )
                return

            self._store_connection_config(parent_node, child_node, input_port, config)

            # 全ノードの位置を再計算
            print("Recalculating all node positions after connection...")
            self.recalculate_all_positions()
            self._notify_joint_structure_changed()

        except Exception as e:
            print(f"Error in port connection: {str(e)}")
            print(f"Detailed connection information:")
            print(f"  Output port: {output_port.name()} from {parent_node.name()}")
            print(f"  Input port: {input_port.name()} from {child_node.name()}")
            traceback.print_exc()

    def on_port_disconnected(self, input_port, output_port):
        """ポートが切断された時の処理"""
        child_node = input_port.node()  # 入力ポートを持つノードが子
        parent_node = output_port.node()  # 出力ポートを持つノードが親

        print(f"\nDisconnecting ports:")
        print(f"Parent node: {parent_node.name()}, Child node: {child_node.name()}")

        try:
            key = (id(parent_node), id(child_node))
            if key in self.connection_settings:
                del self.connection_settings[key]

            if hasattr(child_node, 'parent_joint_config'):
                delattr(child_node, 'parent_joint_config')

            if input_port:
                self._reset_input_port_label(child_node, input_port)

            # 子ノードの位置をリセット
            if hasattr(child_node, 'current_transform'):
                del child_node.current_transform

            # STLの位置をリセット
            self.stl_viewer.reset_stl_transform(child_node)
            print(f"Reset position for node: {child_node.name()}")

            # 全ノードの位置を再計算
            print("Recalculating all node positions after disconnection...")
            self.recalculate_all_positions()
            self._notify_joint_structure_changed()

        except Exception as e:
            print(f"Error in port disconnection: {str(e)}")
            traceback.print_exc()

    def update_robot_name(self, text):
        """ロボット名を更新するメソッド"""
        self.robot_name = text
        print(f"Robot name updated to: {text}")

        # 必要に応じて追加の処理
        # 例：ウィンドウタイトルの更新
        if hasattr(self, 'widget') and self.widget:
            if self.widget.window():
                title = f"URDF Kitchen - Assembler v0.0.1 - {text}"
                self.widget.window().setWindowTitle(title)

    def get_robot_name(self):
        """
        現在のロボット名を取得するメソッド
        Returns:
            str: 現在のロボット名
        """
        return self.robot_name

    def set_robot_name(self, name):
        """
        ロボット名を設定するメソッド
        Args:
            name (str): 設定するロボット名
        """
        self.robot_name = name
        # 入力フィールドが存在する場合は更新
        if hasattr(self, 'name_input') and self.name_input:
            self.name_input.setText(name)
        print(f"Robot name set to: {name}")

    def set_joint_change_callback(self, callback):
        self._joint_change_callback = callback

    def _notify_joint_structure_changed(self):
        if callable(self._joint_change_callback):
            self._joint_change_callback()

    def clean_robot_name(self, name):
        """ロボット名から_descriptionを除去"""
        if name.endswith('_description'):
            return name[:-12]  # '_description'の長さ(12)を削除
        return name

    def _resolve_mesh_dir_name(self, preferred: str | None = None) -> str:
        """Return meshes directory name for URDF references."""
        if preferred:
            return preferred
        if self.meshes_dir:
            dir_name = os.path.basename(self.meshes_dir)
            if dir_name and dir_name.lower().startswith('mesh'):
                return dir_name
        return 'meshes'

    def _format_mesh_path(
        self,
        mesh_filename: str,
        mesh_dir_name: str,
        export_mode: str,
        package_name: str | None,
    ) -> str:
        """Return URDF mesh reference path for the given export mode."""
        safe_filename = mesh_filename.replace('\\', '/')
        safe_dir = mesh_dir_name.replace('\\', '/') if mesh_dir_name else 'meshes'

        if export_mode == 'ros':
            resolved_package = package_name or f"{self.clean_robot_name(self.robot_name)}_description"
            return f"package://{resolved_package}/{safe_dir}/{safe_filename}"

        # Default to relative path for non-ROS exports
        return posixpath.join(safe_dir, safe_filename)

    @staticmethod
    def _axis_vector_to_index(axis_vec: list[float] | tuple[float, ...] | None, tol: float = 1e-4) -> int:
        """Convert an axis vector to the legacy rotation_axis index."""
        if not axis_vec:
            return 3
        if len(axis_vec) < 3:
            return 3
        x, y, z = axis_vec[:3]
        magnitude = math.sqrt(x * x + y * y + z * z)
        if magnitude < tol:
            return 3
        ux, uy, uz = x / magnitude, y / magnitude, z / magnitude
        if abs(ux - 1.0) < tol:
            return 0
        if abs(ux + 1.0) < tol:
            return 0
        if abs(uy - 1.0) < tol:
            return 1
        if abs(uy + 1.0) < tol:
            return 1
        if abs(uz - 1.0) < tol:
            return 2
        if abs(uz + 1.0) < tol:
            return 2
        return 3

    @staticmethod
    def _axis_index_to_vector(index: int) -> list[float]:
        """Return a unit vector for the legacy rotation_axis index."""
        if index == 0:
            return [1.0, 0.0, 0.0]
        if index == 1:
            return [0.0, 1.0, 0.0]
        if index == 2:
            return [0.0, 0.0, 1.0]
        return [0.0, 0.0, 0.0]

    @staticmethod
    def _parse_axis_element(axis_elem: ET.Element | None) -> list[float] | None:
        if axis_elem is None:
            return None
        axis_text = axis_elem.get('xyz') if axis_elem.get('xyz') else axis_elem.text
        if not axis_text:
            return None
        try:
            values = [float(x) for x in axis_text.replace(',', ' ').split()]
        except ValueError:
            return None
        if len(values) < 3:
            return None
        return values[:3]

    @staticmethod
    def _cleanup_axis_values(axis_value: object) -> list[float] | None:
        if axis_value is None:
            return None
        if isinstance(axis_value, (list, tuple)):
            values = list(axis_value)[:3]
        elif isinstance(axis_value, str):
            try:
                values = [float(x) for x in axis_value.replace(',', ' ').split()]
            except ValueError:
                return None
        else:
            return None
        if len(values) < 3:
            return None
        try:
            return [float(values[0]), float(values[1]), float(values[2])]
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _vector3_from_point(point_data: dict | None) -> np.ndarray:
        if not point_data:
            return np.zeros(3, dtype=float)
        xyz = point_data.get('xyz') or point_data.get('point_xyz')
        if xyz is None:
            return np.zeros(3, dtype=float)
        if isinstance(xyz, str):
            try:
                values = [float(x) for x in xyz.replace(',', ' ').split()]
            except ValueError:
                return np.zeros(3, dtype=float)
        else:
            values = list(xyz)
        if len(values) < 3:
            values += [0.0] * (3 - len(values))
        return np.array(values[:3], dtype=float)

    @staticmethod
    def _vector3_from_axis(axis_value: object | None) -> np.ndarray:
        cleaned = CustomNodeGraph._cleanup_axis_values(axis_value)
        if cleaned is None:
            return np.zeros(3, dtype=float)
        return np.array(cleaned, dtype=float)

    @staticmethod
    def _normalize_vector_np(vec: np.ndarray) -> np.ndarray:
        norm = np.linalg.norm(vec)
        if norm < 1e-9:
            return np.zeros(3, dtype=float)
        return vec / norm

    @staticmethod
    def _rotation_matrix_between(source: np.ndarray, target: np.ndarray) -> np.ndarray:
        src = CustomNodeGraph._normalize_vector_np(source)
        tgt = CustomNodeGraph._normalize_vector_np(target)
        if np.linalg.norm(src) < 1e-9 or np.linalg.norm(tgt) < 1e-9:
            return np.identity(3)
        dot_val = np.clip(np.dot(src, tgt), -1.0, 1.0)
        if dot_val > 1.0 - 1e-8:
            return np.identity(3)
        if dot_val < -1.0 + 1e-8:
            # 180° rotation: find orthogonal axis
            ortho = np.array([1.0, 0.0, 0.0])
            if abs(src[0]) > 0.9:
                ortho = np.array([0.0, 1.0, 0.0])
            axis = CustomNodeGraph._normalize_vector_np(np.cross(src, ortho))
            return CustomNodeGraph._rotation_about_axis(axis, math.pi)
        axis = CustomNodeGraph._normalize_vector_np(np.cross(src, tgt))
        angle = math.acos(dot_val)
        return CustomNodeGraph._rotation_about_axis(axis, angle)

    @staticmethod
    def _rotation_about_axis(axis: np.ndarray, angle: float) -> np.ndarray:
        axis_norm = CustomNodeGraph._normalize_vector_np(axis)
        if np.linalg.norm(axis_norm) < 1e-9 or abs(angle) < 1e-9:
            return np.identity(3)
        x, y, z = axis_norm
        c = math.cos(angle)
        s = math.sin(angle)
        C = 1.0 - c
        return np.array([
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ])

    @staticmethod
    def _matrix_to_rpy(rotation: np.ndarray) -> tuple[float, float, float]:
        sy = math.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
        singular = sy < 1e-9
        if not singular:
            roll = math.atan2(rotation[2, 1], rotation[2, 2])
            pitch = math.atan2(-rotation[2, 0], sy)
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        else:
            roll = math.atan2(-rotation[1, 2], rotation[1, 1])
            pitch = math.atan2(-rotation[2, 0], sy)
            yaw = 0.0
        return roll, pitch, yaw

    @staticmethod
    def _vtk_transform_to_matrix(transform: vtk.vtkTransform) -> np.ndarray:
        vtk_mat = transform.GetMatrix()
        return np.array([[vtk_mat.GetElement(i, j) for j in range(4)] for i in range(4)], dtype=float)

    @staticmethod
    def _matrix_to_vtk_transform(matrix: np.ndarray) -> vtk.vtkTransform:
        vtk_matrix = vtk.vtkMatrix4x4()
        for i in range(4):
            for j in range(4):
                vtk_matrix.SetElement(i, j, float(matrix[i, j]))
        tf = vtk.vtkTransform()
        tf.SetMatrix(vtk_matrix)
        return tf

    def _connection_config(self, parent_node, child_node) -> dict | None:
        key = (id(parent_node), id(child_node))
        config = self.connection_settings.get(key)
        if not config:
            config = getattr(child_node, 'parent_joint_config', None)
        if config:
            return dict(config)
        return None

    def _compute_joint_relative(self, parent_node, parent_port_idx: int, child_node, angle_rad: float) -> dict | None:
        if parent_port_idx is None:
            return None
        if parent_port_idx < 0:
            return None

        parent_points = getattr(parent_node, 'points', []) or []
        if parent_port_idx >= len(parent_points):
            return None

        parent_point = parent_points[parent_port_idx]
        parent_axis = self._vector3_from_axis(parent_point.get('axis'))
        parent_axis = self._normalize_vector_np(parent_axis)

        config = self._connection_config(parent_node, child_node) or {}
        child_index = int(config.get('child_point_index', 0))
        child_points = getattr(child_node, 'points', []) or []
        if child_index < 0 or child_index >= len(child_points):
            child_index = 0 if child_points else -1
        child_point = child_points[child_index] if 0 <= child_index < len(child_points) else None

        child_axis = self._vector3_from_axis(child_point.get('axis') if child_point else None)
        child_axis = self._normalize_vector_np(child_axis)

        if np.linalg.norm(parent_axis) < 1e-9:
            parent_axis = self._normalize_vector_np(self._axis_index_to_vector(getattr(child_node, 'rotation_axis', 3)))
        if np.linalg.norm(parent_axis) < 1e-9:
            parent_axis = np.array([1.0, 0.0, 0.0])

        if np.linalg.norm(child_axis) < 1e-9:
            child_axis = parent_axis

        axis_sign = config.get('axis_sign', 1)
        try:
            axis_sign = 1 if int(axis_sign) >= 0 else -1
        except (ValueError, TypeError):
            axis_sign = 1

        source_axis = child_axis * axis_sign
        if np.linalg.norm(source_axis) < 1e-9:
            source_axis = child_axis

        align_rotation = self._rotation_matrix_between(source_axis, parent_axis)
        joint_rotation = self._rotation_about_axis(parent_axis, angle_rad)

        total_rotation = joint_rotation @ align_rotation

        parent_point_vec = self._vector3_from_point(parent_point)
        child_point_vec = self._vector3_from_point(child_point)

        translation = parent_point_vec - total_rotation @ child_point_vec

        relative_matrix = np.identity(4)
        relative_matrix[:3, :3] = total_rotation
        relative_matrix[:3, 3] = translation

        zero_rotation = align_rotation
        zero_translation = parent_point_vec - zero_rotation @ child_point_vec

        zero_matrix = np.identity(4)
        zero_matrix[:3, :3] = zero_rotation
        zero_matrix[:3, 3] = zero_translation

        return {
            'relative_matrix': relative_matrix,
            'zero_matrix': zero_matrix,
            'parent_axis': parent_axis,
            'joint_axis_joint_frame': source_axis,
            'child_point_index': child_index,
            'child_point': child_point,
            'parent_point': parent_point,
        }

    def update_robot_name_from_directory(self, dir_path):
        """ディレクトリ名からロボット名を更新"""
        dir_name = os.path.basename(dir_path)
        if dir_name.endswith('_description'):
            robot_name = dir_name[:-12]
            # UI更新
            if hasattr(self, 'name_input') and self.name_input:
                self.name_input.setText(robot_name)
            self.robot_name = robot_name
            return True
        return False

    def build_urdf_text(
        self,
        mesh_dir_name: str | None = None,
        export_mode: str = 'ros',
        package_name: str | None = None,
        robot_name: str | None = None,
    ) -> str:
        """Generate URDF output for the current graph without writing to disk."""
        base_node = self.get_node_by_name('base_link')
        if base_node is None:
            raise ValueError("Base link node is missing; cannot build URDF")

        active_robot_name = robot_name or self.robot_name
        clean_name = self.clean_robot_name(active_robot_name)
        buffer = io.StringIO()
        buffer.write('<?xml version="1.0"?>\n')
        buffer.write(f'<robot name="{clean_name}">\n\n')

        # Collect material definitions
        materials: dict[str, list[float]] = {}
        for node in self.all_nodes():
            if hasattr(node, 'node_color'):
                rgb = node.node_color
                if isinstance(rgb, (list, tuple)) and len(rgb) >= 3:
                    hex_color = '#{:02x}{:02x}{:02x}'.format(
                        int(max(0.0, min(1.0, rgb[0])) * 255),
                        int(max(0.0, min(1.0, rgb[1])) * 255),
                        int(max(0.0, min(1.0, rgb[2])) * 255)
                    )
                    materials[hex_color] = [float(rgb[0]), float(rgb[1]), float(rgb[2])]

        if materials:
            buffer.write('<!-- material color setting -->\n')
            for hex_color, rgb in materials.items():
                buffer.write(f'<material name="{hex_color}">\n')
                buffer.write(f'  <color rgba="{rgb[0]:.3f} {rgb[1]:.3f} {rgb[2]:.3f} 1.0"/>\n')
                buffer.write('</material>\n')
            buffer.write('\n')

        visited_nodes: set = set()
        resolved_mesh_dir = self._resolve_mesh_dir_name(mesh_dir_name)
        resolved_package = package_name if export_mode != 'ros' else package_name or f"{clean_name}_description"
        self._write_tree_structure(
            buffer,
            base_node,
            None,
            visited_nodes,
            materials,
            resolved_mesh_dir,
            export_mode,
            resolved_package,
        )

        buffer.write('</robot>\n')
        return buffer.getvalue()

    def export_urdf(self):
        """Export URDF with user-selected target format."""
        try:
            if len(self.all_nodes()) <= 1:
                QtWidgets.QMessageBox.warning(
                    self.widget,
                    "Export URDF",
                    "请先添加至少一个部件并建立连接后再导出 URDF。"
                )
                return False

            export_mode = self._prompt_export_mode()
            if export_mode is None:
                print("URDF export cancelled")
                return False

            if export_mode == 'ros':
                return self._export_urdf_ros()

            return self._export_urdf_regular()

        except Exception as e:
            error_msg = f"Error exporting URDF: {str(e)}"
            print(error_msg)
            traceback.print_exc()

            QtWidgets.QMessageBox.critical(
                self.widget,
                "Export Error",
                error_msg
            )
            return False

    def _prompt_export_mode(self) -> str | None:
        """Show a dialog asking the user which export mode to use."""
        dialog = QtWidgets.QMessageBox(self.widget)
        dialog.setIcon(QtWidgets.QMessageBox.Question)
        dialog.setWindowTitle("Export URDF")
        dialog.setText("请选择 URDF 的导出类型：")

        ros_button = dialog.addButton("ROS 包结构", QtWidgets.QMessageBox.YesRole)
        regular_button = dialog.addButton("常规 URDF 打包", QtWidgets.QMessageBox.NoRole)
        dialog.addButton(QtWidgets.QMessageBox.Cancel)

        dialog.exec_()
        clicked = dialog.clickedButton()

        if clicked == ros_button:
            return 'ros'
        if clicked == regular_button:
            return 'regular'
        return None

    def _export_urdf_ros(self) -> bool:
        """Export URDF in ROS package layout (xxx_description)."""
        default_dir = self.last_export_dir or self.last_save_dir or os.getcwd()
        suggested_name = self.clean_robot_name(self.robot_name) or 'robot'
        default_path = os.path.join(default_dir, f"{suggested_name}.urdf")

        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self.widget,
            "保存 URDF (ROS 包)",
            default_path,
            "URDF Files (*.urdf)"
        )

        if not file_path:
            print("ROS export cancelled")
            return False

        base_dir = os.path.dirname(file_path)
        file_stem = os.path.splitext(os.path.basename(file_path))[0]
        package_dir_name = f"{file_stem}_description"
        package_dir = os.path.join(base_dir, package_dir_name)
        urdf_dir = os.path.join(package_dir, 'urdf')
        mesh_dir_name = self._resolve_mesh_dir_name()
        mesh_dir = os.path.join(package_dir, mesh_dir_name)

        os.makedirs(urdf_dir, exist_ok=True)
        os.makedirs(mesh_dir, exist_ok=True)

        copied_meshes = set()
        for node in self.all_nodes():
            if hasattr(node, 'stl_file') and node.stl_file:
                stl_path = node.stl_file
                if not stl_path or not os.path.exists(stl_path):
                    continue
                stl_name = os.path.basename(stl_path)
                if stl_name in copied_meshes:
                    continue
                dest_path = os.path.join(mesh_dir, stl_name)
                try:
                    copy_without_metadata(stl_path, dest_path)
                    copied_meshes.add(stl_name)
                    print(f"Copied mesh without metadata: {stl_name}")
                except Exception as exc:
                    print(f"Failed to copy mesh {stl_path}: {exc}")

        package_name = package_dir_name
        urdf_text = self.build_urdf_text(
            mesh_dir_name=mesh_dir_name,
            export_mode='ros',
            package_name=package_name,
            robot_name=file_stem,
        )

        urdf_file = os.path.join(urdf_dir, f"{file_stem}.urdf")
        with open(urdf_file, 'w', encoding='utf-8') as handle:
            handle.write(urdf_text)

        self.last_export_dir = base_dir
        self.set_robot_name(file_stem)

        print(f"URDF exported to: {urdf_file}")
        QtWidgets.QMessageBox.information(
            self.widget,
            "Export Complete",
            f"ROS 包结构导出完成：\n{urdf_file}"
        )
        return True

    def _export_urdf_regular(self) -> bool:
        """Export URDF as a zipped package for general usage."""
        default_dir = self.last_export_dir or self.last_save_dir or os.getcwd()
        suggested_name = self.clean_robot_name(self.robot_name) or 'robot'
        default_path = os.path.join(default_dir, f"{suggested_name}.zip")

        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self.widget,
            "保存 URDF 打包文件",
            default_path,
            "URDF Package (*.zip);;URDF Files (*.urdf)"
        )

        if not file_path:
            print("Regular export cancelled")
            return False

        if file_path.lower().endswith('.urdf'):
            archive_path = os.path.splitext(file_path)[0] + '.zip'
        elif file_path.lower().endswith('.zip'):
            archive_path = file_path
        else:
            archive_path = f"{file_path}.zip"

        base_dir = os.path.dirname(archive_path)
        file_stem = os.path.splitext(os.path.basename(archive_path))[0]

        with tempfile.TemporaryDirectory() as tmpdir:
            mesh_dir_name = 'meshes'
            mesh_dir = os.path.join(tmpdir, mesh_dir_name)
            os.makedirs(mesh_dir, exist_ok=True)

            copied_meshes = set()
            for node in self.all_nodes():
                if hasattr(node, 'stl_file') and node.stl_file:
                    stl_path = node.stl_file
                    if not stl_path or not os.path.exists(stl_path):
                        continue
                    stl_name = os.path.basename(stl_path)
                    if stl_name in copied_meshes:
                        continue
                    dest_path = os.path.join(mesh_dir, stl_name)
                    try:
                        copy_without_metadata(stl_path, dest_path)
                        copied_meshes.add(stl_name)
                        print(f"Copied mesh without metadata: {stl_name}")
                    except Exception as exc:
                        print(f"Failed to copy mesh {stl_path}: {exc}")

            urdf_text = self.build_urdf_text(
                mesh_dir_name=mesh_dir_name,
                export_mode='regular',
                robot_name=file_stem,
            )

            urdf_temp_path = os.path.join(tmpdir, f"{file_stem}.urdf")
            with open(urdf_temp_path, 'w', encoding='utf-8') as handle:
                handle.write(urdf_text)

            with zipfile.ZipFile(archive_path, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
                archive.write(urdf_temp_path, arcname=os.path.basename(urdf_temp_path))
                for root, _dirs, files in os.walk(mesh_dir):
                    for filename in files:
                        abs_path = os.path.join(root, filename)
                        rel_path = os.path.relpath(abs_path, tmpdir).replace(os.sep, '/')
                        archive.write(abs_path, arcname=rel_path)

        self.last_export_dir = base_dir
        self.set_robot_name(file_stem)

        print(f"URDF package exported to: {archive_path}")
        QtWidgets.QMessageBox.information(
            self.widget,
            "Export Complete",
            f"常规 URDF 打包完成：\n{archive_path}"
        )
        return True

    def preview_urdf(self):
        """Display a modal dialog with the generated URDF text."""
        if len(self.all_nodes()) <= 1:
            QtWidgets.QMessageBox.information(
                self.widget,
                "URDF Preview",
                "请先添加至少一个部件并建立连接后再预览 URDF。"
            )
            return

        try:
            urdf_text = self.build_urdf_text(export_mode='ros')
        except ValueError as exc:
            QtWidgets.QMessageBox.warning(
                self.widget,
                "预览失败",
                str(exc)
            )
            return

        dialog = QtWidgets.QDialog(self.widget)
        dialog.setWindowTitle(f"URDF Preview - {self.clean_robot_name(self.robot_name)}")
        dialog.resize(640, 480)

        layout = QtWidgets.QVBoxLayout(dialog)
        info_label = QtWidgets.QLabel("当前图中的 URDF 文本预览，可复制后用于快速检查。")
        layout.addWidget(info_label)

        text_edit = QtWidgets.QPlainTextEdit()
        text_edit.setPlainText(urdf_text)
        text_edit.setReadOnly(True)
        layout.addWidget(text_edit, 1)

        button_layout = QtWidgets.QHBoxLayout()
        copy_btn = QtWidgets.QPushButton("复制到剪贴板")
        close_btn = QtWidgets.QPushButton("关闭")
        button_layout.addStretch()
        button_layout.addWidget(copy_btn)
        button_layout.addWidget(close_btn)
        layout.addLayout(button_layout)

        def copy_to_clipboard():
            QtWidgets.QApplication.clipboard().setText(text_edit.toPlainText())

        copy_btn.clicked.connect(copy_to_clipboard)
        close_btn.clicked.connect(dialog.accept)

        dialog.exec_()

    def _write_tree_structure(
        self,
        file,
        node,
        parent_node,
        visited_nodes,
        materials,
        mesh_dir_name,
        export_mode,
        package_name,
    ):
        """ツリー構造を順番に出力"""
        if node in visited_nodes:
            return
        visited_nodes.add(node)

        # Massless Decorationノードはスキップ（親ノードの<visual>として処理済み）
        if hasattr(node, 'massless_decoration') and node.massless_decoration:
            return

        if node.name() == "base_link":
            # base_linkの出力
            self._write_base_link(file)

        # 現在のノードに接続されているジョイントとリンクを処理
        for port_idx, port in enumerate(node.output_ports()):
            for connected_port in port.connected_ports():
                child_node = connected_port.node()
                if child_node not in visited_nodes:
                    # Massless Decorationでないノードのみジョイントとリンクを出力
                    if not (hasattr(child_node, 'massless_decoration') and child_node.massless_decoration):
                        # まずジョイントを出力
                        self._write_joint(file, node, child_node, port_idx)
                        file.write('\n')

                        # 次にリンクを出力
                        self._write_link(
                            file,
                            child_node,
                            materials,
                            mesh_dir_name,
                            export_mode,
                            package_name,
                        )
                        file.write('\n')

                    # 再帰的に子ノードを処理
                    self._write_tree_structure(
                        file,
                        child_node,
                        node,
                        visited_nodes,
                        materials,
                        mesh_dir_name,
                        export_mode,
                        package_name,
                    )

    def _write_base_link(self, file):
        """base_linkの出力"""
        file.write('  <link name="base_link">\n')
        file.write('    <inertial>\n')
        file.write('      <origin xyz="0 0 0" rpy="0 0 0"/>\n')
        file.write('      <mass value="0.0"/>\n')
        file.write('      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>\n')
        file.write('    </inertial>\n')
        file.write('  </link>\n\n')

    def _write_urdf_node(self, file, node, parent_node, visited_nodes, materials):
        """再帰的にノードをURDFとして書き出し"""
        if node in visited_nodes:
            return
        visited_nodes.add(node)

        # Massless Decorationフラグのチェック
        is_decoration = hasattr(node, 'massless_decoration') and node.massless_decoration

        if is_decoration:
            # 親ノードに装飾ビジュアルを追加
            if parent_node is not None:
                mesh_dir_name = self._resolve_mesh_dir_name()

                stl_filename = os.path.basename(node.stl_file)
                package_path = self._format_mesh_path(
                    stl_filename,
                    mesh_dir_name,
                    export_mode='ros',
                    package_name=None,
                )

                # 親ノードにビジュアル要素を追加
                file.write(f'''
                <visual>
                    <origin xyz="{node.xyz[0]} {node.xyz[1]} {node.xyz[2]}" rpy="{node.rpy[0]} {node.rpy[1]} {node.rpy[2]}" />
                    <geometry>
                        <mesh filename="{package_path}" />
                    </geometry>
                    <material name="#2e2e2e" />
                </visual>
                ''')
            return  # 装飾要素はこれ以上処理しない

        # 通常ノードの処理
        file.write(f'  <link name="{node.name()}">\n')

        # 慣性パラメータ
        if hasattr(node, 'mass_value') and hasattr(node, 'inertia'):
            file.write('    <inertial>\n')
            file.write(f'      <origin xyz="0 0 0" rpy="0 0 0"/>\n')
            file.write(f'      <mass value="{node.mass_value:.6f}"/>\n')
            file.write('      <inertia')
            for key, value in node.inertia.items():
                file.write(f' {key}="{value:.6f}"')
            file.write('/>\n')
            file.write('    </inertial>\n')

        # ビジュアルとコリジョン
        if hasattr(node, 'stl_file') and node.stl_file:
            mesh_dir_name = self._resolve_mesh_dir_name()

            stl_filename = os.path.basename(node.stl_file)
            package_path = self._format_mesh_path(
                stl_filename,
                mesh_dir_name,
                export_mode='ros',
                package_name=None,
            )

            # メインのビジュアル
            file.write('    <visual>\n')
            file.write(f'      <origin xyz="0 0 0" rpy="0 0 0"/>\n')
            file.write('      <geometry>\n')
            file.write(f'        <mesh filename="{package_path}"/>\n')
            file.write('      </geometry>\n')
            file.write('    </visual>\n')

            # コリジョン
            file.write('    <collision>\n')
            file.write(f'      <origin xyz="0 0 0" rpy="0 0 0"/>\n')
            file.write('      <geometry>\n')
            file.write(f'        <mesh filename="{package_path}"/>\n')
            file.write('      </geometry>\n')
            file.write('    </collision>\n')

        file.write('  </link>\n\n')

        # ジョイントの書き出し
        if parent_node and not is_decoration:
            origin_xyz = [0, 0, 0]  # デフォルト値
            for port in parent_node.output_ports():
                for connected_port in port.connected_ports():
                    if connected_port.node() == node:
                        origin_xyz = port.get_position()  # ポートの位置を取得
                        break

            joint_name = f"{parent_node.name()}_to_{node.name()}"
            file.write(f'  <joint name="{joint_name}" type="fixed">\n')
            file.write(f'    <origin xyz="{origin_xyz[0]} {origin_xyz[1]} {origin_xyz[2]}" rpy="0.0 0.0 0.0"/>\n')
            file.write(f'    <parent link="{parent_node.name()}"/>\n')
            file.write(f'    <child link="{node.name()}"/>\n')
            file.write('  </joint>\n\n')

        # 子ノードの処理
        for port in node.output_ports():
            for connected_port in port.connected_ports():
                child_node = connected_port.node()
                self._write_urdf_node(file, child_node, node, visited_nodes, materials)

    def generate_tree_text(self, node, level=0):
        tree_text = "  " * level + node.name() + "\n"
        for output_port in node.output_ports():
            for connected_port in output_port.connected_ports():
                child_node = connected_port.node()
                tree_text += self.generate_tree_text(child_node, level + 1)
        return tree_text

    def get_node_by_name(self, name):
        for node in self.all_nodes():
            if node.name() == name:
                return node
        return None

    def update_last_stl_directory(self, file_path):
        self.last_stl_directory = os.path.dirname(file_path)

    def show_inspector(self, node, screen_pos=None):
        """
        ノードのインスペクタウィンドウを表示
        """
        try:
            # 既存のインスペクタウィンドウをクリーンアップ
            if hasattr(self, 'inspector_window') and self.inspector_window is not None:
                try:
                    self.inspector_window.close()
                    self.inspector_window.deleteLater()
                except:
                    pass
                self.inspector_window = None

            # 新しいインスペクタウィンドウを作成
            self.inspector_window = InspectorWindow(stl_viewer=self.stl_viewer)

            # ウィンドウサイズを取得
            inspector_size = self.inspector_window.sizeHint()

            if self.widget and self.widget.window():
                # 保存された位置があればそれを使用し、なければデフォルト位置を計算
                if hasattr(self, 'last_inspector_position') and self.last_inspector_position:
                    x = self.last_inspector_position.x()
                    y = self.last_inspector_position.y()

                    # スクリーンの情報を取得して位置を検証
                    screen = QtWidgets.QApplication.primaryScreen()
                    screen_geo = screen.availableGeometry()

                    # 画面外にはみ出していないか確認
                    if x < screen_geo.x() or x + inspector_size.width() > screen_geo.right() or \
                    y < screen_geo.y() or y + inspector_size.height() > screen_geo.bottom():
                        # 画面外の場合はデフォルト位置を使用
                        main_geo = self.widget.window().geometry()
                        x = main_geo.x() + (main_geo.width() - inspector_size.width()) // 2
                        y = main_geo.y() + 50
                else:
                    # デフォルトの位置を計算
                    main_geo = self.widget.window().geometry()
                    x = main_geo.x() + (main_geo.width() - inspector_size.width()) // 2
                    y = main_geo.y() + 50

                # ウィンドウの初期設定と表示
                self.inspector_window.setWindowTitle(f"Node Inspector - {node.name()}")
                self.inspector_window.current_node = node
                self.inspector_window.graph = self
                self.inspector_window.update_info(node)

                self.inspector_window.move(x, y)
                self.inspector_window.show()
                self.inspector_window.raise_()
                self.inspector_window.activateWindow()

                print(f"Inspector window displayed for node: {node.name()}")

        except Exception as e:
            print(f"Error showing inspector: {str(e)}")
            traceback.print_exc()

    def remove_node(self, node):
        self.stl_viewer.remove_stl_for_node(node)
        super(CustomNodeGraph, self).remove_node(node)
        self._notify_joint_structure_changed()

    def create_node(self, node_type, name=None, pos=None):
        new_node = super(CustomNodeGraph, self).create_node(node_type, name)

        if pos is None:
            pos = QPointF(0, 0)
        elif isinstance(pos, (tuple, list)):
            pos = QPointF(*pos)

        print(f"Initial position for new node: {pos}")  # デバッグ情報

        adjusted_pos = self.find_non_overlapping_position(pos)
        print(f"Adjusted position for new node: {adjusted_pos}")  # デバッグ情報

        new_node.set_pos(adjusted_pos.x(), adjusted_pos.y())

        return new_node

    def find_non_overlapping_position(self, pos, offset_x=50, offset_y=30, items_per_row=16):
        all_nodes = self.all_nodes()
        current_node_count = len(all_nodes)

        # 現在の行を計算
        row = current_node_count // items_per_row

        # 行内での位置を計算
        position_in_row = current_node_count % items_per_row

        # 基準となるX座標を計算（各行の開始X座標）
        base_x = pos.x()

        # 基準となるY座標を計算
        # 新しい行は前の行の開始位置から200ポイント下
        base_y = pos.y() + (row * 200)

        # 現在のノードのX,Y座標を計算
        new_x = base_x + (position_in_row * offset_x)
        new_y = base_y + (position_in_row * offset_y)

        new_pos = QPointF(new_x, new_y)

        print(f"Positioning node {current_node_count + 1}")
        print(f"Row: {row + 1}, Position in row: {position_in_row + 1}")
        print(f"Position: ({new_pos.x()}, {new_pos.y()})")

        # オーバーラップチェックと位置の微調整
        iteration = 0
        while any(self.nodes_overlap(new_pos, node.pos()) for node in all_nodes):
            new_pos += QPointF(5, 5)  # 微小なオフセットで調整
            iteration += 1
            if iteration > 10:
                break

        return new_pos

    def nodes_overlap(self, pos1, pos2, threshold=5):
        pos1 = self.ensure_qpointf(pos1)
        pos2 = self.ensure_qpointf(pos2)
        overlap = (abs(pos1.x() - pos2.x()) < threshold and
                abs(pos1.y() - pos2.y()) < threshold)
        # デバッグ出力を条件付きに
        if overlap:
            print(f"Overlap detected: pos1={pos1}, pos2={pos2}")
        return overlap

    def ensure_qpointf(self, pos):
        if isinstance(pos, QPointF):
            return pos
        elif isinstance(pos, (tuple, list)):
            return QPointF(*pos)
        else:
            print(f"Warning: Unsupported position type: {type(pos)}")  # デバッグ情報
            return QPointF(0, 0)  # デフォルト値を返す

    def _save_node_data(self, node, project_dir):
        """ノードデータの保存"""
        print(f"\nStarting _save_node_data for node: {node.name()}")
        node_elem = ET.Element("node")

        try:
            # 基本情報
            print(f"  Saving basic info for node: {node.name()}")
            ET.SubElement(node_elem, "id").text = hex(id(node))
            ET.SubElement(node_elem, "name").text = node.name()
            ET.SubElement(node_elem, "type").text = node.__class__.__name__

            # output_count の保存
            if hasattr(node, 'output_count'):
                ET.SubElement(node_elem, "output_count").text = str(node.output_count)
                print(f"  Saved output_count: {node.output_count}")

            # STLファイル情報
            if hasattr(node, 'stl_file') and node.stl_file:
                print(f"  Processing STL file for node {node.name()}: {node.stl_file}")
                stl_elem = ET.SubElement(node_elem, "stl_file")

                try:
                    stl_path = os.path.abspath(node.stl_file)
                    print(f"    Absolute STL path: {stl_path}")

                    if self.meshes_dir and stl_path.startswith(self.meshes_dir):
                        rel_path = os.path.relpath(stl_path, self.meshes_dir)
                        stl_elem.set('base_dir', 'meshes')
                        stl_elem.text = os.path.join('meshes', rel_path)
                        print(f"    Using meshes relative path: {rel_path}")
                    else:
                        rel_path = os.path.relpath(stl_path, project_dir)
                        stl_elem.set('base_dir', 'project')
                        stl_elem.text = rel_path
                        print(f"    Using project relative path: {rel_path}")

                except Exception as e:
                    print(f"    Error processing STL file: {str(e)}")
                    stl_elem.set('error', str(e))

            # 位置情報
            pos = node.pos()
            pos_elem = ET.SubElement(node_elem, "position")
            if isinstance(pos, (list, tuple)):
                ET.SubElement(pos_elem, "x").text = str(pos[0])
                ET.SubElement(pos_elem, "y").text = str(pos[1])
            else:
                ET.SubElement(pos_elem, "x").text = str(pos.x())
                ET.SubElement(pos_elem, "y").text = str(pos.y())

            # 物理プロパティ
            if hasattr(node, 'volume_value'):
                ET.SubElement(node_elem, "volume").text = str(node.volume_value)
                print(f"  Saved volume: {node.volume_value}")

            if hasattr(node, 'mass_value'):
                ET.SubElement(node_elem, "mass").text = str(node.mass_value)
                print(f"  Saved mass: {node.mass_value}")

            # 慣性テンソル
            if hasattr(node, 'inertia'):
                inertia_elem = ET.SubElement(node_elem, "inertia")
                for key, value in node.inertia.items():
                    inertia_elem.set(key, str(value))
                print("  Saved inertia tensor")

            # 色情報
            if hasattr(node, 'node_color'):
                color_elem = ET.SubElement(node_elem, "color")
                color_elem.text = ' '.join(map(str, node.node_color))
                print(f"  Saved color: {node.node_color}")

            # 回転軸
            if hasattr(node, 'rotation_axis'):
                ET.SubElement(node_elem, "rotation_axis").text = str(node.rotation_axis)
                print(f"  Saved rotation axis: {node.rotation_axis}")

            if hasattr(node, 'joint_limit_lower') and hasattr(node, 'joint_limit_upper'):
                limits_elem = ET.SubElement(node_elem, "joint_limits")
                limits_elem.set('lower', str(node.joint_limit_lower))
                limits_elem.set('upper', str(node.joint_limit_upper))
                limits_elem.set('position', str(getattr(node, 'joint_position', 0.0)))

            # Massless Decoration
            if hasattr(node, 'massless_decoration'):
                ET.SubElement(node_elem, "massless_decoration").text = str(node.massless_decoration)
                print(f"  Saved massless_decoration: {node.massless_decoration}")

            # ポイントデータ
            if hasattr(node, 'points'):
                points_elem = ET.SubElement(node_elem, "points")
                for i, point in enumerate(node.points):
                    point_elem = ET.SubElement(points_elem, "point")
                    point_elem.set('index', str(i))
                    ET.SubElement(point_elem, "name").text = point['name']
                    ET.SubElement(point_elem, "type").text = point['type']
                    ET.SubElement(point_elem, "xyz").text = ' '.join(map(str, point['xyz']))
                    axis_vals = point.get('axis')
                    if axis_vals is not None:
                        axis_elem = ET.SubElement(point_elem, "axis")
                        axis_elem.set('xyz', ' '.join(f"{val:.6f}" for val in axis_vals[:3]))
                print(f"  Saved {len(node.points)} points")

            # 累積座標
            if hasattr(node, 'cumulative_coords'):
                coords_elem = ET.SubElement(node_elem, "cumulative_coords")
                for coord in node.cumulative_coords:
                    coord_elem = ET.SubElement(coords_elem, "coord")
                    ET.SubElement(coord_elem, "point_index").text = str(coord['point_index'])
                    ET.SubElement(coord_elem, "xyz").text = ' '.join(map(str, coord['xyz']))
                print(f"  Saved cumulative coordinates")

            print(f"  Completed saving node data for: {node.name()}")
            return node_elem

        except Exception as e:
            print(f"ERROR in _save_node_data for node {node.name()}: {str(e)}")
            traceback.print_exc()
            raise

    def save_project(self, file_path=None):
        """プロジェクトの保存（循環参照対策版）"""
        print("\n=== Starting Project Save ===")
        try:
            # STLビューアの状態を一時的にバックアップ
            stl_viewer_state = None
            if hasattr(self, 'stl_viewer'):
                print("Backing up STL viewer state...")
                stl_viewer_state = {
                    'actors': dict(self.stl_viewer.stl_actors),
                    'transforms': dict(self.stl_viewer.transforms)
                }
                # STLビューアの参照を一時的にクリア
                self.stl_viewer.stl_actors.clear()
                self.stl_viewer.transforms.clear()

            # ファイルパスの取得
            if not file_path:
                default_filename = f"urdf_pj_{datetime.datetime.now().strftime('%Y%m%d%H%M')}.xml"
                default_dir = self.last_save_dir or self.meshes_dir or os.getcwd()
                file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
                    None,
                    "Save Project",
                    os.path.join(default_dir, default_filename),
                    "XML Files (*.xml)"
                )
                if not file_path:
                    print("Save cancelled by user")
                    return False

            self.project_dir = os.path.dirname(os.path.abspath(file_path))
            self.last_save_dir = self.project_dir
            print(f"Project will be saved to: {file_path}")

            # XMLツリーの作成
            print("Creating XML structure...")
            root = ET.Element("project")

            # ロボット名の保存
            robot_name_elem = ET.SubElement(root, "robot_name")
            robot_name_elem.text = self.robot_name
            print(f"Saving robot name: {self.robot_name}")

            if self.meshes_dir:
                try:
                    meshes_rel_path = os.path.relpath(self.meshes_dir, self.project_dir)
                    ET.SubElement(root, "meshes_directory").text = meshes_rel_path
                    print(f"Added meshes directory reference: {meshes_rel_path}")
                except ValueError:
                    ET.SubElement(root, "meshes_directory").text = self.meshes_dir
                    print(f"Added absolute meshes path: {self.meshes_dir}")

            # ノード情報の保存
            print("\nSaving nodes...")
            nodes_elem = ET.SubElement(root, "nodes")
            total_nodes = len(self.all_nodes())

            for i, node in enumerate(self.all_nodes(), 1):
                print(f"Processing node {i}/{total_nodes}: {node.name()}")
                # 一時的にSTLビューアの参照を削除
                stl_viewer_backup = node.stl_viewer if hasattr(node, 'stl_viewer') else None
                if hasattr(node, 'stl_viewer'):
                    delattr(node, 'stl_viewer')

                node_elem = self._save_node_data(node, self.project_dir)
                nodes_elem.append(node_elem)

                # STLビューアの参照を復元
                if stl_viewer_backup is not None:
                    node.stl_viewer = stl_viewer_backup

            # 接続情報の保存
            print("\nSaving connections...")
            connections = ET.SubElement(root, "connections")
            connection_count = 0

            for node in self.all_nodes():
                for port in node.output_ports():
                    for connected_port in port.connected_ports():
                        conn = ET.SubElement(connections, "connection")
                        ET.SubElement(conn, "from_node").text = node.name()
                        ET.SubElement(conn, "from_port").text = port.name()
                        ET.SubElement(conn, "to_node").text = connected_port.node().name()
                        ET.SubElement(conn, "to_port").text = connected_port.name()
                        connection_count += 1
                        print(f"Added connection: {node.name()}.{port.name()} -> "
                            f"{connected_port.node().name()}.{connected_port.name()}")

            print(f"Total connections saved: {connection_count}")

            # ファイルの保存
            print("\nWriting to file...")
            tree = ET.ElementTree(root)
            tree.write(file_path, encoding='utf-8', xml_declaration=True)

            # STLビューアの状態を復元
            if stl_viewer_state and hasattr(self, 'stl_viewer'):
                print("Restoring STL viewer state...")
                self.stl_viewer.stl_actors = stl_viewer_state['actors']
                self.stl_viewer.transforms = stl_viewer_state['transforms']
                self.stl_viewer.vtkWidget.GetRenderWindow().Render()

            print(f"\nProject successfully saved to: {file_path}")

            QtWidgets.QMessageBox.information(
                None,
                "Save Complete",
                f"Project saved successfully to:\n{file_path}"
            )

            return True

        except Exception as e:
            error_msg = f"Error saving project: {str(e)}"
            print(f"\nERROR: {error_msg}")
            print("Traceback:")
            traceback.print_exc()

            # エラー時もSTLビューアの状態を復元
            if 'stl_viewer_state' in locals() and stl_viewer_state and hasattr(self, 'stl_viewer'):
                print("Restoring STL viewer state after error...")
                self.stl_viewer.stl_actors = stl_viewer_state['actors']
                self.stl_viewer.transforms = stl_viewer_state['transforms']
                self.stl_viewer.vtkWidget.GetRenderWindow().Render()

            QtWidgets.QMessageBox.critical(
                None,
                "Save Error",
                error_msg
            )
            return False

    def _restore_stl_viewer_state(self, backup):
        """STLビューアの状態を復元"""
        if not backup or not hasattr(self, 'stl_viewer'):
            return

        print("Restoring STL viewer state...")
        try:
            self.stl_viewer.stl_actors = dict(backup['actors'])
            self.stl_viewer.transforms = dict(backup['transforms'])
            print("STL viewer state restored successfully")
        except Exception as e:
            print(f"Error restoring STL viewer state: {e}")

    def detect_meshes_directory(self):
        """meshesディレクトリの検出"""
        for node in self.all_nodes():
            if hasattr(node, 'stl_file') and node.stl_file:
                current_dir = os.path.dirname(os.path.abspath(node.stl_file))
                while current_dir and os.path.basename(current_dir).lower() != 'meshes':
                    current_dir = os.path.dirname(current_dir)
                if current_dir and os.path.basename(current_dir).lower() == 'meshes':
                    self.meshes_dir = current_dir
                    print(f"Found meshes directory: {self.meshes_dir}")
                    return

    def load_project(self, file_path=None):
        """プロジェクトの読み込み（コンソール出力版）"""
        print("\n=== Starting Project Load ===")
        try:
            if not file_path:
                file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
                    None,
                    "Load Project",
                    self.last_save_dir or "",
                    "XML Files (*.xml)"
                )

            if not file_path:
                print("Load cancelled by user")
                return False

            print(f"Loading project from: {file_path}")

            self.project_dir = os.path.dirname(os.path.abspath(file_path))
            self.last_save_dir = self.project_dir

            # XMLファイルの解析
            print("Parsing XML file...")
            tree = ET.parse(file_path)
            root = tree.getroot()

            # ロボット名の読み込み
            robot_name_elem = root.find("robot_name")
            if robot_name_elem is not None and robot_name_elem.text:
                self.robot_name = robot_name_elem.text
                # UI上の名前入力フィールドを更新
                if hasattr(self, 'name_input') and self.name_input:
                    self.name_input.setText(self.robot_name)
                print(f"Loaded robot name: {self.robot_name}")
            else:
                print("No robot name found in project file")

            # 既存のノードをクリア
            print("Clearing existing nodes...")
            self.clear_graph()

            # meshesディレクトリの解決
            print("Resolving meshes directory...")
            meshes_dir_elem = root.find("meshes_directory")
            if meshes_dir_elem is not None and meshes_dir_elem.text:
                meshes_path = os.path.normpath(os.path.join(self.project_dir, meshes_dir_elem.text))
                if os.path.exists(meshes_path):
                    self.meshes_dir = meshes_path
                    print(f"Found meshes directory: {meshes_path}")
                else:
                    response = QtWidgets.QMessageBox.question(
                        None,
                        "Meshes Directory Not Found",
                        "The original meshes directory was not found. Would you like to select it?",
                        QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No
                    )

                    if response == QtWidgets.QMessageBox.Yes:
                        self.meshes_dir = QtWidgets.QFileDialog.getExistingDirectory(
                            None,
                            "Select Meshes Directory",
                            self.project_dir
                        )
                        if self.meshes_dir:
                            print(f"Selected new meshes directory: {self.meshes_dir}")
                        else:
                            print("Meshes directory selection cancelled")

            # ノードの復元
            print("\nRestoring nodes...")
            nodes_elem = root.find("nodes")
            total_nodes = len(nodes_elem.findall("node"))
            nodes_dict = {}

            for i, node_elem in enumerate(nodes_elem.findall("node"), 1):
                print(f"Processing node {i}/{total_nodes}")
                node = self._load_node_data(node_elem)
                if node:
                    nodes_dict[node.name()] = node
                    print(f"Successfully restored node: {node.name()}")

            # 接続の復元
            print("\nRestoring connections...")
            connection_count = 0
            for conn in root.findall(".//connection"):
                from_node = nodes_dict.get(conn.find("from_node").text)
                to_node = nodes_dict.get(conn.find("to_node").text)

                if from_node and to_node:
                    from_port = from_node.get_output(conn.find("from_port").text)
                    to_port = to_node.get_input(conn.find("to_port").text)

                    if from_port and to_port:
                        self.connect_ports(from_port, to_port)
                        connection_count += 1
                        print(f"Restored connection: {from_node.name()}.{from_port.name()} -> "
                            f"{to_node.name()}.{to_port.name()}")

            print(f"Total connections restored: {connection_count}")

            # 位置の再計算とビューの更新
            print("\nRecalculating positions...")
            self.recalculate_all_positions()

            print("Updating 3D view...")
            if self.stl_viewer:
                QtCore.QTimer.singleShot(500, lambda: self.stl_viewer.reset_view_to_fit())

            print(f"\nProject successfully loaded from: {file_path}")
            return True

        except Exception as e:
            error_msg = f"Error loading project: {str(e)}"
            print(f"\nERROR: {error_msg}")
            print("Traceback:")
            traceback.print_exc()

            QtWidgets.QMessageBox.critical(
                None,
                "Load Error",
                error_msg
            )
            return False

    def _load_node_data(self, node_elem):
        """ノードデータの読み込み"""
        try:
            node_type = node_elem.find("type").text

            # ノードの作成
            if node_type == "BaseLinkNode":
                node = self.create_base_link()
            else:
                node = self.create_node('insilico.nodes.FooNode')

            # 基本情報の設定
            name_elem = node_elem.find("name")
            if name_elem is not None:
                node.set_name(name_elem.text)
                print(f"Loading node: {name_elem.text}")

            # output_count の復元とポートの追加
            if isinstance(node, FooNode):
                points_elem = node_elem.find("points")
                if points_elem is not None:
                    points = points_elem.findall("point")
                    num_points = len(points)
                    print(f"Found {num_points} points")

                    # 必要な数のポートを追加
                    while len(node.output_ports()) < num_points:
                        node._add_output()
                        print(f"Added output port, total now: {len(node.output_ports())}")

                    # ポイントデータの復元
                    node.points = []
                    for point_elem in points:
                        axis_vals = self._parse_axis_element(point_elem.find("axis"))
                        if axis_vals is None:
                            axis_vals = self._parse_axis_element(point_elem.find("joint_axis"))
                        if axis_vals is None:
                            axis_vals = [1.0, 0.0, 0.0]

                        point_data = {
                            'name': point_elem.find("name").text,
                            'type': point_elem.find("type").text,
                            'xyz': [float(x) for x in point_elem.find("xyz").text.split()],
                            'axis': axis_vals,
                        }
                        node.points.append(point_data)
                        print(f"Restored point: {point_data}")

                    # output_countを更新
                    node.output_count = num_points
                    print(f"Set output_count to {num_points}")

            # 位置の設定
            pos_elem = node_elem.find("position")
            if pos_elem is not None:
                x = float(pos_elem.find("x").text)
                y = float(pos_elem.find("y").text)
                node.set_pos(x, y)
                print(f"Set position: ({x}, {y})")

            # 物理プロパティの復元
            volume_elem = node_elem.find("volume")
            if volume_elem is not None:
                node.volume_value = float(volume_elem.text)
                print(f"Restored volume: {node.volume_value}")

            mass_elem = node_elem.find("mass")
            if mass_elem is not None:
                node.mass_value = float(mass_elem.text)
                print(f"Restored mass: {node.mass_value}")

            # 慣性テンソルの復元
            inertia_elem = node_elem.find("inertia")
            if inertia_elem is not None:
                node.inertia = {
                    'ixx': float(inertia_elem.get('ixx', '0.0')),
                    'ixy': float(inertia_elem.get('ixy', '0.0')),
                    'ixz': float(inertia_elem.get('ixz', '0.0')),
                    'iyy': float(inertia_elem.get('iyy', '0.0')),
                    'iyz': float(inertia_elem.get('iyz', '0.0')),
                    'izz': float(inertia_elem.get('izz', '0.0'))
                }
                print(f"Restored inertia tensor")

            # 色情報の復元
            color_elem = node_elem.find("color")
            if color_elem is not None and color_elem.text:
                node.node_color = [float(x) for x in color_elem.text.split()]
                print(f"Restored color: {node.node_color}")

            # 回転軸の復元
            rotation_axis_elem = node_elem.find("rotation_axis")
            if rotation_axis_elem is not None:
                node.rotation_axis = int(rotation_axis_elem.text)
                print(f"Restored rotation axis: {node.rotation_axis}")

            limits_elem = node_elem.find("joint_limits")
            if limits_elem is not None:
                try:
                    node.joint_limit_lower = float(limits_elem.get('lower', getattr(node, 'joint_limit_lower', -math.pi)))
                    node.joint_limit_upper = float(limits_elem.get('upper', getattr(node, 'joint_limit_upper', math.pi)))
                except ValueError:
                    pass
                try:
                    node.joint_position = float(limits_elem.get('position', getattr(node, 'joint_position', 0.0)))
                except ValueError:
                    node.joint_position = getattr(node, 'joint_position', 0.0)

            # Massless Decorationの復元
            massless_dec_elem = node_elem.find("massless_decoration")
            if massless_dec_elem is not None:
                node.massless_decoration = massless_dec_elem.text.lower() == 'true'
                print(f"Restored massless_decoration: {node.massless_decoration}")

            # 累積座標の復元
            coords_elem = node_elem.find("cumulative_coords")
            if coords_elem is not None:
                node.cumulative_coords = []
                for coord_elem in coords_elem.findall("coord"):
                    coord_data = {
                        'point_index': int(coord_elem.find("point_index").text),
                        'xyz': [float(x) for x in coord_elem.find("xyz").text.split()]
                    }
                    node.cumulative_coords.append(coord_data)
                print("Restored cumulative coordinates")

            # STLファイルの設定と処理
            stl_elem = node_elem.find("stl_file")
            if stl_elem is not None and stl_elem.text:
                stl_path = stl_elem.text
                base_dir = stl_elem.get('base_dir', 'project')

                if base_dir == 'meshes' and self.meshes_dir:
                    if stl_path.startswith('meshes/'):
                        stl_path = stl_path[7:]
                    abs_path = os.path.join(self.meshes_dir, stl_path)
                else:
                    abs_path = os.path.join(self.project_dir, stl_path)

                abs_path = os.path.normpath(abs_path)
                if os.path.exists(abs_path):
                    node.stl_file = abs_path
                    if self.stl_viewer:
                        print(f"Loading STL file: {abs_path}")
                        self.stl_viewer.load_stl_for_node(node)
                else:
                    print(f"Warning: STL file not found: {abs_path}")

            print(f"Node {node.name()} loaded successfully")
            return node

        except Exception as e:
            print(f"Error loading node data: {str(e)}")
            traceback.print_exc()
            return None

    def clear_graph(self):
        for node in self.all_nodes():
            self.remove_node(node)

    def connect_ports(self, from_port, to_port):
        """指定された2つのポートを接続"""
        if from_port and to_port:
            try:
                # 利用可能なメソッドを探して接続を試みる
                if hasattr(self, 'connect_nodes'):
                    connection = self.connect_nodes(
                        from_port.node(), from_port.name(),
                        to_port.node(), to_port.name())
                elif hasattr(self, 'add_edge'):
                    connection = self.add_edge(
                        from_port.node().id, from_port.name(),
                        to_port.node().id, to_port.name())
                elif hasattr(from_port, 'connect_to'):
                    connection = from_port.connect_to(to_port)
                else:
                    raise AttributeError("No suitable connection method found")

                if connection:
                    print(
                        f"Connected {from_port.node().name()}.{from_port.name()} to {to_port.node().name()}.{to_port.name()}")
                    return True
                else:
                    print("Failed to connect ports: Connection not established")
                    return False
            except Exception as e:
                print(f"Error connecting ports: {str(e)}")
                return False
        else:
            print("Failed to connect ports: Invalid port(s)")
            return False

    def calculate_cumulative_coordinates(self, node):
        """ノードの累積座標を計算（ルートからのパスを考慮）"""
        if isinstance(node, BaseLinkNode):
            return [0, 0, 0]  # base_linkは原点

        # 親ノードとの接続情報を取得
        input_port = node.input_ports()[0]  # 最初の入力ポート
        if not input_port.connected_ports():
            return [0, 0, 0]  # 接続されていない場合は原点

        parent_port = input_port.connected_ports()[0]
        parent_node = parent_port.node()

        # 親ノードの累積座標を再帰的に計算
        parent_coords = self.calculate_cumulative_coordinates(parent_node)

        # 接続されているポートのインデックスを取得
        port_index = 0
        parent_outputs = parent_node.output_ports() if hasattr(parent_node, 'output_ports') else []
        for idx, port in enumerate(parent_outputs):
            if port is parent_port:
                port_index = idx
                break

        # 親ノードのポイント座標を取得
        if 0 <= port_index < len(parent_node.points):
            point_xyz = parent_node.points[port_index]['xyz']

            # 累積座標の計算
            return [
                parent_coords[0] + point_xyz[0],
                parent_coords[1] + point_xyz[1],
                parent_coords[2] + point_xyz[2]
            ]
        return parent_coords

    def _ensure_unique_point_names(self, point_data_list):
        seen = set()
        for idx, data in enumerate(point_data_list):
            base = data.get('name') or f"point_{idx + 1}"
            base = str(base).strip() or f"point_{idx + 1}"
            candidate = base
            suffix = 1
            while candidate in seen:
                candidate = f"{base}_{suffix}"
                suffix += 1
            if candidate != data.get('name'):
                data['name'] = candidate
            seen.add(candidate)
        return point_data_list

    def _rebuild_output_ports(self, node, point_data_list):
        if not hasattr(node, 'output_ports') or not point_data_list:
            return

        desired_names = [d.get('name') or f"point_{idx + 1}"
                         for idx, d in enumerate(point_data_list)]

        if hasattr(node, 'port_deletion_allowed') and node.port_deletion_allowed():
            input_data = []
            for port in node.input_ports():
                try:
                    input_data.append({
                        'name': port.name(),
                        'multi_connection': bool(getattr(port.model, 'multi_connection', True)),
                        'display_name': bool(getattr(port.model, 'display_name', True)),
                        'locked': port.locked()
                    })
                except Exception:
                    input_data.append({
                        'name': port.name(),
                        'multi_connection': True,
                        'display_name': True,
                        'locked': False
                    })

            output_data = []
            for name in desired_names:
                output_data.append({
                    'name': name,
                    'multi_connection': True,
                    'display_name': True,
                    'locked': False
                })

            try:
                node.set_ports({'input_ports': input_data, 'output_ports': output_data})
            except Exception as e:
                print(f"Warning: failed to rebuild ports for {node.name()}: {e}")
        else:
            outputs = list(node.output_ports())
            while len(outputs) < len(desired_names) and hasattr(node, '_add_output'):
                node._add_output(name=desired_names[len(outputs)])
                outputs = list(node.output_ports())
            while len(outputs) > len(desired_names) and hasattr(node, 'remove_output'):
                node.remove_output()
                outputs = list(node.output_ports())

            outputs = list(node.output_ports())
            for idx, port in enumerate(outputs):
                label = desired_names[idx if idx < len(desired_names) else -1]
                self._set_port_label(node, port, label)

    def _set_port_label(self, node, port, label):
        try:
            port_view = port.view
            label = str(label)
            port_view.name = label
            port_view.display_name = True
            text_item = node.view.get_output_text_item(port_view)
            text_item.setPlainText(label)
        except Exception:
            pass

    def _refresh_node_port_labels(self, node):
        if not hasattr(node, 'output_ports'):
            return
        outputs = list(node.output_ports())
        for idx, port in enumerate(outputs):
            label = None
            if hasattr(node, 'points') and idx < len(node.points):
                label = node.points[idx].get('name')
            if not label:
                label = port.name()
            self._set_port_label(node, port, label)

    def _populate_node_from_root(self, node, root, stl_path: str | None = None):
        if root.tag != 'urdf_part':
            raise ValueError("Invalid metadata format (expected <urdf_part> root)")

        link_elem = root.find('link')
        if link_elem is not None:
            link_name = link_elem.get('name')
            if link_name:
                node.set_name(link_name)

            inertial_elem = link_elem.find('inertial')
            if inertial_elem is not None:
                mass_elem = inertial_elem.find('mass')
                if mass_elem is not None:
                    try:
                        node.mass_value = float(mass_elem.get('value', '0.0'))
                    except ValueError:
                        node.mass_value = 0.0

                volume_elem = inertial_elem.find('volume')
                if volume_elem is not None:
                    try:
                        node.volume_value = float(volume_elem.get('value', '0.0'))
                    except ValueError:
                        node.volume_value = 0.0

                inertia_elem = inertial_elem.find('inertia')
                if inertia_elem is not None:
                    node.inertia = {
                        'ixx': float(inertia_elem.get('ixx', '0.0')),
                        'ixy': float(inertia_elem.get('ixy', '0.0')),
                        'ixz': float(inertia_elem.get('ixz', '0.0')),
                        'iyy': float(inertia_elem.get('iyy', '0.0')),
                        'iyz': float(inertia_elem.get('iyz', '0.0')),
                        'izz': float(inertia_elem.get('izz', '0.0'))
                    }

        color_elem = root.find('.//material/color')
        if color_elem is not None:
            rgba = color_elem.get('rgba', '1.0 1.0 1.0 1.0').split()
            try:
                node.node_color = [float(x) for x in rgba[:3]]
            except ValueError:
                node.node_color = [1.0, 1.0, 1.0]
        else:
            node.node_color = [1.0, 1.0, 1.0]

        joint_elem = root.find('.//joint')
        rotation_axis_assigned = False
        if joint_elem is not None:
            joint_type = joint_elem.get('type', '')
            if joint_type == 'fixed':
                node.rotation_axis = 3
                rotation_axis_assigned = True
                node.joint_limit_lower = 0.0
                node.joint_limit_upper = 0.0
                node.joint_position = 0.0
            else:
                axis_elem = joint_elem.find('axis')
                axis_vals = self._parse_axis_element(axis_elem)
                if axis_vals is not None:
                    node.rotation_axis = self._axis_vector_to_index(axis_vals)
                    rotation_axis_assigned = True

                limit_elem = joint_elem.find('limit')
                if limit_elem is not None:
                    try:
                        node.joint_limit_lower = float(limit_elem.get('lower', node.joint_limit_lower))
                        node.joint_limit_upper = float(limit_elem.get('upper', node.joint_limit_upper))
                    except ValueError:
                        pass

        point_elements = list(root.findall('point'))
        point_data_list = []
        fallback_axis_index = None
        for point_idx, point_elem in enumerate(point_elements):
            point_name = point_elem.get('name', f"point{point_idx + 1}")
            point_type = point_elem.get('type', 'fixed')
            point_xyz_elem = point_elem.find('point_xyz')
            if point_xyz_elem is not None and point_xyz_elem.text:
                xyz_values = [float(x) for x in point_xyz_elem.text.strip().split()]
            else:
                xyz_values = [0.0, 0.0, 0.0]

            axis_vals = self._parse_axis_element(point_elem.find('joint_axis'))
            if axis_vals is None:
                axis_vals = self._parse_axis_element(point_elem.find('axis'))
            if axis_vals is None:
                axis_vals = [1.0, 0.0, 0.0]

            point_data = {
                'name': point_name,
                'type': point_type,
                'xyz': xyz_values,
                'axis': axis_vals,
            }
            point_data_list.append(point_data)

            axis_index = self._axis_vector_to_index(axis_vals)
            if fallback_axis_index is None or axis_index != 3:
                fallback_axis_index = axis_index

        if point_data_list:
            point_data_list = self._ensure_unique_point_names(point_data_list)
            self._rebuild_output_ports(node, point_data_list)
            node.points = point_data_list
        else:
            # 保持既存のポイント設定
            if not getattr(node, 'points', None):
                node.points = []

        outputs = list(node.output_ports()) if hasattr(node, 'output_ports') else []
        if len(node.points) < len(outputs):
            for idx in range(len(node.points), len(outputs)):
                node.points.append({
                    'name': outputs[idx].name() if outputs else f"point_{idx+1}",
                    'type': 'fixed',
                    'xyz': [0.0, 0.0, 0.0],
                    'axis': [1.0, 0.0, 0.0]
                })
        elif len(node.points) > len(outputs):
            node.points = node.points[:len(outputs)]

        coord_count = len(node.points)
        node.cumulative_coords = [{'point_index': idx, 'xyz': [0.0, 0.0, 0.0]} for idx in range(coord_count)]
        node.output_count = len(outputs)
        self._refresh_node_port_labels(node)

        if stl_path and os.path.exists(stl_path):
            node.stl_file = stl_path

        if not rotation_axis_assigned:
            if fallback_axis_index is not None:
                node.rotation_axis = fallback_axis_index
            else:
                node.rotation_axis = 0

    def import_xmls_from_folder(self):
        """フォルダ内の部品メタデータを読み込む"""
        message_box = QtWidgets.QMessageBox()
        message_box.setIcon(QtWidgets.QMessageBox.Information)
        message_box.setWindowTitle("Select Directory")
        message_box.setText("请选择包含 STL 的 meshes 目录。")
        message_box.exec_()

        folder_path = QtWidgets.QFileDialog.getExistingDirectory(
            None, "Select meshes Directory")

        if not folder_path:
            return

        print(f"Importing parts from: {folder_path}")
        self.meshes_dir = folder_path

        try:
            parent_dir = os.path.dirname(folder_path)
            robot_name = os.path.basename(parent_dir)
            if robot_name.endswith('_description'):
                robot_name = robot_name[:-12]
            self.robot_name = robot_name
            if hasattr(self, 'name_input') and self.name_input:
                self.name_input.setText(robot_name)
            print(f"Robot name set to: {robot_name}")
        except Exception as e:
            print(f"Warning: failed to derive robot name: {e}")

        stl_files = sorted([f for f in os.listdir(folder_path) if f.lower().endswith('.stl')])
        xml_files = {os.path.splitext(f)[0]: os.path.join(folder_path, f)
                     for f in os.listdir(folder_path) if f.lower().endswith('.xml')}

        if not stl_files and not xml_files:
            print("No STL or XML files found in the selected folder")
            return

        processed = set()
        imported_count = 0

        for stl_name in stl_files:
            base = os.path.splitext(stl_name)[0]
            stl_path = os.path.join(folder_path, stl_name)
            metadata_source = None
            root = None

            metadata_text = extract_metadata_text(stl_path)
            if metadata_text:
                try:
                    root = ET.fromstring(metadata_text)
                    metadata_source = "embedded"
                except ET.ParseError as e:
                    print(f"Embedded metadata parse error for {stl_name}: {e}")

            if root is None:
                xml_path = xml_files.get(base)
                if xml_path and os.path.exists(xml_path):
                    try:
                        root = ET.parse(xml_path).getroot()
                        metadata_source = "xml"
                    except Exception as e:
                        print(f"Failed to parse XML {xml_path}: {e}")
                else:
                    print(f"Skipped {stl_name}: no metadata found")
                    continue

            new_node = self.create_node(
                'insilico.nodes.FooNode',
                name=f'Node_{len(self.all_nodes())}',
                pos=QtCore.QPointF(0, 0)
            )

            try:
                self._populate_node_from_root(new_node, root, stl_path=stl_path)
            except Exception as e:
                print(f"Failed to populate node from metadata for {stl_name}: {e}")
                traceback.print_exc()
                self.remove_node(new_node)
                continue

            processed.add(base)
            imported_count += 1

            if self.stl_viewer and new_node.stl_file:
                self.stl_viewer.load_stl_for_node(new_node)
                self.stl_viewer.apply_color_to_node(new_node)

            print(f"Imported {stl_name} ({metadata_source})")

        # Handle legacy XML files without STL counterparts
        for base_name, xml_path in xml_files.items():
            if base_name in processed:
                continue
            try:
                root = ET.parse(xml_path).getroot()
                stl_path = os.path.join(folder_path, f"{base_name}.stl")
                new_node = self.create_node(
                    'insilico.nodes.FooNode',
                    name=f'Node_{len(self.all_nodes())}',
                    pos=QtCore.QPointF(0, 0)
                )
                self._populate_node_from_root(new_node, root, stl_path=stl_path if os.path.exists(stl_path) else None)
                if self.stl_viewer and new_node.stl_file:
                    self.stl_viewer.load_stl_for_node(new_node)
                    self.stl_viewer.apply_color_to_node(new_node)
                imported_count += 1
                print(f"Imported legacy XML: {xml_path}")
            except Exception as e:
                print(f"Failed to import legacy XML {xml_path}: {e}")
                traceback.print_exc()
                continue

        print(f"\nImport process completed. Total parts: {imported_count}")
        self._notify_joint_structure_changed()

    def add_parts_from_stl(self):
        """STL ファイルを選択して部品ノードを追加（メタデータは埋め込み優先）"""
        try:
            files, _ = QtWidgets.QFileDialog.getOpenFileNames(
                None, "Select STL Files", self.meshes_dir or os.getcwd(), "STL Files (*.stl)")
            if not files:
                return
            for stl_path in files:
                root = None
                meta_text = None
                try:
                    meta_text = extract_metadata_text(stl_path)
                    if meta_text:
                        root = ET.fromstring(meta_text)
                except Exception as e:
                    print(f"Warning: failed to parse embedded metadata in {stl_path}: {e}")
                node = self.create_node('insilico.nodes.FooNode', name=f'Node_{len(self.all_nodes())}', pos=QtCore.QPointF(0, 0))
                try:
                    if root is not None:
                        self._populate_node_from_root(node, root, stl_path=stl_path)
                    else:
                        # fallback: 仅设置 STL，其他信息默认
                        node.stl_file = stl_path
                except Exception as e:
                    print(f"Failed to initialize node from STL {stl_path}: {e}")
                    self.remove_node(node)
                    continue
                if self.stl_viewer and node.stl_file:
                    self.stl_viewer.load_stl_for_node(node)
                    self.stl_viewer.apply_color_to_node(node)
            # 更新 meshes_dir 推断
            try:
                first_dir = os.path.dirname(files[0])
                if os.path.isdir(first_dir):
                    self.meshes_dir = first_dir
            except Exception:
                pass
            self._notify_joint_structure_changed()
        except Exception as e:
            print(f"Error adding parts from STL: {e}")

    def recalculate_all_positions(self):
        """すべてのノードの位置を再計算"""
        print("Starting position recalculation for all nodes...")

        try:
            # base_linkノードを探す
            base_node = None
            for node in self.all_nodes():
                if isinstance(node, BaseLinkNode):
                    base_node = node
                    break

            if not base_node:
                print("Error: Base link node not found")
                return

            # 再帰的に位置を更新
            visited_nodes = set()
            print(f"Starting from base node: {base_node.name()}")
            identity = vtk.vtkTransform()
            identity.Identity()
            self._recalculate_node_positions(base_node, identity, visited_nodes)

            # STLビューアの更新
            if hasattr(self, 'stl_viewer'):
                self.stl_viewer.vtkWidget.GetRenderWindow().Render()

            print("Position recalculation completed")

        except Exception as e:
            print(f"Error during position recalculation: {str(e)}")
            traceback.print_exc()

    def _recalculate_node_positions(self, node, parent_transform, visited):
        """再帰的にノードの位置を計算"""
        if node in visited:
            return
        visited.add(node)

        try:
            parent_matrix = self._vtk_transform_to_matrix(parent_transform)
            for port_idx, output_port in enumerate(node.output_ports()):
                for connected_port in output_port.connected_ports():
                    child_node = connected_port.node()

                    joint_info = self._compute_joint_relative(
                        node,
                        port_idx,
                        child_node,
                        getattr(child_node, 'joint_position', 0.0),
                    )

                    if not joint_info:
                        print(f"Warning: No joint data for connection {node.name()} -> {child_node.name()}")
                        continue

                    child_matrix = parent_matrix @ joint_info['relative_matrix']
                    child_transform = self._matrix_to_vtk_transform(child_matrix)

                    world_pos = child_transform.TransformPoint(0.0, 0.0, 0.0)

                    if hasattr(child_node, 'cumulative_coords'):
                        for coord in child_node.cumulative_coords:
                            coord['xyz'] = list(world_pos)

                    child_node.current_transform = child_transform

                    if self.stl_viewer:
                        self.stl_viewer.update_stl_transform(child_node, child_transform)

                    self._recalculate_node_positions(child_node, child_transform, visited)

        except Exception as e:
            print(f"Error processing node {node.name()}: {str(e)}")
            traceback.print_exc()

    def disconnect_ports(self, from_port, to_port):
        """ポートの接続を解除"""
        try:
            print(f"Disconnecting ports: {from_port.node().name()}.{from_port.name()} -> {to_port.node().name()}.{to_port.name()}")

            # 接続を解除する前に位置情報をリセット
            child_node = to_port.node()
            if child_node:
                self.stl_viewer.reset_stl_transform(child_node)

            # 利用可能なメソッドを探して接続解除を試みる
            if hasattr(self, 'disconnect_nodes'):
                success = self.disconnect_nodes(
                    from_port.node(), from_port.name(),
                    to_port.node(), to_port.name())
            elif hasattr(from_port, 'disconnect_from'):
                success = from_port.disconnect_from(to_port)
            else:
                success = False
                print("No suitable disconnection method found")

            if success:
                print("Ports disconnected successfully")
                # on_port_disconnectedイベントを呼び出す
                self.on_port_disconnected(to_port, from_port)
                return True
            else:
                print("Failed to disconnect ports")
                return False

        except Exception as e:
            print(f"Error disconnecting ports: {str(e)}")
            return False

    def _write_joint(self, file, parent_node, child_node, parent_port_idx: int):
        """ジョイントの出力"""
        try:
            joint_info = self._compute_joint_relative(parent_node, parent_port_idx, child_node, 0.0)

            origin_xyz = [0.0, 0.0, 0.0]
            origin_rpy = (0.0, 0.0, 0.0)
            axis_joint = None

            if joint_info:
                zero_matrix = joint_info['zero_matrix']
                origin_xyz = zero_matrix[:3, 3].tolist()
                origin_rpy = self._matrix_to_rpy(zero_matrix[:3, :3])
                axis_joint = self._normalize_vector_np(joint_info['joint_axis_joint_frame'])
                axis_joint = np.where(np.abs(axis_joint) < 1e-8, 0.0, axis_joint)

            if axis_joint is None or np.linalg.norm(axis_joint) < 1e-6:
                fallback_axis = None
                if hasattr(child_node, 'rotation_axis'):
                    fallback_axis = self._axis_index_to_vector(child_node.rotation_axis)
                fallback_axis = fallback_axis or [1.0, 0.0, 0.0]
                axis_joint = self._normalize_vector_np(np.array(fallback_axis, dtype=float))
                axis_joint = np.where(np.abs(axis_joint) < 1e-8, 0.0, axis_joint)

            if np.linalg.norm(axis_joint) < 1e-6:
                joint_type = 'fixed'
                axis_text = None
            else:
                joint_type = 'revolute'
                axis_text = f"{axis_joint[0]:.6f} {axis_joint[1]:.6f} {axis_joint[2]:.6f}"

            joint_name = f"{parent_node.name()}_to_{child_node.name()}"

            file.write(f'  <joint name="{joint_name}" type="{joint_type}">\n')
            file.write(
                '    <origin '
                f'xyz="{origin_xyz[0]:.6f} {origin_xyz[1]:.6f} {origin_xyz[2]:.6f}" '
                f'rpy="{origin_rpy[0]:.6f} {origin_rpy[1]:.6f} {origin_rpy[2]:.6f}"/>'
                '\n'
            )
            if axis_text:
                file.write(f'    <axis xyz="{axis_text}"/>\n')
            file.write(f'    <parent link="{parent_node.name()}"/>\n')
            file.write(f'    <child link="{child_node.name()}"/>\n')
            if joint_type != 'fixed':
                lower = getattr(child_node, 'joint_limit_lower', -math.pi)
                upper = getattr(child_node, 'joint_limit_upper', math.pi)
                file.write(f'    <limit lower="{lower:.6f}" upper="{upper:.6f}" effort="0" velocity="0"/>\n')
            file.write('  </joint>\n')

        except Exception as e:
            print(f"Error writing joint: {str(e)}")
            traceback.print_exc()

    def _write_link(
        self,
        file,
        node,
        materials,
        mesh_dir_name: str | None = None,
        export_mode: str = 'ros',
        package_name: str | None = None,
    ):
        """リンクの出力"""
        try:
            file.write(f'  <link name="{node.name()}">\n')

            # 慣性パラメータ
            if hasattr(node, 'mass_value') and hasattr(node, 'inertia'):
                file.write('    <inertial>\n')
                file.write('      <origin xyz="0 0 0" rpy="0 0 0"/>\n')
                file.write(f'      <mass value="{node.mass_value:.6f}"/>\n')
                file.write('      <inertia')
                for key, value in node.inertia.items():
                    file.write(f' {key}="{value:.6f}"')
                file.write('/>\n')
                file.write('    </inertial>\n')

            # メインのビジュアルとコリジョン
            if hasattr(node, 'stl_file') and node.stl_file:
                try:
                    mesh_dir = self._resolve_mesh_dir_name(mesh_dir_name)

                    # メインのビジュアル
                    stl_filename = os.path.basename(node.stl_file)
                    package_path = self._format_mesh_path(
                        stl_filename,
                        mesh_dir,
                        export_mode,
                        package_name,
                    )

                    file.write('    <visual>\n')
                    file.write('      <origin xyz="0 0 0" rpy="0 0 0"/>\n')
                    file.write('      <geometry>\n')
                    file.write(f'        <mesh filename="{package_path}"/>\n')
                    file.write('      </geometry>\n')
                    if hasattr(node, 'node_color'):
                        hex_color = '#{:02x}{:02x}{:02x}'.format(
                            int(node.node_color[0] * 255),
                            int(node.node_color[1] * 255),
                            int(node.node_color[2] * 255)
                        )
                        file.write(f'      <material name="{hex_color}"/>\n')
                    file.write('    </visual>\n')

                    # 装飾パーツのビジュアルを追加
                    for port in node.output_ports():
                        for connected_port in port.connected_ports():
                            dec_node = connected_port.node()
                            if hasattr(dec_node, 'massless_decoration') and dec_node.massless_decoration:
                                if hasattr(dec_node, 'stl_file') and dec_node.stl_file:
                                    dec_stl = os.path.basename(dec_node.stl_file)
                                    dec_path = self._format_mesh_path(
                                        dec_stl,
                                        mesh_dir,
                                        export_mode,
                                        package_name,
                                    )

                                    file.write('    <visual>\n')
                                    file.write('      <origin xyz="0 0 0" rpy="0 0 0"/>\n')
                                    file.write('      <geometry>\n')
                                    file.write(f'        <mesh filename="{dec_path}"/>\n')
                                    file.write('      </geometry>\n')
                                    if hasattr(dec_node, 'node_color'):
                                        dec_color = '#{:02x}{:02x}{:02x}'.format(
                                            int(dec_node.node_color[0] * 255),
                                            int(dec_node.node_color[1] * 255),
                                            int(dec_node.node_color[2] * 255)
                                        )
                                        file.write(f'      <material name="{dec_color}"/>\n')
                                    file.write('    </visual>\n')

                    # コリジョン
                    file.write('    <collision>\n')
                    file.write('      <origin xyz="0 0 0" rpy="0 0 0"/>\n')
                    file.write('      <geometry>\n')
                    file.write(f'        <mesh filename="{package_path}"/>\n')
                    file.write('      </geometry>\n')
                    file.write('    </collision>\n')

                except Exception as e:
                    print(f"Error processing STL file for node {node.name()}: {str(e)}")
                    traceback.print_exc()

            file.write('  </link>\n')

        except Exception as e:
            print(f"Error writing link: {str(e)}")
            traceback.print_exc()

    def export_for_unity(self):
        """Unityプロジェクト用のファイル構造を作成しエクスポート"""
        try:
            # ディレクトリ選択ダイアログを表示
            message_box = QtWidgets.QMessageBox()
            message_box.setIcon(QtWidgets.QMessageBox.Information)
            message_box.setWindowTitle("Select Directory")
            message_box.setText("Please select the directory where you want to create the Unity project structure.")
            message_box.exec_()

            base_dir = QtWidgets.QFileDialog.getExistingDirectory(
                self.widget,
                "Select Base Directory for Unity Export"
            )

            if not base_dir:
                print("Unity export cancelled")
                return False

            # ロボット名からディレクトリ名を生成
            robot_name = self.get_robot_name()
            unity_dir_name = f"{robot_name}_unity_description"
            unity_dir_path = os.path.join(base_dir, unity_dir_name)

            # メインディレクトリの作成
            os.makedirs(unity_dir_path, exist_ok=True)
            print(f"Created Unity description directory: {unity_dir_path}")

            # meshesディレクトリの作成
            meshes_dir = os.path.join(unity_dir_path, "meshes")
            os.makedirs(meshes_dir, exist_ok=True)
            print(f"Created meshes directory: {meshes_dir}")

            # STLファイルのコピー
            copied_files = []
            for node in self.all_nodes():
                if hasattr(node, 'stl_file') and node.stl_file:
                    if os.path.exists(node.stl_file):
                        # ファイル名のみを取得
                        stl_filename = os.path.basename(node.stl_file)
                        # コピー先のパスを生成
                        dest_path = os.path.join(meshes_dir, stl_filename)
                        copy_without_metadata(node.stl_file, dest_path)
                        copied_files.append(stl_filename)
                        print(f"Copied STL file: {stl_filename}")

            # URDFファイルの生成
            urdf_file = os.path.join(unity_dir_path, f"{robot_name}.urdf")
            with open(urdf_file, 'w', encoding='utf-8') as f:
                # ヘッダー
                f.write('<?xml version="1.0"?>\n')
                f.write(f'<robot name="{robot_name}">\n\n')

                # マテリアル定義の収集
                materials = {}
                for node in self.all_nodes():
                    if hasattr(node, 'node_color'):
                        rgb = node.node_color
                        if len(rgb) >= 3:
                            hex_color = '#{:02x}{:02x}{:02x}'.format(
                                int(rgb[0] * 255),
                                int(rgb[1] * 255),
                                int(rgb[2] * 255)
                            )
                            materials[hex_color] = rgb

                # マテリアルの書き出し
                f.write('<!-- material color setting -->\n')
                for hex_color, rgb in materials.items():
                    f.write(f'<material name="{hex_color}">\n')
                    f.write(f'  <color rgba="{rgb[0]:.3f} {rgb[1]:.3f} {rgb[2]:.3f} 1.0"/>\n')
                    f.write('</material>\n')
                f.write('\n')

                # base_linkから開始して、ツリー構造を順番に出力
                visited_nodes = set()
                base_node = self.get_node_by_name('base_link')
                if base_node:
                    self._write_tree_structure_unity(f, base_node, None, visited_nodes, materials, unity_dir_name)

                f.write('</robot>\n')

            print(f"Unity export completed successfully:")
            print(f"- Directory: {unity_dir_path}")
            print(f"- URDF file: {urdf_file}")
            print(f"- Copied {len(copied_files)} STL files")

            QtWidgets.QMessageBox.information(
                self.widget,
                "Unity Export Complete",
                f"URDF files have been exported for Unity URDF-Importer:\n\n"
                f"Directory Path:\n{unity_dir_path}\n\n"
                f"URDF File:\n{urdf_file}\n\n"
                f"The files are ready to be imported using Unity URDF-Importer."
            )

            return True

        except Exception as e:
            error_msg = f"Error exporting for Unity: {str(e)}"
            print(error_msg)
            traceback.print_exc()

            QtWidgets.QMessageBox.critical(
                self.widget,
                "Export Error",
                error_msg
            )
            return False

    def _write_tree_structure_unity(self, file, node, parent_node, visited_nodes, materials, unity_dir_name):
        """Unity用のツリー構造を順番に出力"""
        if node in visited_nodes:
            return
        visited_nodes.add(node)

        # Massless Decorationノードはスキップ（親ノードの<visual>として処理済み）
        if hasattr(node, 'massless_decoration') and node.massless_decoration:
            return

        if node.name() == "base_link":
            # base_linkの出力
            self._write_base_link(file)

        # 現在のノードに接続されているジョイントとリンクを処理
        for port_idx, port in enumerate(node.output_ports()):
            for connected_port in port.connected_ports():
                child_node = connected_port.node()
                if child_node not in visited_nodes:
                    # Massless Decorationでないノードのみジョイントとリンクを出力
                    if not (hasattr(child_node, 'massless_decoration') and child_node.massless_decoration):
                        # まずジョイントを出力
                        self._write_joint(file, node, child_node, port_idx)
                        file.write('\n')

                        # 次にリンクを出力（Unity用のパスで）
                        self._write_link_unity(file, child_node, materials, unity_dir_name)
                        file.write('\n')

                        # 再帰的に子ノードを処理
                        self._write_tree_structure_unity(file, child_node, node, visited_nodes, materials, unity_dir_name)

    def _write_link_unity(self, file, node, materials, unity_dir_name):
        """Unity用のリンク出力"""
        try:
            file.write(f'  <link name="{node.name()}">\n')

            # 慣性パラメータ
            if hasattr(node, 'mass_value') and hasattr(node, 'inertia'):
                file.write('    <inertial>\n')
                file.write('      <origin xyz="0 0 0" rpy="0 0 0"/>\n')
                file.write(f'      <mass value="{node.mass_value:.6f}"/>\n')
                file.write('      <inertia')
                for key, value in node.inertia.items():
                    file.write(f' {key}="{value:.6f}"')
                file.write('/>\n')
                file.write('    </inertial>\n')

            # ビジュアルとコリジョン
            if hasattr(node, 'stl_file') and node.stl_file:
                try:
                    # メインのビジュアル
                    stl_filename = os.path.basename(node.stl_file)
                    # パスは直接meshesを指定
                    package_path = f"package://meshes/{stl_filename}"

                    # メインのビジュアル
                    file.write('    <visual>\n')
                    file.write('      <origin xyz="0 0 0" rpy="0 0 0"/>\n')
                    file.write('      <geometry>\n')
                    file.write(f'        <mesh filename="{package_path}"/>\n')
                    file.write('      </geometry>\n')
                    if hasattr(node, 'node_color'):
                        hex_color = '#{:02x}{:02x}{:02x}'.format(
                            int(node.node_color[0] * 255),
                            int(node.node_color[1] * 255),
                            int(node.node_color[2] * 255)
                        )
                        file.write(f'      <material name="{hex_color}"/>\n')
                    file.write('    </visual>\n')

                    # 装飾パーツのビジュアルを追加
                    for port in node.output_ports():
                        for connected_port in port.connected_ports():
                            dec_node = connected_port.node()
                            if hasattr(dec_node, 'massless_decoration') and dec_node.massless_decoration:
                                if hasattr(dec_node, 'stl_file') and dec_node.stl_file:
                                    dec_stl = os.path.basename(dec_node.stl_file)
                                    dec_path = f"package://meshes/{dec_stl}"

                                    file.write('    <visual>\n')
                                    file.write('      <origin xyz="0 0 0" rpy="0 0 0"/>\n')
                                    file.write('      <geometry>\n')
                                    file.write(f'        <mesh filename="{dec_path}"/>\n')
                                    file.write('      </geometry>\n')
                                    if hasattr(dec_node, 'node_color'):
                                        dec_color = '#{:02x}{:02x}{:02x}'.format(
                                            int(dec_node.node_color[0] * 255),
                                            int(dec_node.node_color[1] * 255),
                                            int(dec_node.node_color[2] * 255)
                                        )
                                        file.write(f'      <material name="{dec_color}"/>\n')
                                    file.write('    </visual>\n')

                    # コリジョン
                    file.write('    <collision>\n')
                    file.write('      <origin xyz="0 0 0" rpy="0 0 0"/>\n')
                    file.write('      <geometry>\n')
                    file.write(f'        <mesh filename="{package_path}"/>\n')
                    file.write('      </geometry>\n')
                    file.write('    </collision>\n')

                except Exception as e:
                    print(f"Error processing STL file for node {node.name()}: {str(e)}")
                    traceback.print_exc()

            file.write('  </link>\n')

        except Exception as e:
            print(f"Error writing link: {str(e)}")
            traceback.print_exc()

    def calculate_inertia_tensor_for_mirrored(self, poly_data, mass, center_of_mass):
        """
        ミラーリングされたモデルの慣性テンソルを計算。
        CustomNodeGraphクラスのメソッド。
        """
        try:
            print("\nCalculating inertia tensor for mirrored model...")
            print(f"Mass: {mass:.6f}")
            print(f"Center of Mass (before mirroring): {center_of_mass}")

            # Y座標を反転した重心を使用
            mirrored_com = [center_of_mass[0], -center_of_mass[1], center_of_mass[2]]
            print(f"Center of Mass (after mirroring): {mirrored_com}")

            # インスペクタウィンドウのインスタンスを取得
            if not hasattr(self, 'inspector_window') or not self.inspector_window:
                self.inspector_window = InspectorWindow(stl_viewer=self.stl_viewer)

            # 慣性テンソルを計算（ミラーリングモードで）
            inertia_tensor = self.inspector_window._calculate_base_inertia_tensor(
                poly_data, mass, mirrored_com, is_mirrored=True)

            print("\nMirrored model inertia tensor calculated successfully")
            return inertia_tensor

        except Exception as e:
            print(f"Error calculating mirrored inertia tensor: {str(e)}")
            traceback.print_exc()
            return None
