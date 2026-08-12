#!/usr/bin/env python3
"""PyQt5 dashboard for human--AI collaboration in Stage-5 VLM search.

The dashboard deliberately publishes high-level intent only.  It does not issue
navigation goals or low-level velocity commands.  All map clicks become priority
polygons consumed by ``human_interaction_manager.py``.
"""
from __future__ import annotations

import json
import math
import os
import sys
import uuid
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from PyQt5.QtCore import QObject, QPointF, QRectF, Qt, pyqtSignal
    from PyQt5.QtGui import (
        QColor, QFont, QImage, QPainter, QPainterPath,
        QPen, QPixmap, QTextOption
    )
    from PyQt5.QtWidgets import (
        QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout,
        QFrame, QGridLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel,
        QMainWindow, QMessageBox, QPushButton, QPlainTextEdit, QScrollArea,
        QSpinBox, QSplitter, QTabWidget, QTableWidget, QTableWidgetItem,
        QTextEdit, QVBoxLayout, QWidget,
    )
except ImportError as exc:  # pragma: no cover - runtime dependency message
    raise SystemExit('PyQt5 is required. Install it with: sudo apt install python3-pyqt5') from exc

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from vlm_common import compact_json, safe_json_loads


_ROBOT_COLORS = {
    'uav0': QColor(54, 127, 255),
    'uav1': QColor(255, 156, 61),
    'ugv0': QColor(80, 180, 105),
}


def _robot_color(robot_id: str) -> QColor:
    return _ROBOT_COLORS.get(robot_id, QColor(170, 80, 200))


def _pretty(value: Any) -> str:
    try:
        return json.dumps(value, ensure_ascii=False, indent=2, sort_keys=False)
    except Exception:
        return str(value)

def _configure_readable_text_editor(editor: QPlainTextEdit,
                                    font_size: int = 10) -> None:
    """统一设置识别描述和规划输出框的可读性。"""
    font = QFont('Noto Sans CJK SC', font_size)
    font.setLetterSpacing(QFont.PercentageSpacing, 104)  # 轻微增加字符间距
    editor.setFont(font)

    # 长英文单词、JSON 字段、目标编号等均可在窗口边界换行。
    editor.setLineWrapMode(QPlainTextEdit.WidgetWidth)
    editor.setWordWrapMode(QTextOption.WrapAtWordBoundaryOrAnywhere)

    # 增加文本块上下间距，避免多行信息紧贴。
    option = editor.document().defaultTextOption()
    option.setWrapMode(QTextOption.WrapAtWordBoundaryOrAnywhere)
    editor.document().setDefaultTextOption(option)

    block_format = editor.document().rootFrame().frameFormat()
    block_format.setTopMargin(3)
    block_format.setBottomMargin(3)
    editor.document().rootFrame().setFrameFormat(block_format)

def _numpy_to_qimage(array: np.ndarray, image_format: QImage.Format) -> QImage:
    """Safely copy a NumPy image into a PyQt5 QImage.

    In PyQt5 on Ubuntu 20.04, ``ndarray.data`` is exposed as a Python
    ``memoryview``. The QImage overload used here accepts ``bytes`` (or a SIP
    pointer), not a memoryview.  Converting to a contiguous byte buffer and
    immediately calling ``copy()`` gives QImage independent ownership.
    """
    if not isinstance(array, np.ndarray) or array.ndim != 3:
        raise ValueError('Expected an HxWxC NumPy image array.')

    array = np.ascontiguousarray(array)
    height, width = int(array.shape[0]), int(array.shape[1])
    bytes_per_line = int(array.strides[0])
    return QImage(
        array.tobytes(), width, height, bytes_per_line, image_format
    ).copy()


class MapCanvas(QWidget):
    """Renderer for one geometry layer (UAV or UGV) in the shared ``map`` frame.

    The aerial and ground occupancy grids must remain visually separate because
    they encode different clearance assumptions.  Human priority polygons are
    map-frame intent and are intentionally drawn on both views.
    """

    region_drawn = pyqtSignal(object)

    def __init__(self, map_key: str, title: str, topic: str,
                 parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.map_key = str(map_key).strip().lower()
        self.title = str(title)
        self.topic = str(topic)
        self.setMinimumSize(560, 250)
        self.setMouseTracking(True)
        self._map_image: Optional[QImage] = None
        self._map_meta: Dict[str, float] = {}
        self._regions: List[Dict[str, Any]] = []
        self._poses: Dict[str, Tuple[float, float, float]] = {}
        self._goals: Dict[str, Dict[str, Any]] = {}
        self._paths: Dict[str, List[Tuple[float, float]]] = {}
        self._semantic_objects: List[Dict[str, Any]] = []
        self._robot_types: Dict[str, str] = {}
        self._draw_enabled = False
        self._drag_start: Optional[QPointF] = None
        self._drag_end: Optional[QPointF] = None
        self._map_rect = self.rect()

    def set_robot_types(self, robot_types: Dict[str, str]) -> None:
        self._robot_types = {
            str(name): str(kind).strip().lower()
            for name, kind in robot_types.items()
        }

    def _belongs_to_this_map(self, robot_id: str) -> bool:
        kind = self._robot_types.get(str(robot_id), '')
        if not kind:
            # Safe fallback for projects that omit the ``type`` field.
            kind = 'ugv' if str(robot_id).lower().startswith('ugv') else 'uav'
        return kind == self.map_key

    def set_draw_enabled(self, enabled: bool) -> None:
        self._draw_enabled = bool(enabled)
        if not enabled:
            self._drag_start = None
            self._drag_end = None
        self.update()

    def set_map(self, msg: OccupancyGrid) -> None:
        try:
            width = int(msg.info.width)
            height = int(msg.info.height)
            if width <= 0 or height <= 0 or len(msg.data) != width * height:
                return
            data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        except Exception:
            return
        rgba = np.zeros((height, width, 4), dtype=np.uint8)
        unknown = data < 0
        occupied = data >= 65
        free = (~unknown) & (~occupied)
        rgba[unknown] = (145, 145, 145, 255)
        rgba[free] = (245, 245, 245, 255)
        rgba[occupied] = (32, 32, 32, 255)
        # np.flipud() returns a negative-stride view.  QImage needs a
        # conventional, contiguous top-to-bottom byte buffer.
        rgba = np.ascontiguousarray(np.flipud(rgba))
        self._map_image = _numpy_to_qimage(rgba, QImage.Format_RGBA8888)
        self._map_meta = {
            'origin_x': float(msg.info.origin.position.x),
            'origin_y': float(msg.info.origin.position.y),
            'resolution': float(msg.info.resolution),
            'width': float(width),
            'height': float(height),
        }
        self.update()

    def set_context(self, context: Dict[str, Any]) -> None:
        regions = context.get('priority_regions', []) if isinstance(context, dict) else []
        self._regions = [item for item in regions if isinstance(item, dict)] if isinstance(regions, list) else []
        self.update()

    def set_pose(self, robot_id: str, pose: PoseStamped) -> None:
        if not self._belongs_to_this_map(robot_id):
            return
        q = pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._poses[robot_id] = (float(pose.pose.position.x), float(pose.pose.position.y), yaw)
        self.update()

    def set_goal(self, robot_id: str, goal: Dict[str, Any]) -> None:
        if not self._belongs_to_this_map(robot_id):
            return
        if isinstance(goal, dict) and 'x' in goal and 'y' in goal:
            self._goals[robot_id] = dict(goal)
            self.update()

    def set_path(self, robot_id: str, path: Path) -> None:
        if not self._belongs_to_this_map(robot_id):
            return
        points = [(float(item.pose.position.x), float(item.pose.position.y)) for item in path.poses]
        self._paths[robot_id] = points
        self.update()

    def set_semantic_overlay(self, overlay: Dict[str, Any]) -> None:
        objects = overlay.get('objects', []) if isinstance(overlay, dict) else []
        self._semantic_objects = [item for item in objects if isinstance(item, dict)] if isinstance(objects, list) else []
        self.update()

    def _compute_map_rect(self):
        if self._map_image is None or not self._map_meta:
            return self.rect()
        image_ratio = float(self._map_image.width()) / max(1.0, float(self._map_image.height()))
        area = self.rect().adjusted(8, 8, -8, -8)
        area_ratio = float(area.width()) / max(1.0, float(area.height()))
        if area_ratio > image_ratio:
            height = area.height()
            width = int(height * image_ratio)
            x = area.x() + (area.width() - width) // 2
            return area.__class__(x, area.y(), width, height)
        width = area.width()
        height = int(width / image_ratio)
        y = area.y() + (area.height() - height) // 2
        return area.__class__(area.x(), y, width, height)

    def _world_to_screen(self, x: float, y: float) -> QPointF:
        rect = self._map_rect
        meta = self._map_meta
        if not meta or rect.width() <= 0 or rect.height() <= 0:
            return QPointF()
        world_width = meta['width'] * meta['resolution']
        world_height = meta['height'] * meta['resolution']
        sx = rect.left() + (x - meta['origin_x']) / max(world_width, 1e-9) * rect.width()
        sy = rect.top() + (meta['origin_y'] + world_height - y) / max(world_height, 1e-9) * rect.height()
        return QPointF(sx, sy)

    def _screen_to_world(self, p: QPointF) -> Optional[Tuple[float, float]]:
        rect = self._map_rect
        meta = self._map_meta
        if not meta or not rect.contains(int(p.x()), int(p.y())):
            return None
        world_width = meta['width'] * meta['resolution']
        world_height = meta['height'] * meta['resolution']
        x = meta['origin_x'] + (p.x() - rect.left()) / max(rect.width(), 1) * world_width
        y = meta['origin_y'] + world_height - (p.y() - rect.top()) / max(rect.height(), 1) * world_height
        return (float(x), float(y))

    def paintEvent(self, _event) -> None:  # noqa: N802, PyQt API
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(29, 31, 35))
        self._map_rect = self._compute_map_rect()
        if self._map_image is None:
            painter.setPen(QColor(220, 220, 220))
            painter.drawText(
                self.rect(), Qt.AlignCenter,
                '等待 %s 地图...\n%s' % (self.title, self.topic))
            painter.end()
            return
        painter.drawImage(self._map_rect, self._map_image)
        painter.setPen(QPen(QColor(160, 160, 160), 1))
        painter.drawRect(self._map_rect)

        for region in self._regions:
            polygon = region.get('polygon', [])
            if not isinstance(polygon, list) or len(polygon) < 3:
                continue
            path = QPainterPath()
            first = polygon[0]
            if not isinstance(first, dict):
                continue
            path.moveTo(self._world_to_screen(float(first.get('x', 0.0)), float(first.get('y', 0.0))))
            valid = True
            for point in polygon[1:]:
                if not isinstance(point, dict):
                    valid = False
                    break
                path.lineTo(self._world_to_screen(float(point.get('x', 0.0)), float(point.get('y', 0.0))))
            if not valid:
                continue
            path.closeSubpath()
            hard = str(region.get('mode', 'soft')).lower() == 'hard'
            border = QColor(220, 70, 70) if hard else QColor(255, 184, 56)
            fill = QColor(border.red(), border.green(), border.blue(), 55)
            painter.fillPath(path, fill)
            painter.setPen(QPen(border, 2))
            painter.drawPath(path)
            bounds = path.boundingRect()
            painter.setPen(QColor(30, 30, 30))
            painter.drawText(bounds.topLeft() + QPointF(3, 14), '%s  p=%.2f  max=%s' % (
                region.get('region_id', 'H?'), float(region.get('priority', 0.0)), region.get('max_robots', 1)))

        # Map-aligned semantic evidence. Query-specific targets are red;
        # ordinary semantic entities remain cyan and do not imply target match.
        for obj in self._semantic_objects:
            pos = obj.get('position_map', {}) if isinstance(obj.get('position_map'), dict) else {}
            if 'x' not in pos or 'y' not in pos:
                continue
            p = self._world_to_screen(float(pos['x']), float(pos['y']))
            is_target = str(obj.get('label', '')) == 'target_candidate' or str(obj.get('category', '')) == 'query_target'
            confidence = float(obj.get('target_confidence', obj.get('confidence', 0.0)) or 0.0)
            color = QColor(220, 55, 55) if is_target else QColor(35, 180, 185)
            painter.setPen(QPen(color, 2))
            painter.setBrush(QColor(color.red(), color.green(), color.blue(), 75))
            painter.drawEllipse(p, 5, 5)
            if is_target:
                painter.setPen(QColor(25, 25, 25))
                painter.drawText(p + QPointF(7, 12), 'target %.2f' % confidence)

        for robot_id, points in self._paths.items():
            if len(points) < 2:
                continue
            path = QPainterPath(self._world_to_screen(*points[0]))
            for x, y in points[1:]:
                path.lineTo(self._world_to_screen(x, y))
            painter.setPen(QPen(_robot_color(robot_id), 2, Qt.DashLine))
            painter.drawPath(path)

        for robot_id, goal in self._goals.items():
            try:
                p = self._world_to_screen(float(goal['x']), float(goal['y']))
            except Exception:
                continue
            color = _robot_color(robot_id)
            painter.setPen(QPen(color, 2))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(p, 7, 7)
            painter.drawLine(p + QPointF(-10, 0), p + QPointF(10, 0))
            painter.drawLine(p + QPointF(0, -10), p + QPointF(0, 10))

        for robot_id, (x, y, yaw) in self._poses.items():
            p = self._world_to_screen(x, y)
            color = _robot_color(robot_id)
            painter.setBrush(color)
            painter.setPen(QPen(QColor(20, 20, 20), 1))
            painter.drawEllipse(p, 7, 7)
            end = p + QPointF(16.0 * math.cos(yaw), -16.0 * math.sin(yaw))
            painter.drawLine(p, end)
            painter.setPen(QColor(25, 25, 25))
            painter.drawText(p + QPointF(9, -9), robot_id)

        if self._draw_enabled and self._drag_start is not None and self._drag_end is not None:
            painter.setBrush(QColor(70, 175, 255, 45))
            painter.setPen(QPen(QColor(70, 175, 255), 2, Qt.DashLine))
            painter.drawRect(QRectF(self._drag_start, self._drag_end).normalized())
        if self._draw_enabled:
            painter.setPen(QColor(220, 230, 245))
            painter.drawText(12, 22, '绘制模式：在此 %s 地图上拖拽矩形，松开鼠标后创建全局优先探索区' % self.map_key.upper())
        painter.end()

    def mousePressEvent(self, event) -> None:  # noqa: N802, PyQt API
        if self._draw_enabled and event.button() == Qt.LeftButton and self._map_rect.contains(event.pos()):
            self._drag_start = QPointF(event.pos())
            self._drag_end = QPointF(event.pos())
            self.update()
        elif event.button() == Qt.RightButton:
            self._drag_start = None
            self._drag_end = None
            self.update()

    def mouseMoveEvent(self, event) -> None:  # noqa: N802, PyQt API
        if self._draw_enabled and self._drag_start is not None:
            self._drag_end = QPointF(event.pos())
            self.update()

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802, PyQt API
        if not self._draw_enabled or event.button() != Qt.LeftButton or self._drag_start is None:
            return
        self._drag_end = QPointF(event.pos())
        start = self._screen_to_world(self._drag_start)
        end = self._screen_to_world(self._drag_end)
        self._drag_start = None
        self._drag_end = None
        self.update()
        if start is None or end is None:
            return
        x0, x1 = sorted((start[0], end[0]))
        y0, y1 = sorted((start[1], end[1]))
        if x1 - x0 < 0.20 or y1 - y0 < 0.20:
            return
        self.region_drawn.emit([
            {'x': x0, 'y': y0}, {'x': x1, 'y': y0},
            {'x': x1, 'y': y1}, {'x': x0, 'y': y1},
        ])


class ImagePanel(QLabel):
    def __init__(self, title: str) -> None:
        super().__init__()
        self._title = title
        self._image: Optional[QImage] = None
        self.setMinimumSize(280, 190)
        self.setAlignment(Qt.AlignCenter)
        self.setFrameShape(QFrame.StyledPanel)
        self.setText('%s\n等待图像...' % title)

    def set_image(self, image: QImage) -> None:
        self._image = image
        self._refresh()

    def resizeEvent(self, event) -> None:  # noqa: N802
        super().resizeEvent(event)
        self._refresh()

    def _refresh(self) -> None:
        if self._image is None:
            return
        self.setPixmap(QPixmap.fromImage(self._image).scaled(
            self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))


class RosBridge(QObject):
    # First argument identifies the geometry layer: ``uav`` or ``ugv``.
    map_received = pyqtSignal(str, object)
    pose_received = pyqtSignal(str, object)
    path_received = pyqtSignal(str, object)
    image_received = pyqtSignal(str, str, object)
    context_received = pyqtSignal(object)
    feedback_received = pyqtSignal(object)
    status_received = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        rospy.init_node('human_ai_dashboard', anonymous=True, disable_signals=True)
        self.bridge = CvBridge()
        self.robots = list(rospy.get_param('/vehicles', [])) + list(rospy.get_param('/ground_robots', []))
        self.region_pub = rospy.Publisher('/hri/set_priority_region', String, queue_size=10)
        self.remove_region_pub = rospy.Publisher('/hri/remove_priority_region', String, queue_size=10)
        self.clear_region_pub = rospy.Publisher('/hri/clear_priority_regions', String, queue_size=5)
        self.target_pub = rospy.Publisher('/hri/set_target_instruction', String, queue_size=10)
        self.force_replan_pub = rospy.Publisher('/hri/force_replan', String, queue_size=5)

        # These are intentionally two independent subscriptions.  The UAV map
        # represents a flight-height traversability layer, while the UGV map
        # represents near-ground traversability and must not overwrite it.
        self.uav_map_topic = rospy.get_param('~uav_map_topic', '/global_map_2d')
        ground_robots = [robot for robot in self.robots if str(robot.get('type', '')).lower() == 'ugv']
        default_ugv_map_topic = '/ugv0/ground_map_2d'
        if ground_robots:
            default_ugv_map_topic = str(ground_robots[0].get('ground_map_topic', default_ugv_map_topic))
        self.ugv_map_topic = rospy.get_param('~ugv_map_topic', default_ugv_map_topic)
        rospy.Subscriber(self.uav_map_topic, OccupancyGrid,
                         lambda msg: self.map_received.emit('uav', msg), queue_size=1)
        rospy.Subscriber(self.ugv_map_topic, OccupancyGrid,
                         lambda msg: self.map_received.emit('ugv', msg), queue_size=1)
        rospy.Subscriber('/hri/shared_context', String, self._context_cb, queue_size=10)
        rospy.Subscriber('/hri/decision_feedback', String, self._feedback_cb, queue_size=10)
        rospy.Subscriber('/hri/status', String, lambda msg: self.status_received.emit(str(msg.data)), queue_size=10)
        for robot in self.robots:
            name = str(robot.get('name', ''))
            if not name:
                continue
            rospy.Subscriber(str(robot['global_pose_topic']), PoseStamped,
                             lambda msg, rid=name: self.pose_received.emit(rid, msg), queue_size=10)
            rospy.Subscriber(str(robot.get('planned_path_topic', '/%s/search/planned_path' % name)), Path,
                             lambda msg, rid=name: self.path_received.emit(rid, msg), queue_size=3)
            rospy.Subscriber(str(robot['rgb_topic']), Image,
                             lambda msg, rid=name: self._image_cb(rid, 'rgb', msg), queue_size=1, buff_size=2**24)
            rospy.Subscriber('/%s/vlm/debug_image' % name, Image,
                             lambda msg, rid=name: self._image_cb(rid, 'debug', msg), queue_size=1, buff_size=2**24)

    def _image_cb(self, robot_id: str, kind: str, msg: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb = np.ascontiguousarray(bgr[:, :, ::-1])
            image = _numpy_to_qimage(rgb, QImage.Format_RGB888)
        except Exception:
            return
        self.image_received.emit(robot_id, kind, image)

    def _context_cb(self, msg: String) -> None:
        data = safe_json_loads(msg.data, None)
        if isinstance(data, dict):
            self.context_received.emit(data)

    def _feedback_cb(self, msg: String) -> None:
        data = safe_json_loads(msg.data, None)
        if isinstance(data, dict):
            self.feedback_received.emit(data)

    def set_region(self, payload: Dict[str, Any]) -> None:
        self.region_pub.publish(compact_json(payload))

    def remove_region(self, region_id: str) -> None:
        self.remove_region_pub.publish(compact_json({'region_id': region_id}))

    def clear_regions(self) -> None:
        self.clear_region_pub.publish('{}')

    def set_target_instruction(self, payload: Dict[str, Any]) -> None:
        self.target_pub.publish(compact_json(payload))

    def force_replan(self, refresh_local_perception: bool = True) -> None:
        self.force_replan_pub.publish(compact_json({
            'refresh_local_perception': bool(refresh_local_perception),
            'operator_note': 'Dashboard manual replan request',
        }))


class Dashboard(QMainWindow):
    def __init__(self, bridge: RosBridge) -> None:
        super().__init__()
        self.bridge = bridge
        self.current_context: Dict[str, Any] = {}
        self.current_feedback: Dict[str, Any] = {}
        self.image_panels: Dict[Tuple[str, str], ImagePanel] = {}
        self.robot_types: Dict[str, str] = {
            str(robot.get('name', '')): str(robot.get('type', 'uav')).lower()
            for robot in bridge.robots if robot.get('name')
        }
        self.setWindowTitle('Stage-5 VLM Human--AI Collaboration Dashboard')
        self.resize(1660, 960)
        self._build_ui()
        self._wire_ros()

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)
        splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(splitter)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(2, 2, 2, 2)
        left_layout.setSpacing(4)

        # Keep the two traversability layers in separate, vertically stacked
        # views.  Both use the common map frame, but only platform-compatible
        # robots and routes are rendered on each map.
        self.uav_map_canvas = MapCanvas('uav', 'UAV 空中占据地图', self.bridge.uav_map_topic)
        self.ugv_map_canvas = MapCanvas('ugv', 'UGV 近地占据地图', self.bridge.ugv_map_topic)
        self.uav_map_canvas.set_robot_types(self.robot_types)
        self.ugv_map_canvas.set_robot_types(self.robot_types)

        map_splitter = QSplitter(Qt.Vertical)
        map_splitter.setChildrenCollapsible(False)
        uav_section = self._make_map_section(
            'UAV 地图：/global_map_2d（仅由 UAV LiDAR 构建，适用于飞行高度通行性）',
            self.uav_map_canvas)
        ugv_section = self._make_map_section(
            'UGV 地图：/ugv0/ground_map_2d（由 UGV LiDAR 构建，适用于近地通行性）',
            self.ugv_map_canvas)
        map_splitter.addWidget(uav_section)
        map_splitter.addWidget(ugv_section)
        map_splitter.setSizes([470, 470])
        left_layout.addWidget(map_splitter, 1)

        left_help = QLabel(
            '图例：实线圆点=当前平台机器人；虚线=当前平台 A* 路径；十字=当前目标点；'
            '橙/红色区域=操作员优先探索区。优先区域使用统一 map 坐标，可在任一地图上绘制。')
        left_help.setWordWrap(True)
        left_layout.addWidget(left_help)
        splitter.addWidget(left)

        right = QTabWidget()
        right.addTab(self._build_input_tab(), '人类输入')
        right.addTab(self._build_visual_tab(), '视觉与调试')
        right.addTab(self._build_feedback_tab(), 'AI 决策反馈')
        splitter.addWidget(right)
        splitter.setSizes([920, 720])
        self.statusBar().showMessage('等待 ROS 数据...')

    @staticmethod
    def _make_map_section(title: str, canvas: MapCanvas) -> QWidget:
        section = QWidget()
        layout = QVBoxLayout(section)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        label = QLabel(title)
        label.setStyleSheet('font-weight: bold; padding: 2px 4px;')
        layout.addWidget(label)
        layout.addWidget(canvas, 1)
        return section

    def _set_region_draw_mode(self, enabled: bool) -> None:
        # The priority polygon is global ``map``-frame intent; either canvas can
        # be the source of the mouse drag.
        self.uav_map_canvas.set_draw_enabled(enabled)
        self.ugv_map_canvas.set_draw_enabled(enabled)

    def _build_input_tab(self) -> QWidget:
        widget = QWidget()
        outer = QVBoxLayout(widget)

        target_group = QGroupBox('目标描述与线索')
        target_layout = QVBoxLayout(target_group)
        self.current_query_label = QPlainTextEdit()
        self.current_query_label.setReadOnly(True)
        self.current_query_label.setMaximumBlockCount(200)
        self.current_query_label.setFixedHeight(110)
        target_layout.addWidget(QLabel('当前有效目标描述：'))
        target_layout.addWidget(self.current_query_label)
        self.target_mode = QComboBox()
        self.target_mode.addItem('替换当前目标描述', 'replace')
        self.target_mode.addItem('追加目标线索', 'append_hint')
        self.target_text = QTextEdit()
        self.target_text.setPlaceholderText('例如：搜索蓝色低矮箱体；或：目标可能位于西侧走廊尽头。')
        self.target_text.setFixedHeight(95)
        self.color_field = QLabel('颜色/形状属性可选：使用下方文本框中的逗号分隔。')
        self.colors = QTextEdit()
        self.colors.setPlaceholderText('颜色，例如：red, blue')
        self.colors.setFixedHeight(44)
        self.shapes = QTextEdit()
        self.shapes.setPlaceholderText('形状，例如：box, cylinder, irregular')
        self.shapes.setFixedHeight(44)
        send_target = QPushButton('提交目标描述/线索')
        send_target.clicked.connect(self._submit_target_instruction)
        target_layout.addWidget(self.target_mode)
        target_layout.addWidget(self.target_text)
        target_layout.addWidget(self.color_field)
        target_layout.addWidget(self.colors)
        target_layout.addWidget(self.shapes)
        target_layout.addWidget(send_target)
        outer.addWidget(target_group)

        region_group = QGroupBox('优先探索区域')
        region_layout = QGridLayout(region_group)
        self.draw_region_btn = QPushButton('开始在任一地图绘制矩形区域')
        self.draw_region_btn.setCheckable(True)
        self.draw_region_btn.toggled.connect(self._set_region_draw_mode)
        self.region_priority = QDoubleSpinBox()
        self.region_priority.setRange(0.0, 1.0)
        self.region_priority.setSingleStep(0.05)
        self.region_priority.setValue(0.80)
        self.region_mode = QComboBox()
        self.region_mode.addItem('软偏好：提高候选优先级', 'soft')
        self.region_mode.addItem('强偏好：VLM 应优先覆盖', 'hard')
        self.region_max_robots = QSpinBox()
        self.region_max_robots.setRange(1, 10)
        self.region_max_robots.setValue(1)
        self.region_ttl = QDoubleSpinBox()
        self.region_ttl.setRange(0.0, 36000.0)
        self.region_ttl.setDecimals(0)
        self.region_ttl.setSuffix(' s；0=持续有效')
        self.region_ttl.setValue(0.0)
        self.region_note = QTextEdit()
        self.region_note.setPlaceholderText('区域备注，例如：人类判断该区域更可能出现目标。')
        self.region_note.setFixedHeight(58)
        region_layout.addWidget(self.draw_region_btn, 0, 0, 1, 2)
        region_layout.addWidget(QLabel('优先级'), 1, 0)
        region_layout.addWidget(self.region_priority, 1, 1)
        region_layout.addWidget(QLabel('模式'), 2, 0)
        region_layout.addWidget(self.region_mode, 2, 1)
        region_layout.addWidget(QLabel('区域最大机器人数'), 3, 0)
        region_layout.addWidget(self.region_max_robots, 3, 1)
        region_layout.addWidget(QLabel('有效时长'), 4, 0)
        region_layout.addWidget(self.region_ttl, 4, 1)
        region_layout.addWidget(QLabel('备注'), 5, 0, 1, 2)
        region_layout.addWidget(self.region_note, 6, 0, 1, 2)
        outer.addWidget(region_group)

        self.region_table = QTableWidget(0, 5)
        self.region_table.setHorizontalHeaderLabels(['ID', '优先级', '模式', '最大机器人', '备注'])
        self.region_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.region_table.setSelectionBehavior(QTableWidget.SelectRows)
        outer.addWidget(QLabel('当前优先区域：'))
        outer.addWidget(self.region_table, 1)
        row_buttons = QHBoxLayout()
        remove_btn = QPushButton('删除选中区域')
        remove_btn.clicked.connect(self._remove_selected_region)
        clear_btn = QPushButton('清除全部区域')
        clear_btn.clicked.connect(self._clear_regions)
        replan_btn = QPushButton('立即重新规划（同时刷新局部 VLM）')
        replan_btn.clicked.connect(lambda: self.bridge.force_replan(True))
        row_buttons.addWidget(remove_btn)
        row_buttons.addWidget(clear_btn)
        row_buttons.addWidget(replan_btn)
        outer.addLayout(row_buttons)
        return widget

    def _build_visual_tab(self) -> QWidget:
        tabs = QTabWidget()
        for robot in self.bridge.robots:
            robot_id = str(robot.get('name', ''))
            if not robot_id:
                continue
            page = QWidget()
            grid = QGridLayout(page)
            raw = ImagePanel('%s 原始视觉图像' % robot_id)
            debug = ImagePanel('%s Local VLM Debug Image' % robot_id)
            self.image_panels[(robot_id, 'rgb')] = raw
            self.image_panels[(robot_id, 'debug')] = debug
            grid.addWidget(raw, 0, 0)
            grid.addWidget(debug, 0, 1)
            report = QPlainTextEdit()
            report.setReadOnly(True)
            report.setObjectName('report_' + robot_id)
            report.setMinimumHeight(145)
            _configure_readable_text_editor(report, font_size=10)
            grid.addWidget(report, 1, 0, 1, 2)
            tabs.addTab(page, robot_id)
        return tabs

    def _build_feedback_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        self.status_text = QPlainTextEdit()
        self.status_text.setReadOnly(True)
        self.status_text.setFixedHeight(100)
        _configure_readable_text_editor(self.status_text, font_size=10)
        self.assignment_table = QTableWidget(0, 6)
        self.assignment_table.setHorizontalHeaderLabels(
            ['机器人', '任务', '候选点', '目标点', '人类区域', '解释']
        )
        self.assignment_table.setWordWrap(True)
        self.assignment_table.verticalHeader().setDefaultSectionSize(52)
        self.assignment_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch
        )
        self.plan_text = QPlainTextEdit()
        self.plan_text.setReadOnly(True)
        self.plan_text.setMaximumBlockCount(1500)
        _configure_readable_text_editor(self.plan_text, font_size=10)
        layout.addWidget(QLabel('系统状态：'))
        layout.addWidget(self.status_text)
        layout.addWidget(QLabel('当前验证后任务与目标点：'))
        layout.addWidget(self.assignment_table, 1)
        layout.addWidget(QLabel('最近中央 VLM 计划：'))
        layout.addWidget(self.plan_text, 2)
        return page

    def _wire_ros(self) -> None:
        self.uav_map_canvas.region_drawn.connect(self._region_drawn)
        self.ugv_map_canvas.region_drawn.connect(self._region_drawn)
        self.bridge.map_received.connect(self._update_map)
        self.bridge.pose_received.connect(self.uav_map_canvas.set_pose)
        self.bridge.pose_received.connect(self.ugv_map_canvas.set_pose)
        self.bridge.path_received.connect(self.uav_map_canvas.set_path)
        self.bridge.path_received.connect(self.ugv_map_canvas.set_path)
        self.bridge.image_received.connect(self._update_image)
        self.bridge.context_received.connect(self._update_context)
        self.bridge.feedback_received.connect(self._update_feedback)
        self.bridge.status_received.connect(lambda text: self.statusBar().showMessage(text))

    def _update_map(self, map_key: str, msg: OccupancyGrid) -> None:
        key = str(map_key).strip().lower()
        if key == 'uav':
            self.uav_map_canvas.set_map(msg)
        elif key == 'ugv':
            self.ugv_map_canvas.set_map(msg)

    def _region_drawn(self, polygon: List[Dict[str, float]]) -> None:
        payload = {
            'region_id': 'H_%s' % uuid.uuid4().hex[:6],
            'polygon': polygon,
            'priority': float(self.region_priority.value()),
            'mode': self.region_mode.currentData(),
            'max_robots': int(self.region_max_robots.value()),
            'ttl_sec': float(self.region_ttl.value()),
            'operator_note': self.region_note.toPlainText().strip(),
        }
        self.bridge.set_region(payload)
        self.draw_region_btn.setChecked(False)

    def _submit_target_instruction(self) -> None:
        text = self.target_text.toPlainText().strip()
        mode = str(self.target_mode.currentData())
        if not text:
            QMessageBox.warning(self, '缺少输入', '请输入目标描述或线索。')
            return
        colors = [item.strip() for item in self.colors.toPlainText().split(',') if item.strip()]
        shapes = [item.strip() for item in self.shapes.toPlainText().split(',') if item.strip()]
        payload: Dict[str, Any] = {
            'mode': mode,
            'target_attributes': {'colors': colors, 'shapes': shapes} if colors or shapes else {},
        }
        if mode == 'replace':
            payload['query_text'] = text
        else:
            payload['hint_text'] = text
        self.bridge.set_target_instruction(payload)
        self.target_text.clear()

    def _remove_selected_region(self) -> None:
        rows = self.region_table.selectionModel().selectedRows()
        if not rows:
            return
        item = self.region_table.item(rows[0].row(), 0)
        if item:
            self.bridge.remove_region(item.text())

    def _clear_regions(self) -> None:
        self.bridge.clear_regions()

    def _update_image(self, robot_id: str, kind: str, image: QImage) -> None:
        panel = self.image_panels.get((robot_id, kind))
        if panel is not None:
            panel.set_image(image)

    def _update_context(self, context: Dict[str, Any]) -> None:
        self.current_context = context
        self.uav_map_canvas.set_context(context)
        self.ugv_map_canvas.set_context(context)
        target_query = context.get('target_query', {})
        self.current_query_label.setPlainText(_pretty({
            'query_id': target_query.get('query_id'),
            'query_version': target_query.get('query_version'),
            'query_text': target_query.get('query_text', ''),
            'target_attributes': target_query.get('target_attributes', {}),
            'human_hints': target_query.get('human_hints', []),
        }))
        regions = context.get('priority_regions', [])
        self.region_table.setRowCount(0)
        if isinstance(regions, list):
            for region in regions:
                if not isinstance(region, dict):
                    continue
                row = self.region_table.rowCount()
                self.region_table.insertRow(row)
                values = [
                    str(region.get('region_id', '')),
                    '%.2f' % float(region.get('priority', 0.0)),
                    str(region.get('mode', 'soft')),
                    str(region.get('max_robots', 1)),
                    str(region.get('operator_note', '')),
                ]
                for col, value in enumerate(values):
                    self.region_table.setItem(row, col, QTableWidgetItem(value))

    def _update_feedback(self, feedback: Dict[str, Any]) -> None:
        self.current_feedback = feedback
        sync = str(feedback.get('sync_status', ''))
        trigger = str(feedback.get('trigger_status', ''))
        self.status_text.setPlainText('同步协调器：%s\n触发调度器：%s\n有效人机上下文版本：%s' % (
            sync, trigger, feedback.get('context_version', '?')))
        assignments = feedback.get('current_assignments', [])
        self.assignment_table.setRowCount(0)
        if isinstance(assignments, list):
            for assignment in assignments:
                if not isinstance(assignment, dict):
                    continue
                row = self.assignment_table.rowCount()
                self.assignment_table.insertRow(row)
                metadata = assignment.get('candidate_metadata', {}) if isinstance(assignment.get('candidate_metadata'), dict) else {}
                regions = metadata.get('human_priority_regions', [])
                region_text = ', '.join(str(item.get('region_id', '')) for item in regions if isinstance(item, dict))
                goal = assignment.get('goal', {}) if isinstance(assignment.get('goal'), dict) else {}
                goal_text = '(%.2f, %.2f)' % (float(goal.get('x', 0.0)), float(goal.get('y', 0.0))) if goal else '-'
                values = [
                    str(assignment.get('robot_id', '')),
                    str(assignment.get('task_type', '')),
                    str(assignment.get('candidate_id', '')),
                    goal_text,
                    region_text or '-',
                    str(assignment.get('explanation', '')),
                ]
                for col, value in enumerate(values):
                    item = QTableWidgetItem(value)
                    item.setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                    self.assignment_table.setItem(row, col, item)

                self.assignment_table.resizeRowToContents(row)
                
                robot_id = str(assignment.get('robot_id', ''))
                if robot_id and goal:
                    self.uav_map_canvas.set_goal(robot_id, goal)
                    self.ugv_map_canvas.set_goal(robot_id, goal)

        central_plan = feedback.get('latest_central_plan', {})
        self.plan_text.setPlainText(_pretty(central_plan))
        overlay = feedback.get('semantic_overlay', {})
        self.uav_map_canvas.set_semantic_overlay(overlay)
        self.ugv_map_canvas.set_semantic_overlay(overlay)
        reports = feedback.get('latest_local_reports', [])
        if isinstance(reports, list):
            for report in reports:
                if not isinstance(report, dict):
                    continue
                robot_id = str(report.get('robot_id', ''))
                editor = self.findChild(QPlainTextEdit, 'report_' + robot_id)
                if editor is not None:
                    editor.setPlainText(_pretty(report))

    def closeEvent(self, event) -> None:  # noqa: N802
        try:
            rospy.signal_shutdown('dashboard closed')
        except Exception:
            pass
        super().closeEvent(event)


def main() -> None:
    app = QApplication(sys.argv)
    bridge = RosBridge()
    window = Dashboard(bridge)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()