import sys
from PyQt6.QtWidgets import (QApplication, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem,QSizePolicy, QGraphicsItem,QGraphicsPathItem,QGraphicsLineItem, QGraphicsTextItem, QDialog, QFormLayout, QLineEdit, QComboBox, QPushButton,QToolBar, QMainWindow,QWidget,QHBoxLayout,QMenu,QLabel,QDialogButtonBox,QInputDialog,QFileDialog,QVBoxLayout,QGroupBox,QMessageBox, QCheckBox)
from PyQt6.QtGui import QPen, QBrush, QColor, QPainter,QAction,QPainterPath
from PyQt6.QtCore import Qt, QPointF, QLineF,QPoint,QTimer
import math
import json
import numpy as np
import matplotlib.pyplot as plt
from path_finder.path_finder import PathFinder,Obstacle,Robot,TagNode,TagMap,Graph




ROLE_COLORS = {"depot": QColor("red"), "station": QColor("green"), "viapoint": QColor("grey")}


class NodeItem(QGraphicsEllipseItem):
    def __init__(self, tag_id, name, pose, role, radius=20):
        super().__init__(-radius, -radius, 2*radius, 2*radius)
        self.tag_id = tag_id
        self.name = name
        self.pose = pose
        self.role = role
        self.radius = radius
        self.properties_widget = None
        self.setBrush(QBrush(ROLE_COLORS.get(role, QColor("skyblue"))))
        self.setFlags(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable |
                      QGraphicsItem.GraphicsItemFlag.ItemIsMovable)
        self.setPos(pose[0], pose[1])
        self.text = QGraphicsTextItem(name, self)
        self.text.setDefaultTextColor(Qt.GlobalColor.black)
        self.text.setPos(-radius, -radius-15)
        self.arrow = QGraphicsLineItem(self)
        self.setZValue(10)
        self.update_arrow()

    def update_properties(self, tag_id, name, pose, role):
        self.tag_id = tag_id
        self.name = name
        self.pose = pose
        self.role = role
        self.setPos(pose[0], pose[1])
        self.text.setPlainText(name)
        self.setBrush(QBrush(ROLE_COLORS.get(role, QColor("skyblue"))))
        self.update_arrow()

    def update_arrow(self):
        angle = self.pose[2] if self.pose else 0
        length = self.radius
        x2 = length * math.cos(math.radians(angle))
        y2 = length * math.sin(math.radians(angle))
        self.arrow.setLine(0, 0, x2, y2)
        self.arrow.setPen(QPen(Qt.GlobalColor.black, 2))

    def itemChange(self, change, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            current_x, current_y = value.x(), value.y()
            self.pose = (current_x, current_y, self.pose[2])
            if self.properties_widget and self.properties_widget.current_node == self:
                self.properties_widget.update_fields_from_node()
            
        return super().itemChange(change, value)
 
class IntervalPoint(QGraphicsEllipseItem):
    def __init__(self, x, y, parent_edge, radius=5):
        super().__init__(-radius, -radius, 2*radius, 2*radius)
        
        self.setPos(x, y)
        self.setBrush(QBrush(Qt.GlobalColor.yellow))
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsScenePositionChanges, True)
        self.setZValue(20)

        self.pt_pos = (x,y)
        self.parent_edge = parent_edge

    def itemChange(self, change, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            if self.parent_edge:
                self.parent_edge.update_edge()
        return super().itemChange(change, value)


class EdgeItem(QGraphicsPathItem):
    def __init__(self, node_from, node_to, out_angle=0, in_angle=0):
        super().__init__()
        self.robot_properties = RobotProperties()
        self.path_finder = PathFinder(Obstacle,self.robot_properties.robot,TagMap)
        self.node_from = node_from
        self.node_to = node_to
        self.out_angle = out_angle
        self.in_angle = in_angle
        self.out_offset = 0.0
        self.in_offset = 0.0
        self.interval_points = []
        self.pre_start,self.pre_end = self.node_from.pos(),self.node_to.pos()
        self.is_one_way = True
        self.path_type = "auto" 
        self.setZValue(5)
        self.setPen(QPen(QColor("green"), 2))
        self.setFlags(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)
        self.update_edge()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            for item in self.scene().items():
                if isinstance(item, EdgeItem) and item is not self:
                    item.setPen(QPen(QColor("green"), 2))
            self.setPen(QPen(QColor("red"), 2))
        elif event.button() == Qt.MouseButton.RightButton:
            menu = QMenu()
            angle_action = menu.addAction("Out/In Angles")
            offset_action = menu.addAction("Out/In Offset")
            interval_action = menu.addAction("Set Interval Points")
            dir_menu = menu.addMenu("Direction")
            dir_one_direct = dir_menu.addAction("one-way")
            dir_two_direct = dir_menu.addAction("two-way")

            path_menu = menu.addMenu("Path types")
            path_custom = path_menu.addAction("Custom")
            path_auto = path_menu.addAction("Auto")
            path_dubin = path_menu.addAction("Dubin")
            
            delete_action = menu.addAction("Delete Edge") 

            action = menu.exec(event.screenPos())
            if action == angle_action:
                dialog = EdgeEditDialog(self,"angle")
                dialog.exec()

            elif action == offset_action:
                dialog = EdgeEditDialog(self,"offset")
                dialog.exec()
                
            elif action == interval_action and self.path_type == "custom":
                n, ok = QInputDialog.getInt(None, "Interval Points", "Enter number of points:", 1, 1, 100)
                if ok:
                    self.set_interval_points(n)
            elif action == dir_one_direct:
                self.is_one_way = True
                self.update_edge()
            elif action == dir_two_direct:
                self.is_one_way = False
                self.update_edge()
          
            elif action == path_custom:
                self.path_type = "custom"
                self.update_edge()

            elif action == path_auto:
                self.path_type = "auto"
                self.update_edge()
            
            elif action == path_dubin:
                self.path_type = "dubin"
                self.update_edge()

            elif action == delete_action:
                scene = self.scene()
                for pt in self.interval_points:
                    scene.removeItem(pt)
                if self in self.scene().views()[0].edges:
                    self.scene().views()[0].edges.remove(self)

                scene.removeItem(self)
                self.interval_points = []


        super().mousePressEvent(event)

    def set_interval_points(self, n):
        for pt in self.interval_points:
            self.scene().removeItem(pt)
        self.interval_points = []

        if n < 1:
            self.update_edge()
            return

        x1, y1 = self.node_from.pos().x(), self.node_from.pos().y()
        x2, y2 = self.node_to.pos().x(), self.node_to.pos().y()

        for i in range(1, n+1):
            t = i / (n+1)
            xi = x1 + t*(x2 - x1)
            yi = y1 + t*(y2 - y1)
            pt = IntervalPoint(xi, yi, parent_edge=self)
            self.scene().addItem(pt)
            self.interval_points.append(pt)

        self.update_edge()

    def sigmoid_s(self,t):
        return 1 / (1 + np.exp(-12*(t-0.5)))  # steepness adjustable

    def generate_s_shaped_path(self,start, goal, n_points=40):
        x0, y0, theta0_deg = start
        x1, y1, theta1_deg = goal
 
        theta0 = np.deg2rad(theta0_deg)
        theta1 = np.deg2rad(theta1_deg)
        
        dist = np.hypot(x1 - x0, y1 - y0)
        m0 = dist * 0.5
        m1 = dist * 0.5

        v0 = np.array([m0 * np.cos(theta0), m0 * np.sin(theta0)])
        v1 = np.array([m1 * np.cos(theta1), m1 * np.sin(theta1)])
        
        t = np.linspace(0, 1, n_points)
        t_s = self.sigmoid_s(t)
        
        h00 = 2*t_s**3 - 3*t_s**2 + 1
        h10 = t_s**3 - 2*t_s**2 + t_s
        h01 = -2*t_s**3 + 3*t_s**2
        h11 = t_s**3 - t_s**2
        
        x = h00*x0 + h10*v0[0] + h01*x1 + h11*v1[0]
        y = h00*y0 + h10*v0[1] + h01*y1 + h11*v1[1]
        
        return x, y

    def offset_start(self,length:float,pose:tuple,angle)->tuple:
        x,y = pose[0],pose[1]
        angle_rad = math.radians(angle)
        pose_ctrl = (x + length * math.cos(angle_rad),y + length * math.sin(angle_rad),angle)

        return pose_ctrl
    
    def offset_end(self,length:float,pose:tuple,angle)->tuple:
        x,y = pose[0],pose[1]
        angle_rad = math.radians(angle)
        pose_ctrl = (x - length * math.cos(angle_rad), y - length * math.sin(angle_rad),angle)
        return pose_ctrl
    
    def update_edge(self):
        path = QPainterPath()
        start = self.node_from.pos()
        end = self.node_to.pos()
        pts = []
        if self.path_type == "custom":
            if self.is_one_way == True or self.is_one_way == False :
                angle_rad = math.radians(self.out_angle)
                start_ctrl = QPointF(start.x() + self.out_offset * math.cos(angle_rad),
                                    start.y() + self.out_offset * math.sin(angle_rad))
                pts.append(start)
                pts.append(start_ctrl)
            for p in self.interval_points:
                pts.append(p.pos())
            
            if self.is_one_way == True or self.is_one_way == False :
                angle_rad2 = math.radians(self.in_angle)
                end_ctrl = QPointF(end.x() - self.in_offset * math.cos(angle_rad2),
                                end.y() - self.in_offset * math.sin(angle_rad2))
                pts.append(end_ctrl)
                pts.append(end)
   
        elif self.path_type == "auto":
            start_pos = (float(start.x()), float(start.y()), float(self.out_angle))
            end_pos = (float(end.x()), float(end.y()), float(self.in_angle))
            start_ctrl = self.offset_start(self.out_offset,(start_pos[0],start_pos[1]),self.out_angle)
            end_ctrl = self.offset_end(self.in_offset,(end_pos[0],end_pos[1]),self.in_angle)
            x,y = self.generate_s_shaped_path(start_ctrl,end_ctrl)
            scene = self.scene()
            for pt in self.interval_points:
                scene.removeItem(pt)
            pt_start = QPointF(start_pos[0],start_pos[1])
            pts.append(pt_start)
            for i in range(len(x)):
                pts.append(QPointF(x[i],y[i]))
            pts.append(QPointF(end_pos[0],end_pos[1]))

        elif self.path_type == "dubin":
            start_pos = [float(start.x()), float(start.y()), float(self.out_angle)]
            end_pos = [float(end.x()), float(end.y()), float(self.in_angle)]
            start_ctrl = self.offset_start(self.out_offset,(start_pos[0],start_pos[1]),self.out_angle)
            end_ctrl = self.offset_end(self.in_offset,(end_pos[0],end_pos[1]),self.in_angle)
            start_ctrl = (start_ctrl[0],start_ctrl[1],start_ctrl[2])
            end_ctrl = (end_ctrl[0],end_ctrl[1],end_ctrl[2])
            path_list, path_type, lengthh = self.path_finder.dubins_path_planning(start_ctrl, end_ctrl,self.robot_properties.turning_radius,step_size=1.0)
            scene = self.scene()
            for pt in self.interval_points:
                scene.removeItem(pt)
            pt_start = QPointF(start_pos[0],start_pos[1])
            pts.append(pt_start)
            for point in path_list:
                pts.append(QPointF(point[0],point[1]))
            pts.append(QPointF(end_pos[0],end_pos[1]))

        try:
            path.moveTo(pts[0])
            for pt in pts[1:]:
                path.lineTo(pt)
            self.setPath(path)
        except Exception as e:
            print(e)

    def set_angles(self, out_angle, in_angle):
        self.out_angle = out_angle
        self.in_angle = in_angle
        self.update_edge()

    def set_offset(self,out_offset,in_offset):
        self.out_offset = out_offset
        self.in_offset  = in_offset
        self.update_edge()

class EdgeEditDialog(QDialog):
    def __init__(self, edge: EdgeItem, type: str):
        super().__init__()
        self.edge = edge

        if type == "angle":
            self.setWindowTitle("Edit Edge Angles")
            self.layout = QFormLayout()
            self.angle_choice = ["0", "90", "180", "270"]
            self.out_input = QComboBox(self)
            self.out_input.addItems(self.angle_choice)
            self.in_input = QComboBox(self)
            self.in_input.addItems(self.angle_choice)

            self.layout.addRow("Node1 Out Angle:", self.out_input)
            self.layout.addRow("Node2 In Angle:", self.in_input)

            self.btns = QDialogButtonBox(
                QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
            )
            self.btns.accepted.connect(self.apply_angle)
            self.btns.rejected.connect(self.reject)
            self.layout.addWidget(self.btns)
            self.setLayout(self.layout)

        elif type == "offset":
            self.setWindowTitle("Edit out/in offset")
            self.layout = QFormLayout()

            self.out_input_offset = QLineEdit(str(edge.out_offset))
            self.in_input_offset = QLineEdit(str(edge.in_offset))
            self.fixed_input_checkbox = QCheckBox("Use fixed offset (both)")
            self.fixed_offset_edit = QLineEdit()
            self.fixed_offset_edit.setEnabled(False)  # disabled until checked

            self.layout.addRow("Node1 Out Offset:", self.out_input_offset)
            self.layout.addRow("Node2 In Offset:", self.in_input_offset)
            self.layout.addRow("", self.fixed_input_checkbox)
            self.layout.addRow("Fixed offset value:", self.fixed_offset_edit)

            self.fixed_input_checkbox.stateChanged.connect(self.toggle_offset_mode)

            self.btns = QDialogButtonBox(
                QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
            )
            self.btns.accepted.connect(self.apply_offset)
            self.btns.rejected.connect(self.reject)
            self.layout.addWidget(self.btns)
            self.setLayout(self.layout)

    def toggle_offset_mode(self, state):
        if state == 2:
            is_checked = state == 2  # 0 = unchecked, 2 = checked
            self.out_input_offset.setEnabled(not is_checked)
            self.in_input_offset.setEnabled(not is_checked)
            self.fixed_offset_edit.setEnabled(is_checked)
        elif state == 0:
            is_checked = state == 0  # 0 = unchecked, 2 = checked
            self.out_input_offset.setEnabled(is_checked)
            self.in_input_offset.setEnabled(is_checked)
            self.fixed_offset_edit.setEnabled(not is_checked)

    def apply_angle(self):
        try:
            self.edge.set_angles(
                float(self.out_input.currentText()), float(self.in_input.currentText())
            )
            self.accept()
        except ValueError:
            pass

    def apply_offset(self):
        try:
            if self.fixed_input_checkbox.isChecked():
                fixed_val = float(self.fixed_offset_edit.text())
                self.edge.set_offset(fixed_val, fixed_val)
            else:
                out_val = float(self.out_input_offset.text())
                in_val = float(self.in_input_offset.text())
                self.edge.set_offset(out_val, in_val)
            self.accept()
        except ValueError:
            pass


class NodeProperties(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QFormLayout()
        self.tag_input = QLineEdit()
        self.name_input = QLineEdit()
        self.x_input = QLineEdit()
        self.y_input = QLineEdit()
        self.yaw_input = QLineEdit()
        self.role_input = QComboBox()
        self.role_input.addItems(["depot", "station", "viapoint"])
        self.out_angle_input = QLineEdit()
        self.in_angle_input = QLineEdit()
        self.apply_btn = QPushButton("Apply")
        self.delete_btn = QPushButton("🗑️")
        self.delete_btn.setToolTip('Delete Node')
        self.apply_btn.clicked.connect(self.apply_changes)
        self.delete_btn.clicked.connect(self.delete_node)
        self.layout.addRow("Tag ID:", self.tag_input)
        self.layout.addRow("Name:", self.name_input)
        self.layout.addRow("X:", self.x_input)
        self.layout.addRow("Y:", self.y_input)
        self.layout.addRow("Yaw:", self.yaw_input)
        self.layout.addRow("Role:", self.role_input)
        self.layout.addRow("Out Angle:", self.out_angle_input)
        self.layout.addRow("In Angle:", self.in_angle_input)
        self.layout.addWidget(self.apply_btn)
        self.layout.addWidget(self.delete_btn)
        self.setLayout(self.layout)
        self.current_node = None
        self.current_edge = None
        self.view = None


    def set_node(self, node: NodeItem):
        self.current_node = node
        self.current_edge = None
        if node:
            self.tag_input.setText(str(node.tag_id))
            self.name_input.setText(node.name)
            self.x_input.setText(str(node.pose[0])[:9])
            self.y_input.setText(str(node.pose[1])[:9])
            self.yaw_input.setText(str(node.pose[2])[:9])
            self.role_input.setCurrentText(node.role)
        else:
            self.tag_input.clear()
            self.name_input.clear()
            self.x_input.clear()
            self.y_input.clear()
            self.yaw_input.clear()
            self.role_input.setCurrentIndex(0)
        self.out_angle_input.clear()
        self.in_angle_input.clear()

    def set_edge(self, edge: EdgeItem):
        self.current_edge = edge
        self.current_node = None
        if edge:
            self.out_angle_input.setText(str(edge.out_angle))
            self.in_angle_input.setText(str(edge.in_angle))
        else:
            self.out_angle_input.clear()
            self.in_angle_input.clear()

    def apply_changes(self):
        if self.current_node:
            tag_id = int(self.tag_input.text())
            name = self.name_input.text()
            pose = (float(self.x_input.text()), float(self.y_input.text()), float(self.yaw_input.text()))
            role = self.role_input.currentText()
            self.current_node.update_properties(tag_id, name, pose, role)
        elif self.current_edge:
            out_angle = float(self.out_angle_input.text())
            in_angle = float(self.in_angle_input.text())
            self.current_edge.set_angles(out_angle, in_angle)

    def delete_node(self):
        if self.current_node and self.view:
            self.view.delete_node(self.current_node)
            self.set_node(None)


class ConfigDialog(QDialog):
    def __init__(self, parent, origin_x, origin_y, default_out_offset=0.0, default_in_offset=0.0):
        super().__init__(parent)
        self.setWindowTitle("Configuration")
        self.layout = QFormLayout()

        # Origin settings
        self.origin_x_input = QLineEdit(str(origin_x))
        self.origin_y_input = QLineEdit(str(origin_y))
        self.layout.addRow("Origin X:", self.origin_x_input)
        self.layout.addRow("Origin Y:", self.origin_y_input)

        # Default offsets for all edges
        self.default_out_offset_input = QLineEdit(str(default_out_offset))
        self.default_in_offset_input = QLineEdit(str(default_in_offset))
        self.layout.addRow("Default Out Offset:", self.default_out_offset_input)
        self.layout.addRow("Default In Offset:", self.default_in_offset_input)

        # Buttons
        self.buttons = QDialogButtonBox( QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        self.buttons.accepted.connect(self.accept)
        self.buttons.rejected.connect(self.reject)
        self.layout.addWidget(self.buttons)
        self.setLayout(self.layout)

    def get_origin(self):
        try:
            x = float(self.origin_x_input.text())
            y = float(self.origin_y_input.text())
            return x, y
        except ValueError:
            return None, None

    def get_default_offsets(self):
        try:
            out_val = float(self.default_out_offset_input.text())
            in_val = float(self.default_in_offset_input.text())
            return out_val, in_val
        except ValueError:
            return None, None

class GraphicsView(QGraphicsView):
    def __init__(self, scene, properties_widget: NodeProperties, main_window):
        super().__init__(scene)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.zoom = 1.0
        self.mode = 'draw'
        self.grid_size = 50
        self.origin = QPointF(0, 0)
        self.temp_node = None
        self.start_pos = None
        self.setMouseTracking(True)
        self.properties_widget = properties_widget
        self.main_window = main_window
        self.node_stack = []
        self.edge_mode_nodes = []
        self.edges = []
        self.nodes = []
        self.scene().setSceneRect(-10000, -10000, 20000, 20000)
        self.initial_click = False
        self._is_panning = False
        self._pan_start = QPoint()

        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)
        self.setDragMode(QGraphicsView.DragMode.NoDrag)
                


    def drawBackground(self, painter, rect):
        left = int(rect.left()) - int(rect.left()) % self.grid_size
        top = int(rect.top()) - int(rect.top()) % self.grid_size
        pen = QPen(QColor(200, 200, 200))
        painter.setPen(pen)
        x = left
        while x < rect.right():
            painter.drawLine(QLineF(x, rect.top(), x, rect.bottom()))
            x += self.grid_size
        y = top
        while y < rect.bottom():
            painter.drawLine(QLineF(rect.left(), y, rect.right(), y))
            y += self.grid_size
        origin_pen = QPen(QColor(255, 0, 0))
        painter.setPen(origin_pen)
        painter.drawLine(QLineF(self.origin.x(), rect.top(), self.origin.x(), rect.bottom()))
        painter.drawLine(QLineF(rect.left(), self.origin.y(), rect.right(), self.origin.y()))

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        factor = 1.25 if delta > 0 else 0.8
        self.zoom *= factor
        self.scale(factor, factor)

    def contextMenuEvent(self, event):
        scene_pos = self.mapToScene(event.pos())
        items = self.scene().items(scene_pos)
        for item in items:
            if isinstance(item, NodeItem):
                menu = QMenu()
                edit_action = QAction("Edit Node")
                delete_action = QAction("Delete Node")
                edit_action.triggered.connect(lambda i=item: self.properties_widget.set_node(i))
                delete_action.triggered.connect(lambda i=item: self.delete_node(i))
                menu.addAction(edit_action)
                menu.addAction(delete_action)
                menu.exec(event.globalPos())
                break

    def delete_node(self, node):
        for edge in list(self.edges):
            if edge.node_from == node or edge.node_to == node:
                for pt in edge.interval_points:
                    self.scene().removeItem(pt)
                self.scene().removeItem(edge)
                if edge in self.edges:
                    self.edges.remove(edge)
        if node in self.nodes:
            self.nodes.remove(node)
        if node in self.node_stack:
            self.node_stack.remove(node)

        self.scene().removeItem(node)
        self.properties_widget.set_node(None)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.MiddleButton:
            self._is_panning = True
            self._pan_start = event.pos()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            event.accept()
            return

        scene_pos = self.mapToScene(event.pos())
        clicked_items = self.scene().items(scene_pos)
        if not any(isinstance(i, EdgeItem) for i in clicked_items):
            for item in self.edges:
                item.setPen(QPen(QColor("green"), 2))

        if self.mode == 'draw' and event.button() == Qt.MouseButton.LeftButton:
            self.start_pos = scene_pos
            tag_id = len(self.nodes) + 1
            self.temp_node = NodeItem(
                tag_id, f"Node{tag_id}", (scene_pos.x(), scene_pos.y(), 0), "viapoint"
            )
            self.temp_node.properties_widget = self.properties_widget
            self.scene().addItem(self.temp_node)
            self.nodes.append(self.temp_node)

        elif self.mode == 'select':
            items = self.scene().items(scene_pos)
            for item in items:
                if isinstance(item, NodeItem):
                    self.properties_widget.set_node(item)
                    break
                elif isinstance(item, EdgeItem):
                    self.properties_widget.set_edge(item)
                    break

        elif self.mode == 'edge' and event.button() == Qt.MouseButton.LeftButton:
            for node in self.node_stack:
                node.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, False)
            items = self.scene().items(scene_pos)
            for item in items:
                if isinstance(item, NodeItem):
                    self.edge_mode_nodes.append(item)
                    if len(self.edge_mode_nodes) == 2:
                        n1, n2 = self.edge_mode_nodes
                        edge = EdgeItem(n1, n2)
                        self.scene().addItem(edge)
                        self.edges.append(edge)
                        self.properties_widget.set_edge(edge)
                        self.edge_mode_nodes = []
                    break
        else:
            for node in self.node_stack:
                node.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)

        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._is_panning:
            delta = event.pos() - self._pan_start
            self._pan_start = event.pos()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - delta.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - delta.y())
            return

        scene_pos = self.mapToScene(event.pos())
        if self.main_window and hasattr(self.main_window, "mouse_pos_label"):
            self.main_window.mouse_pos_label.setText(
                f"({scene_pos.x():.2f}, {scene_pos.y():.2f})"
            )

        if self.mode == 'draw' and self.temp_node and self.start_pos:
            dx = scene_pos.x() - self.start_pos.x()
            dy = scene_pos.y() - self.start_pos.y()
            angle = math.degrees(math.atan2(dy, dx))
            self.temp_node.pose = (self.start_pos.x(), self.start_pos.y(), angle)
            self.temp_node.setPos(self.start_pos.x(), self.start_pos.y())
            self.temp_node.update_arrow()

        scene_pos = self.mapToScene(event.pos())
        clicked_items = self.scene().items(scene_pos)

        if self.mode == 'select' and clicked_items :
            
            scene_pos = self.mapToScene(event.pos())
            items = self.scene().items(scene_pos)
            for item in items:
                if isinstance(item, NodeItem) and item.isSelected():
                    if event.buttons() == Qt.MouseButton.LeftButton:
                        if  self.initial_click == False:
                            self.dx,self.dy = scene_pos.x()-item.pose[0] , scene_pos.y()-item.pose[1]
                            self.initial_click = True
                        yaw = item.pose[2]
                        item.pose = (scene_pos.x()-self.dx, scene_pos.y()-self.dy,yaw)
                        self.properties_widget.set_node(item)
                    elif event.buttons() == Qt.MouseButton.NoButton:
                            yaw = item.pose[2]
                            item.pose = (item.pose[0], item.pose[1],yaw)
                            self.properties_widget.set_node(item)
                            self.initial_click = False
                
        for edge in self.edges:
            if hasattr(edge, "node_from") and hasattr(edge, "node_to"):
                if edge.node_from.isSelected() or edge.node_to.isSelected():
                    edge.update_edge()

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.MiddleButton:
            self._is_panning = False
            self.setCursor(Qt.CursorShape.ArrowCursor)
            event.accept()
            return

        if self.mode == 'draw' and self.temp_node:
            node = self.temp_node
            self.node_stack.append(node)
            self.properties_widget.set_node(node)
            self.temp_node = None
            self.start_pos = None

        super().mouseReleaseEvent(event)

    def undo(self):
        if self.node_stack:
            node = self.node_stack.pop()
            self.scene().removeItem(node)
            self.properties_widget.set_node(None)

    def clear_all(self):
        for node in list(self.node_stack) + list(self.nodes):
            self.scene().removeItem(node)

        for edge in list(self.edges):
            for pt in edge.interval_points:
                self.scene().removeItem(pt)
            self.scene().removeItem(edge)
        self.nodes.clear()
        self.node_stack.clear()
        self.edges.clear()
        self.properties_widget.set_node(None)
        self.properties_widget.set_edge(None)

class RobotProperties():
    def __init__(self):
        try:
            with open("config/_robot.json", "r") as f:
                self.data = json.load(f)
        except Exception as e:
            QMessageBox.warning(self, "Error", " robot's configuration file not found !")
            exit()
        
        self.connect_server = False
        self.name = self.data.get("robot_name", "agv_dummy")
        self.type = self.data.get("robot_type", "diffdrive")
        self.length = self.data.get("length", 2.0)
        self.max_speed = self.data.get("max_speed", 2.0)
        self.max_steer = self.data.get("max_steer", 45.0)
        self.current_pose = self.data.get("current_pose",[0,0,0])
        self.turning_radius = self.data.get("turning_radius", 1.0)
        self.robot = Robot(self.name,self.type,self.length,self.max_speed,self.max_steer,self.current_pose,self.turning_radius)
        

    def update_data(self):
        print("Call API")

        self.msg = QMessageBox()
        self.msg.setIcon(QMessageBox.Icon.Information)      # PyQt6
        self.msg.setWindowTitle("Status")
        self.msg.setText("Connecting to Server . . .")
        self.msg.setStandardButtons(QMessageBox.StandardButton.NoButton)  # PyQt6

        self.msg.show()
        QTimer.singleShot(3000, lambda: self.msg.accept())
        # if self.msg.dis:
        #     QMessageBox.information(None,"Status","Connected to server ✅")


    
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.robot_properties = RobotProperties()
        self.scene = QGraphicsScene(-500, -500, 1000, 1000)
        self.properties_panel = NodeProperties()
        self.select_graph = None
        self.initial_one_way = False
        self.initial_two_way = False
        self.loaded_data = {'nodes':[],'edges':[]}
        self.one_way_graph = Graph('direct')
        self.two_way_graph = Graph('bi-direct')

        self.view = GraphicsView(self.scene, self.properties_panel, self)
        self.properties_panel.view = self.view
        self.app_mode = "finder"  
        self.right_panel = QWidget()
        self.right_layout = QVBoxLayout()
        self.right_panel.setLayout(self.right_layout)
        self.right_layout.addWidget(self.properties_panel)
        self.pathfinder_group = QGroupBox("Path Finder")
        pathfinder_layout = QVBoxLayout()
        self.start_combo = QComboBox()
        self.goal_combo = QComboBox()
        self.graph_type_check_btn = QCheckBox(" 2 ways")
        self.find_path_btn = QPushButton("Find Path")
        pathfinder_layout.addWidget(QLabel("Start Node:"))
        pathfinder_layout.addWidget(self.start_combo)
        pathfinder_layout.addWidget(QLabel("Goal Node:"))
        pathfinder_layout.addWidget(self.goal_combo)
        pathfinder_layout.addWidget(self.graph_type_check_btn)
        pathfinder_layout.addWidget(self.find_path_btn)
        
        self.pathfinder_group.setLayout(pathfinder_layout)
        self.right_layout.addWidget(self.pathfinder_group)

        central = QWidget()
        layout = QHBoxLayout()
        layout.addWidget(self.view, 4)
        layout.addWidget(self.right_panel, 1)
        central.setLayout(layout)
        self.setCentralWidget(central)
        
        self.toolbar = QToolBar("Main Toolbar")
        self.addToolBar(self.toolbar)

        self.sub_toolbar = QToolBar("Sub_Toolbar")
        self.addToolBar(self.sub_toolbar)


        self.mode_label = QLabel("Mode: None")
        self.toolbar.addWidget(self.mode_label)

        self.create_graph_action = QAction("📝 Create Graph", self)
        self.create_graph_action.setCheckable(True)
        self.create_graph_action.setChecked(True)
        self.create_graph_action.triggered.connect(lambda: self.set_app_mode("create"))

        self.path_finder_action = QAction("🔍 Path Finder", self)
        self.path_finder_action.setCheckable(True)
        self.path_finder_action.triggered.connect(lambda: self.set_app_mode("finder"))

        self.toolbar.addAction(self.create_graph_action)
        self.toolbar.addAction(self.path_finder_action)

        self.create_graph_action.setChecked(True)

        self.find_path_btn.clicked.connect(self.on_find_path)

        self.highlight_items = []
        self.interval_points = [] 

        self.select_action = QAction("↖", self)
        self.select_action.triggered.connect(lambda: self.set_mode('select'))
        self.select_action.setToolTip("Select / Move")
        self.toolbar.addAction(self.select_action)
    
        self.draw_action = QAction("✏️", self)
        self.draw_action.triggered.connect(lambda: self.set_mode('draw'))
        self.draw_action.setToolTip("Draw node")
        self.toolbar.addAction(self.draw_action)

        self.edge_action = QAction("🔗", self)
        self.edge_action.triggered.connect(lambda: self.set_mode('edge'))
        self.edge_action.setToolTip("Pair edge")
        self.toolbar.addAction(self.edge_action)

        self.undo_action = QAction("↺", self)
        self.undo_action.triggered.connect(self.view.undo)
        self.undo_action.setToolTip("Undo")
        self.toolbar.addAction(self.undo_action)

        self.config_action = QAction("⚙️", self)
        self.config_action.triggered.connect(self.open_config)
        self.config_action.setToolTip("Configuration")
        self.toolbar.addAction(self.config_action)

        self.save_act = QAction("💾", self)
        self.save_act.triggered.connect(self.save_graph)
        self.save_act.setToolTip("Save to file")
        self.toolbar.addAction(self.save_act)

        self.load_act = QAction("📤", self)
        self.load_act.triggered.connect(self.load_graph)
        self.load_act.setToolTip("Upload file")
        self.toolbar.addAction(self.load_act)

        self.clear_action = QAction("🧹", self)
        self.clear_action.triggered.connect(self.view.clear_all)
        self.clear_action.setToolTip("Clear all")
        self.toolbar.addAction(self.clear_action)

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.sub_toolbar.addWidget(spacer)
        self.update_btn = QAction("🌐", self)
        self.update_btn.triggered.connect(self.robot_properties.update_data)
        self.update_btn.setToolTip("Syncing data with DataBase")
        self.sub_toolbar.addAction(self.update_btn)
       


        

 
        self.mouse_pos_label = QLabel("(0, 0)")
        self.statusBar().addPermanentWidget(self.mouse_pos_label)

        self.set_app_mode("finder")

        self.setWindowTitle("TY  -  GUI")
        self.resize(1200, 700)
        self.show()

        

    def set_app_mode(self, mode):
        self.app_mode = mode
        if mode == "create":
            self.mode_label.setText("Mode: Create Graph")
            self.create_graph_action.setChecked(True)
            self.path_finder_action.setChecked(False)
            self.properties_panel.setEnabled(True)
            self.pathfinder_group.setEnabled(False)
            self.view.setInteractive(True)
            self.toolbar.addAction(self.draw_action)
            self.toolbar.addAction(self.edge_action)
            self.toolbar.addAction(self.undo_action)
            self.view.mode = 'select'
            self.mode_label.setText("Mode: Create Graph")
        elif mode == "finder":
            self.mode_label.setText("Mode: Path Finder")
            self.create_graph_action.setChecked(False)
            self.path_finder_action.setChecked(True)
            self.properties_panel.setEnabled(False)  # still visible
            self.pathfinder_group.setEnabled(True)
            self.view.setInteractive(False)
            self.view.mode = 'select'
            self.toolbar.removeAction(self.draw_action)
            self.toolbar.removeAction(self.edge_action)
            self.toolbar.removeAction(self.undo_action)
        self.view.clear_all()

    def set_mode(self, mode):
        self.view.mode = mode
        self.mode_label.setText(f"Mode: {mode.capitalize()}")
        movable = mode != 'edge'
        for node in self.view.node_stack:
            node.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, movable)

    def open_config(self):
        dialog = ConfigDialog(self, origin_x=self.view.origin.x(), origin_y=self.view.origin.y())
        if dialog.exec() == QDialog.DialogCode.Accepted:
            x, y = dialog.get_origin()
            if x is not None and y is not None:
                self.view.origin = QPointF(x, y)

            out_off, in_off = dialog.get_default_offsets()
            if out_off is not None and in_off is not None:
                for edge in self.view.edges:
                    edge.set_offset(out_off, in_off)

    def save_graph(self):
        data = {"nodes": [], "edges": []}
        existing_nodes = set(self.view.nodes)
        for node in self.view.nodes:
            if not self.scene.items().__contains__(node):
                continue  
            data["nodes"].append({
                "tag_id": node.tag_id,
                "name": node.name,
                "pose": node.pose,
                "role": node.role
            })

        for edge in list(self.view.edges):
            if not self.scene.items().__contains__(edge):
                continue
            if edge.node_from not in existing_nodes or edge.node_to not in existing_nodes:
                if edge in self.view.edges:
                    self.view.edges.remove(edge)
                continue

            edge_data = {
                "from": edge.node_from.tag_id,
                "to": edge.node_to.tag_id,
                "out_angle": edge.out_angle,
                "in_angle": edge.in_angle,
                "out_offset": edge.out_offset,
                "in_offset": edge.in_offset,
                "is_one_way": edge.is_one_way,
                "path_type": edge.path_type,
                "interval_points": [(p.pos().x(), p.pos().y()) for p in edge.interval_points],
                "path_points": []
            }
            poly = edge.path().toSubpathPolygons()
            if poly:
                edge_data["path_points"] = [(pt.x(), pt.y()) for pt in poly[0]]
            data["edges"].append(edge_data)

        fname, _ = QFileDialog.getSaveFileName(self, "Save Graph", "", "JSON Files (*.json)")
        if fname:
            with open(fname, "w") as f:
                json.dump(data, f, indent=4)
            QMessageBox.information(self, "Notice", "File has been saved !")

    def load_graph(self):
        fname, _ = QFileDialog.getOpenFileName(self, "Load Graph", "", "JSON Files (*.json)")
        if not fname:
            return
        with open(fname, "r") as f:
            data = json.load(f)
        self.scene.clear()
        self.view.nodes.clear()
        self.view.edges.clear()
        id_to_node = {}
        self.node_ids = []
        self.edge_info = []
        for nd in data.get("nodes", []):
            try:
                node = NodeItem(nd["tag_id"], nd["name"], tuple(nd["pose"]), nd["role"])
                self.scene.addItem(node)
                self.view.nodes.append(node)
                id_to_node[nd["tag_id"]] = node
                if self.app_mode == "pathfinder":
                    node.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, False)
            except Exception as e:
                print(f"Skipping invalid node: {nd}, error={e}")
        
        for node in self.view.nodes:
            self.node_ids.append(node.tag_id)
        self.start_combo.clear()
        self.goal_combo.clear()
        self.start_combo.addItems([str(i) for i in self.node_ids])
        self.goal_combo.addItems([str(i) for i in self.node_ids])

        for ed in data.get("edges", []):
                if ed["from"] not in id_to_node or ed["to"] not in id_to_node:
                    print(f"Skipping orphan edge: {ed}")
                    continue
                self.edge_info.append({"from": ed["from"] , 'to': ed['to'] ,'out_angle' : ed['out_angle'],
                                       'in_angle': ed["in_angle"],'out_offset': ed['out_offset'],
                                       'in_offset': ed['in_offset'] ,'is_one_way': ed['is_one_way'],
                                       'path_type': ed['path_type'],'path_pts': ed["path_points"]})
                n1, n2 = id_to_node[ed["from"]], id_to_node[ed["to"]]
                edge = EdgeItem(n1, n2, ed.get("out_angle", 0), ed.get("in_angle", 0))
                edge.is_one_way = ed['is_one_way']
                edge.path_type = ed['path_type']
                edge.out_offset = ed['out_offset']
                edge.in_offset = ed['in_offset']
                self.scene.addItem(edge)
                self.view.edges.append(edge)

                if self.app_mode == "create":
                    for (x, y) in ed.get("interval_points", []):
                        pt = IntervalPoint(x, y, edge)
                        self.scene.addItem(pt)
                        edge.interval_points.append(pt)
                        self.interval_points.append((x,y))
                    edge.update_edge()

                if self.app_mode == "finder":
                    for (x, y) in ed.get("interval_points", []):
                        pt = IntervalPoint(x, y, edge)
                        edge.interval_points.append(pt)
                    for item in self.scene.items():
                        if isinstance(item, NodeItem) or isinstance(item, EdgeItem):
                            item.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, False)
                            item.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, False)
                        if isinstance(item, IntervalPoint):
                            item.setVisible(False)
                  
                    edge.update_edge()

    def clear_path_highlight(self):
        for node in self.view.nodes:
            if node.role == 'depot':
                node.setBrush(QBrush(Qt.GlobalColor.red))
            elif node.role == 'station':
                node.setBrush(QBrush(Qt.GlobalColor.green))
            else:
                node.setBrush(QBrush(Qt.GlobalColor.gray))
        for edge in self.view.edges:
            edge.setPen((QPen(QColor("green"), 2)))
        for item in self.highlight_items:
            self.scene.removeItem(item)
        self.highlight_items.clear()
                
    def on_find_path(self):
        start_id = int(self.start_combo.currentText())
        goal_id = int(self.goal_combo.currentText())
        if start_id == goal_id:
            QMessageBox.warning(self, "Warning", "start and goal nodes are same id")
            return
        if not start_id or not goal_id:
            QMessageBox.warning(self, "Warning", "Please select both start and goal nodes")
            return
        start_node = None
        goal_node = None
        for node in self.view.nodes:
            if node.tag_id == start_id:
                start_node = node
            elif node.tag_id == goal_id:
                goal_node = node
        if not start_node or not goal_node:
            QMessageBox.warning(self, "Warning", "Start or goal node not found")
            return 
        for node in self.view.nodes:
            this_node_info = {'id': node.tag_id,'name': node.name, 'pose':node.pose, 'role':node.role}
            self.loaded_data['nodes'].append(this_node_info)
        for line in self.edge_info:
            self.loaded_data['edges'].append(line)

        if self.graph_type_check_btn.isChecked():
            self.select_graph = self.two_way_graph
            if not self.initial_two_way:
                self.select_graph.initial_graph(self.loaded_data)
                self.initial_two_way = True

        elif self.graph_type_check_btn.isChecked() == False :
            self.select_graph = self.one_way_graph
            if not self.initial_one_way:
                self.select_graph.initial_graph(self.loaded_data)
                self.initial_one_way = True
        
        path_nodes, _, path_points = self.select_graph.astar_path(start_id,goal_id)
        if path_nodes == None:
            QMessageBox.warning(self, "Warning", str(_))
            return
        else: 
            self.highlight_path(path_nodes)

    def get_edge_between(self, id1, id2):
        for edge in self.view.edges:
            if (edge.node_from.tag_id == id1 and edge.node_to.tag_id == id2) or \
            (edge.node_from.tag_id == id2 and edge.node_to.tag_id == id1):
                return edge
        return None
    
    def highlight_path(self, tag_path):
        if not tag_path or len(tag_path) < 2:
            return
        self.clear_path_highlight()
        for i in range(len(tag_path) - 1):
            edge = self.get_edge_between(tag_path[i], tag_path[i+1])
            if edge:
                edge.setPen(QPen(Qt.GlobalColor.blue, 3))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec())