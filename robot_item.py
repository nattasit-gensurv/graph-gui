from PyQt6.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QApplication
from PyQt6.QtGui import QPixmap, QTransform, QPainter
from PyQt6.QtCore import QPointF, Qt
import sys
import math

class RobotItem(QGraphicsPixmapItem):
    def __init__(self, image_path, parent=None):
        pixmap = QPixmap(image_path)
        super().__init__(pixmap, parent)
        # Set center of rotation to robot center
        self.setTransformOriginPoint(self.boundingRect().center())

    def move_to(self, x, y, yaw):
        """Update robot position and rotation (yaw in degrees)"""
        self.setPos(x, y)
        self.setRotation(math.degrees(yaw))  # if yaw in radians, convert to degrees

class GraphicsView(QGraphicsView):
    def __init__(self, scene):
        super().__init__(scene)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Add robot item
        self.robot = RobotItem("car_bg.png")  # your 2D top-view robot image
        scene.addItem(self.robot)
        self.robot.setZValue(10)  # make sure it's on top

    def update_robot(self, x, y, yaw):
        self.robot.move_to(x, y, yaw)
    def set_scale(self,value):
        self.robot.setScale(value)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    scene = QGraphicsScene(-500, -500, 1000, 1000)
    view = GraphicsView(scene)
    view.show()

    # Move robot example
    view.set_scale(0.5)
    view.update_robot(100, 50, math.radians(0))  # x=100, y=50, yaw=45deg
    
    sys.exit(app.exec())
