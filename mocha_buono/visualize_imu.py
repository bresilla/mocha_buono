import pygame
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.executors import MultiThreadedExecutor


class Window:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.video_flags = OPENGL | DOUBLEBUF
        pygame.init()
        self.screen = pygame.display.set_mode((self.width, self.height), self.video_flags)
        pygame.display.set_caption("orientation visualization")
        self.resizewin()
        self.init()
    
    def resizewin(self):
        """
        For resizing window
        """
        if self.height == 0:
            self.height = 1
        glViewport(0, 0, self.width, self.height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, 1.0*self.width/self.height, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def init(self):
        glShadeModel(GL_SMOOTH)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

class Cube:
    def __init__(self, width, height):
        self.window = Window(width, height)

    def quat_to_ypr(self, q):
        yaw   = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
        pitch = -math.asin(2.0 * (q[1] * q[3] - q[0] * q[2]))
        roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
        pitch *= 180.0 / math.pi
        yaw   *= 180.0 / math.pi
        # yaw   -= -0.13  # Declination at Chandrapur, Maharashtra is - 0 degress 13 min
        roll  *= 180.0 / math.pi
        return [yaw, pitch, roll]

    def draw(self, w, nx, ny, nz):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0, 0.0, -7.0)

        self.drawText((-2.6, 1.8, 2), "VIZI", 18)
        self.drawText((-2.6, -2, 2), "Press Escape to exit.", 16)
        [yaw, pitch , roll] = self.quat_to_ypr([w, nx, ny, nz])
        self.drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
        glRotatef(2 * math.acos(w) * 180.00/math.pi, -1 * nx, nz, ny)
        glBegin(GL_QUADS)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(1.0, 0.2, 1.0)

        glColor3f(1.0, 0.5, 0.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(1.0, -0.2, -1.0)

        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)

        glColor3f(1.0, 1.0, 0.0)
        glVertex3f(1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, -1.0)

        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, 1.0)

        glColor3f(1.0, 0.0, 1.0)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, -1.0)
        glEnd()

    def drawText(self, position, textString, size):
        font = pygame.font.SysFont("Courier", size, True)
        textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
        textData = pygame.image.tostring(textSurface, "RGBA", True)
        glRasterPos3d(*position)
        glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription( Imu, '/imu/data', self.listener_callback, 10)
        self.subscription

        self.cube = Cube(640, 480)


    def listener_callback(self, msg):
        w = msg.orientation.w
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        self.cube.draw(w, x, y, z)
        pygame.display.flip()


def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()