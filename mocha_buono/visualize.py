import pygame
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
import board
import adafruit_bno055

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


# def main():
    cube = Cube(640, 480)
    sensor = adafruit_bno055.BNO055_I2C(board.I2C())
    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break
        [w, nx, ny, nz] = sensor.quaternion
        cube.draw(w, nx, ny, nz)
        pygame.display.flip()

if __name__ == '__main__':
    main()

# """
# credits to:
# https://raw.githubusercontent.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation
# """

# import pygame
# import math
# from OpenGL.GL import *
# from OpenGL.GLU import *
# from pygame.locals import *
# import board
# import adafruit_bno055

# def resizewin(width, height):
#     """
#     For resizing window
#     """
#     if height == 0:
#         height = 1
#     glViewport(0, 0, width, height)
#     glMatrixMode(GL_PROJECTION)
#     glLoadIdentity()
#     gluPerspective(45, 1.0*width/height, 0.1, 100.0)
#     glMatrixMode(GL_MODELVIEW)
#     glLoadIdentity()


# def init():
#     glShadeModel(GL_SMOOTH)
#     glClearColor(0.0, 0.0, 0.0, 0.0)
#     glClearDepth(1.0)
#     glEnable(GL_DEPTH_TEST)
#     glDepthFunc(GL_LEQUAL)
#     glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


# def read_data():
#     if(useQuat):
#         return sensor.quaternion
#     else:
#         return sensor.euler


# def draw(w, nx, ny, nz):
#     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
#     glLoadIdentity()
#     glTranslatef(0, 0.0, -7.0)

#     drawText((-2.6, 1.8, 2), "VIZI", 18)
#     drawText((-2.6, -2, 2), "Press Escape to exit.", 16)

#     if(useQuat):
#         [yaw, pitch , roll] = quat_to_ypr([w, nx, ny, nz])
#         drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
#         glRotatef(2 * math.acos(w) * 180.00/math.pi, -1 * nx, nz, ny)
#     else:
#         yaw = nx
#         pitch = ny
#         roll = nz
#         drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
#         glRotatef(-roll, 0.00, 0.00, 1.00)
#         glRotatef(pitch, 1.00, 0.00, 0.00)
#         glRotatef(yaw, 0.00, 1.00, 0.00)

#     glBegin(GL_QUADS)
#     glColor3f(0.0, 1.0, 0.0)
#     glVertex3f(1.0, 0.2, -1.0)
#     glVertex3f(-1.0, 0.2, -1.0)
#     glVertex3f(-1.0, 0.2, 1.0)
#     glVertex3f(1.0, 0.2, 1.0)

#     glColor3f(1.0, 0.5, 0.0)
#     glVertex3f(1.0, -0.2, 1.0)
#     glVertex3f(-1.0, -0.2, 1.0)
#     glVertex3f(-1.0, -0.2, -1.0)
#     glVertex3f(1.0, -0.2, -1.0)

#     glColor3f(1.0, 0.0, 0.0)
#     glVertex3f(1.0, 0.2, 1.0)
#     glVertex3f(-1.0, 0.2, 1.0)
#     glVertex3f(-1.0, -0.2, 1.0)
#     glVertex3f(1.0, -0.2, 1.0)

#     glColor3f(1.0, 1.0, 0.0)
#     glVertex3f(1.0, -0.2, -1.0)
#     glVertex3f(-1.0, -0.2, -1.0)
#     glVertex3f(-1.0, 0.2, -1.0)
#     glVertex3f(1.0, 0.2, -1.0)

#     glColor3f(0.0, 0.0, 1.0)
#     glVertex3f(-1.0, 0.2, 1.0)
#     glVertex3f(-1.0, 0.2, -1.0)
#     glVertex3f(-1.0, -0.2, -1.0)
#     glVertex3f(-1.0, -0.2, 1.0)

#     glColor3f(1.0, 0.0, 1.0)
#     glVertex3f(1.0, 0.2, -1.0)
#     glVertex3f(1.0, 0.2, 1.0)
#     glVertex3f(1.0, -0.2, 1.0)
#     glVertex3f(1.0, -0.2, -1.0)
#     glEnd()


# def drawText(position, textString, size):
#     font = pygame.font.SysFont("Courier", size, True)
#     textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
#     textData = pygame.image.tostring(textSurface, "RGBA", True)
#     glRasterPos3d(*position)
#     glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

# def quat_to_ypr(q):
#     yaw   = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
#     pitch = -math.asin(2.0 * (q[1] * q[3] - q[0] * q[2]))
#     roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
#     pitch *= 180.0 / math.pi
#     yaw   *= 180.0 / math.pi
#     yaw   -= -0.13  # Declination at Chandrapur, Maharashtra is - 0 degress 13 min
#     roll  *= 180.0 / math.pi
#     return [yaw, pitch, roll]


# i2c = board.I2C()
# sensor = adafruit_bno055.BNO055_I2C(i2c)

# useQuat = True

# def main():
#     video_flags = OPENGL | DOUBLEBUF
#     pygame.init()
#     screen = pygame.display.set_mode((640,480), video_flags)
#     pygame.display.set_caption("orientation visualization")
#     resizewin(640,480)
#     init()
#     frames = 0
#     ticks = pygame.time.get_ticks()
#     while 1:
#         event = pygame.event.poll()
#         if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
#             break
#         if(useQuat):
#             [w, nx, ny, nz] = read_data()
#         else:
#             [yaw, roll, pitch] = read_data()
#         if(useQuat):
#             draw(w, nx, ny, nz)
#         else:
#             draw(1, yaw, roll, pitch)
#         pygame.display.flip()
#         frames += 1
#     print("fps: %d" % ((frames*1000)/(pygame.time.get_ticks()-ticks)))

# if __name__ == '__main__':
#     main()