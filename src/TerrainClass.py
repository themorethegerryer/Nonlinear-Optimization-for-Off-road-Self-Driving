# import os.path, pkgutil
# import pyqtgraph.opengl.items.GLVolumeItem as gl
# pkgpath = os.path.dirname(gl.__file__)
# print([name for _, name, _ in pkgutil.iter_modules([pkgpath])])

import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import sys
import numpy as np

class Terrain(object):
    def __init__(self):
        self.app = QtWidgets.QApplication(sys.argv)
        self.w = gl.GLViewWidget()
        self.w.setGeometry(0, 110, 1920, 1080)
        self.w.show()
        self.w.setWindowTitle('Terrain')
        self.w.setCameraPosition(distance=30, elevation=8)

        # grid = gl.GLGridItem()
        # grid.scale(2, 2, 2)
        # self.w.addItem(grid)

        # 0,0,0
        # 1,0,0
        # 0,1,0
        # 1,1,0
        # 0,0,1
        # 1,0,1
        # 0,1,1
        # 1,1,1

        # car_coords = np.array([
        #     [0,0,0,1],
        #     [1,0,0,1],
        #     [0,1,0,1],
        #     [1,1,0,1],
        #     [0,0,1,1],
        #     [1,0,1,1],
        #     [0,1,1,1],
        #     [1,1,1,1]
        #     ])

        # car_coords = np.zeros((2,2,2,4))
        # print("car_coords: ", car_coords)
        # print("car_coords[0,0,0,:]: ", car_coords[0,0,0,:])
        # print("car_corrds: ", car_coords)
        # car = gl.GLVolumeItem(car_coords)
        # self.w.addItem(car)

        # Generate Car Vertices
        self.nstep = 1 #Distance between each vertice
        self.y_max = 2
        self.x_max = 1
        self.ypoints = range(-self.y_max, self.y_max, self.nstep)
        self.xpoints = range(-self.x_max, self.x_max, self.nstep)
        self.nfaces = len(self.ypoints)

        self.car_verts = np.array([
            [
                x, y, 4
            ] for n, x in enumerate(self.xpoints) for m, y in enumerate(self.ypoints)
        ], dtype=np.float32)

        faces = []
        colors = []
        ii = 0
        for m in range(self.nfaces - 1):
            yoff = m * self.nfaces
            for n in range(self.nfaces - 1):
                faces.append([n + yoff, yoff + n + self.nfaces, yoff + n + self.nfaces + 1])
                faces.append([n + yoff, yoff + n + 1, yoff + n + self.nfaces + 1])
                color = [0.3,0.3,1,1]
                colors.append(color)
                colors.append(color)
                ii += 1

        self.car_faces = np.array(faces)
        self.car_colors = np.array(colors)

        self.m2 = gl.GLMeshItem(
            vertexes=self.car_verts,
            faces=self.car_faces, faceColors=self.car_colors,
            smooth=True, drawEdges=True, edgeColor=[0.3,0.2,0.05,1]
        )
        self.m2.setGLOptions('additive')
        self.w.addItem(self.m2)

        # Generate Terrain Vertices
        self.nstep = 1 #Distance between each vertice
        self.y_max = 50
        self.x_max = 50
        self.ypoints = range(-self.y_max, self.y_max, self.nstep)
        self.xpoints = range(-self.x_max, self.x_max, self.nstep)
        self.nfaces = len(self.ypoints)

        self.verts = np.array([
            [
                x, y, 0
            ] for n, x in enumerate(self.xpoints) for m, y in enumerate(self.ypoints)
        ], dtype=np.float32)

        # Create Depths
        # Depth is created from the bottom left to the top right with bilinear interpolation
        # ->  -> ->
        # ^  ^  ^  ^
        car_width = 2
        car_length = 3
        depth_std = 0.6
        ii = 0
        for x_idx in self.xpoints:
            for y_idx in self.ypoints:
                if x_idx <= -self.x_max + car_width and y_idx <= -self.y_max + car_length:
                    self.verts[ii,2] = 0
                elif x_idx == 0:
                    self.verts[ii,2] = self.verts[ii-len(self.xpoints), 2] + depth_std * np.random.normal()
                else:
                    self.verts[ii,2] = (self.verts[ii-len(self.xpoints), 2] + self.verts[ii-1, 2]) / 2 + depth_std * np.random.normal()
                ii += 1


        # Generate Faces
        self.gen_Faces()

        self.m1 = gl.GLMeshItem(
            vertexes=self.verts,
            faces=self.faces, faceColors=self.colors,
            smooth=False, drawEdges=True, edgeColor=[0.3,0.2,0.05,1]
        )
        self.m1.setGLOptions('additive')
        self.w.addItem(self.m1)

    def start(self):
        if(sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtWidgets.QApplication.instance().exec_()

    def animation(self):
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(10)
        self.start()
        self.update()

    def update(self):
        print("self.car_verts: ", self.car_verts)
        print("self.car_verts.shape: ", self.car_verts.shape)
        self.car_verts[:,0] += 0.01
        self.car_verts[:,1] += 0.02
        # self.car_faces[:,0] += 0.01
        # self.car_faces[:,1] += 0.02
        print("self.car_faces: ", self.car_faces)
        print("self.car_faces.shape: ", self.car_faces.shape)
        # quit()
        self.m2.setMeshData(
            vertexes = self.car_verts, faces = self.car_faces, faceColors = self.car_colors
        )

    def gen_Faces(self):
        faces = []
        colors = []
        color_light = [238/255, 222/255, 154/255]
        color_dark = [85/255, 63/255, 19/255]
        height_max = np.max(self.verts[:,2])
        height_min = np.min(self.verts[:,2])
        opacity = 1000
        ii = 0
        for m in range(self.nfaces - 1):
            yoff = m * self.nfaces
            for n in range(self.nfaces - 1):
                faces.append([n + yoff, yoff + n + self.nfaces, yoff + n + self.nfaces + 1])
                faces.append([n + yoff, yoff + n + 1, yoff + n + self.nfaces + 1])
                color_ratio = (self.verts[ii,2] - height_min) / (height_max - height_min)
                color = [
                    (color_light[0] - color_dark[0]) * color_ratio + color_dark[0],
                    (color_light[1] - color_dark[1]) * color_ratio + color_dark[1],
                    (color_light[2] - color_dark[2]) * color_ratio + color_dark[2],
                    opacity
                    ]
                colors.append(color)
                colors.append(color)
                ii += 1

        self.faces = np.array(faces)
        self.colors = np.array(colors)


if __name__ == '__main__':
    t = Terrain()
    t.animation()