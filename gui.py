import numpy as np
import cv2

import matplotlib.pyplot as plt
from geometry import Transform

from tkinter import *

class Gui():

    def __init__(self):
        plt.ion()
        self._fig = plt.figure()
        self._rbax = self._fig.add_subplot(projection="3d")
        self._world = Transform.id()
        plt.show()


    def draw(self, target):
        for t in target:
            tran = np.concatenate([t.astype(int)/10, [0]])
            tran = Transform.from_t(tran)
            tran.draw_on_ax(self._rbax, scale=2)
        self._rbax.set_xlim((0,40))
        self._rbax.set_ylim((0,40))
        self._rbax.set_zlim((0,40))

    def draw_transform(self, transform: Transform, c_i=0):
        transform.draw_on_ax(self._rbax, c_i=c_i)
        self._rbax.set_aspect('equal')

    def image_visualization(self, current_estimated_pixel, image):
        for estimated_pixel in current_estimated_pixel:
            marked_image = cv2.circle(image, estimated_pixel.astype(int), 0, (0, 0, 255), 2)

        cv2.imshow('img_1', marked_image)
        cv2.waitKey(25000)

    def transform(self, data):
        trans = Transform(data)
        return trans

    def world_id(self):
        return Transform.id()
    
class InteractiveGui():
    def __init__(self):
        pass

    def debug_answer(self):
        debug_monitor = Tk()

        label = Label(debug_monitor, text = 'Enter debug mode:')

        positive = Button(debug_monitor, text = 'yes', command = positive())
        negative = Button(debug_monitor, text = 'no', command = negative())

        debug_monitor.mainloop()

        return self.answer

    def positive(self, debug_monitor):
        self.answer = True
        debug_monitor.destroy
    
    def negative(self, debug_monitor):
        self.answer = False
        debug_monitor.destroy