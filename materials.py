import numpy as np

class Colour:
    data = np.ndarray

    def RGBA(r, g, b, a, ofBytes = False):
        if ofBytes:
            r /= 255
            g /= 255
            b /= 255
            a /= 255
        array = np.array((r, g, b, a), float)
        return Colour(array)
    
    def RGB(r, g, b):
        return Colour.RGBA(r, g, b, 1)
    
    def __init__(self, array):
        self.data = array
    
    def blend(self, other):
        self.data *= other.data