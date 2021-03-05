from optics import *
from spatial import *

class Scene:
    bodies : list

    def __init__(self, bodies, light, camera):
        self.bodies = bodies
        self.light = light
        self.camera = camera
    
    def __iadd__(self, bodies):
        self.add_bodies(bodies)
    
    def add_bodies(self, bodies):
        if not type(bodies) == list:
            bodies = [bodies]
        self.bodies += bodies
    
    def apply(self, transform : Mat4):
        for b in self.bodies:
            b.apply(transform)
        return self