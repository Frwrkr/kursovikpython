from bodies import *
from optics import *
from scene import *
from spatial import *

from PIL import Image
import time



def example1(width = 200, height = 150):
    t0 = time.perf_counter()
    
    ball1 = Sphere(3).coloured(Colour.RGB(0.9,0.3,0)).apply(Mat4.Translator(3,-1,-2))
    ball2 = Sphere(2).coloured(Colour.RGB(0,0.6,0.6)).apply(Mat4.Translator(-2,2,-4))
    ball3 = Sphere().coloured(Colour.RGB(0.7,1,0)).apply(Mat4.Translator(-2,0,1))
    
    eye = Eye(transform = Mat4.Translator(0,0,4))
    light = PointSource(Vec4.Point(0,6,6), Colour.RGB(1.2,1.2,1.2))
    
    scene = Scene([ball1, ball2, ball3], light, eye)
    
    render = eye.render(scene, width, height)
    
    image = Image.fromarray(render)
    print('time taken:', time.perf_counter()-t0)
    image.save('result.png')
