from bodies import Body
from materials import *
from scene import Scene
from spatial import *



class LightSource:
    position : Vec4

    def apply(self, transform : Mat4):
        np.matmul(self.position, transform, self.position)

# Класс точечного источника света
class PointSource(LightSource):
    intensity : Colour

    def __init__(self, position, intensity):
        self.position = position
        self.intensity = intensity



# Вычисление цвета пикселя от цвета поверности, ориентации векторов источника света, камеры и нормали
def lighting(surf_colour : Colour, source : LightSource, point : np.ndarray, eye : np.ndarray, normal : np.ndarray):
    source_colour = source.intensity.data[:3]
    eff_colour = surf_colour.data[:3] * source_colour
    ambient = 0.2 * eff_colour
    source_vec = (source.position.data - point).T
    source_vec /= math.sqrt(np.sum(source_vec*source_vec))
    eye_vec = eye - point
    eye_vec /= math.sqrt(np.sum(eye_vec*eye_vec))
    ldn = sum(source_vec*normal)
    if ldn < 0.0:
        return ambient
    diffuse = eff_colour * ldn
    refl_vec = reflect_array(-source_vec, normal)
    rde = np.sum((refl_vec*eye_vec)[:3])
    if rde <= 0.0:
        return ambient + diffuse
    specular = source_colour * math.pow(rde, 10.0)
    return ambient + diffuse + specular



# Класс перспективной камеры
class Eye(Body):
    origin = np.array(( 0.0,  0.0,  0.0,  1.0))
    fov : float
    
    def __init__(self, fov = 90.0, transform = None):
        super().__init__(transform)
        self.fov = fov
    
    # Вспомогательный класс для прохода по лучам матрицы
    class GridIterator:
        width : int
        height : int
        anchor : np.ndarray
        hor_inc : np.ndarray
        ver_inc : np.ndarray
    
        def __init__(self, root, width : int, height : int):
            self.root = root
            self.width = width
            self.height = height
            hor_span = 2.0*math.tan(root.fov*math.pi/360.0)
            ver_span = hor_span*height/width
            # Горизонтальный и вертикальный инкременты
            # Они будут добавляться к направлению луча при движении по сетке
            self.hor_inc = np.array((hor_span/(width-1), 0.0, 0.0, 0.0))
            self.ver_inc = np.array((0.0, -(ver_span/(height-1)), 0.0, 0.0))
            # Верхний левый угол сетки, от которого начнется отсчет
            self.anchor = np.array((-hor_span/2.0, ver_span/2.0, -1.0, 0.0))-self.hor_inc

        def __iter__(self):
            for y in range(self.height):
                dirc = self.anchor+y*self.ver_inc
                for x in range(self.width):
                    dirc += self.hor_inc
                    # Возвращение величины 3*индекс_пикселя упрощает расчеты в функции рендера
                    yield 3*(y*self.width+x), (Ray.make(self.root.origin, dirc)*self.root.transform).normalised()
    
    # Основной метод, отрисовка сцены с данным разрешением
    def render(self, scene : Scene, width : int, height : int):
        img_data = np.empty(3*width*height, float)
        blank = np.zeros(3, int)
        for i, ray in self.GridIterator(self, width, height):
            hit = ray.cast_into(scene)
            if hit == None:
                pixel = blank
            else:
                point = ray.loc_at_t(hit.t)
                normal_array = hit.body.normal_array_at(point)
                pixel = lighting(hit.body.colour, scene.light, point, self.origin*self.transform, normal_array)
            img_data[i:i+3] = pixel
        np.clip(img_data, 0.0, 1.0, img_data)
        return (255.0*img_data).astype(np.uint8).reshape((height, width, 3))
