from materials import *
from spatial import *
from utils import *



# Суперкласс объекта сцены
class Body:
        transform : np.ndarray
        inv_transform : np.ndarray
        colour : Colour

        def __init__(self, transform = None, inv_transform = None):
            self.transform = Mat4.Identity().data if transform == None else transform.data
            if inv_transform == None:
                self.set_inverse()
            else:
                self.inv_transform = inv_transform

        # Применение матричного преобразования к объекту - меняет его собственные матрицы
        def apply(self, transform : Mat4):
            np.matmul(self.transform, transform.data, self.transform)
            self.set_inverse()
            return self

        # Определение обратного преобразования из локальной системы координат в глобальную
        def set_inverse(self):
            self.inv_transform = np.linalg.inv(self.transform)

        def intersect(self, ray : Ray):
            return [Intersection(t, self) for t in self.find_intersections(ray)]

        # Абстрактный метод поиска пересечений тела с лучом, имплементируется примитивами
        def find_intersections(self, ray : Ray):
            pass

        # Задание цвета объекта
        def coloured(self, colour : Colour):
            self.colour = colour
            return self

# Класс многоугольников
class Polygon(Body):
    std : bool
    points : np.ndarray
    normal : np.ndarray
    D : float
    
    def __init__(self, points, transform = None, inv_transform = None, std = False):
        super().__init__(transform, inv_transform)
        self.std = std
        self.points = points
        if std:
            self.normal = np.array((0.0, 1.0, 0.0))
            self.D = 0.0
        else:
            self.normal = self.get_normal()
            self.D = self.get_D()

    # Абстрактный метод проверки вхождения точки в полигон, имплементируется субклассами
    def includes(self, Q):
        pass

    # Вычисление инварианта плоскости
    def get_D(self):
        A = self.points[0,:3]
        return np.dot(self.normal, A)

    # Вычисление нормали
    def get_normal(self):
        A, B, C = self.points[:,:3]
        normal = np.cross(B-A, C-A)
        magnitude = math.sqrt(sum(normal*normal))
        return normal/magnitude
    
    def find_intersections(self, ray : Ray):
        normal = self.get_normal_array()
        objspace_ray = np.matmul(ray.data, self.inv_transform)
        orig, dirc = objspace_ray[:,:3]
        denom = np.dot(normal, dirc)
        if floeq(denom, 0.0):
            return []
        t = (self.get_D()-np.dot(normal, orig))/denom
        Q = orig+t*dirc
        return [t] if self.includes(Q) else []

class Triangle(Polygon):
    def includes(self, Q):
        A, B, C = self.points
        tup = ((B-A, A), (A-C, C), (C-B, B))
        for side, vertex in tup:
            if np.dot(self.normal, np.cross(side, Q-vertex)) < 0.0:
                return False
        return True

# Класс четырехугольника/плоскости
class Quad(Polygon):
    std : bool

    points = np.array((( 0.5,  0.0,  0.5),
                       ( 0.5,  0.0, -0.5),
                       (-0.5,  0.0, -0.5),
                       (-0.5,  0.0,  0.5)))
    
    indices = np.array(((0, 1, 3),
                        (2, 3, 1)))

    # Конструкторы плоскостей
    def X_pos(y_scale = 1.0, z_scale = 1.0):
        return Quad(transform = Mat4.Scaler(y_scale, 1.0, z_scale)*Mat4.Rotor(2, math.pi/2))
    
    def X_neg(y_scale = 1.0, z_scale = 1.0):
        return Quad(transform = Mat4.Scaler(y_scale, 1.0, z_scale)*Mat4.Rotor(2, -math.pi/2))
    
    def Y_pos(x_scale = 1.0, z_scale = 1.0):
        return Quad(transform = Mat4.Scaler(x_scale, 1.0, z_scale))
    
    def Y_neg(x_scale = 1.0, z_scale = 1.0):
        return Quad(transform = Mat4.Scaler(-x_scale, 1.0, z_scale))
    
    def Z_pos(x_scale = 1.0, y_scale = 1.0):
        return Quad(transform = Mat4.Scaler(x_scale, 1.0, y_scale)*Mat4.Rotor(0, math.pi/2))
    
    def Z_neg(x_scale = 1.0, y_scale = 1.0):
        return Quad(transform = Mat4.Scaler(x_scale, 1.0, y_scale)*Mat4.Rotor(0, -math.pi/2))
    
    def __init__(self, points = None, transform = None, std = True):
        if points == None:
            super().__init__(Quad.points, transform, std = std)
        else:
            super().__init__(points, transform)
    
    def includes(self, Q):
        A, B, C, D = self.points
        if np.dot(self.normal, np.cross(C-A, Q-A)) >= 0.0:
            # Q в полуплоскости вершины D
            tup = ((D-C, C), (A-D, D))
        else:
            # Q в полуплоскости вершины B
            tup = ((B-A, A), (B-C, B))
        for side, vertex in tup:
            if np.dot(self.normal, np.cross(side, Q-vertex)) < 0.0:
                return False
        return True



class Sphere(Body):
    def __init__(self, radius = None, center = None, transform = None):
        if radius == None and center == None:
            super().__init__(transform)
        else:
            t = Mat4.Identity() if radius == None else Mat4.Scaler(radius, radius, radius)
            if not transform == None:
                t *= transform
            if not center == None:
                t *= Mat4.Translator(*center[:3])
            super().__init__(t)
    
    def find_intersections(self, ray : Ray):
        object_ray = np.matmul(ray.data, self.inv_transform)
        orig, dirc = np.vsplit(object_ray[:,:3], 2)
        a = np.dot(dirc, dirc.T)
        b = 2.0*np.dot(dirc, orig.T)
        c = np.dot(orig, orig.T)-1.0
        discriminant = b*b-4*a*c
        if discriminant < 0:
            return []
        elif discriminant == 0:
            return [-b/(2.0*a)]
        else:
            d = math.sqrt(discriminant)
            return [(-b-d)/(2.0*a), (-b+d)/(2.0*a)]
    
    def normal_array_at(self, world_point):
        object_normal = np.matmul(world_point.data, self.inv_transform)
        world_normal = np.matmul(object_normal, self.transform.T).T
        world_normal[3] = 0
        world_normal /= math.sqrt(np.sum(world_normal*world_normal))
        return world_normal

            



# Класс многогранника
class Polyhedron(Body):
    points : np.ndarray
    indices : np.ndarray

    def __init__(self, points, indices):
        super().__init__()
        self.points = points
        self.indices = indices
    
    def apply(self, *transforms):
        for t in transforms:
            self.transforms.insert(0, t)
            self.points = np.matmul(self.points, t)
        return self

class Box(Polyhedron):
    def __init__(self, scale = None, center = None):
        points = np.array((( 0.5,  0.5,  0.5,  1.0),
                           ( 0.5,  0.5, -0.5,  1.0),
                           (-0.5,  0.5, -0.5,  1.0),
                           (-0.5,  0.5,  0.5,  1.0),
                           ( 0.5, -0.5,  0.5,  1.0),
                           ( 0.5, -0.5, -0.5,  1.0),
                           (-0.5, -0.5, -0.5,  1.0),
                           (-0.5, -0.5,  0.5,  1.0)))
        indices = np.array(((0, 4, 1),
                            (5, 1, 4),
                            (3, 2, 7),
                            (6, 7, 2),
                            (0, 1, 3),
                            (2, 3, 1),
                            (4, 7, 5),
                            (6, 5, 7),
                            (0, 3, 4),
                            (7, 4, 3),
                            (1, 5, 2),
                            (6, 2, 5)))
        super().__init__(points, indices)
