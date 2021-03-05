import math
import numpy as np



# Векторы (4 компонента) для описания точек и стрелок
class Vec4:
    data : np.ndarray

    # Конструктор точки
    def Point(x, y, z):
        return Vec4(np.array((x, y, z, 1), float))

    # Стрелки - векторы, инвариантные под переносами
    # Конструктор стрелки
    def Arrow(x, y, z):
        return Vec4(np.array((x, y, z, 0), float))

    def __init__(self, array):
        self.data = array

    def __repr__(self):
        return f'Vector:{repr(self.data)}'
    
    def dot(self, v):
        return np.dot(self.data[:3], v.data[:3])
    
    def __abs__(self):
        return self.dot(self)
    def __add__(self, other):
        return Vec4(self.data+other.data)
    def __sub__(self, other):
        return Vec4(self.data-other.data)
    def __rmul__(self, scalar):
        return Vec4(self.data*scalar)
    def __truediv__(self, scalar):
        return Vec4(self.data/scalar)
    
    def normalised(self):
        return self/abs(self)



def reflect_array(a, n):
    return a-n*2*sum(a[:3]*n[:3])



# Матрицы
class Matrix:
    data : np.ndarray

    def __init__(self, array):
        self.data = array

    def __repr__(self):
        return f'Matrix:\n{repr(self.data)}'

    # Вычисляет определитель
    def __abs__(self):
        return np.linalg.det(self.data)
    # Вычисляет обратную матрицу
    def __invert__(self):
        return np.linalg.inv(self.data)
    def __add__(self, other):
        return type(self)(self.data+other.data)
    def __sub__(self, other):
        return type(self)(self.data-other.data)
    def __mul__(self, other):
        return type(self)(np.matmul(self.data, other.data))
    def __imul__(self, other):
        self.data = (self*other).data

# Матрицы 4х4 для трансформаций векторов
class Mat4(Matrix):
    data : np.ndarray

    # Конструктор единичной матрицы
    def Identity():
        return Mat4.Scaler(1, 1, 1)

    # Конструктор матрицы маштабирования
    def Scaler(x = 1, y = 1, z = 1):
        array = np.zeros((4, 4))
        for i, v in enumerate((x, y, z, 1)):
            array[i,i] = v
        return Mat4(array)

    # Конструктор матрицы переноса
    def Translator(x = 0, y = 0, z = 0):
        result = Mat4.Identity()
        for i, v in enumerate((x, y, z)):
            result.data[3,i] = v
        return result
    
    # Конструктор матрицы поворота
    def Rotor(axis, angle):
        result = Mat4.Identity()
        c = math.cos(angle)
        s = math.sin(angle)
        if axis == 0:
            result.data[1,1] = c
            result.data[2,2] = c
            result.data[1,2] = s
            result.data[2,1] = -s
        elif axis == 1:
            result.data[0,0] = c
            result.data[2,2] = c
            result.data[0,2] = -s
            result.data[2,0] = s
        elif axis == 2:
            result.data[0,0] = c
            result.data[1,1] = c
            result.data[0,1] = s
            result.data[1,0] = -s
        return result
    
    def Shearer(xy = 0, xz = 0, yx = 0,
                yz = 0, zx = 0, zy = 0):
        result = Mat4.Identity()
        result.data[1,0] = xy
        result.data[2,0] = xz
        result.data[0,1] = yx
        result.data[2,1] = yz
        result.data[0,2] = zx
        result.data[1,2] = zy
        return result

    def __init__(self, array):
        super().__init__(array)
    
    # Применяет матричную трансформацию к вектору
    def __rmul__(self, other : Vec4):
        return Vec4(np.matmul(other.data, self.data))



# Верзоры для поворотов векторов
class Versor:
    # Вспомогательные массивы индексов и знаков для умножения кватернионов
    __mul_indices = np.array((0,1,2,3, 1,0,3,2, 2,3,0,1, 3,2,1,0))
    __mul_signs = np.array((1,1,1,1, 1,-1,1,-1, 1,-1,-1,1, 1,1,-1,-1))
    
    data : np.ndarray

    # Конструктор верзора
    def make(angle, x, y, z):
        angle /= 2
        magnitude = math.sqrt(x*x+y*y+z*z)
        s_coeff = math.sin(angle)/magnitude
        return Versor(np.array((math.cos(angle),
                                s_coeff*x,
                                s_coeff*y,
                                s_coeff*z),
                               float))

    def __init__(self, array):
        self.data = array

    def __repr__(self):
        return f'Versor:\n{repr(self.data)}'

    def copy(self):
        return Versor(self.data.copy())

    # Вычисляет массив компоннтов сопряжённого кватерниона
    def conj_array(self):
        return self.data*np.array((1, -1, -1, -1))

    # Применяет кватернионное вращение к вектору
    def __rmul__(self, vector : Vec4):
        # w-компонента нужна для восстановления вектора исходного типа
        w = vector.data[3]
        # Вычисление массива компонентов кватерниона от исходного вектора
        quat_vec_data = np.concatenate((np.zeros(1), vector.data[:3]))
        res_quat = self.copy()
        # Умножение верзора на кватернион точки и сопряжённый верзор
        res_quat.imul_with_array(quat_vec_data)
        res_quat.imul_with_array(self.conj_array())
        # Построение массива компонентов повёрнутого вектора
        res_array = np.concatenate((res_quat.data[1:], np.array((w,))))
        return Vec4(res_array)

    # Умножает верзор на другой кватернион по его массиву компонентов
    def imul_with_array(self, array):
        res_table = np.outer(self.data, array)
        self.data = np.zeros(4)
        for v, i, s in zip(res_table.flat, self.__mul_indices, self.__mul_signs):
            self.data[i] += v*s



# Класс лучей
class Ray:
    data : np.ndarray
    fields = ('origin', 'direction')

    # Конструктор из массивов источника и направления
    def make(origin, direction):
        return Ray(np.vstack((origin, direction)))
    
    def __init__(self, array):
        self.data = array
    
    def __getattr__(self, attr):
        return self.data[self.fields.index(attr)]
    
    def __mul__(self, other : Mat4):
        return Ray(np.matmul(self.data, other.data))

    # Определение точки на луче
    def loc_at_t(self, t):
        return self.origin+t*self.direction

    # Поиск ближайшего пересечения с объектами сцены
    def cast_into(self, scene):
        result = []
        for b in scene.bodies:
            result += b.intersect(self)
        return self.find_hit(result)

    # Определения ближайшего пересечения в списке
    def find_hit(self, intersections : list):
        intersections = sorted(intersections, key = lambda x : x.t)
        for i in intersections:
            if i.t >= 0.0:
                return i
        return None

    # Нормализация
    def normalised(self):
        self.data[1,:] *= 1.0/np.linalg.norm(self.data[1,:])
        return self

# Структура для хранения информации о пересечении
class Intersection:
    t : float

    def __init__(self, t, body):
        self.t = t
        self.body = body
