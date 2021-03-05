EPSILON = 1e-8

# Проверяет равенство чисел с плаваюей точкой
def floeq(a, b):
    return abs(a-b) < EPSILON
