import math

a = ((1, 1), (1, -1), (-1, 1), (-1, -1))
for i in a:
    print(math.atan2(i[0], i[1]))


def vision_yaw(cx, cy):
    if cx * cy > 0:
        return math.atan2()