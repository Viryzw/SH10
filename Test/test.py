import math


def vision_yaw(cx, cy):
    if cx * cy > 0:
        return math.atan2(abs(cx), abs(cy))
    else:
        return math.atan2(-1*abs(cx), abs(cy))
    
'''a = ((1, 1), (1, -1), (-1, 1), (-1, -1), (0.5, 1), (0, 1), (1, 0))
for i in a:
    print(vision_yaw(i[0], i[1]))'''

