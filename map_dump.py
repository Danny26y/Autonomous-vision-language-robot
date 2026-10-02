import os
import re

import cv2

import sys
import sys
yaml_path = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/turtlebot3_map.yaml')
txt = open(yaml_path).read()
print(txt)
img_name = re.search(r'image:\s*(\S+)', txt).group(1)
res = float(re.search(r'resolution:\s*([\d.eE+-]+)', txt).group(1))
ox, oy = [float(v) for v in
          re.search(r'origin:\s*\[([^\]]+)\]', txt).group(1).split(',')[:2]]
pgm = img_name if os.path.isabs(img_name) else os.path.join(os.path.dirname(yaml_path), img_name)
img = cv2.imread(pgm, cv2.IMREAD_GRAYSCALE)
h, w = img.shape
print(f'{w}x{h} px, {res} m/px, origin ({ox}, {oy}); each character = 2x2 px; top row = +y')
print('R robot, G goal, H hydrant | # occupied, ? unknown, blank free')

marks = {}
for ch, (x, y) in {'R': (-1.91, -0.82), 'G': (-0.45, -0.49), 'H': (0.55, -0.49)}.items():
    col = int((x - ox) / res)
    row = h - 1 - int((y - oy) / res)
    marks[(col // 2, row // 2)] = ch

for J in range((h + 1) // 2):
    line = []
    for I in range((w + 1) // 2):
        if (I, J) in marks:
            line.append(marks[(I, J)])
            continue
        v = int(img[2 * J:2 * J + 2, 2 * I:2 * I + 2].min())   # darkest pixel wins
        line.append('#' if v < 100 else ' ' if v > 240 else '?')
    print(''.join(line).rstrip())
