from shapely.geometry import LineString, Point
import random
import copy

"""
If you are creating a new room, make sure to run this file first

1) Run this file and generate random dust points, copy terminal output and copy it into a dust list
2) Run main.py
3) See which dust particles are in unusual places (outside boarders, inside polygons)
4) Click the centre of the particles you want to remove, the coordinates will be displayed in the terminal
5) Eliminate them from the dust list you have created
6) Run main.py again
"""

dust = [None for i in range(780)]
zero_rows = [0 for _ in range(860)]
row = [0 for _ in range(860)]

for i in range(860):
    row[i] = 1 if i > 15 and i % 15 == 0 else 0

for i in range(780):
    dust[i] = zero_rows if i < 21 else copy.deepcopy(row)

total_dust = sum([sum(row) for row in dust])

walls_1 = [
    # Standard walls
    LineString([(860, 20), (15, 20)]),
    LineString([(15, 20), (15, 780)]),
    LineString([(15, 780), (860, 780)]),
    LineString([(860, 780), (860, 20)]),

    LineString([(170, 210), (231, 151)]),
    LineString([(231, 151), (290, 210)]),
    LineString([(290, 210), (230, 290)]),
    LineString([(230, 290), (170, 210)]),

    LineString([(470, 700), (705, 660)]),
    LineString([(705, 660), (706, 420)]),
    LineString([(706, 420), (730, 700)]),
    LineString([(730, 700), (470, 700)]),

]

dust_1 = [
    Point((226, 330)),
    Point((620, 51)),
    Point((481, 772)),
    Point((195, 373)),
    Point((134, 645)),
    Point((539, 261)),
    Point((527, 685)),
    Point((672, 777)),
    Point((211, 46)),
    Point((94, 421)),
    Point((424, 553)),
    Point((655, 285)),
    Point((529, 634)),
    Point((344, 84)),
    Point((754, 533)),
    Point((111, 98)),
    Point((335, 145)),
    Point((136, 74)),
    Point((356, 753)),
    Point((253, 348)),
    Point((486, 550)),
    Point((180, 30)),
    Point((600, 274)),
    Point((706, 408)),
    Point((498, 681)),
    Point((639, 633)),
    Point((788, 199)),
    Point((118, 392)),
    Point((587, 572)),
    Point((797, 344)),
    Point((24, 441)),
    Point((91, 139)),
    Point((216, 38)),
    Point((351, 768)),
    Point((791, 361)),
    Point((263, 155)),
    Point((798, 606)),
    Point((349, 651)),
    Point((827, 97)),
    Point((624, 644)),
    Point((112, 419)),
    Point((801, 341)),
    Point((709, 155)),
    Point((302, 745)),
    Point((160, 81)),
    Point((421, 179)),
]

room_1 = [walls_1, dust, total_dust]

####################################################

walls_2 = [
    LineString([(860, 20), (15, 20)]),
    LineString([(15, 20), (15, 780)]),
    LineString([(15, 780), (860, 780)]),
    LineString([(860, 780), (860, 20)]),
]

room_2 = [walls_2, dust, total_dust]

####################################################

walls_3 = [
    # Standard walls
    LineString([(860, 20), (15, 20)]),
    LineString([(15, 20), (15, 780)]),
    LineString([(15, 780), (860, 780)]),
    LineString([(860, 780), (860, 20)]),

    LineString([(150, 218), (175, 170)]),
    LineString([(750, 218), (150, 218)]),
    LineString([(725, 170), (750, 218)]),
    LineString([(175, 170), (725, 170)]),

    LineString([(725, 700), (175, 700)]),
    LineString([(150, 650), (750, 650)]),
    LineString([(175, 700), (150, 650)]),
    LineString([(750, 650), (725, 700)]),

    LineString([(230, 340), (230, 460)]),
    LineString([(195, 490), (195, 310)]),
    LineString([(195, 310), (230, 340)]),
    LineString([(230, 460), (195, 490)])

]

room_3 = [walls_3, dust, total_dust]

####################################################

walls_4 = [
    # Standard walls
    LineString([(422, 20), (15, 780)]),
    LineString([(860, 780), (422, 20)]),
    LineString([(15, 780), (860, 780)]),


]

room_4 = [walls_4, dust, total_dust]

####################################################

walls_5 = [
    # Standard walls
    LineString([(860, 20), (15, 20)]),
    LineString([(15, 20), (15, 780)]),
    LineString([(15, 780), (860, 780)]),
    LineString([(860, 780), (860, 20)]),

    LineString([(170, 210), (231, 151)]),
    LineString([(231, 151), (290, 210)]),
    LineString([(290, 210), (230, 290)]),
    LineString([(230, 290), (170, 210)]),

    LineString([(788, 225), (860, 157)]),
    LineString([(860, 257), (788, 225)]),

    LineString([(200, 780), (200, 480)]),
    LineString([(200, 480), (450, 780)]),

    LineString([(860, 780), (714, 515)]),
    LineString([(714, 515), (860, 400)]),
]

room_5 = [walls_5, dust, total_dust]

####################################################

walls_6 = [
    LineString([(441, 20), (244, 340)]),
    LineString([(244, 340), (15, 340)]),
    LineString([(15, 340), (244, 523)]),
    LineString([(244, 523), (135, 780)]),
    LineString([(135, 780), (441, 585)]),
    LineString([(441, 585), (747, 780)]),
    LineString([(747, 780), (638, 523)]),
    LineString([(638, 523), (860, 340)]),
    LineString([(860, 340), (638, 340)]),
    LineString([(638, 340), (441, 20)]),
]

room_6 = [walls_6, dust, total_dust]


def generate_random_dust(amount_of_dust):

    for _ in range(amount_of_dust):
        print(f"Point(({random.randint(15, 860)}, {random.randint(20, 780)})),")


if __name__ == '__main__':
    print(len(dust))
