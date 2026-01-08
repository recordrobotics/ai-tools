import math, random
import matplotlib.pyplot as plt  # type: ignore

FIELD_OCTOGON = [
    (7.0570570, 4.025900),
    (8.7741250, 2.736850),
    (8.7741250, -2.73685),
    (7.0570570, -4.02590),
    (-7.057057, -4.02590),
    (-8.774125, -2.73685),
    (-8.774125, 2.736850),
    (-7.057057, 4.025900),
]
FIELD_CENTER_SQUARE = [
    (0.15240, 0.15240),
    (0.15240, -0.1524),
    (-0.1524, -0.1524),
    (-0.1524, 0.15240),
]
FIELD_BLUE_REEF = [  # generate hexagon centered on (4.284614, 0) with radius 0.9604
    (4.284614 + 0.9604 * math.cos(theta), 0 + 0.9604 * math.sin(theta))
    for theta in [i * (2 * math.pi / 6) + math.pi / 2 for i in range(6)]
]
FIELD_RED_REEF = [  # invert field blue reef across y axis
    (-x, y) for x, y in FIELD_BLUE_REEF
]
ROBOT_SQUARE = [
    (0.463550, 0.463550),
    (0.463550, -0.46355),
    (-0.46355, -0.46355),
    (-0.46355, 0.463550),
]
CORAL_RECTANGLE = [
    (0.15081250, 0.057150),
    (0.15081250, -0.05715),
    (-0.1508125, -0.05715),
    (-0.1508125, 0.057150),
]
# generate circle with 0.41275 diameter estimated by 16 points
ALGAE_CIRCLE = [
    (0.206375 * math.cos(theta), 0.206375 * math.sin(theta))
    for theta in [i * (-2 * math.pi / 16) for i in range(16)]
]


def project(poly, ax, ay):
    min_p = max_p = poly[0][0] * ax + poly[0][1] * ay
    for x, y in poly[1:]:
        p = x * ax + y * ay
        if p < min_p:
            min_p = p
        elif p > max_p:
            max_p = p
    return min_p, max_p


def axes(poly):
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        dx, dy = x2 - x1, y2 - y1
        # perpendicular axis (edge normal)
        yield -dy, dx


def convex_intersection(poly1, poly2):
    for poly in (poly1, poly2):
        for ax, ay in axes(poly):
            min1, max1 = project(poly1, ax, ay)
            min2, max2 = project(poly2, ax, ay)
            if max1 < min2 or max2 < min1:
                return False  # separating axis found

    return True


def convex_containment(inner_polygon, outer_polygon):
    # Using SAT for containment check
    for ax, ay in axes(outer_polygon):
        min_outer, max_outer = project(outer_polygon, ax, ay)
        min_inner, max_inner = project(inner_polygon, ax, ay)
        if min_inner < min_outer or max_inner > max_outer:
            return False  # inner polygon extends beyond outer polygon on this axis
    return True


def draw_polygons(polygons):
    for polygon in polygons:
        color = (
            random.random(),
            random.random(),
            random.random(),
        )  # Generate random RGB color
        xs, ys = zip(*polygon)
        xs += (xs[0],)
        ys += (ys[0],)
        plt.plot(xs, ys, color=color)

    plt.axis("equal")
    plt.show()


def rotate_polygon(polygon, theta, origin=(0, 0)):
    theta_rad = math.radians(theta)
    cos_theta = math.cos(theta_rad)
    sin_theta = math.sin(theta_rad)
    ox, oy = origin

    rotated_polygon = []
    for px, py in polygon:
        rotated_x = cos_theta * (px - ox) - sin_theta * (py - oy) + ox
        rotated_y = sin_theta * (px - ox) + cos_theta * (py - oy) + oy
        rotated_polygon.append((rotated_x, rotated_y))

    return rotated_polygon


def translate_polygon(polygon, dx, dy):
    translated_polygon = []
    for px, py in polygon:
        translated_x = px + dx
        translated_y = py + dy
        translated_polygon.append((translated_x, translated_y))
    return translated_polygon


def move_polygon_to_pose(polygon, pose):  # pose: (x, y, theta)
    x, y, theta = pose
    rotated_polygon = rotate_polygon(polygon, theta)
    translated_polygon = translate_polygon(rotated_polygon, x, y)
    return translated_polygon


def check_convex_not_intersecting(convex, pose, other_convexes=[]):
    moved_convex = move_polygon_to_pose(convex, pose)

    return (
        convex_containment(moved_convex, FIELD_OCTOGON)
        and not convex_intersection(FIELD_CENTER_SQUARE, moved_convex)
        and not convex_intersection(FIELD_BLUE_REEF, moved_convex)
        and not convex_intersection(FIELD_RED_REEF, moved_convex)
        and all(
            not convex_intersection(other, moved_convex) for other in other_convexes
        )
    )


def random_convex_in_field(convex, other_convexes=[]):
    while True:
        pose = (
            random.uniform(-9.0, 9.0),
            random.uniform(-4.5, 4.5),
            random.uniform(0, 360),
        )
        if check_convex_not_intersecting(convex, pose, other_convexes):
            return pose

def fill_field_with_objects():
    robot_pose = random_convex_in_field(ROBOT_SQUARE)
    robot_polygon = move_polygon_to_pose(ROBOT_SQUARE, robot_pose)
    coral_poses = []
    coral_polygons = []
    for _ in range(25):
        pose = random_convex_in_field(CORAL_RECTANGLE, coral_polygons + [robot_polygon])
        coral_poses.append(pose)
        coral_polygons.append(move_polygon_to_pose(CORAL_RECTANGLE, pose))
    algae_poses = []
    algae_polygons = []
    for _ in range(10):
        pose = random_convex_in_field(
            ALGAE_CIRCLE, coral_polygons + [robot_polygon] + algae_polygons
        )
        algae_poses.append(pose)
        algae_polygons.append(move_polygon_to_pose(ALGAE_CIRCLE, pose))

    return robot_polygon, coral_polygons, algae_polygons

robot_polygon, coral_polygons, algae_polygons = fill_field_with_objects()
draw_polygons(
    [FIELD_OCTOGON, FIELD_CENTER_SQUARE, FIELD_BLUE_REEF, FIELD_RED_REEF, robot_polygon]
    + coral_polygons
    + algae_polygons
)
