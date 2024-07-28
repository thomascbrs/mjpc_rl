import math

class Rectangle:
  def __init__(self, x, y, halfWidth, halfHeight):
    self.x = x
    self.y = y
    self.halfWidth = halfWidth
    self.halfHeight = halfHeight

  def get_boundaries(self):
    """ return boundaries as [[x_lim-,x_lim+] , [ylim-,ylim+]]
    """
    return [[self.x - self.halfWidth, self.x + self.halfWidth],
            [self.y - self.halfHeight, self.y + self.halfHeight]]

class Hole:
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.size = size

    def is_inside(self, x, y):
        """Check if the point (x, y) is inside this hole"""
        distance = math.sqrt((x - self.x) ** 2 + (y - self.y) ** 2)
        return distance <= self.size

    def intersects(self, x, y, dwidth):
        """Check if a sphere of diameter dwidth centered at (x, y) intersects with this hole"""
        sphere_radius = dwidth / 2
        distance = math.sqrt((x - self.x) ** 2 + (y - self.y) ** 2)
        return distance <= (self.size + sphere_radius)


def create_environment():
    # Define environments with rectangles
    environments = [
        [Rectangle(0.0, 0.0, 4.0, 4.0)],    # Environment 0
        [Rectangle(0.0, 8.0, 2.0, 2.0),
         Rectangle(4.06, 8.0, 2.0, 2.0)],   # Environment 1
        [Rectangle(0.0, 14.0, 2.0, 2.0),
         Rectangle(4.12, 14.0, 2.0, 2.0)],  # Environment 2
        [Rectangle(0.0, 22.0, 2.0, 2.0),
         Rectangle(4.2, 22.0, 2.0, 2.0)],   # Environment 3
        [Rectangle(0.0, 28.0, 2.0, 2.0),
         Rectangle(4.25, 28.0, 2.0, 2.0)],  # Environment 4
        [Rectangle(0.0, 34.0, 2.0, 2.0),
         Rectangle(4.3, 34.0, 2.0, 2.0)],    # Environment 5
        [Rectangle(0.0, 40.0, 2.0, 2.0),
         Rectangle(4.5, 40.0, 2.0, 2.0)],    # Environment 6
        [Rectangle(0.0, 46.0, 2.0, 2.0),
         Rectangle(4., 46.0, 2.0, 2.0)]    # Environment 7
    ]

    start_zones = [
        Rectangle(0.0, 0.0, 0.25, 0.25),   # Environment 0
        Rectangle(1., 8.0, 0.5, 0.5),      # Environment 1
        Rectangle(1., 14.0, 0.5, 0.5),     # Environment 2
        Rectangle(1., 22.0, 0.5, 0.5),     # Environment 3
        Rectangle(1., 28.0, 0.5, 0.5),     # Environment 4
        Rectangle(1., 34.0, 0.5, 0.5),      # Environment 5
        Rectangle(1., 40.0, 0.5, 0.5),      # Environment 6
        Rectangle(1., 46.0, 0.5, 0.5)      # Environment 7
    ]

    goal_zones = [
        Rectangle(2., 0.0, 0.5, 0.5),      # Environment 0
        Rectangle(3.06, 8.0, 0.5, 0.5),     # Environment 1
        Rectangle(3.12, 14.0, 0.5, 0.5),    # Environment 2
        Rectangle(3.2, 22.0, 0.5, 0.5),     # Environment 3
        Rectangle(3.55, 28.0, 0.5, 0.5),    # Environment 4
        Rectangle(3.3, 34.0, 0.5, 0.5),      # Environment 5
        Rectangle(3.3, 40.0, 0.5, 0.5),      # Environment 6
        Rectangle(3., 46.0, 0.5, 0.5)      # Environment 7
    ]

    holes = []

    return environments, start_zones, goal_zones, holes

def create_environment_baseline():
    # Define environments with rectangles
    environments = [
        [Rectangle(0.0, 0.0, 4.0, 4.0)],    # Environment 0
        [Rectangle(0.0, 0.0, 4.0, 4.0)],    # Environment 1
        [Rectangle(0.0, 0.0, 4.0, 4.0)],    # Environment 2
    ]

    start_zones = [
        Rectangle(0., 0., 0.25, 0.25),   # Environment 0
        Rectangle(0., 0., 0.25, 0.25),   # Environment 1
        Rectangle(0., 0., 0.25, 0.25),   # Environment 2
    ]

    goal_zones = [
        Rectangle(0.6, 0.0, 0.5, 0.25),   # Environment 0
        Rectangle(0.8, 0.0, 0.5, 2.),   # Environment 1
        Rectangle(2., 0.0, 0.5, 2.),   # Environment 2
    ]

    holes = []

    return environments, start_zones, goal_zones, holes

def create_environment_baseline_holes():
    # Define environments with rectangles
    environments = [
        [Rectangle(0.0, -1.0, 4.0, 4.0)],    # Environment 0
        [Rectangle(0.0, -1.0, 4.0, 4.0)],    # Environment 1
        [Rectangle(0.0, 8.06, 4.0, 4.0)],    # Environment 2
    ]

    start_zones = [
        Rectangle(0., -1., 0.25, 0.25),   # Environment 0
        Rectangle(0., -1., 0.25, 0.25),   # Environment 1
        Rectangle(0., 8.06, 0.25, 0.25),   # Environment 2
    ]

    goal_zones = [
        Rectangle(0.8, -1.0, 0.5, 2.),   # Environment 0
        Rectangle(2., -1.0, 0.5, 2.),   # Environment 1
        Rectangle(0., 8.06, 3.5, 3.5),   # Environment 2
    ]

    holes = [[],
        [],
        [Hole(-1.0, 9.260000000000002, 0.2),
        Hole(0.7999999999999998, 10.06, 0.4),
        Hole(-2.0, 10.06, 0.28),
        Hole(-1.0, 11.06, 0.44),
        Hole(3.0, 8.86, 0.48),
        Hole(2.08, 10.18, 0.24),
        Hole(1.4800000000000004, 8.06, 0.32),
        Hole(-2.12, 8.540000000000001, 0.4),
        Hole(-1.96, 6.18, 0.4),
        Hole(2.88, 6.66, 0.32),
        Hole(0.6000000000000005, 4.66, 0.4),
        Hole(-0.8799999999999999, 5.42, 0.2),
        Hole(2.4000000000000004, 5.5, 0.32),
        Hole(-2.0, 5.0600000000000005, 0.32),
        Hole(1.0, 6.460000000000001, 0.28),
        # Hole(0.4800000000000004, 8.86, 0.2),
        Hole(-0.7999999999999998, 9.86, 0.2),
        Hole(-2.8, 7.0600000000000005, 0.4),
        Hole(1.2000000000000002, 9.06, 0.2)]
    ]

    return environments, start_zones, goal_zones, holes



def is_inside_holes(x, y, holes):
    for hole in holes:
        if hole.is_inside(x, y):
            return True
    return False

def intersect_holes(x, y, holes, dwidth):
    for hole in holes:
        if hole.intersects(x, y, dwidth):
            return True
    return False