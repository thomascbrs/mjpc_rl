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

    return environments, start_zones, goal_zones


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

    return environments, start_zones, goal_zones