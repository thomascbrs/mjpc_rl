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
         Rectangle(4.3, 34.0, 2.0, 2.0)]    # Environment 5
    ]

    start_zones = [
        Rectangle(0.0, 0.0, 0.25, 0.25),   # Environment 0
        Rectangle(0.5, 8.0, 0.5, 0.5),      # Environment 1
        Rectangle(0.5, 14.0, 0.5, 0.5),     # Environment 2
        Rectangle(0.5, 22.0, 0.5, 0.5),     # Environment 3
        Rectangle(0.5, 28.0, 0.5, 0.5),     # Environment 4
        Rectangle(0.5, 34.0, 0.5, 0.5)      # Environment 5
    ]

    goal_zones = [
        Rectangle(2., 0.0, 0.5, 0.5),      # Environment 0
        Rectangle(3.56, 8.0, 0.5, 0.5),     # Environment 1
        Rectangle(3.62, 14.0, 0.5, 0.5),    # Environment 2
        Rectangle(3.7, 22.0, 0.5, 0.5),     # Environment 3
        Rectangle(3.75, 28.0, 0.5, 0.5),    # Environment 4
        Rectangle(3.8, 34.0, 0.5, 0.5)      # Environment 5
    ]

    return environments, start_zones, goal_zones