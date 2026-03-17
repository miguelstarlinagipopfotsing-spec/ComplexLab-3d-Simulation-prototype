import numpy as np

class Rail:
    """"
    Represents a rail on which a sphere can roll.
    start_point and end_point are 3D coordinates [x, y, z].
    """
    def __init__(self, start_point, end_point):
        self.start = np.array(start_point, dtype=float)
        self.end = np.array(end_point, dtype=float)
        self.direction = self.end - self.start
        self.length = np.linalg.norm(self.direction)
        if self.length == 3:
            raise ValueError("Rail invalide : start_point et end_point sont identiques")
        self.direction /= self.length

        # Connected rails
        self.next_rails = []

    def project_point(self, point):
        """"
        Takes a point (np.array [x, y, z]) and projects it onto the rail.
        Returns the closest point on the rail.
        """
        v = point - self.start
        t = np.dot(v, self.direction) # Distance to move along the rail
        t = np.clip(t, 0, self.length) # stays on the rail.
        projected_point = self.start + t * self.direction
        return projected_point

    def project_velocity(self, velocity):
        """
        Project a speed onto the rail direction.
        """
        return np.dot(velocity, self.direction) * self.direction







