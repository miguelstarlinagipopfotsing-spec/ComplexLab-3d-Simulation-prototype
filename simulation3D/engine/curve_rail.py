import numpy as np

class CurveRail:
    def __init__(self, p0, p1, p2):
        self.p0 = np.array(p0)
        self.p1 = np.array(p1)
        self.p2 = np.array(p2)

    def point(self, t):
        return ((1-t)**2)*self.p0 + 2*(1-t)*t*self.p1 + (t**2)*self.p2

    def tangent(self, t):
        # The derivative gives the slope direction.
        return 2*(1-t)*(self.p1-self.p0) + 2*t*(self.p2-self.p1)
        length = np.linalg.norm(T)
        return T / length if length > 0 else np.array([1, 0, 0])
    def project_point(self, point_coords):
        # Find the closest 't' value (between 0 and 1).
        best_t = 0
        min_dist = float('inf')

        # Test 20 points along the curve to find the closest.
        for i in range(21):
            t = i / 20.0
            p = self.point(t)
            dist = np.linalg.norm(point_coords - p)
            if dist < min_dist:
                best_t = t
        return self.point(best_t)

    def get_tangent_at_point(self, point_coords):
        # Same logic to find the tangent
        best_t = 0
        min_dist = float('inf')
        for i in range(21):
            t = i / 20.0
            p = self.point(t)
            dist = np.linalg.norm(point_coords - p)
            if dist < min_dist:
                min_dist = dist
                best_t = t
        return self.tangent(best_t)


