import numpy as np

class Particle:
    def __init__(self, mass, position, velocity):
        self.mass = mass
        # The force accumulator is used to store the sum of all forces acting on the particle before the numerical integration step
        self.force_accumulator = np.zeros(3, dtype=np.float64)
        self.position = np.array(position, dtype=np.float64)
        self.velocity = np.array(velocity, dtype=np.float64)
        # To store the total applied force (optional, useful for display/debugging)
        self.force = np.zeros(3, dtype=np.float64)
        self.radius = 1.0

    def apply_force(self, force):
        self.force_accumulator += force
        self.force += np.copy(self.force_accumulator)

    def integrate(self, dt):
        acceleration = self.force_accumulator / self.mass
        # Velocity update
        self.velocity += acceleration * dt
        # Position update
        self.position = self.position + (self.velocity * dt)
        # Clear the force accumulator
        self.force_accumulator[:] = 0.0

    def update(self, dt):
        acceleration = self.force / self.mass
        self.velocity += acceleration * dt
        self.position += self.velocity * dt

        self.force = np.zeros(3, dtype=np.float64)