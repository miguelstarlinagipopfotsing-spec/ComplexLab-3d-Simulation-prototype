import panda3d
from direct.showbase.ShowBase import ShowBase
from direct.showbase.ShowBaseGlobal import globalClock
from direct.task import Task
import numpy as np
from panda3d.core import LineSegs # Allows drawing lines in 3D space.
from panda3d.core import CollisionTraverser
from panda3d.core import CollisionNode
from panda3d.core import CollisionRay
from panda3d.core import CollisionHandlerQueue
from panda3d.core import CollisionSphere
from panda3d.core import BitMask32
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import TextNode
from panda3d.core import Point2
from panda3d.core import CollisionTube
from wcwidth import center

from engine.curve_rail import CurveRail
from engine.physics import Particle
from engine.Rail import Rail

class SimulationApp(ShowBase):
    def __init__(self):
        super().__init__()
        # rail creation system
        self.creating_rail = False
        self.rail_start_point = None

        self.particles = []
        self.models = []
        self.rails = []
        self.rail_nodes = []

        self.selected_rail = None
        self.rail_visuals = {} # Dictionary to map the rail to its drawing
        self.is_d_pressed = False

        self.accept("d", self.set_d_on)
        self.accept("d-up", self.set_d_off)

        # Add and Delete ('P' and 'Delete' keys)
        self.accept("p", self.add_particle_at_mouse)
        self.accept("delete", self.delete_selected)
        self.accept("control-z", self.undo_last_rail)

        self.accept("r", self.toggle_rail_creation)
        # Collision system
        self.picker = CollisionTraverser()
        self.pq = CollisionHandlerQueue()

        self.pickerNode = CollisionNode("mouseRay")
        self.pickerNP = self.camera.attachNewNode(self.pickerNode)

        self.pickerRay = CollisionRay()
        self.pickerNode.addSolid(self.pickerRay)

        self.pickerNode.setFromCollideMask(BitMask32.bit(1))
        self.picker.addCollider(self.pickerNP, self.pq)

        curve = CurveRail(
            [0,0,5],
            [0,10,5],
            [10,0,0]
        )
        curve.next_rails = []
        self.draw_curve(curve)
        self.rails.append(curve)

        # Rail building height
        self.build_z = 0.0
        self.accept("q", self.increase_z)
        self.accept("e", self.decrease_z)

        # Disable default mouse control
        self.disableMouse()
        # Set the camera position
        self.camera.setPos(0, 0, 0)
        # Point the camera at the center
        self.camera.lookAt(0, 0, 0)
        # Camera variable.
        self.cam_distance = 40
        self.cam_yaw = 0
        self.cam_pitch = 20

        self.last_mouse = None

        # Add mouse avents
        self.cam_yaw = 0
        self.cam_pitch = 0
        self.cam_distance = 20  # Ajuste la distance selon ton besoin
        self.last_mouse = None
        self.accept("mouse3", self.start_camera_rotate)
        self.accept("mouse3-up", self.stop_camera_rotate)
        self.accept("wheel_up", self.zoom_in)
        self.accept("wheel_down", self.zoom_out)

        # Creation Preview Slider
        self.cursor = self.loader.loadModel("models/smiley")
        self.cursor.setScale(0.2)
        self.cursor.setAlphaScale(0.5) # A bit transparent
        self.cursor.reparentTo(self.render)
        self.cursor.hide() # Hidden by default

        # Creates a rail between two points
        railA = Rail(start_point=[-20, 0, 0], end_point=[0, 0, 5])
        railB = Rail(start_point=[0, 0, 5], end_point=[10, 5, 5])
        railC = Rail(start_point=[0, 0, 5], end_point=[10, -5, 5])
        railD = Rail(start_point=[10, -5, 5], end_point=[25, -5, 1])
        railE = Rail(start_point=[25, -5, 1], end_point=[40, -5, 3])
        railF = Rail(start_point=[40, -5, 3], end_point= [55, -5, 0])
        railG = Rail(start_point=[20, -5, 1], end_point=[25, 5, 4])
        railH = Rail(start_point=[20, 10, 3], end_point=[15, 5, 2])
        railI = Rail(start_point=[20, 0, 1], end_point=[30, 0, 0])

        railA.next_rails.append(railB)
        railA.next_rails.append(railC)
        railA.next_rails.append(curve)
        railC.next_rails.append(railD)
        railD.next_rails.append(railE)
        railE.next_rails.append(railF)
        railD.next_rails.append(railG)
        railG.next_rails.append(railH)
        railH.next_rails.append(railI)

        self.rails = [railA, railB, railC, railD, railE, railF, railG, railH, railI]

        # Draw the rail at startup
        for rail in self.rails:

            self.draw_rails(rail)

        # -----------------
        # 1️⃣ Create a particle
        # -----------------
        self.particles = []
        self.models = []
        self.gravity = np.array([0, 0, -9.81])
        # Create two particle
        for i in range(5):

            v = np.array([0, 0, 0])

            if i == 0:
                v = np.array([5.0, 0.0, 0.0])

            p = Particle(
                position = np.array([-5.0 + i*2.2, 0.0, 0.0]),
                velocity = v,
                mass = 1.0
            )
            p.rail = self.rails[0]
            self.particles.append(p)

        # Create the particle's graphical mode
        for p in self.particles:

            model = self.loader.loadModel("models/smiley") # Panda3D has a simple sphere model
            cnode = CollisionNode('sphere')
            cnode.addSolid(CollisionSphere(0,0,0,p.radius))
            cnode.setIntoCollideMask(BitMask32.bit(1))

            cnodepath = model.attachNewNode(cnode)
            model.reparentTo(self.render) # attach it to the main render
            model.setScale(p.radius) # Height of the sphere
            # Create the list to store the models
            self.models.append(model) # Add the model to the list

        # Add the coef of restitution
        self.restitution = 0.9
        # Add the friction coef
        self.friction = 0.2

        # Tracks the selected sphere
        self.selected_particle = None

        # Event: left click
        self.accept("mouse1", self.on_mouse_click)
        self.accept("mouse1-up", self.on_mouse_release)
        # -----------------
        # 2️⃣ Add the update function (simulation loop)
        # -----------------
        self.taskMgr.add(self.update_simulation, "update_simulation")
        self.ui_text = OnscreenText(
            text="Select Particle",
            pos=(-1.3, 0.9),
            scale=0.06,
            fg=(1, 1, 1, 1),
            align=TextNode.ALeft,
            mayChange=True,
        )
        self.build_giant_network()

        # Set up the Help Menu
        self.setup_help_menu()
        self.accept("h", self.toggle_help)

        # Add a small permanent hint on the screen
        self.hint_text = OnscreenText(
            text="Press [H] for Help",
            pos=(1.0, -0.9),  # Bottom-right corner
            scale=0.05,
            fg=(1, 1, 0, 1),  # Yellow text
            align=TextNode.ARight
        )

    def draw_rails(self, rail):

        lines = LineSegs()

        lines.setThickness(8)
        lines.setColor(1, 0, 0, 1) # Red

        lines.moveTo(
            rail.start[0],
            rail.start[1],
            rail.start[2]
        )

        lines.drawTo(
            rail.end[0],
            rail.end[1],
            rail.end[2]
        )

        node = lines.create()
        nodepath = self.render.attachNewNode(node)

        cnode = CollisionNode('rail_col')
        # Create a tube between point A and point B.
        tube = CollisionTube(rail.start[0], rail.start[1], rail.start[2],
                             rail.end[0], rail.end[1], rail.end[2], 0.5)
        cnode.addSolid(tube)
        cnode.setIntoCollideMask(BitMask32.bit(1))

        cnode_path = nodepath.attachNewNode(cnode)
        cnode_path.setPythonTag("owner", rail) # Link collision to the rail object


        self.rail_nodes.append(nodepath)
        self.rail_visuals[rail] = nodepath # Store the visual to move it later.

    # -----------------
    # 3️⃣ Update function
    # -----------------
    def update_simulation(self, task):
        self.update_camera()
        self.draw_forces()
        dt = globalClock.getDt()
        if dt > 0.1: dt = 0.016

        # --- 1. MOUSE MANAGEMENT (DRAG & DROP) ---
        # --- DRAGGING THE PARTICLE WITH THE MOUSE ---
        if self.selected_particle is not None and self.mouseWatcherNode.hasMouse():
            # Temporarily set ground plane to particle height (build_z)
            old_z = self.build_z
            self.build_z = self.selected_particle.position[2]
            pos_souris_3d = self.mouse_to_world()
            self.build_z = old_z  # Reset ground height to default


            if pos_souris_3d is not None:
                # Find the nearest rail to the mouse cursor
                rail_le_plus_proche = self.selected_particle.rail
                distance_min = float('inf')

                for rail in self.rails:
                    point_projete = rail.project_point(pos_souris_3d)
                    distance = np.linalg.norm(pos_souris_3d - point_projete)
                    if distance < distance_min:
                        distance_min = distance
                        rail_le_plus_proche = rail

                # Assign the new rail and force the position
                self.selected_particle.rail = rail_le_plus_proche
                self.selected_particle.position = rail_le_plus_proche.project_point(pos_souris_3d)
                self.selected_particle.velocity = np.zeros(3)  # Stop the ball when held

                # Visual cursor update
                if self.creating_rail:
                    pos = self.mouse_to_world()
                    if pos is not None:
                        self.cursor.show()
                        self.cursor.setPos(pos[0], pos[1], pos[2])
                    else:
                        self.cursor.hide()
                else:
                    self.cursor.hide()

        for p in self.particles:
            if p == self.selected_particle:
                continue

            # Apply gravity
            p.apply_force(np.array([0.0, 0.0, -9.81 * p.mass]))

            # Physic integration
            p.integrate(dt)

            # --- PROJECTION ONTO THE RAIL (Straight or Curved) ---
            # Ask the rail to snap the ball back onto it
            p.position = p.rail.project_point(p.position)

            # Adjust the velocity to follow the rail's direction
            if hasattr(p.rail, "get_tangent_at_point"):  # It's a curve
                tangent = p.rail.get_tangent_at_point(p.position)
            else:  # C'est un rail droit
                if hasattr(p.rail, 'direction'):
                    tangent = p.rail.direction
                else:
                    diff = p.rail.end - p.rail.start
                    tangent = diff / np.linalg.norm(diff)

            p.velocity = np.dot(p.velocity, tangent) * tangent
        # --- DRAGGING THE RAIL WITH THE MOUSE ---
        if self.selected_rail is not None and self.mouseWatcherNode.hasMouse() and self.is_d_pressed:
            pos_3d = self.mouse_to_world()
            if pos_3d is not None:
                # Calculate the current rail center.
                center = (self.selected_rail.start + self.selected_rail.end) / 2

                # How much should the center move to follow the mouse?
                delta = pos_3d - center

                # Move the physical points.
                self.selected_rail.start += delta
                self.selected_rail.end += delta

                # Update the visual by moving its entire NodePath.
                nodepath = self.rail_visuals[self.selected_rail]
                nodepath.setPos(nodepath.getPos() + panda3d.core.Point3(delta[0], delta[1], delta[2]))

        # --- 2. PHYSICS AND CONSTRAINTS ---
        for particle in self.particles:
            if particle == self.selected_particle:
                continue  # Skip gravity while holding the bal

            rail = particle.rail

            if hasattr(rail, "get_tangent_at_point"):
                # It's a curve, we request the tangent at the current position
                rail_dir = rail.get_tangent_at_point(particle.position)
            else:
                # It's a straight rail, we use its fixed direction.
                rail_dir = rail.end - rail.start
                rail_dir = rail_dir / np.linalg.norm(rail_dir)

            # Forces
            gravity_along_rail = np.dot(self.gravity, rail_dir) * rail_dir
            friction_force = -self.friction * particle.velocity
            total_force = (particle.mass * gravity_along_rail) + friction_force
            particle.apply_force(total_force)

            # Integration (Mouvement)
            particle.integrate(dt)

            # Constraint (Snap particle back to the rail)
            particle.position = rail.project_point(particle.position)
            # Ensure the velocity stays aligned with the rail axis.
            if hasattr(rail, "project_velocity"):
                particle.velocity = rail.project_velocity(particle.velocity)
            else:
                particle.velocity = np.dot(particle.velocity, rail_dir) * rail_dir

            # End of rail (Automatic transition)
            # For a curve, check the distance to P2 (the end point)
            target_end = rail.p2 if hasattr(rail, "p2") else rail.end
            dist_to_end = np.linalg.norm(particle.position - target_end)
            if dist_to_end < 0.5 and len(rail.next_rails) > 0:
                particle.rail = np.random.choice(rail.next_rails)

        # --- 3. Collisions ---
        for i in range(len(self.particles)):
            for j in range(i + 1, len(self.particles)):
                p1 = self.particles[i]
                p2 = self.particles[j]

                delta = p2.position - p1.position
                distance = np.linalg.norm(delta)

                if 0 < distance < (p1.radius + p2.radius):
                    normal = delta / distance
                    overlap = (p1.radius + p2.radius) - distance

                    # --- New logic for dragging ---
                    if p1 == self.selected_particle:
                        # If we hold p1, only p2 moves !
                        p2.position = p2.position + normal * overlap
                        p2.position = p2.rail.project_point(p2.position)
                        # Give p2 a little "Kick" in velocity
                        p2.velocity = p2.velocity + normal * 2.0
                        continue # Skip the rest of the math for this pair
                    elif p2 == self.selected_particle:
                        # If we hold p2, only p1 moves !
                        p1.position = p1.position - normal * overlap
                        p1.position = p1.rail.project_point(p1.position)
                        p1.velocity = p1.velocity - normal * 2.0
                        continue
                    # --- Standard collision (None are selected) ---
                    else:
                        # Separate both equally (0.5)
                        correction = normal * overlap * 0.5
                        p1.position = p1.position - correction
                        p2.position = p2.position + correction

                        p1.position = p1.rail.project_point(p1.position)
                        p2.position = p2.rail.project_point(p2.position)

                        # Re-calculate velocity bounce
                        relative_velocity = p1.velocity - p2.velocity
                        vel_along_normal = np.dot(relative_velocity, normal)

                        if vel_along_normal > 0:
                            j_impulse = -(1 + self.restitution) * vel_along_normal
                            j_impulse /= (1/p1.mass + 1/p2.mass)
                            impulse = j_impulse * normal
                            p1.velocity += impulse / p1.mass
                            p2.velocity -= impulse / p2.mass

        # --- 4. 3D RENDERING (Always last!) ---
        for i, particle in enumerate(self.particles):
            self.models[i].setPos(
                particle.position[0],
                particle.position[1],
                particle.position[2]
            )
        self.update_ui_advanced()

        return Task.cont

    def update_ui_advanced(self):
        if self.selected_particle:
            p = self.selected_particle
            v_mag = np.linalg.norm(p.velocity)

            # Real-time calculations
            momentum = p.mass * v_mag
            ke = 0.5 * p.mass * (v_mag ** 2)
            # Power is the dot product of Force and Velocity
            power = np.dot(p.force, p.velocity) if hasattr(p, 'force') else 0

            text = (
                f"--- ANALYSE TEMPS RÉEL ---\n"
                f"Position: {p.position[0]:.2f}, {p.position[1]:.2f}, {p.position[2]:.2f}\n"
                f"Vitesse: {v_mag:.3f} m/s\n"
                f"Force: {np.linalg.norm(p.force):.3f} N\n"
                f"Momentum: {momentum:.3f} N/s\n"
                f"Énergie Cinétique: {ke:.3f} J\n"
                f"Puissance: {power:.3f} W\n"
                f"--------------------------"
            )
            self.ui_text.setText(text)
        else:
            self.ui_text.setText("Click a ball to see physics data")

    # Add functions to handle mouse clicks
    def on_mouse_click(self):
        """" Checks if a sphere is clicked and selects it"""

        if self.creating_rail:
            pos = self.mouse_to_world()

            if pos is None:
                return

            if self.rail_start_point is None:
                self.rail_start_point = pos
                print("First point selected")
            else:
                new_rail = Rail(
                    start_point=self.rail_start_point,
                    end_point=pos
                )
                self.rails.append(new_rail)
                self.draw_rails(new_rail)
                print("Rails added")
                self.rail_start_point = None
            return

        if not self.mouseWatcherNode.hasMouse():
            return

        mpos = self.mouseWatcherNode.getMouse()

        self.pickerRay.setFromLens(
            self.camNode,
            mpos.getX(),
            mpos.getY()
        )

        self.picker.traverse(self.render)

        if self.pq.getNumEntries() > 0:

            pickedObj = self.pq.getEntry(0).getIntoNodePath()

            # 1. Did we click on a rail?
            if pickedObj.hasPythonTag('owner') and self.is_d_pressed:
                self.selected_rail = pickedObj.getPythonTag('owner')
                print("Rail selected !")
                return

            # 2. Did we click on a particle?
            pickedModel = pickedObj.getParent()

            for i, model in enumerate(self.models):
                if model == pickedModel:
                    self.selected_particle = self.particles[i]
                    print("Particle selected !")
                    break

    def on_mouse_release(self):
        """" Release the sphere"""
        self.selected_particle = None
        self.selected_rail = None

    def toggle_rail_creation(self):
        self.creating_rail = not self.creating_rail

        if self.creating_rail:
            print("Rail creation mode ON")
        else:
            print("Rail creation mode OFF")

    def set_d_on(self):
        self.is_d_pressed = True
    def set_d_off(self):
        self.is_d_pressed = False

    def mouse_to_world(self):
        if not self.mouseWatcherNode.hasMouse():
            return None

        mpos = self.mouseWatcherNode.getMouse()
        self.pickerRay.setFromLens(self.camNode, mpos.getX(), mpos.getY())
        # Get world space position and direction.
        origin = self.camera.getPos(self.render)
        direction = self.render.getRelativeVector(self.camera, self.pickerRay.getDirection())

        # If the ray is almost parallel to the ground, cancel it.
        if abs(direction.getZ()) < 0.001:
            return None

        t = (self.build_z - origin.getZ()) / direction.getZ()

        # Limit: Ignore if behind the camera (t < 0) or too far (t > 200 units) to prevent the 'infinite' effect
        if t < 0 or t > 200:
            return None

        pos = origin + direction * t
        return np.array([pos.getX(), pos.getY(), pos.getZ()])

    def draw_curve(self, curve):
        lines = LineSegs()
        lines.setThickness(8)
        lines.setColor(0,1,0,1)
        steps = 30

        for i in range (steps):
            t1 = i/steps
            t2 = (i + 1)/steps
            p1 = curve.point(t1)
            p2 = curve.point(t2)

            lines.moveTo(p1[0], p1[1], p1[2])
            lines.drawTo(p2[0], p2[1], p2[2])
        node = lines.create()
        self.render.attachNewNode(node)

    def increase_z(self):
        self.build_z += 1.0
        print(f"Building height z: {self.build_z}")
    def decrease_z(self):
        self.build_z -= 1.0
        print(f"Building height z: {self.build_z}")

    # Camera control function
    def start_camera_rotate(self):
        if self.mouseWatcherNode.hasMouse():
            self.last_mouse = Point2(self.mouseWatcherNode.getMouse())
            print("Rotating started with a copy of point A")
    def stop_camera_rotate(self):
        self.last_mouse = None
    def zoom_in(self):
        self.cam_distance -= 2
        if self.cam_distance < 5:
            self.cam_distance = 5
    def zoom_out(self):
        self.cam_distance += 2

    # Camera's update
    def update_camera(self):
        if self.last_mouse is not None and self.mouseWatcherNode.hasMouse():
            mpos = self.mouseWatcherNode.getMouse()

            # Displacement calculation
            dx = mpos.getX() - self.last_mouse.getX()
            dy = mpos.getY() - self.last_mouse.getY()

            # Updating angles
            self.cam_yaw += dx * 100
            self.cam_pitch += dy * 100

            # Vertical limit to prevent camera flipping
            self.cam_pitch = max(-80, min(80, self.cam_pitch))

            # Reference update (Static copy)
            self.last_mouse = Point2(mpos)

        # 2. Position calculation (Always active to maintain view)
        yaw_rad = np.radians(self.cam_yaw)
        pitch_rad = np.radians(self.cam_pitch)

        x = self.cam_distance * np.cos(pitch_rad) * np.sin(yaw_rad)
        y = self.cam_distance * np.cos(pitch_rad) * np.cos(yaw_rad)
        z = self.cam_distance * np.sin(pitch_rad)

        self.camera.setPos(x, y, z)
        self.camera.lookAt(0, 0, 0)

    def add_particle_at_mouse(self):
        mouse_pos = self.mouse_to_world()
        if mouse_pos is None or not self.rails:
            return

        # 1. Trouver le rail le plus proche
        best_rail = self.rails[0]
        min_dist = float('inf')
        for r in self.rails:
            projected = r.project_point(mouse_pos)
            dist = np.linalg.norm(mouse_pos - projected)
            if dist < min_dist:
                min_dist = dist
                best_rail = r

        # 2. Create the particle and its model (ONCE ONLY)
        if best_rail:
            new_p = Particle(mass=1.0, position=best_rail.project_point(mouse_pos), velocity=[0,0,0])
            new_p.rail = best_rail

            # --- NEW: 3D Model + Collision
            model = self.loader.loadModel("models/smiley")

            cnode = CollisionNode('sphere')
            cnode.addSolid(CollisionSphere(0,0,0, new_p.radius))
            cnode.setIntoCollideMask(BitMask32.bit(1))

            model.attachNewNode(cnode)
            model.reparentTo(self.render)
            model.setScale(new_p.radius)

            self.models.append(model)
            self.particles.append(new_p)
            print('Ball added and clickable!')

    def delete_selected(self):
        if self.selected_particle:
            self.selected_particle.model.removeNode()
            self.particles.remove(self.selected_particle)
            self.selected_particle = None

    def draw_forces(self):
        if hasattr(self, 'force_lines'):
            self.force_lines.removeNode()

        if self.selected_particle:
            segs = LineSegs()
            p = self.selected_particle

            # 1. Velocity Vector (Blue) - Multiply by 0.5 for visibility
            segs.setColor(0, 0, 1, 1)
            segs.moveTo(p.position[0], p.position[1], p.position[2])
            segs.drawTo(p.position[0] + p.velocity[0] * 1.0,
                        p.position[1] + p.velocity[1] * 0.5,
                        p.position[2] + p.velocity[2] * 0.5)

            # Vector Gravity (red)
            gravity = np.array([0, 0, -9.81])
            segs.setColor(1, 0, 0, 1)
            segs.moveTo(p.position[0], p.position[1], p.position[2])
            segs.drawTo(p.position[0], p.position[1], p.position[2] + gravity[2] * 0.2)

            self.force_lines = self.render.attachNewNode(segs.create())

    def undo_last_rail(self):
        if self.rail_nodes and self.rails:
            # On retire le dernier rail de la vue 3D.
            last_node = self.rail_nodes.pop()
            last_node.removeNode()
            # On le retire de la physique
            self.rails.pop()
            print("Dernier rail annulé !")

    def build_giant_network(self):
        # Liste de points [x, y, z]
        points = [
            [-20, 0, 10], [0, 0, 5], [10, 10, 8],
            [20, 0, 5], [10, -10, 2], [0, 0, 5],
            [10, 5,	5], [25, 10, 4], [35, 5, 3],
            [30, -5, 2], [20, -10, 1], [10, -10, 0]
        ]

        for i in range(len(points) - 1):
            start = points[i]
            end = points[i + 1]
            new_rail = Rail(start, end)

            # On connecte les rails entre eux pour que la bille continue sa route
            if len(self.rails) > 0:
                self.rails[-1].next_rails.append(new_rail)

            self.rails.append(new_rail)
            self.draw_rails(new_rail)

    def setup_help_menu(self):
        """Creates the on-screen help menu."""
        help_text = (
            "--- ComplexLab Controls ---\n"
            "[H] : Show / Hide this menu\n"
            "\n"
            "--- Camera ---\n"
            "Right-Click (Hold) : Rotate camera\n"
            "Mouse Wheel : Zoom in / out\n"
            "\n"
            "--- Particles ---\n"
            "Left-Click : Select a ball\n"
            "Left-Click (Hold) : Move selected ball\n"
            "[P] : Add a ball at mouse cursor\n"
            "[Delete] : Remove selected ball\n"
            "\n"
            "--- Rails & Building ---\n"
            "[R] : Toggle Rail Building Mode\n"
            "[Q] / [E] : Change building height\n"
            "[D] (Hold) + Click : Move a rail\n"
            "[Ctrl] + [Z] : Undo last rail\n"
        )

        # Create the text object on the right side of the screen
        self.help_display = OnscreenText(
            text=help_text,
            pos=(0.8, 0.8),  # Placed on the top-right to avoid the physics UI
            scale=0.05,
            fg=(1, 1, 1, 1),  # White text
            bg=(0, 0, 0, 0.6),  # Semi-transparent black background for readability
            align=TextNode.ALeft,
            mayChange=True
        )

        # Hide the menu by default when the program starts
        self.help_display.hide()

    def toggle_help(self):
        """Shows or hides the help menu."""
        if self.help_display.isHidden():
            self.help_display.show()
        else:
            self.help_display.hide()


# -----------------
# 4️⃣ Start the simulation
# -----------------
app = SimulationApp()
app.run()