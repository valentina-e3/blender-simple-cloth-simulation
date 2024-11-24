import bpy, bmesh
import mathutils as m

DAMPING = 0.1  # Damping constant to reduce force
TIME_STEP = 0.01  # Sampling time
CONSTRAINT_ITERATOR = 50  # Determines how many times the spring length will be adjusted; higher values make the cloth stiffer


class Particle:
    """
    Particle class, describes a particle within the cloth model
    """

    def __init__(self, pos, movable):
        """
        Particle object constructor
        :param pos: Blender reference, used as the current position of the particle
        :param movable: Boolean, indicates if the particle is fixed or not
        :ivar pos: Blender reference, stores the parameter `pos`
        :ivar old_pos: mathutils.Vector, the previous position of the particle
        :ivar acceleration: mathutils.Vector, particle's acceleration
        :ivar movable: Stores the `movable` parameter
        :ivar mass: Integer, particle's mass
        """
        self.pos = pos
        self.old_pos = pos.co
        self.acceleration = m.Vector((0, 0, 0))
        self.movable = movable
        self.mass = 1

    def addForce(self, force):
        """
        Method to convert force into acceleration
        :param force: mathutils.Vector, force acting on the particle
        """
        self.acceleration += force / self.mass

    def update(self):
        """
        Method to update the position of the particle using Verlet integration
        """
        if self.movable:
            temp = self.pos.co
            self.pos.co += (self.pos.co - self.old_pos) * (1.0 - DAMPING) + self.acceleration * TIME_STEP
            self.old_pos = temp
            self.resetAcceleration()

    def resetAcceleration(self):
        """
        Method to reset the particle's acceleration to zero
        """
        self.acceleration = m.Vector((0, 0, 0))

    def makeUnmovable(self):
        """
        Method to fix the particle's position
        """
        self.movable = False


class Spring:
    """
    Spring class, describes a spring within the cloth model
    """

    def __init__(self, p1, p2):
        """
        Spring object constructor
        :param p1: Particle, first particle connected to the spring
        :param p2: Particle, second particle connected to the spring
        :ivar p1: Particle, stores the parameter `p1`
        :ivar p2: Particle, stores the parameter `p2`
        :ivar rest_distance: Double, initial distance between `p1` and `p2`
        """
        self.p1 = p1
        self.p2 = p2
        v = m.Vector(p1.pos.co - p2.pos.co)
        self.rest_distance = v.magnitude

    def satisfyConstraint(self):
        """
        Method to adjust the positions of particles to maintain the realistic spring length
        """
        p1_p2 = m.Vector(self.p1.pos.co - self.p2.pos.co)
        current_distance = p1_p2.magnitude
        correctionVector = p1_p2 * (1 - self.rest_distance / current_distance)
        correctionVectorHalf = correctionVector * 0.5
        if self.p1.movable:
            self.p1.pos.co -= correctionVectorHalf
        if self.p2.movable:
            self.p2.pos.co -= (correctionVectorHalf * -1)


class Cloth:
    """
    Cloth class, describes the cloth model
    """

    def __init__(self, obj):
        """
        Cloth object constructor
        :param obj: Blender reference, the object representing the cloth
        :ivar obj: Blender reference, stores the parameter `obj`
        :ivar num_of_particles: Integer, number of particles in the cloth
        :ivar num_of_springs: Integer, number of springs in the cloth
        :ivar particles: List of Particle objects, contains all particles in the cloth
        :ivar springs: List of Spring objects, contains all springs in the cloth
        """
        self.obj = obj
        self.num_of_particles = len(self.obj.data.vertices)
        self.num_of_springs = len(self.obj.data.edges)
        self.particles = []
        self.springs = []

    def fillParticles(self):
        """
        Method to populate the `particles` list with Particle objects, creating each object by passing 
        a reference to an individual vertex of the object representing the cloth
        """
        for i in range(self.num_of_particles):
            self.particles.append(Particle(self.obj.data.vertices[i], True))

    def fillSprings(self):
        """
        Method to populate the `springs` list with Spring objects, creating them by passing two Particle 
        objects from the `particles` list based on edge data from the object representing the cloth
        """
        for i in range(self.num_of_springs):
            self.springs.append(Spring(self.particles[self.obj.data.edges[i].vertices[0]], self.particles[self.obj.data.edges[i].vertices[1]]))

    def makeParticlesUnmovable(self, particle_indices):
        """
        Method to fix specific particles
        :param particle_indices: List of integers, indices of particles to fix
        """
        for i in particle_indices:
            self.particles[i].makeUnmovable()

    def update(self):
        """
        Method to update the cloth
        """
        for i in self.particles:
            i.update()
        for _ in range(CONSTRAINT_ITERATOR):
            for j in self.springs:
                j.satisfyConstraint()

    def addForce(self, force):
        """
        Method to apply a force to each particle in the cloth
        :param force: mathutils.Vector, force acting on the cloth
        """
        for i in self.particles:
            i.addForce(force)

    def calculateTriangleNormal(self, pos1, pos2, pos3):
        """
        Method to calculate the normal of a triangle defined by three particles
        :param pos1: mathutils.Vector, first particle defining the triangle
        :param pos2: mathutils.Vector, second particle defining the triangle
        :param pos3: mathutils.Vector, third particle defining the triangle
        :return: mathutils.Vector, cross product of two edges of the triangle
        """
        v1 = pos2 - pos1
        v2 = pos3 - pos1
        return v1.cross(v2)

    def addWindForce(self, direction):
        """
        Method to simulate wind effects using a vector describing the wind direction and the normals 
        of triangles to calculate the force acting on the particles of each triangle. Triangles are 
        derived from the vertex indices of the object polygons.
        :param direction: mathutils.Vector, vector describing the wind direction
        """
        for i in range(len(self.obj.data.polygons)):
            p1 = self.particles[self.obj.data.polygons[i].vertices[0]]
            p2 = self.particles[self.obj.data.polygons[i].vertices[1]]
            p3 = self.particles[self.obj.data.polygons[i].vertices[2]]
            p4 = self.particles[self.obj.data.polygons[i].vertices[3]]

            normal1 = self.calculateTriangleNormal(p1.pos.co, p2.pos.co, p3.pos.co)
            n1 = m.Vector(normal1)
            n1.normalize()
            force1 = normal1 * (n1.dot(direction))
            p1.addForce(force1)
            p2.addForce(force1)
            p3.addForce(force1)

            normal2 = self.calculateTriangleNormal(p1.pos.co, p4.pos.co, p3.pos.co)
            n2 = m.Vector(normal2)
            n2.normalize()
            force2 = normal2 * (n2.dot(direction))
            p1.addForce(force2)
            p4.addForce(force2)
            p3.addForce(force2)

    def ballCollision(self, radius, location):
        """
        Method to handle collision with a sphere
        :param radius: Double, radius of the sphere
        :param location: mathutils.Vector, center of the sphere
        """
        for i in self.particles:
            if i.movable:
                v = i.pos.co - location
                l = v.magnitude
                if l < radius:
                    v.normalize()
                    v *= radius - l
                    i.pos.co += v


def makeTriangles(data):
    """
    Function that diagonally connects the vertices within the polygons of a Plane object. 
    It modifies edge data but does not alter polygon data.
    :param data: Blender reference, data of the object
    """
    bpy.ops.object.editmode_toggle()
    bm = bmesh.from_edit_mesh(data)

    bm.verts.ensure_lookup_table()
    for i in range(len(data.polygons)):
        v1 = bm.verts[data.polygons[i].vertices[0]]
        v2 = bm.verts[data.polygons[i].vertices[1]]
        v3 = bm.verts[data.polygons[i].vertices[2]]
        v4 = bm.verts[data.polygons[i].vertices[3]]
        bm.edges.new((v1, v3))
        bm.edges.new((v2, v4))

    bmesh.update_edit_mesh(data)
    bpy.ops.object.editmode_toggle()


# Documentation: To see printed documentation, go to Window -> Toggle System Console
# help(Particle)
# help(Spring)
# help(Cloth)
# help(makeTriangles)

# Delete all existing objects
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

# Add a Plane object representing the cloth and perform modifications
bpy.ops.mesh.primitive_plane_add(enter_editmode=False, location=(0, 0, 0), size=10)
bpy.ops.object.editmode_toggle()
bpy.ops.mesh.subdivide(number_cuts=30)
bpy.ops.object.editmode_toggle()
makeTriangles(bpy.data.objects['Plane'].data)
bpy.ops.object.modifier_add(type='BEVEL')
bpy.context.object.modifiers["Bevel"].segments = 4

# Add material for the cloth
cloth_material = bpy.data.materials.get("ClothMaterial")
bpy.data.objects['Plane'].data.materials.append(cloth_material)

# Add a sphere
ball_radius = 3
ball_location = (-10, 0, -7)
bpy.ops.mesh.primitive_uv_sphere_add(radius=ball_radius, enter_editmode=False, location=ball_location)
bpy.ops.object.shade_smooth()
ball_radius += 0.57  # Increase sphere radius slightly to prevent parts of the cloth from ending up inside the sphere

# Add material for the sphere
sphere_material = bpy.data.materials.get("SphereMaterial")
bpy.data.objects['Sphere'].data.materials.append(sphere_material)

# Add background
bpy.ops.mesh.primitive_plane_add(enter_editmode=False, location=(-30, 0, 5), size=60)
bpy.data.objects["Plane.001"].rotation_euler = (0.0, 1.57, 0.0)

bpy.ops.mesh.primitive_plane_add(enter_editmode=False, location=(0, -30, 5), size=60)
bpy.data.objects["Plane.002"].rotation_euler = (1.57, 0.0, 0.0)

bpy.ops.mesh.primitive_plane_add(enter_editmode=False, location=(0, 0, -25), size=60)

# Add material for the background
wall_material1 = bpy.data.materials.get("WallMaterial1")
wall_material2 = bpy.data.materials.get("WallMaterial2")
floor_material = bpy.data.materials.get("FloorMaterial")
bpy.data.objects['Plane.001'].data.materials.append(wall_material1)
bpy.data.objects['Plane.002'].data.materials.append(wall_material2)
bpy.data.objects['Plane.003'].data.materials.append(floor_material)

# Add lighting
lamp_data = bpy.data.lights.new(name="lamp", type='SUN')
lamp_object = bpy.data.objects.new(name="Sun", object_data=lamp_data)
bpy.context.collection.objects.link(lamp_object)
lamp_object.location = (37.0, 5.392792701721191, 36.33957290649414)
lamp_object.rotation_euler = (0.16045404970645905, 0.801373302936554, 0.6719042062759399)
lamp_object.data.energy = 2

# Add a camera
cam_data = bpy.data.cameras.new(name="cam")
cam_ob = bpy.data.objects.new(name="Camera", object_data=cam_data)
bpy.context.collection.objects.link(cam_ob)
cam_ob.location = (42.30911636352539, 2.881232738494873, 16.553810119628906)
cam_ob.rotation_euler = (1.1182160377502441, 5.3443007345777005e-05, -4.648222923278809)
bpy.context.scene.camera = bpy.data.objects['Camera']

# Create a Cloth object
cloth = Cloth(bpy.data.objects['Plane'])
cloth.fillParticles()
cloth.fillSprings()
unmovable = [0, 2]
cloth.makeParticlesUnmovable(unmovable)

gravity = m.Vector((0, 0, -9.8))
wind = m.Vector((80, 0, -30))

bpy.context.scene.frame_set(0)
bpy.context.scene.frame_end = 300

# Update the scene
def my_handler(scene):
    cloth.addForce(gravity)
    cloth.addWindForce(wind)
    bpy.data.objects['Sphere'].location.x += 0.1
    bpy.data.objects['Sphere'].data.update()
    cloth.ballCollision(ball_radius, m.Vector(bpy.data.objects['Sphere'].location))
    cloth.update()

# Call the `my_handler` function for every frame
bpy.app.handlers.frame_change_pre.append(my_handler)

# Rendering
# replace with your filepath 
# bpy.context.scene.render.filepath = "C:\\Users\\Valentina\\Desktop\\blender_projects\\collision_and_wind"
# bpy.ops.render.render(animation=True)
