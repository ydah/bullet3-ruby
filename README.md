# bullet3

Ruby bindings for the Bullet Physics SDK.

This project exposes a two-layer API:

- low-level Bullet3 C++ bindings exposed under `Bullet3`
- Ruby-friendly helpers and simulation APIs built on top of those bindings

The current native extension covers LinearMath, collision shapes,
`CollisionObject`/`CollisionWorld`, rigid bodies, dynamics worlds, constraints,
ray/contact/closest-point queries, raycast vehicles, soft bodies, multibodies,
primitive URDF loading, bundled data paths, and the high-level
`Bullet3::Simulation` facade.

## Installation

Add the gem from this repository:

```bash
gem "bullet3", path: "path/to/bullet3"
```

Then install dependencies:

```bash
brew install bullet
bundle install
```

On Linux, install the Bullet development package for your distribution before
running `bundle install`:

```bash
sudo apt install libbullet-dev
sudo dnf install bullet-devel
```

Compile the native extension before using the Bullet3-backed classes:

```bash
bundle exec rake compile
```

## Usage

```ruby
require "bullet3"

v = Bullet3::Vector3.new(1, 2, 3)
axis = Bullet3::Vector3.new(0, 0, 1)
rotation = Bullet3::Quaternion.from_axis_angle(axis, Math::PI / 2)
transform = Bullet3::Transform.new(rotation, Bullet3::Vector3.new(1, 0, 0))

p transform * v
```

High-level direct simulation:

```ruby
sim = Bullet3::Simulation.new
sim.set_gravity(0, -10, 0)

plane = sim.create_collision_shape(:static_plane, normal: [0, 1, 0], offset: 0)
sphere = sim.create_collision_shape(:sphere, radius: 1.0)

sim.create_rigid_body(mass: 0.0, collision_shape: plane)
body = sim.create_rigid_body(mass: 1.0, collision_shape: sphere, position: [0, 10, 0])

120.times { sim.step_simulation(time_step: 1.0 / 60.0, fixed_time_step: 1.0 / 60.0) }
p sim.get_base_position_and_orientation(body)
puts sim.get_contact_points(body_a: body).inspect
```

By default `require "bullet3"` loads the native extension when it has been
compiled. Set `BULLET3_SKIP_NATIVE=1` to force the pure Ruby fallback for
non-native classes.

Primitive URDF loading and data paths:

```ruby
sim = Bullet3::Simulation.new
sim.set_gravity(0, 0, -10)

plane = sim.load_urdf("plane.urdf", use_fixed_base: true)
cube = sim.load_urdf("cube.urdf", base_position: [0, 0, 3])

120.times { sim.step_simulation(time_step: 1.0 / 60.0) }
p sim.get_aabb(cube)
p sim.get_contact_points(body_a: plane, body_b: cube)
```

Lower-level APIs are available for direct Bullet3 usage:

```ruby
world = Bullet3::CollisionWorld.create
shape = Bullet3::Shapes::SphereShape.new(1.0)
object = Bullet3::CollisionObject.new(shape)
world.add_collision_object(object)

p world.ray_test([0, 5, 0], [0, -5, 0])
```

Native `.bullet` serialization and import:

```ruby
sim = Bullet3::Simulation.new
shape = sim.create_collision_shape(:sphere, radius: 1.0)
sim.create_rigid_body(mass: 0.0, collision_shape: shape)

sim.save_bullet("world.bullet")

loaded = Bullet3::Simulation.new
importer = loaded.load_bullet("world.bullet")
p importer.num_rigid_bodies
```

`.bullet` import support is enabled when Bullet's WorldImporter extras are
available at build time. Set `BULLET3_EXTRAS_SOURCE_DIR` to Bullet's
`Extras/Serialize` source directory before compiling if your system Bullet
package does not ship those libraries.

Debug drawing via Bullet3's `btIDebugDraw` interface:

```ruby
drawer = Bullet3::DebugDraw.new
drawer.debug_mode = Bullet3::DebugDraw::DRAW_WIREFRAME | Bullet3::DebugDraw::DRAW_AABB
sim.debug_draw_world(drawer)

p drawer.lines.first
```

## Implemented Scope

- LinearMath: `Vector3`, `Quaternion`, `Matrix3x3`, `Transform`
- Collision shapes: primitive, compound, convex hull, triangle mesh,
  convex triangle mesh, multi-sphere, heightfield
- Collision world: standalone collision objects, broadphases, ray tests,
  contact tests, pair tests, closest points
- Dynamics: rigid bodies, motion states, default/explicit worlds,
  constraints, contact manifolds, GVL-free stepping
- Extras: raycast vehicles, soft bodies, multibodies
- High-level API: direct/gui/shared-memory compatible local simulation modes,
  primitive body creation, single- and multi-link URDF, SDF and MJCF primitive
  import, ray/contact/AABB helpers, base pose, joint state/control helpers,
  camera ray rendering, JSON world snapshots, native `.bullet` serialization
  and import, debug drawing, dynamics updates, reset/disconnect

Remaining limitations: GUI and shared-memory modes run through the same local
simulation backend, and camera images use the built-in ray renderer rather than
TinyRenderer/OpenGL.

## Development

After checking out the repo, install Bullet and Ruby dependencies:

```bash
brew install bullet
bundle install
```

Use `BULLET_ROOT=/path/to/bullet` when Bullet is installed outside the standard
system prefixes. `.bullet` import support also needs Bullet's WorldImporter
extras; set `BULLET3_EXTRAS_SOURCE_DIR=/path/to/bullet3/Extras/Serialize` before
compiling to enable it from a Bullet source checkout.

Run the Ruby fallback test suite:

```bash
BULLET3_SKIP_NATIVE=1 bundle exec rake spec
```

Compile and test the native extension:

```bash
bundle exec rake compile
bundle exec rake spec
```

Run examples:

```bash
bundle exec ruby examples/hello_world.rb
bundle exec ruby examples/ray_casting.rb
bundle exec ruby examples/urdf.rb
```

## Contributing

Bug reports and pull requests are welcome on GitHub.

## License

The gem is available as open source under the terms of the [MIT License](https://opensource.org/licenses/MIT).
