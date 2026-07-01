# bullet_ruby

Ruby bindings for the Bullet Physics SDK.

This project is building a two-layer API:

- low-level Bullet C++ bindings exposed under `Bullet`
- Ruby-friendly helpers and simulation APIs built on top of those bindings

The current implementation establishes the gem/native-extension build
foundation and the first LinearMath APIs: `Bullet::Vector3`,
`Bullet::Quaternion`, `Bullet::Matrix3x3`, `Bullet::Transform`, collision
shapes, rigid bodies, dynamics worlds, constraints, ray/contact queries, and a
small high-level `Bullet::Simulation` facade.

## Installation

Add the gem from this repository:

```bash
gem "bullet_ruby", path: "path/to/bullet_ruby"
```

Then install dependencies:

```bash
bundle install
```

## Usage

```ruby
require "bullet_ruby"

v = Bullet::Vector3.new(1, 2, 3)
axis = Bullet::Vector3.new(0, 0, 1)
rotation = Bullet::Quaternion.from_axis_angle(axis, Math::PI / 2)
transform = Bullet::Transform.new(rotation, Bullet::Vector3.new(1, 0, 0))

p transform * v
```

High-level direct simulation:

```ruby
sim = Bullet::Simulation.new
sim.set_gravity(0, -10, 0)

plane = sim.create_collision_shape(:static_plane, normal: [0, 1, 0], offset: 0)
sphere = sim.create_collision_shape(:sphere, radius: 1.0)

sim.create_rigid_body(mass: 0.0, collision_shape: plane)
body = sim.create_rigid_body(mass: 1.0, collision_shape: sphere, position: [0, 10, 0])

120.times { sim.step_simulation(time_step: 1.0 / 60.0, fixed_time_step: 1.0 / 60.0) }
p sim.get_base_position_and_orientation(body)
```

By default the Ruby implementation is loaded. Set `BULLET_RUBY_USE_NATIVE=1`
after compiling the extension to load the native `Bullet::Vector3` binding.

## Development

After checking out the repo, initialize submodules and install dependencies:

```bash
git submodule update --init --recursive
bundle install
```

Run the Ruby fallback test suite:

```bash
BULLET_RUBY_SKIP_NATIVE=1 bundle exec rake spec
```

Compile and test the native extension:

```bash
bundle exec rake compile
BULLET_RUBY_USE_NATIVE=1 bundle exec rake spec
```

## Contributing

Bug reports and pull requests are welcome on GitHub.

## License

The gem is available as open source under the terms of the [MIT License](https://opensource.org/licenses/MIT).
