# frozen_string_literal: true

require "bullet3"

sim = Bullet3::Simulation.new
sim.set_gravity(0, -10, 0)

plane_shape = sim.create_collision_shape(:static_plane, normal: [0, 1, 0], offset: 0)
sphere_shape = sim.create_collision_shape(:sphere, radius: 1.0)

sim.create_rigid_body(mass: 0.0, collision_shape: plane_shape)
sphere_id = sim.create_rigid_body(mass: 1.0, collision_shape: sphere_shape, position: [0, 10, 0])

120.times { sim.step_simulation(time_step: 1.0 / 60.0, fixed_time_step: 1.0 / 60.0) }

position, orientation = sim.get_base_position_and_orientation(sphere_id)
puts "position=#{position.inspect} orientation=#{orientation.inspect}"

sim.disconnect
