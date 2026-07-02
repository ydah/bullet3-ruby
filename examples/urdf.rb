# frozen_string_literal: true

require "bullet_ruby"

sim = Bullet::Simulation.new
sim.set_gravity(0, 0, -10)

plane_id = sim.load_urdf("plane.urdf", use_fixed_base: true)
cube_id = sim.load_urdf("cube.urdf", base_position: [0, 0, 3])

120.times { sim.step_simulation(time_step: 1.0 / 60.0, fixed_time_step: 1.0 / 60.0) }

position, orientation = sim.get_base_position_and_orientation(cube_id)
contacts = sim.get_contact_points(body_a: plane_id, body_b: cube_id)

puts "cube_position=#{position.inspect} cube_orientation=#{orientation.inspect}"
puts "cube_aabb=#{sim.get_aabb(cube_id).inspect}"
puts "contacts=#{contacts.length}"

sim.disconnect
