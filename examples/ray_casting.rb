# frozen_string_literal: true

require "bullet_ruby"

sim = Bullet::Simulation.new
sim.set_gravity(0, -10, 0)

plane_shape = sim.create_collision_shape(:static_plane, normal: [0, 1, 0], offset: 0)
sphere_shape = sim.create_collision_shape(:sphere, radius: 1.0)

plane_id = sim.create_rigid_body(mass: 0.0, collision_shape: plane_shape)
sphere_id = sim.create_rigid_body(mass: 1.0, collision_shape: sphere_shape, position: [0, 1, 0])
sim.step_simulation(time_step: 1.0 / 60.0)

hit = sim.ray_test([0, 5, 0], [0, -5, 0])
contacts = sim.get_contact_points(body_a: plane_id, body_b: sphere_id)

puts "hit_fraction=#{hit[:fraction].round(4)} hit_point=#{hit[:point].to_a.inspect}"
puts "contacts=#{contacts.length} first_distance=#{contacts.first[:distance].round(4)}"

sim.disconnect
