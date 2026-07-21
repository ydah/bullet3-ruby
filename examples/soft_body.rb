# frozen_string_literal: true

require "bullet3"

world = Bullet3::SoftBody::SoftRigidDynamicsWorld.create
world.gravity = [0, -9.81, 0]
info = world.world_info
info.air_density = 1.2
info.water_normal = [0, 1, 0]

rope = Bullet3::SoftBody::Helpers.create_rope(info, [0, 2, 0], [2, 2, 0], 8, 1)
rope.set_total_mass(1.0)
rope.generate_bending_constraints(2)
world.add_soft_body(rope)

60.times { world.step_simulation(1.0 / 60.0) }

puts "nodes=#{rope.node_count} links=#{rope.link_count}"
puts "first_node=#{rope.node_position(0).to_a.inspect}"
puts "aabb=#{rope.aabb.map(&:to_a).inspect}"
