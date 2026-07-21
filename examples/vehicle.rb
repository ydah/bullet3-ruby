# frozen_string_literal: true

require "bullet3"

def rigid_body(shape, mass, origin)
  transform = Bullet3::Transform.new(Bullet3::Quaternion.identity, origin)
  motion_state = Bullet3::MotionState.new(transform)
  inertia = shape.calculate_local_inertia(mass)
  info = Bullet3::RigidBodyConstructionInfo.new(mass, motion_state, shape, inertia)
  Bullet3::RigidBody.new(info)
end

world = Bullet3::DiscreteDynamicsWorld.create
world.gravity = [0, -9.81, 0]

ground = rigid_body(Bullet3::Shapes::StaticPlaneShape.new(Bullet3::Vector3.new(0, 1, 0), 0), 0.0, [0, 0, 0])
chassis = rigid_body(Bullet3::Shapes::BoxShape.new(Bullet3::Vector3.new(1.0, 0.4, 2.0)), 800.0, [0, 1.0, 0])
world.add_rigid_body(ground)
world.add_rigid_body(chassis)

tuning = Bullet3::VehicleTuning.new
tuning.suspension_stiffness = 6.0
tuning.friction_slip = 12.0

vehicle = Bullet3::RaycastVehicle.new(tuning, world, chassis)
vehicle.set_coordinate_system(0, 1, 2)
vehicle.add_wheel([0.8, 0.2, 1.2], [0, -1, 0], [-1, 0, 0], 0.6, 0.35, tuning, true)
vehicle.add_wheel([-0.8, 0.2, 1.2], [0, -1, 0], [-1, 0, 0], 0.6, 0.35, tuning, true)
vehicle.add_wheel([0.8, 0.2, -1.2], [0, -1, 0], [-1, 0, 0], 0.6, 0.35, tuning, false)
vehicle.add_wheel([-0.8, 0.2, -1.2], [0, -1, 0], [-1, 0, 0], 0.6, 0.35, tuning, false)
world.add_vehicle(vehicle)

vehicle.set_steering_value(0.1, 0)
vehicle.set_steering_value(0.1, 1)
vehicle.apply_engine_force(250.0, 2)
vehicle.apply_engine_force(250.0, 3)

120.times { world.step_simulation(1.0 / 60.0) }

puts "chassis_position=#{chassis.world_transform.origin.to_a.inspect}"
puts "forward=#{vehicle.forward_vector.to_a.inspect}"
puts "wheels=#{vehicle.num_wheels}"

world.remove_vehicle(vehicle)
