# frozen_string_literal: true

RSpec.describe "Bullet3 vehicles" do
  before do
    skip "native extension only" unless Bullet3.native?
  end

  def rigid_body(shape, mass, origin)
    transform = Bullet3::Transform.new(Bullet3::Quaternion.identity, origin)
    motion_state = Bullet3::MotionState.new(transform)
    inertia = shape.calculate_local_inertia(mass)
    info = Bullet3::RigidBodyConstructionInfo.new(mass, motion_state, shape, inertia)
    Bullet3::RigidBody.new(info)
  end

  it "creates raycast vehicles and updates wheel controls" do
    world = Bullet3::DiscreteDynamicsWorld.create
    world.gravity = [0, -9.81, 0]

    ground_shape = Bullet3::Shapes::StaticPlaneShape.new(Bullet3::Vector3.new(0, 1, 0), 0)
    ground = rigid_body(ground_shape, 0.0, Bullet3::Vector3.new(0, 0, 0))
    chassis_shape = Bullet3::Shapes::BoxShape.new(Bullet3::Vector3.new(1.0, 0.4, 2.0))
    chassis = rigid_body(chassis_shape, 800.0, Bullet3::Vector3.new(0, 1.0, 0))
    world.add_rigid_body(ground)
    world.add_rigid_body(chassis)

    tuning = Bullet3::VehicleTuning.new
    tuning.suspension_stiffness = 6.0
    tuning.friction_slip = 12.0

    vehicle = Bullet3::RaycastVehicle.new(tuning, world, chassis)
    vehicle.set_coordinate_system(0, 1, 2)
    wheel0 = vehicle.add_wheel([0.8, 0.2, 1.2], [0, -1, 0], [-1, 0, 0], 0.6, 0.35, tuning, true)
    vehicle.add_wheel([-0.8, 0.2, 1.2], [0, -1, 0], [-1, 0, 0], 0.6, 0.35, tuning, true)
    vehicle.add_wheel([0.8, 0.2, -1.2], [0, -1, 0], [-1, 0, 0], 0.6, 0.35, tuning, false)
    vehicle.add_wheel([-0.8, 0.2, -1.2], [0, -1, 0], [-1, 0, 0], 0.6, 0.35, tuning, false)
    world.add_vehicle(vehicle)

    vehicle.set_steering_value(0.15, 0)
    vehicle.apply_engine_force(150.0, 2)
    vehicle.set_brake(0.0, 2)
    world.step_simulation(1.0 / 60.0)
    vehicle.update_wheel_transform(0)

    expect(tuning.suspension_stiffness).to be_within(1e-6).of(6.0)
    expect(vehicle.coordinate_system).to eq([0, 1, 2])
    expect(vehicle.num_wheels).to eq(4)
    expect(vehicle.steering_value(0)).to be_within(1e-6).of(0.15)
    expect(vehicle.wheel_info(2).engine_force).to be_within(1e-6).of(150.0)
    expect(wheel0.index).to eq(0)
    expect(wheel0.front_wheel?).to be(true)
    expect(wheel0.radius).to be_within(1e-6).of(0.35)
    expect(wheel0.world_transform).to be_a(Bullet3::Transform)
    expect(vehicle.chassis_world_transform).to be_a(Bullet3::Transform)
    expect(vehicle.forward_vector).to be_a(Bullet3::Vector3)

    world.remove_vehicle(vehicle)
  end
end
