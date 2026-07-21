# frozen_string_literal: true

RSpec.describe "Bullet3 Hello World integration" do
  before do
    skip "native extension only" unless ENV["BULLET3_USE_NATIVE"] == "1"
  end

  it "drops a dynamic sphere onto a static plane" do
    world = Bullet3::DiscreteDynamicsWorld.create
    world.gravity = [0, -10, 0]

    ground_shape = Bullet3::Shapes::StaticPlaneShape.new(Bullet3::Vector3.new(0, 1, 0), 0)
    ground_motion_state = Bullet3::MotionState.new(Bullet3::Transform.identity)
    ground_info = Bullet3::RigidBodyConstructionInfo.new(0.0, ground_motion_state, ground_shape)
    ground = Bullet3::RigidBody.new(ground_info)

    sphere_shape = Bullet3::Shapes::SphereShape.new(1.0)
    sphere_transform = Bullet3::Transform.new(Bullet3::Quaternion.identity, Bullet3::Vector3.new(0, 20, 0))
    sphere_motion_state = Bullet3::MotionState.new(sphere_transform)
    sphere_info = Bullet3::RigidBodyConstructionInfo.new(1.0, sphere_motion_state, sphere_shape)
    sphere = Bullet3::RigidBody.new(sphere_info)

    world.add_rigid_body(ground)
    world.add_rigid_body(sphere)

    180.times { world.step_simulation(1.0 / 60.0) }

    expect(sphere.world_transform.origin.y).to be_between(0.95, 1.1)
  end
end
