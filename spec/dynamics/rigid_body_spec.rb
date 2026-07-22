# frozen_string_literal: true

RSpec.describe "Bullet3 rigid bodies" do
  before do
    skip "native extension only" unless Bullet3.native?
  end

  it "builds rigid bodies from construction info" do
    shape = Bullet3::Shapes::SphereShape.new(1.0)
    motion_state = Bullet3::MotionState.new(Bullet3::Transform.new(Bullet3::Quaternion.identity, Bullet3::Vector3.new(0, 4, 0)))
    info = Bullet3::RigidBodyConstructionInfo.new(2.0, motion_state, shape)
    body = Bullet3::RigidBody.new(info)

    expect(info.mass).to be_within(1e-6).of(2.0)
    expect(info.local_inertia.length).to be > 0
    expect(body.mass).to be_within(1e-6).of(2.0)
    expect(body.collision_shape.shape_type).to eq(:sphere)
    expect(body.world_transform.origin).to eq(Bullet3::Vector3.new(0, 4, 0))
  end

  it "updates rigid body state" do
    shape = Bullet3::Shapes::BoxShape.new(Bullet3::Vector3.new(1, 1, 1))
    motion_state = Bullet3::MotionState.new
    info = Bullet3::RigidBodyConstructionInfo.new(1.0, motion_state, shape, Bullet3::Vector3.zero)
    body = Bullet3::RigidBody.new(info)

    body.linear_velocity = [1, 2, 3]
    body.angular_velocity = Bullet3::Vector3.new(4, 5, 6)
    body.friction = 0.25
    body.restitution = 0.5
    body.set_damping(0.1, 0.2)
    body.apply_central_force([0, 10, 0])

    expect(body.linear_velocity).to eq(Bullet3::Vector3.new(1, 2, 3))
    expect(body.angular_velocity).to eq(Bullet3::Vector3.new(4, 5, 6))
    expect(body.friction).to be_within(1e-6).of(0.25)
    expect(body.restitution).to be_within(1e-6).of(0.5)
    expect(body.damping).to contain_exactly(be_within(1e-6).of(0.1), be_within(1e-6).of(0.2))
    expect(body.total_force).to eq(Bullet3::Vector3.new(0, 10, 0))
  end
end
