# frozen_string_literal: true

RSpec.describe "Bullet rigid bodies" do
  before do
    skip "native extension only" unless ENV["BULLET_RUBY_USE_NATIVE"] == "1"
  end

  it "builds rigid bodies from construction info" do
    shape = Bullet::Shapes::SphereShape.new(1.0)
    motion_state = Bullet::MotionState.new(Bullet::Transform.new(Bullet::Quaternion.identity, Bullet::Vector3.new(0, 4, 0)))
    info = Bullet::RigidBodyConstructionInfo.new(2.0, motion_state, shape)
    body = Bullet::RigidBody.new(info)

    expect(info.mass).to be_within(1e-6).of(2.0)
    expect(info.local_inertia.length).to be > 0
    expect(body.mass).to be_within(1e-6).of(2.0)
    expect(body.collision_shape.shape_type).to eq(:sphere)
    expect(body.world_transform.origin).to eq(Bullet::Vector3.new(0, 4, 0))
  end

  it "updates rigid body state" do
    shape = Bullet::Shapes::BoxShape.new(Bullet::Vector3.new(1, 1, 1))
    motion_state = Bullet::MotionState.new
    info = Bullet::RigidBodyConstructionInfo.new(1.0, motion_state, shape, Bullet::Vector3.zero)
    body = Bullet::RigidBody.new(info)

    body.linear_velocity = [1, 2, 3]
    body.angular_velocity = Bullet::Vector3.new(4, 5, 6)
    body.friction = 0.25
    body.restitution = 0.5
    body.set_damping(0.1, 0.2)
    body.apply_central_force([0, 10, 0])

    expect(body.linear_velocity).to eq(Bullet::Vector3.new(1, 2, 3))
    expect(body.angular_velocity).to eq(Bullet::Vector3.new(4, 5, 6))
    expect(body.friction).to be_within(1e-6).of(0.25)
    expect(body.restitution).to be_within(1e-6).of(0.5)
    expect(body.damping).to contain_exactly(be_within(1e-6).of(0.1), be_within(1e-6).of(0.2))
    expect(body.total_force).to eq(Bullet::Vector3.new(0, 10, 0))
  end
end
