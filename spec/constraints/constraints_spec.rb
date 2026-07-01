# frozen_string_literal: true

RSpec.describe "Bullet constraints" do
  before do
    skip "native extension only" unless ENV["BULLET_RUBY_USE_NATIVE"] == "1"
  end

  def rigid_body(origin)
    shape = Bullet::Shapes::SphereShape.new(0.5)
    transform = Bullet::Transform.new(Bullet::Quaternion.identity, origin)
    motion_state = Bullet::MotionState.new(transform)
    info = Bullet::RigidBodyConstructionInfo.new(1.0, motion_state, shape)
    Bullet::RigidBody.new(info)
  end

  it "creates and updates point-to-point constraints" do
    body_a = rigid_body(Bullet::Vector3.new(0, 0, 0))
    body_b = rigid_body(Bullet::Vector3.new(1, 0, 0))
    constraint = Bullet::Constraints::Point2PointConstraint.new(body_a, body_b, [0, 0, 0], [0, 0, 0])

    constraint.pivot_in_a = [0.25, 0, 0]
    constraint.enabled = false
    constraint.breaking_impulse_threshold = 10.0
    constraint.debug_draw_size = 0.5

    expect(constraint.constraint_type).to eq(:point2point)
    expect(constraint.pivot_in_a).to eq(Bullet::Vector3.new(0.25, 0, 0))
    expect(constraint.enabled?).to be(false)
    expect(constraint.breaking_impulse_threshold).to be_within(1e-6).of(10.0)
    expect(constraint.debug_draw_size).to be_within(1e-6).of(0.5)
  end

  it "creates hinge and fixed constraints and adds them to worlds" do
    world = Bullet::DiscreteDynamicsWorld.create
    body_a = rigid_body(Bullet::Vector3.new(0, 2, 0))
    body_b = rigid_body(Bullet::Vector3.new(1, 2, 0))
    world.add_rigid_body(body_a)
    world.add_rigid_body(body_b)

    hinge = Bullet::Constraints::HingeConstraint.new(
      body_a,
      body_b,
      [0, 0, 0],
      [0, 0, 0],
      [0, 1, 0],
      [0, 1, 0]
    )
    hinge.set_limit(-0.5, 0.5)
    hinge.enable_angular_motor(true, 1.0, 2.0)

    fixed = Bullet::Constraints::FixedConstraint.new(
      body_a,
      body_b,
      Bullet::Transform.identity,
      Bullet::Transform.identity
    )

    world.add_constraint(hinge, true)
    world.add_constraint(fixed)

    expect(hinge.constraint_type).to eq(:hinge)
    expect(hinge.lower_limit).to be_within(1e-6).of(-0.5)
    expect(hinge.upper_limit).to be_within(1e-6).of(0.5)
    expect(hinge.angular_motor_enabled?).to be(true)
    expect(hinge.motor_target_velocity).to be_within(1e-6).of(1.0)
    expect(hinge.max_motor_impulse).to be_within(1e-6).of(2.0)
    expect(fixed.constraint_type).to eq(:fixed)
    expect(world.num_constraints).to eq(2)

    world.remove_constraint(hinge)
    expect(world.num_constraints).to eq(1)
  end
end
