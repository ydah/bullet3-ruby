# frozen_string_literal: true

RSpec.describe "Bullet3 multi bodies" do
  before do
    skip "native extension only" unless ENV["BULLET3_USE_NATIVE"] == "1"
  end

  def rigid_body
    shape = Bullet3::Shapes::SphereShape.new(0.25)
    motion_state = Bullet3::MotionState.new(Bullet3::Transform.identity)
    info = Bullet3::RigidBodyConstructionInfo.new(0.0, motion_state, shape)
    Bullet3::RigidBody.new(info)
  end

  it "creates multi bodies, colliders, and constraints" do
    world = Bullet3::MultiBody::MultiBodyDynamicsWorld.create
    world.gravity = [0, -9.81, 0]

    multi_body = Bullet3::MultiBody::MultiBody.new(1, 1.0, [1, 1, 1], false, true)
    multi_body.base_position = [0, 1, 0]
    multi_body.setup_revolute(
      0,
      1.0,
      [1, 1, 1],
      -1,
      Bullet3::Quaternion.identity,
      [0, 0, 1],
      [0, 0, 0],
      [0, 1, 0]
    )
    multi_body.finalize_multi_dof
    multi_body.self_collision = true
    multi_body.linear_damping = 0.1
    multi_body.angular_damping = 0.2

    base_collider = Bullet3::MultiBody::MultiBodyLinkCollider.new(multi_body, -1)
    base_collider.collision_shape = Bullet3::Shapes::BoxShape.new(Bullet3::Vector3.new(0.5, 0.5, 0.5))
    base_collider.world_transform = Bullet3::Transform.identity
    multi_body.set_base_collider(base_collider)

    motor = Bullet3::MultiBody::MultiBodyJointMotor.new(multi_body, 0, 0.0, 10.0)
    motor.set_velocity_target(0.5)
    motor.set_position_target(0.1)
    motor.erp = 0.2
    motor.rhs_clamp = 1.0

    limit = Bullet3::MultiBody::MultiBodyJointLimitConstraint.new(multi_body, 0, -0.5, 0.5)
    limit.lower_bound = -0.25
    limit.upper_bound = 0.25

    p2p = Bullet3::MultiBody::MultiBodyPoint2Point.new(multi_body, 0, rigid_body, [0, 0, 0], [0, 0, 0])
    p2p.pivot_in_b = [0.1, 0, 0]

    world.add_multi_body(multi_body)
    world.add_link_collider(base_collider)
    world.add_constraint(motor)
    world.add_constraint(limit)
    world.step_simulation(1.0 / 60.0)
    world.forward_kinematics

    expect(world.gravity).to eq(Bullet3::Vector3.new(0, -9.81, 0))
    expect(world.num_multi_bodies).to eq(1)
    expect(world.num_collision_objects).to eq(1)
    expect(world.num_multi_body_constraints).to eq(2)
    expect(multi_body.num_links).to eq(1)
    expect(multi_body.num_dofs).to eq(1)
    expect(multi_body.parent(0)).to eq(-1)
    expect(multi_body.base_position).to be_a(Bullet3::Vector3)
    expect(multi_body.self_collision?).to be(true)
    expect(multi_body.linear_damping).to be_within(1e-6).of(0.1)
    expect(motor.constraint_type).to eq(:joint_motor)
    expect(motor.erp).to be_within(1e-6).of(0.2)
    expect(limit.constraint_type).to eq(:joint_limit)
    expect(limit.lower_bound).to be_within(1e-6).of(-0.25)
    expect(p2p.constraint_type).to eq(:point2point)
    expect(p2p.pivot_in_b).to eq(Bullet3::Vector3.new(0.1, 0, 0))
    expect(base_collider.link).to eq(-1)
    expect(base_collider.collision_shape).to be_a(Bullet3::Shapes::CollisionShape)

    world.remove_constraint(limit)
    expect(world.num_multi_body_constraints).to eq(1)
  end
end
