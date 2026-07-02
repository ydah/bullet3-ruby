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

  it "creates slider, cone twist, six-dof, gear, and hinge2 constraints" do
    world = Bullet::DiscreteDynamicsWorld.create
    body_a = rigid_body(Bullet::Vector3.new(0, 3, 0))
    body_b = rigid_body(Bullet::Vector3.new(1, 3, 0))
    world.add_rigid_body(body_a)
    world.add_rigid_body(body_b)
    frame = Bullet::Transform.identity

    slider = Bullet::Constraints::SliderConstraint.new(body_a, body_b, frame, frame)
    slider.lower_linear_limit = -1.0
    slider.upper_linear_limit = 1.0
    slider.lower_angular_limit = -0.5
    slider.upper_angular_limit = 0.5
    slider.powered_linear_motor = true
    slider.target_linear_motor_velocity = 2.0
    slider.max_linear_motor_force = 3.0
    slider.powered_angular_motor = true
    slider.target_angular_motor_velocity = 4.0
    slider.max_angular_motor_force = 5.0

    cone_twist = Bullet::Constraints::ConeTwistConstraint.new(body_a, body_b, frame, frame)
    cone_twist.set_limit(0.3, 0.4, 0.5, 0.8, 0.2, 0.9)
    cone_twist.angular_only = true
    cone_twist.damping = 0.75
    cone_twist.enable_motor(true)
    cone_twist.max_motor_impulse = 1.5
    cone_twist.motor_target = Bullet::Quaternion.identity

    generic = Bullet::Constraints::Generic6DofConstraint.new(body_a, body_b, frame, frame)
    generic.linear_lower_limit = [-1, -2, -3]
    generic.linear_upper_limit = [1, 2, 3]
    generic.angular_lower_limit = [-0.1, -0.2, -0.3]
    generic.angular_upper_limit = [0.1, 0.2, 0.3]
    generic.use_frame_offset = true
    generic.use_linear_reference_frame_a = false
    generic.set_limit(0, -0.25, 0.25)

    spring2 = Bullet::Constraints::Generic6DofSpring2Constraint.new(body_a, body_b, frame, frame, 0)
    spring2.rotation_order = 1
    spring2.linear_lower_limit = [-0.5, -0.5, -0.5]
    spring2.linear_upper_limit = [0.5, 0.5, 0.5]
    spring2.set_limit(3, -0.25, 0.25)
    spring2.enable_motor(0, true)
    spring2.set_target_velocity(0, 1.25)
    spring2.set_max_motor_force(0, 2.5)
    spring2.enable_spring(1, true)
    spring2.set_stiffness(1, 3.5)
    spring2.set_damping(1, 0.6)
    spring2.set_axis_equilibrium_point_value(1, 0.1)

    gear = Bullet::Constraints::GearConstraint.new(body_a, body_b, [0, 1, 0], [0, 1, 0], 2.0)
    gear.axis_a = [1, 0, 0]
    gear.axis_b = [0, 0, 1]
    gear.ratio = 3.0

    hinge2 = Bullet::Constraints::Hinge2Constraint.new(body_a, body_b, [0, 3, 0], [0, 1, 0], [1, 0, 0])
    hinge2.set_lower_limit(-0.4)
    hinge2.set_upper_limit(0.4)

    [slider, cone_twist, generic, spring2, gear, hinge2].each do |constraint|
      world.add_constraint(constraint, true)
    end

    expect(slider.constraint_type).to eq(:slider)
    expect(slider.lower_linear_limit).to be_within(1e-6).of(-1.0)
    expect(slider.upper_angular_limit).to be_within(1e-6).of(0.5)
    expect(slider.powered_linear_motor?).to be(true)
    expect(slider.target_angular_motor_velocity).to be_within(1e-6).of(4.0)

    expect(cone_twist.constraint_type).to eq(:cone_twist)
    expect(cone_twist.swing_span1).to be_within(1e-6).of(0.3)
    expect(cone_twist.twist_span).to be_within(1e-6).of(0.5)
    expect(cone_twist.angular_only?).to be(true)
    expect(cone_twist.motor_enabled?).to be(true)

    expect(generic.constraint_type).to eq(:generic_6dof)
    expect(generic.linear_lower_limit).to eq(Bullet::Vector3.new(-0.25, -2, -3))
    expect(generic.angular_upper_limit).to eq(Bullet::Vector3.new(0.1, 0.2, 0.3))
    expect(generic.limited?(0)).to be(true)
    expect(generic.use_linear_reference_frame_a?).to be(false)

    expect(spring2.constraint_type).to eq(:generic_6dof_spring2)
    expect(spring2.rotation_order).to eq(1)
    expect(spring2.linear_upper_limit).to eq(Bullet::Vector3.new(0.5, 0.5, 0.5))
    expect(spring2.limited?(3)).to be(true)

    expect(gear.constraint_type).to eq(:gear)
    expect(gear.axis_a).to eq(Bullet::Vector3.new(1, 0, 0))
    expect(gear.axis_b).to eq(Bullet::Vector3.new(0, 0, 1))
    expect(gear.ratio).to be_within(1e-6).of(3.0)

    expect(hinge2.constraint_type).to eq(:hinge2)
    expect(hinge2.axis1).to eq(Bullet::Vector3.new(0, 1, 0))
    expect(hinge2.axis2).to eq(Bullet::Vector3.new(1, 0, 0))
    expect(world.num_constraints).to eq(6)
  end
end
