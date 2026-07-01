# frozen_string_literal: true

RSpec.describe "Bullet::DiscreteDynamicsWorld" do
  before do
    skip "native extension only" unless ENV["BULLET_RUBY_USE_NATIVE"] == "1"
  end

  def rigid_body(shape:, mass:, origin:)
    transform = Bullet::Transform.new(Bullet::Quaternion.identity, origin)
    motion_state = Bullet::MotionState.new(transform)
    info = Bullet::RigidBodyConstructionInfo.new(mass, motion_state, shape)
    Bullet::RigidBody.new(info)
  end

  it "creates worlds from default and explicit components" do
    default_world = Bullet::DiscreteDynamicsWorld.create
    default_world.gravity = [0, -9.81, 0]

    configuration = Bullet::CollisionConfiguration.new
    dispatcher = Bullet::CollisionDispatcher.new(configuration)
    broadphase = Bullet::DbvtBroadphase.new
    solver = Bullet::SequentialImpulseConstraintSolver.new
    explicit_world = Bullet::DiscreteDynamicsWorld.new(dispatcher, broadphase, solver, configuration)

    expect(default_world.gravity).to eq(Bullet::Vector3.new(0, -9.81, 0))
    expect(explicit_world.num_collision_objects).to eq(0)
  end

  it "adds, steps, and removes rigid bodies" do
    world = Bullet::DiscreteDynamicsWorld.create
    world.gravity = [0, -10, 0]

    body = rigid_body(
      shape: Bullet::Shapes::SphereShape.new(1.0),
      mass: 1.0,
      origin: Bullet::Vector3.new(0, 10, 0)
    )

    world.add_rigid_body(body)
    expect(world.num_collision_objects).to eq(1)

    10.times { world.step_simulation(1.0 / 60.0) }

    expect(body.world_transform.origin.y).to be < 10.0

    world.remove_rigid_body(body)
    expect(world.num_collision_objects).to eq(0)
  end
end
