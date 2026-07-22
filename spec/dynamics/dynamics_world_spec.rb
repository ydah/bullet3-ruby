# frozen_string_literal: true

RSpec.describe "Bullet3::DiscreteDynamicsWorld" do
  before do
    skip "native extension only" unless Bullet3.native?
  end

  def rigid_body(shape:, mass:, origin:)
    transform = Bullet3::Transform.new(Bullet3::Quaternion.identity, origin)
    motion_state = Bullet3::MotionState.new(transform)
    info = Bullet3::RigidBodyConstructionInfo.new(mass, motion_state, shape)
    Bullet3::RigidBody.new(info)
  end

  it "creates worlds from default and explicit components" do
    default_world = Bullet3::DiscreteDynamicsWorld.create
    default_world.gravity = [0, -9.81, 0]

    configuration = Bullet3::CollisionConfiguration.new
    dispatcher = Bullet3::CollisionDispatcher.new(configuration)
    broadphase = Bullet3::DbvtBroadphase.new
    solver = Bullet3::SequentialImpulseConstraintSolver.new
    explicit_world = Bullet3::DiscreteDynamicsWorld.new(dispatcher, broadphase, solver, configuration)

    expect(default_world.gravity).to eq(Bullet3::Vector3.new(0, -9.81, 0))
    expect(explicit_world.num_collision_objects).to eq(0)
  end

  it "adds, steps, and removes rigid bodies" do
    world = Bullet3::DiscreteDynamicsWorld.create
    world.gravity = [0, -10, 0]

    body = rigid_body(
      shape: Bullet3::Shapes::SphereShape.new(1.0),
      mass: 1.0,
      origin: Bullet3::Vector3.new(0, 10, 0)
    )

    world.add_rigid_body(body)
    expect(world.num_collision_objects).to eq(1)

    10.times { world.step_simulation(1.0 / 60.0) }

    expect(body.world_transform.origin.y).to be < 10.0

    world.remove_rigid_body(body)
    expect(world.num_collision_objects).to eq(0)
  end
end
