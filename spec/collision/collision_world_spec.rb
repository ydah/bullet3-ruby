# frozen_string_literal: true

RSpec.describe "Bullet::CollisionWorld" do
  before do
    skip "native extension only" unless ENV["BULLET_RUBY_USE_NATIVE"] == "1"
  end

  it "adds and removes standalone collision objects" do
    world = Bullet::CollisionWorld.create
    shape = Bullet::Shapes::BoxShape.new(Bullet::Vector3.new(1, 1, 1))
    object = Bullet::CollisionObject.new(shape)
    object.world_transform = Bullet::Transform.new(Bullet::Quaternion.identity, [0, 0, 0])
    object.friction = 0.7
    object.restitution = 0.25

    world.add_collision_object(object)
    world.update_aabbs
    world.compute_overlapping_pairs

    expect(world.num_collision_objects).to eq(1)
    expect(object.collision_shape.shape_type).to eq(:box)
    expect(object.friction).to be_within(1e-6).of(0.7)
    expect(object.restitution).to be_within(1e-6).of(0.25)
    expect(object.aabb.map(&:to_a)).to eq([[-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]])

    world.remove_collision_object(object)

    expect(world.num_collision_objects).to eq(0)
  end

  it "returns closest and all ray hits with Ruby collision object references" do
    world = Bullet::CollisionWorld.create
    object = Bullet::CollisionObject.new(Bullet::Shapes::SphereShape.new(1.0))
    object.world_transform = Bullet::Transform.new(Bullet::Quaternion.identity, [0, 0, 0])

    world.add_collision_object(object)

    closest = world.ray_test_closest([0, 5, 0], [0, -5, 0])
    all_hits = world.ray_test_all([0, 5, 0], [0, -5, 0])

    expect(closest).not_to be_nil
    expect(closest[:body]).to equal(object)
    expect(closest[:point].y).to be_within(1e-3).of(1.0)
    expect(closest[:normal].y).to be > 0.0

    expect(all_hits.length).to eq(1)
    expect(all_hits.first[:body]).to equal(object)
  end

  it "can use explicit dispatcher and broadphase objects" do
    configuration = Bullet::CollisionConfiguration.new
    dispatcher = Bullet::CollisionDispatcher.new(configuration)
    dbvt_world = Bullet::CollisionWorld.new(dispatcher, Bullet::DbvtBroadphase.new, configuration)
    axis_world = Bullet::CollisionWorld.new(
      dispatcher,
      Bullet::AxisSweep3.new([-100, -100, -100], [100, 100, 100]),
      configuration
    )

    expect(dbvt_world.num_collision_objects).to eq(0)
    expect(axis_world.num_collision_objects).to eq(0)
  end
end
