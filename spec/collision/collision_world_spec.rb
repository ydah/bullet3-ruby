# frozen_string_literal: true

RSpec.describe "Bullet3::CollisionWorld" do
  before do
    skip "native extension only" unless Bullet3.native?
  end

  it "adds and removes standalone collision objects" do
    world = Bullet3::CollisionWorld.create
    shape = Bullet3::Shapes::BoxShape.new(Bullet3::Vector3.new(1, 1, 1))
    object = Bullet3::CollisionObject.new(shape)
    object.world_transform = Bullet3::Transform.new(Bullet3::Quaternion.identity, [0, 0, 0])
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
    world = Bullet3::CollisionWorld.create
    object = Bullet3::CollisionObject.new(Bullet3::Shapes::SphereShape.new(1.0))
    object.world_transform = Bullet3::Transform.new(Bullet3::Quaternion.identity, [0, 0, 0])

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
    configuration = Bullet3::CollisionConfiguration.new
    dispatcher = Bullet3::CollisionDispatcher.new(configuration)
    dbvt_world = Bullet3::CollisionWorld.new(dispatcher, Bullet3::DbvtBroadphase.new, configuration)
    axis_world = Bullet3::CollisionWorld.new(
      dispatcher,
      Bullet3::AxisSweep3.new([-100, -100, -100], [100, 100, 100]),
      configuration
    )

    expect(dbvt_world.num_collision_objects).to eq(0)
    expect(axis_world.num_collision_objects).to eq(0)
  end

  it "reports contact pairs and closest points" do
    world = Bullet3::CollisionWorld.create
    object_a = Bullet3::CollisionObject.new(Bullet3::Shapes::SphereShape.new(1.0))
    object_b = Bullet3::CollisionObject.new(Bullet3::Shapes::SphereShape.new(1.0))
    object_a.world_transform = Bullet3::Transform.new(Bullet3::Quaternion.identity, [0, 0, 0])
    object_b.world_transform = Bullet3::Transform.new(Bullet3::Quaternion.identity, [0, 1.5, 0])

    world.add_collision_object(object_a)
    world.add_collision_object(object_b)

    contacts = world.contact_pair_test(object_a, object_b)
    closest = world.closest_points(object_a, object_b)
    object_contacts = world.contact_test(object_a)

    expect(contacts).not_to be_empty
    expect(contacts.first.values_at(:body0, :body1)).to include(object_a, object_b)
    expect(contacts.first[:distance]).to be <= 0.0
    expect(closest.first.values_at(:body0, :body1)).to include(object_a, object_b)
    expect(object_contacts.any? { |contact| contact.values_at(:body0, :body1).include?(object_b) }).to be(true)
  end
end
