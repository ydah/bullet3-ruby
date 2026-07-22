# frozen_string_literal: true

RSpec.describe "Bullet3 ray and contact queries" do
  before do
    skip "native extension only" unless Bullet3.native?
  end

  def rigid_body(shape:, mass:, origin:)
    transform = Bullet3::Transform.new(Bullet3::Quaternion.identity, origin)
    motion_state = Bullet3::MotionState.new(transform)
    info = Bullet3::RigidBodyConstructionInfo.new(mass, motion_state, shape)
    Bullet3::RigidBody.new(info)
  end

  it "returns closest and all ray hits with Ruby body references" do
    world = Bullet3::DiscreteDynamicsWorld.create
    world.gravity = [0, -10, 0]

    sphere = rigid_body(
      shape: Bullet3::Shapes::SphereShape.new(1.0),
      mass: 0.0,
      origin: Bullet3::Vector3.new(0, 0, 0)
    )
    world.add_rigid_body(sphere)

    closest = world.ray_test_closest([0, 5, 0], [0, -5, 0])
    all_hits = world.ray_test_all([0, 5, 0], [0, -5, 0])

    expect(closest).not_to be_nil
    expect(closest[:body]).to equal(sphere)
    expect(closest[:point].y).to be_within(1e-3).of(1.0)
    expect(closest[:normal].y).to be > 0.0

    expect(all_hits.length).to eq(1)
    expect(all_hits.first[:body]).to equal(sphere)
  end

  it "returns contact manifolds after stepping" do
    world = Bullet3::DiscreteDynamicsWorld.create
    world.gravity = [0, -10, 0]

    ground = rigid_body(
      shape: Bullet3::Shapes::StaticPlaneShape.new(Bullet3::Vector3.new(0, 1, 0), 0),
      mass: 0.0,
      origin: Bullet3::Vector3.zero
    )
    sphere = rigid_body(
      shape: Bullet3::Shapes::SphereShape.new(1.0),
      mass: 1.0,
      origin: Bullet3::Vector3.new(0, 1, 0)
    )

    world.add_rigid_body(ground)
    world.add_rigid_body(sphere)
    world.step_simulation(1.0 / 60.0)

    manifolds = world.contact_manifolds
    manifold = manifolds.find { |entry| [entry[:body0], entry[:body1]].include?(sphere) }

    expect(manifold).not_to be_nil
    expect([manifold[:body0], manifold[:body1]]).to include(ground, sphere)
    expect(manifold[:points]).not_to be_empty
    expect(manifold[:points].first[:distance]).to be <= 0.05

    contacts = world.contact_pair_test(ground, sphere)
    sphere_contacts = world.contact_test(sphere)
    closest = world.closest_points(ground, sphere)

    expect(contacts).not_to be_empty
    expect(contacts.first.values_at(:body0, :body1)).to include(ground, sphere)
    expect(sphere_contacts.any? { |contact| contact.values_at(:body0, :body1).include?(ground) }).to be(true)
    expect(closest.first[:distance]).to be <= 0.05
  end
end
