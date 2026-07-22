# frozen_string_literal: true

RSpec.describe "Bullet3 soft bodies" do
  before do
    skip "native extension only" unless Bullet3.native?
  end

  def rigid_body(origin)
    shape = Bullet3::Shapes::SphereShape.new(0.25)
    transform = Bullet3::Transform.new(Bullet3::Quaternion.identity, origin)
    motion_state = Bullet3::MotionState.new(transform)
    info = Bullet3::RigidBodyConstructionInfo.new(0.0, motion_state, shape)
    Bullet3::RigidBody.new(info)
  end

  it "creates helper soft bodies and steps a soft-rigid world" do
    world = Bullet3::SoftBody::SoftRigidDynamicsWorld.create
    world.gravity = [0, -9.81, 0]
    world.draw_flags = 1
    info = world.world_info
    info.air_density = 1.1
    info.water_normal = [0, 1, 0]

    anchor_body = rigid_body(Bullet3::Vector3.new(0, 1, 0))
    world.add_rigid_body(anchor_body)

    rope = Bullet3::SoftBody::Helpers.create_rope(info, [0, 1, 0], [1, 1, 0], 4, 1)
    rope.set_total_mass(1.0)
    rope.generate_bending_constraints(2)
    rope.wind_velocity = [0.1, 0, 0]
    rope.append_anchor(0, anchor_body)

    patch = Bullet3::SoftBody::Helpers.create_patch(
      info,
      [-1, 1, -1],
      [1, 1, -1],
      [-1, 1, 1],
      [1, 1, 1],
      4,
      4,
      1,
      true
    )
    patch.set_total_mass(2.0)
    patch.self_collision = true

    ellipsoid = Bullet3::SoftBody::Helpers.create_ellipsoid(info, [0, 2, 0], [0.5, 0.25, 0.5], 8)

    world.add_soft_body(rope)
    world.add_soft_body(patch)
    world.add_soft_body(ellipsoid)
    world.step_simulation(1.0 / 60.0)

    expect(info.air_density).to be_within(1e-6).of(1.1)
    expect(info.water_normal).to eq(Bullet3::Vector3.new(0, 1, 0))
    expect(world.draw_flags).to eq(1)
    expect(world.num_collision_objects).to eq(4)
    expect(world.num_soft_bodies).to eq(3)
    expect(rope.node_count).to eq(6)
    expect(rope.link_count).to be > 0
    expect(rope.total_mass).to be_within(1e-6).of(1.0)
    expect(rope.wind_velocity).to eq(Bullet3::Vector3.new(0.1, 0, 0))
    expect(rope.node_position(0)).to be_a(Bullet3::Vector3)
    expect(rope.aabb).to all(be_a(Bullet3::Vector3))
    expect(patch.face_count).to be > 0
    expect(patch.self_collision?).to be(true)
    expect(ellipsoid.node_count).to be > 0

    world.remove_soft_body(ellipsoid)
    expect(world.num_soft_bodies).to eq(2)
  end
end
