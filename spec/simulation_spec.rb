# frozen_string_literal: true

require "tmpdir"

RSpec.describe Bullet::Simulation do
  before do
    skip "native extension only" unless ENV["BULLET_RUBY_USE_NATIVE"] == "1"
  end

  it "creates primitive bodies and steps the simulation" do
    sim = described_class.new
    sim.set_gravity(0, -10, 0)

    plane_shape = sim.create_collision_shape(:static_plane, normal: [0, 1, 0], offset: 0)
    sphere_shape = sim.create_collision_shape(:sphere, radius: 1.0)
    sim.create_rigid_body(mass: 0.0, collision_shape: plane_shape)
    sphere_id = sim.create_rigid_body(mass: 1.0, collision_shape: sphere_shape, position: [0, 10, 0])

    60.times { sim.step_simulation(time_step: 1.0 / 60.0, fixed_time_step: 1.0 / 60.0) }

    position, orientation = sim.get_base_position_and_orientation(sphere_id)
    hit = sim.ray_test([0, 10, 0], [0, -10, 0])

    expect(position[1]).to be < 10.0
    expect(orientation).to eq([0.0, 0.0, 0.0, 1.0])
    expect(hit[:body]).to equal(sim.body(sphere_id))
  ensure
    sim&.disconnect
  end

  it "removes bodies and validates ids" do
    sim = described_class.new
    shape = sim.create_collision_shape(:box, half_extents: [1, 1, 1])
    body = sim.create_rigid_body(mass: 1.0, collision_shape: shape)

    expect(sim.body(body)).to be_a(Bullet::RigidBody)
    sim.remove_body(body)
    expect { sim.body(body) }.to raise_error(ArgumentError)
  ensure
    sim&.disconnect
  end

  it "loads single-link primitive URDF files from data paths" do
    sim = described_class.new
    sim.set_gravity(0, -10, 0)

    plane_id = sim.load_urdf("plane.urdf", use_fixed_base: true)
    cube_id = sim.load_urdf("cube.urdf", base_position: [0, 3, 0])
    30.times { sim.step_simulation(time_step: 1.0 / 60.0, fixed_time_step: 1.0 / 60.0) }

    position, = sim.get_base_position_and_orientation(cube_id)
    expect(sim.body(plane_id)).to be_static
    expect(sim.body(cube_id).mass).to be_within(1e-6).of(1.0)
    expect(sim.body(cube_id).friction).to be_within(1e-6).of(1.0)
    expect(position[1]).to be < 3.0
  ensure
    sim&.disconnect
  end

  it "loads single-link OBJ mesh URDF collision geometry" do
    Dir.mktmpdir do |dir|
      File.write(
        File.join(dir, "tetra.obj"),
        <<~OBJ
          v 0 0 0
          v 1 0 0
          v 0 1 0
          v 0 0 1
          f 1 2 3
          f 1 2 4
          f 1 3 4
          f 2 3 4
        OBJ
      )
      File.write(
        File.join(dir, "mesh.urdf"),
        <<~URDF
          <robot name="mesh">
            <link name="base">
              <collision>
                <geometry>
                  <mesh filename="tetra.obj" scale="2 2 2"/>
                </geometry>
              </collision>
            </link>
          </robot>
        URDF
      )

      sim = described_class.new
      body_id = sim.load_urdf(File.join(dir, "mesh.urdf"), use_fixed_base: true)

      expect(sim.body(body_id)).to be_static
      expect(sim.body(body_id).collision_shape.shape_type).to eq(:triangle_mesh)
      expect(sim.get_aabb(body_id)).to eq([[0.0, 0.0, 0.0], [2.0, 2.0, 2.0]])
    ensure
      sim&.disconnect
    end
  end

  it "exposes high-level query and dynamics helpers" do
    sim = described_class.new
    plane_shape = sim.create_collision_shape(:static_plane, normal: [0, 1, 0], offset: 0)
    sphere_shape = sim.create_collision_shape(:sphere, radius: 1.0)
    plane_id = sim.create_rigid_body(mass: 0.0, collision_shape: plane_shape)
    sphere_id = sim.create_rigid_body(mass: 1.0, collision_shape: sphere_shape, position: [0, 1, 0])

    sim.change_dynamics(sphere_id, lateral_friction: 0.2, restitution: 0.5)
    sim.step_simulation(time_step: 1.0 / 60.0)

    contacts = sim.get_contact_points(body_a: plane_id, body_b: sphere_id)
    batch = sim.ray_test_batch([[[0, 5, 0], [0, -5, 0]], [[5, 5, 0], [5, 4, 0]]])
    aabb = sim.get_aabb(sphere_id)

    expect(sim.body(sphere_id).friction).to be_within(1e-6).of(0.2)
    expect(sim.body(sphere_id).restitution).to be_within(1e-6).of(0.5)
    expect(contacts.first.values_at(:body0, :body1)).to contain_exactly(plane_id, sphere_id)
    expect(batch.first[:body]).to equal(sim.body(sphere_id))
    expect(batch.last).to be_nil
    expect(aabb).to eq([[-1.0, 0.0, -1.0], [1.0, 2.0, 1.0]])

    sim.set_base_position_and_orientation(sphere_id, [0, 4, 0])
    position, = sim.get_base_position_and_orientation(sphere_id)
    expect(position).to eq([0.0, 4.0, 0.0])

    sim.reset_simulation
    expect { sim.body(sphere_id) }.to raise_error(ArgumentError)
    expect(sim.world.num_collision_objects).to eq(0)
  ensure
    sim&.disconnect
  end

  it "creates and removes high-level constraints" do
    sim = described_class.new
    shape = sim.create_collision_shape(:sphere, radius: 0.5)
    body_a = sim.create_rigid_body(mass: 1.0, collision_shape: shape, position: [0, 1, 0])
    body_b = sim.create_rigid_body(mass: 1.0, collision_shape: shape, position: [1, 1, 0])

    p2p = sim.create_constraint(
      :point2point,
      body_a: body_a,
      body_b: body_b,
      pivot_in_a: [0, 0, 0],
      pivot_in_b: [0, 0, 0],
      disable_collisions_between_linked_bodies: true
    )
    fixed = sim.create_constraint(
      :fixed,
      body_a: body_a,
      body_b: body_b,
      frame_in_a: Bullet::Transform.identity,
      frame_in_b: Bullet::Transform.identity
    )

    expect(sim.constraint(p2p).constraint_type).to eq(:point2point)
    expect(sim.constraint(fixed).constraint_type).to eq(:fixed)
    expect(sim.world.num_constraints).to eq(2)

    sim.remove_constraint(p2p)
    expect { sim.constraint(p2p) }.to raise_error(ArgumentError)
    expect(sim.world.num_constraints).to eq(1)
  ensure
    sim&.disconnect
  end
end
