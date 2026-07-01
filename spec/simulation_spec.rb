# frozen_string_literal: true

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
end
