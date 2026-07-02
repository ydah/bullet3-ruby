# frozen_string_literal: true

RSpec.describe Bullet::DebugDraw do
  before do
    skip "native extension only" unless ENV["BULLET_RUBY_USE_NATIVE"] == "1"
  end

  it "collects lines from Bullet debug drawing" do
    sim = Bullet::Simulation.new
    shape = sim.create_collision_shape(:box, half_extents: [1, 1, 1])
    sim.create_rigid_body(mass: 0.0, collision_shape: shape)
    drawer = described_class.new
    drawer.debug_mode = described_class::DRAW_WIREFRAME | described_class::DRAW_AABB

    result = sim.debug_draw_world(drawer)

    expect(result).to equal(drawer)
    expect(drawer.lines).not_to be_empty
    expect(drawer.lines.first).to include(:from, :to, :color)
  ensure
    sim&.disconnect
  end
end
