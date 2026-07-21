# frozen_string_literal: true

require "tmpdir"

RSpec.describe Bullet3::IO do
  before do
    skip "native extension only" unless ENV["BULLET3_USE_NATIVE"] == "1"
  end

  it "saves and imports native .bullet world files" do
    Dir.mktmpdir do |dir|
      sim = Bullet3::Simulation.new
      shape = sim.create_collision_shape(:sphere, radius: 1.0)
      sim.create_rigid_body(mass: 0.0, collision_shape: shape)

      path = File.join(dir, "world.bullet")
      serializer = described_class::Serializer.new(sim)
      serializer.save_world(sim, path)

      loaded = Bullet3::Simulation.new
      importer = serializer.load_world(loaded, path)

      expect(File.binread(path, 6)).to eq("BULLET")
      expect(importer.num_rigid_bodies).to eq(1)
      expect(importer.num_collision_shapes).to be >= 1
      expect(loaded.world.num_collision_objects).to eq(1)
    ensure
      loaded&.disconnect
      sim&.disconnect
    end
  end

  it "exposes URDF/SDF/MJCF importer facades" do
    sim = Bullet3::Simulation.new
    importer = described_class::URDFImporter.new(sim)

    body_id = importer.load("plane.urdf", use_fixed_base: true)

    expect(sim.body(body_id)).to be_static
  ensure
    sim&.disconnect
  end
end
