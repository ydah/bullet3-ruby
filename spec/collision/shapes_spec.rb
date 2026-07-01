# frozen_string_literal: true

RSpec.describe "Bullet::Shapes" do
  before do
    skip "native extension only" unless ENV["BULLET_RUBY_USE_NATIVE"] == "1"
  end

  it "creates primitive shapes with symbolic shape types" do
    expect(Bullet::Shapes::BoxShape.new(Bullet::Vector3.new(1, 1, 1)).shape_type).to eq(:box)
    expect(Bullet::Shapes::SphereShape.new(1).shape_type).to eq(:sphere)
    expect(Bullet::Shapes::CapsuleShape.new(0.5, 2).shape_type).to eq(:capsule)
    expect(Bullet::Shapes::CylinderShape.new(Bullet::Vector3.new(1, 1, 1)).shape_type).to eq(:cylinder)
    expect(Bullet::Shapes::ConeShape.new(1, 2).shape_type).to eq(:cone)
    expect(Bullet::Shapes::StaticPlaneShape.new(Bullet::Vector3.new(0, 1, 0), 0).shape_type).to eq(:static_plane)
  end

  it "updates common collision shape properties" do
    shape = Bullet::Shapes::BoxShape.new(Bullet::Vector3.new(1, 1, 1))

    shape.local_scaling = Bullet::Vector3.new(2, 3, 4)
    shape.margin = 0.1

    expect(shape.local_scaling).to eq(Bullet::Vector3.new(2, 3, 4))
    expect(shape.margin).to be_within(1e-6).of(0.1)
  end

  it "calculates local inertia and bounding spheres" do
    shape = Bullet::Shapes::SphereShape.new(2)

    inertia = shape.calculate_local_inertia(3)
    center, radius = shape.bounding_sphere

    expect(inertia.x).to be > 0
    expect(inertia.y).to be > 0
    expect(inertia.z).to be > 0
    expect(center).to eq(Bullet::Vector3.zero)
    expect(radius).to be >= 2.0
  end
end
