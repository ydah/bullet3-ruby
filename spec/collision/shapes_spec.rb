# frozen_string_literal: true

RSpec.describe "Bullet3::Shapes" do
  before do
    skip "native extension only" unless ENV["BULLET3_USE_NATIVE"] == "1"
  end

  it "creates primitive shapes with symbolic shape types" do
    expect(Bullet3::Shapes::BoxShape.new(Bullet3::Vector3.new(1, 1, 1)).shape_type).to eq(:box)
    expect(Bullet3::Shapes::SphereShape.new(1).shape_type).to eq(:sphere)
    expect(Bullet3::Shapes::CapsuleShape.new(0.5, 2).shape_type).to eq(:capsule)
    expect(Bullet3::Shapes::CylinderShape.new(Bullet3::Vector3.new(1, 1, 1)).shape_type).to eq(:cylinder)
    expect(Bullet3::Shapes::ConeShape.new(1, 2).shape_type).to eq(:cone)
    expect(Bullet3::Shapes::StaticPlaneShape.new(Bullet3::Vector3.new(0, 1, 0), 0).shape_type).to eq(:static_plane)
  end

  it "updates common collision shape properties" do
    shape = Bullet3::Shapes::BoxShape.new(Bullet3::Vector3.new(1, 1, 1))

    shape.local_scaling = Bullet3::Vector3.new(2, 3, 4)
    shape.margin = 0.1

    expect(shape.local_scaling).to eq(Bullet3::Vector3.new(2, 3, 4))
    expect(shape.margin).to be_within(1e-6).of(0.1)
  end

  it "calculates local inertia and bounding spheres" do
    shape = Bullet3::Shapes::SphereShape.new(2)

    inertia = shape.calculate_local_inertia(3)
    center, radius = shape.bounding_sphere
    min, max = shape.aabb(Bullet3::Transform.identity)

    expect(inertia.x).to be > 0
    expect(inertia.y).to be > 0
    expect(inertia.z).to be > 0
    expect(center).to eq(Bullet3::Vector3.zero)
    expect(radius).to be >= 2.0
    expect(min).to eq(Bullet3::Vector3.new(-2, -2, -2))
    expect(max).to eq(Bullet3::Vector3.new(2, 2, 2))
  end

  it "creates compound shapes" do
    compound = Bullet3::Shapes::CompoundShape.new
    box = Bullet3::Shapes::BoxShape.new(Bullet3::Vector3.new(1, 1, 1))
    transform = Bullet3::Transform.new(Bullet3::Quaternion.identity, Bullet3::Vector3.new(3, 0, 0))

    compound.add_child_shape(transform, box)

    expect(compound.shape_type).to eq(:compound)
    expect(compound.num_child_shapes).to eq(1)
    expect(compound.child_shape(0).shape_type).to eq(:box)
    expect(compound.child_transform(0).origin).to eq(Bullet3::Vector3.new(3, 0, 0))
  end

  it "creates convex hull shapes" do
    shape = Bullet3::Shapes::ConvexHullShape.new([
      [0, 0, 0],
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1]
    ])
    shape.add_point(Bullet3::Vector3.new(1, 1, 1))

    expect(shape.shape_type).to eq(:convex_hull)
    expect(shape.num_points).to eq(5)
    expect(shape.point(4)).to eq(Bullet3::Vector3.new(1, 1, 1))
  end

  it "creates triangle mesh shapes" do
    triangles = [
      [[0, 0, 0], [1, 0, 0], [0, 1, 0]],
      [[0, 0, 0], [0, 1, 0], [0, 0, 1]]
    ]

    concave = Bullet3::Shapes::TriangleMeshShape.new(triangles)
    convex = Bullet3::Shapes::ConvexTriangleMeshShape.new(triangles)

    expect(concave.shape_type).to eq(:triangle_mesh)
    expect(concave.num_triangles).to eq(2)
    expect(concave.uses_quantized_aabb_compression).to be(true)
    expect(convex.shape_type).to eq(:convex_triangle_mesh)
    expect(convex.num_triangles).to eq(2)
  end

  it "creates multi-sphere and heightfield shapes" do
    multi_sphere = Bullet3::Shapes::MultiSphereShape.new(
      [[0, 0, 0], [1, 0, 0]],
      [0.5, 0.25]
    )
    heightfield = Bullet3::Shapes::HeightfieldShape.new(2, 2, [0.0, 1.0, 2.0, 3.0], 0.0, 3.0)

    expect(multi_sphere.shape_type).to eq(:multi_sphere)
    expect(multi_sphere.sphere_count).to eq(2)
    expect(multi_sphere.sphere_position(1)).to eq(Bullet3::Vector3.new(1, 0, 0))
    expect(multi_sphere.sphere_radius(0)).to be_within(1e-6).of(0.5)

    expect(heightfield.shape_type).to eq(:heightfield)
    expect(heightfield.width).to eq(2)
    expect(heightfield.length).to eq(2)
    expect(heightfield.height(1, 1)).to be_within(1e-6).of(3.0)
    expect(heightfield.up_axis).to eq(1)
    expect(heightfield.vertex(1, 1)).to be_a(Bullet3::Vector3)
  end
end
