# frozen_string_literal: true

RSpec.describe Bullet3::Vector3 do
  it "stores and exposes vector components" do
    vector = described_class.new(1, 2, 3)

    expect(vector.x).to eq(1.0)
    expect(vector.y).to eq(2.0)
    expect(vector.z).to eq(3.0)
    expect(vector[0]).to eq(1.0)
    expect(vector.to_a).to eq([1.0, 2.0, 3.0])
  end

  it "updates components by writer and index" do
    vector = described_class.zero

    vector.x = 4
    vector[1] = 5
    vector.z = 6

    expect(vector.to_a).to eq([4.0, 5.0, 6.0])
  end

  it "performs vector arithmetic" do
    left = described_class.new(1, 2, 3)
    right = described_class.new(4, 5, 6)

    expect(left + right).to eq(described_class.new(5, 7, 9))
    expect(right - left).to eq(described_class.new(3, 3, 3))
    expect(left * 2).to eq(described_class.new(2, 4, 6))
    expect(right / 2).to eq(described_class.new(2, 2.5, 3))
    expect(-left).to eq(described_class.new(-1, -2, -3))
  end

  it "calculates vector products and distances" do
    x_axis = described_class.new(1, 0, 0)
    y_axis = described_class.new(0, 1, 0)

    expect(x_axis.dot(y_axis)).to eq(0.0)
    expect(x_axis.cross(y_axis)).to eq(described_class.new(0, 0, 1))
    expect(x_axis.distance(y_axis)).to be_within(1e-6).of(Math.sqrt(2))
    expect(x_axis.angle(y_axis)).to be_within(1e-6).of(Math::PI / 2)
  end

  it "normalizes vectors" do
    vector = described_class.new(3, 0, 4)

    expect(vector.normalized).to eq(described_class.new(0.6, 0, 0.8))
    expect(vector.normalize).to equal(vector)
    expect(vector.length).to be_within(1e-6).of(1.0)
  end

  it "interpolates and formats vectors" do
    vector = described_class.new(-1, 2, -3)

    expect(vector.absolute).to eq(described_class.new(1, 2, 3))
    expect(vector.lerp(described_class.new(1, 4, 1), 0.5)).to eq(described_class.new(0, 3, -1))
    expect(vector.to_s).to eq("(-1.0, 2.0, -3.0)")
    expect(vector.inspect).to eq("#<Bullet3::Vector3 (-1.0, 2.0, -3.0)>")
  end
end
