# frozen_string_literal: true

RSpec.describe Bullet3::Matrix3x3 do
  it "creates identity matrices" do
    matrix = described_class.identity

    expect(matrix.to_a).to eq([
      [1.0, 0.0, 0.0],
      [0.0, 1.0, 0.0],
      [0.0, 0.0, 1.0]
    ])
  end

  it "builds a rotation matrix from a quaternion" do
    matrix = described_class.from_quaternion(
      Bullet3::Quaternion.from_axis_angle(Bullet3::Vector3.new(0, 0, 1), Math::PI / 2)
    )

    rotated = matrix * Bullet3::Vector3.new(1, 0, 0)

    expect(rotated.x).to be_within(1e-6).of(0.0)
    expect(rotated.y).to be_within(1e-6).of(1.0)
    expect(rotated.z).to be_within(1e-6).of(0.0)
  end

  it "transposes matrices" do
    matrix = described_class.new(1, 2, 3, 4, 5, 6, 7, 8, 9)

    expect(matrix.transpose.to_a).to eq([
      [1.0, 4.0, 7.0],
      [2.0, 5.0, 8.0],
      [3.0, 6.0, 9.0]
    ])
  end
end
