# frozen_string_literal: true

RSpec.describe Bullet3::Transform do
  it "creates identity transforms" do
    transform = described_class.identity

    expect(transform.origin).to eq(Bullet3::Vector3.zero)
    expect(transform.rotation).to eq(Bullet3::Quaternion.identity)
    expect(transform.to_a).to eq([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
  end

  it "transforms vectors" do
    transform = described_class.new(
      Bullet3::Quaternion.from_axis_angle(Bullet3::Vector3.new(0, 0, 1), Math::PI / 2),
      Bullet3::Vector3.new(1, 2, 3)
    )

    expect(transform * Bullet3::Vector3.new(1, 0, 0)).to eq(Bullet3::Vector3.new(1, 3, 3))
  end

  it "composes and inverts transforms" do
    transform = described_class.new(
      Bullet3::Quaternion.from_axis_angle(Bullet3::Vector3.new(0, 0, 1), Math::PI / 4),
      Bullet3::Vector3.new(3, 4, 5)
    )

    vector = Bullet3::Vector3.new(6, 7, 8)

    expect(transform.inverse * (transform * vector)).to eq(vector)
    expect(transform.inverse_times(transform).origin).to eq(Bullet3::Vector3.zero)
  end
end
