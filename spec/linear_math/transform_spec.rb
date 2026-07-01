# frozen_string_literal: true

RSpec.describe Bullet::Transform do
  it "creates identity transforms" do
    transform = described_class.identity

    expect(transform.origin).to eq(Bullet::Vector3.zero)
    expect(transform.rotation).to eq(Bullet::Quaternion.identity)
    expect(transform.to_a).to eq([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
  end

  it "transforms vectors" do
    transform = described_class.new(
      Bullet::Quaternion.from_axis_angle(Bullet::Vector3.new(0, 0, 1), Math::PI / 2),
      Bullet::Vector3.new(1, 2, 3)
    )

    expect(transform * Bullet::Vector3.new(1, 0, 0)).to eq(Bullet::Vector3.new(1, 3, 3))
  end

  it "composes and inverts transforms" do
    transform = described_class.new(
      Bullet::Quaternion.from_axis_angle(Bullet::Vector3.new(0, 0, 1), Math::PI / 4),
      Bullet::Vector3.new(3, 4, 5)
    )

    vector = Bullet::Vector3.new(6, 7, 8)

    expect(transform.inverse * (transform * vector)).to eq(vector)
    expect(transform.inverse_times(transform).origin).to eq(Bullet::Vector3.zero)
  end
end
