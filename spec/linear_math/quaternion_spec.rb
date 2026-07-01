# frozen_string_literal: true

RSpec.describe Bullet::Quaternion do
  it "builds identity and array representations" do
    quaternion = described_class.identity

    expect(quaternion.to_a).to eq([0.0, 0.0, 0.0, 1.0])
    expect(quaternion.axis).to eq(Bullet::Vector3.new(1, 0, 0))
    expect(quaternion.angle).to eq(0.0)
  end

  it "builds a quaternion from an axis and angle" do
    quaternion = described_class.from_axis_angle(Bullet::Vector3.new(0, 0, 1), Math::PI)

    expect(quaternion.x).to be_within(1e-12).of(0.0)
    expect(quaternion.y).to be_within(1e-12).of(0.0)
    expect(quaternion.z).to be_within(1e-12).of(1.0)
    expect(quaternion.w).to be_within(1e-12).of(0.0)
  end

  it "round-trips euler angles" do
    quaternion = described_class.from_euler(0.1, 0.2, 0.3)
    roll, pitch, yaw = quaternion.to_euler

    expect(roll).to be_within(1e-12).of(0.1)
    expect(pitch).to be_within(1e-12).of(0.2)
    expect(yaw).to be_within(1e-12).of(0.3)
  end

  it "multiplies and inverts quaternions" do
    quaternion = described_class.from_axis_angle(Bullet::Vector3.new(0, 1, 0), Math::PI / 4)
    identity = quaternion * quaternion.inverse

    expect(identity.normalized).to eq(described_class.identity)
  end

  it "spherically interpolates rotations" do
    start = described_class.identity
    finish = described_class.from_axis_angle(Bullet::Vector3.new(0, 0, 1), Math::PI)

    halfway = start.slerp(finish, 0.5)

    expect(halfway.angle).to be_within(1e-12).of(Math::PI / 2)
    expect(halfway.axis).to eq(Bullet::Vector3.new(0, 0, 1))
  end
end
