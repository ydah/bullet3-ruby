# frozen_string_literal: true

RSpec.describe Bullet::Data do
  it "finds bundled pybullet data assets" do
    path = described_class.find("plane.urdf")

    expect(path).to end_with("plane.urdf")
    expect(File).to exist(path)
  end

  it "manages additional search paths" do
    described_class.add_search_path(Dir.pwd)

    expect(described_class.search_paths).to include(Dir.pwd)
  ensure
    described_class.remove_search_path(Dir.pwd)
  end
end
