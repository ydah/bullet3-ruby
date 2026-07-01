# frozen_string_literal: true

RSpec.describe Bullet do
  it "has a version number" do
    expect(described_class::VERSION).not_to be nil
  end

  it "keeps the legacy gem namespace as an alias" do
    expect(BulletRuby::VERSION).to eq(described_class::VERSION)
    expect(BulletRuby::Error).to eq(described_class::Error)
  end
end
