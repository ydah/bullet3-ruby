# frozen_string_literal: true

require_relative "bullet/version"

module Bullet
  class Error < StandardError; end
end

begin
  require "bullet_ruby/bullet_ruby" if ENV["BULLET_RUBY_USE_NATIVE"] == "1"
rescue LoadError
  raise if ENV["BULLET_RUBY_REQUIRE_NATIVE"] == "1"
end

require_relative "bullet/linear_math"
require_relative "bullet/collision"
require_relative "bullet/dynamics"
require_relative "bullet/constraints"
require_relative "bullet/vehicle"
require_relative "bullet/soft_body"
require_relative "bullet/simulation"

module BulletRuby
  VERSION = Bullet::VERSION unless const_defined?(:VERSION, false)
  Error = Bullet::Error unless const_defined?(:Error, false)
end
