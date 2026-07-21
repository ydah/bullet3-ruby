# frozen_string_literal: true

require_relative "bullet3/version"

module Bullet3
  class Error < StandardError; end
end

begin
  require "bullet3/bullet3" if ENV["BULLET3_USE_NATIVE"] == "1"
rescue LoadError
  raise if ENV["BULLET3_REQUIRE_NATIVE"] == "1"
end

require_relative "bullet3/linear_math"
require_relative "bullet3/collision"
require_relative "bullet3/dynamics"
require_relative "bullet3/constraints"
require_relative "bullet3/data"
require_relative "bullet3/debug_draw"
require_relative "bullet3/vehicle"
require_relative "bullet3/soft_body"
require_relative "bullet3/multi_body"
require_relative "bullet3/simulation"
require_relative "bullet3/io"
