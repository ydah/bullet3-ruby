# frozen_string_literal: true

require_relative "bullet3/version"

module Bullet3
  class Error < StandardError; end

  class << self
    attr_reader :native_load_error

    def native?
      const_defined?(:DiscreteDynamicsWorld, false)
    end

    def load_native_extension(required: false)
      return true if native?
      return false if ENV["BULLET3_SKIP_NATIVE"] == "1" && !required

      require "bullet3/bullet3"
      true
    rescue LoadError => error
      @native_load_error = error
      raise if required || ENV["BULLET3_REQUIRE_NATIVE"] == "1"

      false
    end

    def require_native_extension!(feature)
      return true if load_native_extension

      detail = native_load_error ? " #{native_load_error.message}" : ""
      raise Error, "native extension is required for #{feature}. Run `bundle exec rake compile` before using it.#{detail}"
    end
  end
end

Bullet3.load_native_extension(required: ENV["BULLET3_REQUIRE_NATIVE"] == "1")

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
