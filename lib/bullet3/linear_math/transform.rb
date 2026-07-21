# frozen_string_literal: true

module Bullet3
  class Transform
    attr_accessor :origin, :rotation

    def self.identity
      new
    end

    def initialize(rotation = Quaternion.identity, origin = Vector3.zero)
      @rotation = Quaternion.coerce(rotation)
      @origin = Vector3.coerce(origin)
    end

    def basis
      rotation.to_matrix
    end

    def *(other)
      case other
      when Transform
        self.class.new(rotation * other.rotation, transform_vector(other.origin))
      else
        transform_vector(other)
      end
    end

    def inverse
      inverse_rotation = rotation.inverse.normalized
      self.class.new(inverse_rotation, inverse_rotation.to_matrix * -origin)
    end

    def inverse_times(other)
      inverse * other
    end

    def to_a
      [origin.to_a, rotation.to_a]
    end

    private

    def transform_vector(vector)
      (basis * vector) + origin
    end
  end unless const_defined?(:Transform, false)
end
