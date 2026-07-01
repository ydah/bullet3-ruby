# frozen_string_literal: true

module Bullet
  class Quaternion
    EPSILON = 1e-6

    attr_accessor :x, :y, :z, :w

    def self.identity
      new(0.0, 0.0, 0.0, 1.0)
    end

    def self.from_axis_angle(axis, angle)
      axis = Vector3.coerce(axis).normalized
      half_angle = Float(angle) / 2.0
      scale = Math.sin(half_angle)

      new(axis.x * scale, axis.y * scale, axis.z * scale, Math.cos(half_angle))
    end

    def self.from_euler(roll, pitch, yaw)
      cr = Math.cos(Float(roll) / 2.0)
      sr = Math.sin(Float(roll) / 2.0)
      cp = Math.cos(Float(pitch) / 2.0)
      sp = Math.sin(Float(pitch) / 2.0)
      cy = Math.cos(Float(yaw) / 2.0)
      sy = Math.sin(Float(yaw) / 2.0)

      new(
        (sr * cp * cy) - (cr * sp * sy),
        (cr * sp * cy) + (sr * cp * sy),
        (cr * cp * sy) - (sr * sp * cy),
        (cr * cp * cy) + (sr * sp * sy)
      )
    end

    def initialize(x = 0.0, y = 0.0, z = 0.0, w = 1.0)
      @x = Float(x)
      @y = Float(y)
      @z = Float(z)
      @w = Float(w)
    end

    def *(other)
      other = self.class.coerce(other)
      self.class.new(
        (w * other.x) + (x * other.w) + (y * other.z) - (z * other.y),
        (w * other.y) - (x * other.z) + (y * other.w) + (z * other.x),
        (w * other.z) + (x * other.y) - (y * other.x) + (z * other.w),
        (w * other.w) - (x * other.x) - (y * other.y) - (z * other.z)
      )
    end

    def ==(other)
      other = self.class.try_coerce(other)
      return false unless other

      to_a.zip(other.to_a).all? { |left, right| (left - right).abs <= EPSILON }
    end

    def inverse
      magnitude = length2
      raise ZeroDivisionError, "cannot invert a zero-length quaternion" if magnitude <= EPSILON

      self.class.new(-x / magnitude, -y / magnitude, -z / magnitude, w / magnitude)
    end

    def angle
      2.0 * Math.acos(w.clamp(-1.0, 1.0))
    end

    def axis
      scale = Math.sqrt(1.0 - (w * w))
      return Vector3.new(1.0, 0.0, 0.0) if scale <= EPSILON

      Vector3.new(x / scale, y / scale, z / scale)
    end

    def slerp(other, t)
      other = self.class.coerce(other)
      t = Float(t)
      cosine = dot(other)

      if cosine.negative?
        other = self.class.new(-other.x, -other.y, -other.z, -other.w)
        cosine = -cosine
      end

      return linear_interpolate(other, t).normalized if cosine > 0.9995

      theta = Math.acos(cosine.clamp(-1.0, 1.0))
      sin_theta = Math.sin(theta)
      left = Math.sin((1.0 - t) * theta) / sin_theta
      right = Math.sin(t * theta) / sin_theta

      self.class.new(
        (x * left) + (other.x * right),
        (y * left) + (other.y * right),
        (z * left) + (other.z * right),
        (w * left) + (other.w * right)
      )
    end

    def to_euler
      roll = Math.atan2(2.0 * ((w * x) + (y * z)), 1.0 - (2.0 * ((x * x) + (y * y))))
      pitch_sin = 2.0 * ((w * y) - (z * x))
      pitch = if pitch_sin.abs >= 1.0
                pitch_sin.negative? ? -Math::PI / 2.0 : Math::PI / 2.0
              else
                Math.asin(pitch_sin)
              end
      yaw = Math.atan2(2.0 * ((w * z) + (x * y)), 1.0 - (2.0 * ((y * y) + (z * z))))

      [roll, pitch, yaw]
    end

    def to_matrix
      Matrix3x3.from_quaternion(self)
    end

    def normalized
      magnitude = Math.sqrt(length2)
      return self.class.identity if magnitude <= EPSILON

      self.class.new(x / magnitude, y / magnitude, z / magnitude, w / magnitude)
    end

    def to_a
      [x, y, z, w]
    end

    def self.coerce(value)
      coerced = try_coerce(value)
      return coerced if coerced

      raise TypeError, "expected Bullet::Quaternion or a 4-element Array"
    end

    def self.try_coerce(value)
      return value if value.is_a?(self)

      return new(*value) if value.respond_to?(:to_ary) && value.to_ary.length == 4

      nil
    end

    protected

    def dot(other)
      (x * other.x) + (y * other.y) + (z * other.z) + (w * other.w)
    end

    private

    def length2
      dot(self)
    end

    def linear_interpolate(other, t)
      self.class.new(
        x + ((other.x - x) * t),
        y + ((other.y - y) * t),
        z + ((other.z - z) * t),
        w + ((other.w - w) * t)
      )
    end
  end unless const_defined?(:Quaternion, false)
end
