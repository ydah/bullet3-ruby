# frozen_string_literal: true

module Bullet
  class Vector3
    include Enumerable

    EPSILON = 1e-6
    COMPONENTS = { 0 => :x, 1 => :y, 2 => :z }.freeze

    attr_accessor :x, :y, :z

    def self.zero
      new
    end

    def initialize(x = 0.0, y = 0.0, z = 0.0)
      @x = Float(x)
      @y = Float(y)
      @z = Float(z)
    end

    def [](index)
      public_send(component_name(index))
    end

    def []=(index, value)
      public_send("#{component_name(index)}=", Float(value))
    end

    def each(&block)
      to_a.each(&block)
    end

    def +(other)
      other = self.class.coerce(other)
      self.class.new(x + other.x, y + other.y, z + other.z)
    end

    def -(other)
      other = self.class.coerce(other)
      self.class.new(x - other.x, y - other.y, z - other.z)
    end

    def -@
      self.class.new(-x, -y, -z)
    end

    def *(scalar)
      self.class.new(x * Float(scalar), y * Float(scalar), z * Float(scalar))
    end

    def /(scalar)
      scalar = Float(scalar)
      raise ZeroDivisionError, "divided by 0" if scalar.zero?

      self.class.new(x / scalar, y / scalar, z / scalar)
    end

    def ==(other)
      other = self.class.try_coerce(other)
      return false unless other

      distance2(other) <= EPSILON * EPSILON
    end

    def length
      Math.sqrt(length2)
    end

    def length2
      dot(self)
    end

    def normalize
      normalized = self.normalized
      self.x = normalized.x
      self.y = normalized.y
      self.z = normalized.z
      self
    end

    alias normalize! normalize

    def normalized
      magnitude = length
      return self.class.zero if magnitude <= EPSILON

      self / magnitude
    end

    def dot(other)
      other = self.class.coerce(other)
      (x * other.x) + (y * other.y) + (z * other.z)
    end

    def cross(other)
      other = self.class.coerce(other)
      self.class.new(
        (y * other.z) - (z * other.y),
        (z * other.x) - (x * other.z),
        (x * other.y) - (y * other.x)
      )
    end

    def distance(other)
      Math.sqrt(distance2(other))
    end

    def distance2(other)
      (self - other).length2
    end

    def lerp(other, t)
      other = self.class.coerce(other)
      self + ((other - self) * Float(t))
    end

    def angle(other)
      other = self.class.coerce(other)
      denominator = length * other.length
      return 0.0 if denominator <= EPSILON

      cosine = dot(other) / denominator
      Math.acos(cosine.clamp(-1.0, 1.0))
    end

    def absolute
      self.class.new(x.abs, y.abs, z.abs)
    end

    def to_a
      [x, y, z]
    end

    def to_s
      "(#{x}, #{y}, #{z})"
    end

    def inspect
      "#<Bullet::Vector3 (#{x}, #{y}, #{z})>"
    end

    def self.coerce(value)
      coerced = try_coerce(value)
      return coerced if coerced

      raise TypeError, "expected Bullet::Vector3 or a 3-element Array"
    end

    def self.try_coerce(value)
      return value if value.is_a?(self)

      return new(*value) if value.respond_to?(:to_ary) && value.to_ary.length == 3

      nil
    end

    private

    def component_name(index)
      COMPONENTS.fetch(Integer(index))
    rescue KeyError
      raise IndexError, "index #{index} outside vector"
    end
  end unless const_defined?(:Vector3, false)

  class Vector3
    EPSILON = 1e-6 unless const_defined?(:EPSILON, false)

    include Enumerable unless include?(Enumerable)

    def each(&block)
      to_a.each(&block)
    end

    def to_s
      "(#{x}, #{y}, #{z})"
    end

    def inspect
      "#<Bullet::Vector3 (#{x}, #{y}, #{z})>"
    end

    def self.coerce(value)
      coerced = try_coerce(value)
      return coerced if coerced

      raise TypeError, "expected Bullet::Vector3 or a 3-element Array"
    end

    def self.try_coerce(value)
      return value if value.is_a?(self)

      return new(*value) if value.respond_to?(:to_ary) && value.to_ary.length == 3

      nil
    end
  end
end
