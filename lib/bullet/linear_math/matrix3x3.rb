# frozen_string_literal: true

module Bullet
  class Matrix3x3
    attr_reader :rows

    def self.identity
      new([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    end

    def self.from_quaternion(quaternion)
      q = Quaternion.coerce(quaternion).normalized
      xx = q.x * q.x
      yy = q.y * q.y
      zz = q.z * q.z
      xy = q.x * q.y
      xz = q.x * q.z
      yz = q.y * q.z
      wx = q.w * q.x
      wy = q.w * q.y
      wz = q.w * q.z

      new([
        [1.0 - (2.0 * (yy + zz)), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - (2.0 * (xx + zz)), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - (2.0 * (xx + yy))]
      ])
    end

    def initialize(rows = self.class.identity.rows)
      @rows = rows.map { |row| row.map { |value| Float(value) } }
      raise ArgumentError, "matrix must be 3x3" unless @rows.length == 3 && @rows.all? { |row| row.length == 3 }
    end

    def *(vector)
      vector = Vector3.coerce(vector)
      Vector3.new(
        dot_row(0, vector),
        dot_row(1, vector),
        dot_row(2, vector)
      )
    end

    def transpose
      self.class.new(rows.transpose)
    end

    def to_a
      rows.map(&:dup)
    end

    private

    def dot_row(index, vector)
      row = rows[index]
      (row[0] * vector.x) + (row[1] * vector.y) + (row[2] * vector.z)
    end
  end unless const_defined?(:Matrix3x3, false)
end
