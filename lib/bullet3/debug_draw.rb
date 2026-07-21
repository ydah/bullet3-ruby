# frozen_string_literal: true

module Bullet3
  class DebugDraw
    NO_DEBUG = 0 unless const_defined?(:NO_DEBUG, false)
    DRAW_WIREFRAME = 1 unless const_defined?(:DRAW_WIREFRAME, false)
    DRAW_AABB = 2 unless const_defined?(:DRAW_AABB, false)
    DRAW_CONTACT_POINTS = 8 unless const_defined?(:DRAW_CONTACT_POINTS, false)
    DRAW_CONSTRAINTS = 1 << 11 unless const_defined?(:DRAW_CONSTRAINTS, false)
    DRAW_CONSTRAINT_LIMITS = 1 << 12 unless const_defined?(:DRAW_CONSTRAINT_LIMITS, false)
    DRAW_FRAMES = 1 << 15 unless const_defined?(:DRAW_FRAMES, false)

    attr_accessor :debug_mode
    attr_reader :lines, :contact_points, :warnings, :texts

    def initialize(target = nil)
      @target = target
      @debug_mode = DRAW_WIREFRAME | DRAW_AABB | DRAW_CONSTRAINTS | DRAW_CONSTRAINT_LIMITS
      clear
    end

    def clear
      @lines = []
      @contact_points = []
      @warnings = []
      @texts = []
      nil
    end

    def draw_line(from, to, color)
      entry = { from: vector_array(from), to: vector_array(to), color: vector_array(color) }
      lines << entry
      @target.draw_line(entry[:from], entry[:to], entry[:color]) if @target&.respond_to?(:draw_line)
      nil
    end

    def draw_contact_point(point, normal, distance = nil, life_time = nil, color = nil, **options)
      distance = options.fetch(:distance, distance)
      life_time = options.fetch(:life_time, life_time)
      color = options.fetch(:color, color)
      entry = {
        point: vector_array(point),
        normal: vector_array(normal),
        distance: Float(distance),
        life_time: Integer(life_time),
        color: vector_array(color)
      }
      contact_points << entry
      @target.draw_contact_point(entry) if @target&.respond_to?(:draw_contact_point)
      nil
    end

    def report_error_warning(message)
      warning = message.to_s
      warnings << warning
      @target.report_error_warning(warning) if @target&.respond_to?(:report_error_warning)
      nil
    end

    def draw_3d_text(location, text)
      entry = { location: vector_array(location), text: text.to_s }
      texts << entry
      @target.draw_3d_text(entry[:location], entry[:text]) if @target&.respond_to?(:draw_3d_text)
      nil
    end

    private

    def vector_array(value)
      Vector3.coerce(value).to_a
    end
  end unless const_defined?(:DebugDraw, false)
end
