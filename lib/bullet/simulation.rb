# frozen_string_literal: true

module Bullet
  class Simulation
    attr_reader :world

    def initialize(mode: :direct)
      raise ArgumentError, "only :direct mode is currently supported" unless mode == :direct
      raise Bullet::Error, "native extension is required for Bullet::Simulation" unless defined?(DiscreteDynamicsWorld)

      @world = DiscreteDynamicsWorld.create
      @shapes = []
      @bodies = []
    end

    def set_gravity(x, y = nil, z = nil)
      vector = vector_argument(x, y, z)
      world.gravity = vector
    end

    def step_simulation(time_step: 1.0 / 240.0, max_sub_steps: 1, fixed_time_step: 1.0 / 240.0)
      world.step_simulation(time_step, max_sub_steps, fixed_time_step)
    end

    def create_collision_shape(type, **options)
      shape = case type.to_sym
              when :box
                Shapes::BoxShape.new(Vector3.coerce(options.fetch(:half_extents)))
              when :sphere
                Shapes::SphereShape.new(Float(options.fetch(:radius)))
              when :capsule
                Shapes::CapsuleShape.new(Float(options.fetch(:radius)), Float(options.fetch(:height)))
              when :cylinder
                Shapes::CylinderShape.new(Vector3.coerce(options.fetch(:half_extents)))
              when :cone
                Shapes::ConeShape.new(Float(options.fetch(:radius)), Float(options.fetch(:height)))
              when :plane, :static_plane
                normal = Vector3.coerce(options.fetch(:normal, [0, 1, 0]))
                Shapes::StaticPlaneShape.new(normal, Float(options.fetch(:offset, 0.0)))
              else
                raise ArgumentError, "unsupported collision shape: #{type}"
              end

      @shapes << shape
      @shapes.length - 1
    end

    def create_rigid_body(mass:, collision_shape:, position: [0, 0, 0], orientation: Quaternion.identity)
      shape = shape_for(collision_shape)
      transform = Transform.new(Quaternion.coerce(orientation), Vector3.coerce(position))
      motion_state = MotionState.new(transform)
      info = RigidBodyConstructionInfo.new(Float(mass), motion_state, shape)
      body = RigidBody.new(info)
      world.add_rigid_body(body)

      @bodies << body
      @bodies.length - 1
    end

    def get_base_position_and_orientation(body_id)
      transform = body_for(body_id).world_transform
      [transform.origin.to_a, transform.rotation.to_a]
    end

    def ray_test(from, to)
      world.ray_test(from, to)
    end

    def ray_test_all(from, to)
      world.ray_test_all(from, to)
    end

    def body(body_id)
      body_for(body_id)
    end

    def collision_shape(shape_id)
      shape_for(shape_id)
    end

    def remove_body(body_id)
      body = body_for(body_id)
      world.remove_rigid_body(body)
      @bodies[Integer(body_id)] = nil
      nil
    end

    def disconnect
      @bodies.compact.each { |body| world.remove_rigid_body(body) }
      @bodies.clear
      @shapes.clear
      nil
    end

    private

    def body_for(body_id)
      @bodies.fetch(Integer(body_id)) || raise(ArgumentError, "unknown body id: #{body_id}")
    end

    def shape_for(shape_id)
      @shapes.fetch(Integer(shape_id)) || raise(ArgumentError, "unknown collision shape id: #{shape_id}")
    end

    def vector_argument(x, y, z)
      return Vector3.coerce(x) if y.nil? && z.nil?

      Vector3.new(x, y, z)
    end
  end
end
