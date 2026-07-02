# frozen_string_literal: true

require "rexml/document"

module Bullet
  class Simulation
    attr_reader :world

    def initialize(mode: :direct)
      raise ArgumentError, "only :direct mode is currently supported" unless mode == :direct
      raise Bullet::Error, "native extension is required for Bullet::Simulation" unless defined?(DiscreteDynamicsWorld)

      @world = DiscreteDynamicsWorld.create
      @shapes = []
      @bodies = []
      @constraints = []
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

    def create_constraint(type, **options)
      constraint = build_constraint(type.to_sym, options)
      world.add_constraint(constraint, options.fetch(:disable_collisions_between_linked_bodies, false))

      @constraints << constraint
      @constraints.length - 1
    end

    def load_urdf(filename, base_position: [0, 0, 0], base_orientation: Quaternion.identity, use_fixed_base: false, global_scaling: 1.0)
      path = Data.find(filename)
      document = REXML::Document.new(File.read(path))
      links = REXML::XPath.match(document, "/robot/link")
      joints = REXML::XPath.match(document, "/robot/joint")
      raise ArgumentError, "URDF must contain at least one link: #{filename}" if links.empty?
      raise NotImplementedError, "multi-link URDF loading is not implemented yet" if links.length > 1 || joints.any?

      link = links.first
      mass = use_fixed_base ? 0.0 : urdf_link_mass(link)
      shape = urdf_collision_shape(link, Float(global_scaling))
      shape_id = register_shape(shape)
      body_id = create_rigid_body(
        mass: mass,
        collision_shape: shape_id,
        position: base_position,
        orientation: base_orientation
      )
      apply_urdf_contact(link, body_for(body_id))
      body_id
    end

    def get_base_position_and_orientation(body_id)
      transform = body_for(body_id).world_transform
      [transform.origin.to_a, transform.rotation.to_a]
    end

    def set_base_position_and_orientation(body_id, position, orientation = Quaternion.identity)
      body = body_for(body_id)
      body.world_transform = Transform.new(Quaternion.coerce(orientation), Vector3.coerce(position))
      nil
    end

    def get_aabb(body_id)
      body = body_for(body_id)
      body.collision_shape.aabb(body.world_transform).map(&:to_a)
    end

    def ray_test(from, to)
      world.ray_test(from, to)
    end

    def ray_test_all(from, to)
      world.ray_test_all(from, to)
    end

    def ray_test_batch(rays)
      rays.map do |ray|
        from, to = ray
        ray_test(from, to)
      end
    end

    def get_contact_points(body_a: nil, body_b: nil)
      contacts = if body_a && body_b
                   world.contact_pair_test(body_for(body_a), body_for(body_b))
                 elsif body_a
                   world.contact_test(body_for(body_a))
                 elsif body_b
                   world.contact_test(body_for(body_b))
                 else
                   manifold_contacts
                 end

      contacts.map { |contact| contact_with_body_ids(contact) }
    end

    def change_dynamics(body_id, lateral_friction: nil, restitution: nil)
      body = body_for(body_id)
      body.friction = Float(lateral_friction) unless lateral_friction.nil?
      body.restitution = Float(restitution) unless restitution.nil?
      nil
    end

    def reset_simulation
      @constraints.compact.each { |constraint| world.remove_constraint(constraint) }
      @bodies.compact.each { |body| world.remove_rigid_body(body) }
      @world = DiscreteDynamicsWorld.create
      @bodies.clear
      @shapes.clear
      @constraints.clear
      nil
    end

    def body(body_id)
      body_for(body_id)
    end

    def collision_shape(shape_id)
      shape_for(shape_id)
    end

    def constraint(constraint_id)
      constraint_for(constraint_id)
    end

    def remove_body(body_id)
      body = body_for(body_id)
      world.remove_rigid_body(body)
      @bodies[Integer(body_id)] = nil
      nil
    end

    def remove_constraint(constraint_id)
      constraint = constraint_for(constraint_id)
      world.remove_constraint(constraint)
      @constraints[Integer(constraint_id)] = nil
      nil
    end

    def disconnect
      @constraints.compact.each { |constraint| world.remove_constraint(constraint) }
      @bodies.compact.each { |body| world.remove_rigid_body(body) }
      @constraints.clear
      @bodies.clear
      @shapes.clear
      nil
    end

    private

    def body_for(body_id)
      @bodies.fetch(Integer(body_id)) { raise ArgumentError, "unknown body id: #{body_id}" } ||
        raise(ArgumentError, "unknown body id: #{body_id}")
    end

    def body_id_for(body)
      return nil unless body

      @bodies.index(body)
    end

    def shape_for(shape_id)
      @shapes.fetch(Integer(shape_id)) { raise ArgumentError, "unknown collision shape id: #{shape_id}" } ||
        raise(ArgumentError, "unknown collision shape id: #{shape_id}")
    end

    def constraint_for(constraint_id)
      @constraints.fetch(Integer(constraint_id)) { raise ArgumentError, "unknown constraint id: #{constraint_id}" } ||
        raise(ArgumentError, "unknown constraint id: #{constraint_id}")
    end

    def register_shape(shape)
      @shapes << shape
      @shapes.length - 1
    end

    def vector_argument(x, y, z)
      return Vector3.coerce(x) if y.nil? && z.nil?

      Vector3.new(x, y, z)
    end

    def build_constraint(type, options)
      body_a = body_for(options.fetch(:body_a))
      body_b = options.key?(:body_b) && !options[:body_b].nil? ? body_for(options[:body_b]) : nil

      case type
      when :point2point, :p2p
        build_point2point_constraint(body_a, body_b, options)
      when :hinge
        build_hinge_constraint(body_a, body_b, options)
      when :fixed
        require_body_b!(body_b, type)
        Constraints::FixedConstraint.new(body_a, body_b, transform_option(options, :frame_in_a), transform_option(options, :frame_in_b))
      when :slider
        build_frame_constraint(Constraints::SliderConstraint, body_a, body_b, options)
      when :cone_twist
        build_frame_constraint(Constraints::ConeTwistConstraint, body_a, body_b, options)
      when :generic_6dof
        build_frame_constraint(Constraints::Generic6DofConstraint, body_a, body_b, options)
      when :generic_6dof_spring2
        build_frame_constraint(Constraints::Generic6DofSpring2Constraint, body_a, body_b, options, options.fetch(:rotate_order, 0))
      when :gear
        require_body_b!(body_b, type)
        Constraints::GearConstraint.new(
          body_a,
          body_b,
          options.fetch(:axis_in_a, [0, 1, 0]),
          options.fetch(:axis_in_b, [0, 1, 0]),
          Float(options.fetch(:ratio, 1.0))
        )
      when :hinge2
        require_body_b!(body_b, type)
        Constraints::Hinge2Constraint.new(
          body_a,
          body_b,
          options.fetch(:anchor, [0, 0, 0]),
          options.fetch(:axis_in_a, [0, 1, 0]),
          options.fetch(:axis_in_b, [1, 0, 0])
        )
      else
        raise ArgumentError, "unsupported constraint type: #{type}"
      end
    end

    def build_point2point_constraint(body_a, body_b, options)
      pivot_a = options.fetch(:pivot_in_a, [0, 0, 0])
      return Constraints::Point2PointConstraint.new(body_a, pivot_a) unless body_b

      Constraints::Point2PointConstraint.new(body_a, body_b, pivot_a, options.fetch(:pivot_in_b, [0, 0, 0]))
    end

    def build_hinge_constraint(body_a, body_b, options)
      pivot_a = options.fetch(:pivot_in_a, [0, 0, 0])
      axis_a = options.fetch(:axis_in_a, [0, 1, 0])
      use_reference_frame_a = options.fetch(:use_reference_frame_a, false)
      return Constraints::HingeConstraint.new(body_a, pivot_a, axis_a, use_reference_frame_a) unless body_b

      Constraints::HingeConstraint.new(
        body_a,
        body_b,
        pivot_a,
        options.fetch(:pivot_in_b, [0, 0, 0]),
        axis_a,
        options.fetch(:axis_in_b, [0, 1, 0]),
        use_reference_frame_a
      )
    end

    def build_frame_constraint(klass, body_a, body_b, options, *extra_args)
      frame_a = transform_option(options, :frame_in_a)
      return klass.new(body_a, frame_a, *extra_args) unless body_b

      klass.new(body_a, body_b, frame_a, transform_option(options, :frame_in_b), *extra_args)
    end

    def transform_option(options, key)
      value = options.fetch(key, Transform.identity)
      return value if value.is_a?(Transform)

      Transform.new(Quaternion.coerce(value.fetch(:orientation, Quaternion.identity)), Vector3.coerce(value.fetch(:position, [0, 0, 0])))
    end

    def require_body_b!(body_b, type)
      raise ArgumentError, "#{type} constraint requires body_b" unless body_b
    end

    def manifold_contacts
      world.contact_manifolds.flat_map do |manifold|
        manifold[:points].map do |point|
          point.merge(body0: manifold[:body0], body1: manifold[:body1])
        end
      end
    end

    def contact_with_body_ids(contact)
      contact.merge(
        body0: body_id_for(contact[:body0]),
        body1: body_id_for(contact[:body1])
      )
    end

    def urdf_link_mass(link)
      mass_element = REXML::XPath.first(link, "inertial/mass")
      return 0.0 unless mass_element

      Float(mass_element.attributes["value"])
    end

    def urdf_collision_shape(link, global_scaling)
      collisions = REXML::XPath.match(link, "collision")
      raise ArgumentError, "URDF link has no collision geometry" if collisions.empty?

      shapes_with_origins = collisions.map do |collision|
        geometry = REXML::XPath.first(collision, "geometry")
        raise ArgumentError, "URDF collision has no geometry" unless geometry

        [urdf_geometry_shape(geometry, global_scaling), urdf_origin_transform(REXML::XPath.first(collision, "origin"), global_scaling)]
      end

      return shapes_with_origins.first.first if shapes_with_origins.length == 1 && identity_transform?(shapes_with_origins.first.last)

      compound = Shapes::CompoundShape.new
      shapes_with_origins.each do |shape, transform|
        compound.add_child_shape(transform, shape)
      end
      compound
    end

    def urdf_geometry_shape(geometry, global_scaling)
      if (box = REXML::XPath.first(geometry, "box"))
        size = float_list(box.attributes["size"]).map { |value| value * global_scaling * 0.5 }
        return Shapes::BoxShape.new(Vector3.coerce(size))
      end

      if (sphere = REXML::XPath.first(geometry, "sphere"))
        return Shapes::SphereShape.new(Float(sphere.attributes["radius"]) * global_scaling)
      end

      if (cylinder = REXML::XPath.first(geometry, "cylinder"))
        radius = Float(cylinder.attributes["radius"]) * global_scaling
        length = Float(cylinder.attributes["length"]) * global_scaling
        return Shapes::CylinderShape.new(Vector3.new(radius, length * 0.5, radius))
      end

      if (capsule = REXML::XPath.first(geometry, "capsule"))
        return Shapes::CapsuleShape.new(
          Float(capsule.attributes["radius"]) * global_scaling,
          Float(capsule.attributes["length"]) * global_scaling
        )
      end

      mesh = REXML::XPath.first(geometry, "mesh")
      raise NotImplementedError, "URDF mesh collision geometry is not implemented yet: #{mesh.attributes["filename"]}" if mesh

      raise ArgumentError, "unsupported URDF collision geometry"
    end

    def urdf_origin_transform(origin, global_scaling)
      return Transform.identity unless origin

      xyz = float_list(origin.attributes["xyz"] || "0 0 0").map { |value| value * global_scaling }
      rpy = float_list(origin.attributes["rpy"] || "0 0 0")
      Transform.new(Quaternion.from_euler(*rpy), Vector3.coerce(xyz))
    end

    def identity_transform?(transform)
      transform.origin == Vector3.zero && transform.rotation == Quaternion.identity
    end

    def apply_urdf_contact(link, body)
      lateral_friction = REXML::XPath.first(link, "contact/lateral_friction")
      body.friction = Float(lateral_friction.attributes["value"]) if lateral_friction

      restitution = REXML::XPath.first(link, "contact/restitution")
      body.restitution = Float(restitution.attributes["value"]) if restitution
    end

    def float_list(value)
      value.to_s.split.map { |part| Float(part) }
    end
  end
end
