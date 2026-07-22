# frozen_string_literal: true

require "json"
require "rexml/document"

module Bullet3
  class Simulation
    attr_reader :world, :mode

    def initialize(mode: :direct)
      mode = mode.to_sym
      raise ArgumentError, "unsupported connection mode: #{mode}" unless %i[direct gui shared_memory].include?(mode)
      Bullet3.require_native_extension!("Bullet3::Simulation") unless Bullet3.native?

      @mode = mode
      @world = DiscreteDynamicsWorld.create
      @shapes = []
      @shape_specs = []
      @bodies = []
      @body_specs = []
      @constraints = []
      @constraint_specs = []
      @models = {}
      @importers = []
    end

    def set_gravity(x, y = nil, z = nil)
      vector = vector_argument(x, y, z)
      world.gravity = vector
    end

    def step_simulation(time_step: 1.0 / 240.0, max_sub_steps: 1, fixed_time_step: 1.0 / 240.0)
      world.step_simulation(time_step, max_sub_steps, fixed_time_step)
    end

    def create_collision_shape(type, **options)
      shape = build_collision_shape(type.to_sym, options)
      register_shape(shape, type: type.to_sym, options: serializable_options(options))
    end

    def create_rigid_body(mass:, collision_shape:, position: [0, 0, 0], orientation: Quaternion.identity, rgba_color: [220, 220, 220, 255])
      shape = shape_for(collision_shape)
      position_vector = Vector3.coerce(position)
      orientation_quaternion = Quaternion.coerce(orientation)
      transform = Transform.new(orientation_quaternion, position_vector)
      motion_state = MotionState.new(transform)
      info = RigidBodyConstructionInfo.new(Float(mass), motion_state, shape)
      body = RigidBody.new(info)
      world.add_rigid_body(body)

      @bodies << body
      @body_specs << {
        mass: Float(mass),
        collision_shape: Integer(collision_shape),
        position: position_vector.to_a,
        orientation: orientation_quaternion.to_a,
        rgba_color: rgba_color_argument(rgba_color)
      }
      @bodies.length - 1
    end

    def create_constraint(type, **options)
      constraint = build_constraint(type.to_sym, options)
      world.add_constraint(constraint, options.fetch(:disable_collisions_between_linked_bodies, false))

      @constraints << constraint
      @constraint_specs << { type: type.to_sym, options: serializable_options(options) }
      @constraints.length - 1
    end

    def load_urdf(filename, base_position: [0, 0, 0], base_orientation: Quaternion.identity, use_fixed_base: false, global_scaling: 1.0)
      path = Data.find(filename)
      document = REXML::Document.new(File.read(path))
      links = REXML::XPath.match(document, "/robot/link")
      joints = REXML::XPath.match(document, "/robot/joint")
      raise ArgumentError, "URDF must contain at least one link: #{filename}" if links.empty?
      return load_multilink_urdf(links, joints, base_position, base_orientation, use_fixed_base, Float(global_scaling), File.dirname(path)) if links.length > 1 || joints.any?

      link = links.first
      mass = use_fixed_base ? 0.0 : urdf_link_mass(link)
      shape = urdf_collision_shape(link, Float(global_scaling), File.dirname(path))
      shape_id = register_shape(shape, urdf_shape_spec(link, Float(global_scaling), File.dirname(path)))
      body_id = create_rigid_body(
        mass: mass,
        collision_shape: shape_id,
        position: base_position,
        orientation: base_orientation
      )
      apply_urdf_contact(link, body_for(body_id))
      body_id
    end

    def load_sdf(filename, base_position: [0, 0, 0], base_orientation: Quaternion.identity, global_scaling: 1.0)
      path = Data.find(filename)
      document = REXML::Document.new(File.read(path))
      model_elements = REXML::XPath.match(document, "//model")
      raise ArgumentError, "SDF must contain at least one model: #{filename}" if model_elements.empty?

      model_elements.flat_map do |model|
        load_sdf_model(model, base_position, base_orientation, Float(global_scaling), File.dirname(path))
      end
    end

    def load_mjcf(filename, base_position: [0, 0, 0], base_orientation: Quaternion.identity, global_scaling: 1.0)
      path = Data.find(filename)
      document = REXML::Document.new(File.read(path))
      body_elements = REXML::XPath.match(document, "/mujoco/worldbody/body")
      raise ArgumentError, "MJCF must contain at least one worldbody body: #{filename}" if body_elements.empty?

      base_transform = Transform.new(Quaternion.coerce(base_orientation), Vector3.coerce(base_position))
      body_elements.flat_map do |body|
        load_mjcf_body(body, base_transform, Float(global_scaling), File.dirname(path))
      end
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

    def get_num_joints(body_id)
      model_for(body_id)[:joints].length
    end

    def get_joint_info(body_id, joint_index)
      model_for(body_id)[:joints].fetch(Integer(joint_index)) { raise ArgumentError, "unknown joint index: #{joint_index}" }.dup
    end

    def get_joint_state(body_id, joint_index)
      joint = get_joint_info(body_id, joint_index)
      constraint = joint[:constraint_id] && constraint_for(joint[:constraint_id])
      position = joint_position(joint, constraint)
      {
        position: position,
        velocity: joint[:target_velocity] || 0.0,
        reaction_forces: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        applied_torque: joint[:applied_torque] || 0.0
      }
    end

    def set_joint_motor_control(body_id, joint_index, control_mode:, target_position: nil, target_velocity: 0.0, force: 0.0)
      model = model_for(body_id)
      joint = model[:joints].fetch(Integer(joint_index)) { raise ArgumentError, "unknown joint index: #{joint_index}" }
      constraint = joint[:constraint_id] && constraint_for(joint[:constraint_id])
      mode = control_mode.to_sym

      case mode
      when :velocity
        set_velocity_control(joint, constraint, Float(target_velocity), Float(force))
      when :position
        set_position_control(joint, constraint, Float(target_position || 0.0), Float(force))
      when :torque
        set_torque_control(joint, Float(force))
      else
        raise ArgumentError, "unsupported joint control mode: #{control_mode}"
      end

      nil
    end

    def get_camera_image(width, height, camera_eye_position: [0, 0, 5], camera_target_position: [0, 0, 0], camera_up_vector: [0, 1, 0], fov: 60.0, near: 0.01, far: 100.0, background_color: [0, 0, 0, 255], body_colors: {})
      width = Integer(width)
      height = Integer(height)
      raise ArgumentError, "width and height must be positive" unless width.positive? && height.positive?

      camera = camera_basis(camera_eye_position, camera_target_position, camera_up_vector, Float(fov), Float(far))
      background_color = rgba_color_argument(background_color)
      body_colors = body_colors.to_h { |body_id, color| [Integer(body_id), rgba_color_argument(color)] }
      rgb = []
      depth = []
      segmentation = []

      height.times do |row|
        width.times do |column|
          hit = camera_ray_hit(camera, column, row, width, height)
          append_camera_pixel(hit, rgb, depth, segmentation, background_color, body_colors)
        end
      end

      [width, height, rgb, depth, segmentation]
    end

    def debug_draw_world(drawer = DebugDraw.new, mode: nil)
      drawer.debug_mode = Integer(mode) if mode

      if world.respond_to?(:debug_drawer=) && world.respond_to?(:debug_draw_world)
        world.debug_drawer = drawer
        world.debug_draw_world
      else
        draw_world_fallback(drawer)
      end

      drawer
    end

    def save_world(filename)
      File.write(filename, JSON.pretty_generate(world_snapshot))
      filename
    end

    def save_bullet(filename)
      serializer = IO::Serializer.new(self)
      serializer.save_bullet(self, filename)
    end

    def load_world(filename)
      data = JSON.parse(File.read(filename), symbolize_names: true)
      reset_simulation

      data.fetch(:shapes).each { |shape| create_collision_shape(shape.fetch(:type), **symbolize_hash(shape.fetch(:options))) }
      data.fetch(:bodies).each do |body|
        next if body.nil?

        create_rigid_body(
          mass: body.fetch(:mass),
          collision_shape: body.fetch(:collision_shape),
          position: body.fetch(:position),
          orientation: body.fetch(:orientation),
          rgba_color: body.fetch(:rgba_color, [220, 220, 220, 255])
        )
      end
      data.fetch(:constraints, []).each do |constraint|
        next if constraint.nil?

        create_constraint(constraint.fetch(:type), **symbolize_hash(constraint.fetch(:options)))
      end

      nil
    end

    def load_bullet(filename)
      raise Bullet3::Error, "native extension is required for .bullet import" unless IO.const_defined?(:BulletWorldImporter, false)

      importer = IO::BulletWorldImporter.new(world)
      raise Bullet3::Error, "failed to load .bullet file: #{filename}" unless importer.load_file(filename)

      @importers << importer
      importer
    end

    def reset_simulation
      clear_importers
      @constraints.compact.each { |constraint| world.remove_constraint(constraint) }
      @bodies.compact.each { |body| world.remove_rigid_body(body) }
      @world = DiscreteDynamicsWorld.create
      @bodies.clear
      @body_specs.clear
      @shapes.clear
      @shape_specs.clear
      @constraints.clear
      @constraint_specs.clear
      @models.clear
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
      @body_specs[Integer(body_id)] = nil
      nil
    end

    def remove_constraint(constraint_id)
      constraint = constraint_for(constraint_id)
      world.remove_constraint(constraint)
      @constraints[Integer(constraint_id)] = nil
      @constraint_specs[Integer(constraint_id)] = nil
      nil
    end

    def disconnect
      clear_importers
      @constraints.compact.each { |constraint| world.remove_constraint(constraint) }
      @bodies.compact.each { |body| world.remove_rigid_body(body) }
      @constraints.clear
      @constraint_specs.clear
      @bodies.clear
      @body_specs.clear
      @shapes.clear
      @shape_specs.clear
      @models.clear
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

    def model_for(body_id)
      @models.fetch(Integer(body_id)) { raise ArgumentError, "body has no joints: #{body_id}" }
    end

    def register_shape(shape, spec = nil)
      @shapes << shape
      @shape_specs << spec
      @shapes.length - 1
    end

    def vector_argument(x, y, z)
      return Vector3.coerce(x) if y.nil? && z.nil?

      Vector3.new(x, y, z)
    end

    def rgba_color_argument(value)
      values = value.to_a
      raise ArgumentError, "RGBA color must contain four components" unless values.length == 4

      values.map { |component| Integer(component).clamp(0, 255) }
    end

    def build_collision_shape(type, options)
      case type
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
        Shapes::StaticPlaneShape.new(Vector3.coerce(options.fetch(:normal, [0, 1, 0])), Float(options.fetch(:offset, 0.0)))
      when :mesh, :triangle_mesh
        triangles = options[:triangles] || load_obj_triangles(Data.find(options.fetch(:filename)), vector_scale(options[:scale], 1.0))
        Shapes::TriangleMeshShape.new(triangles)
      else
        raise ArgumentError, "unsupported collision shape: #{type}"
      end
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

    def load_multilink_urdf(links, joints, base_position, base_orientation, use_fixed_base, global_scaling, base_path)
      links_by_name = links.to_h { |link| [link.attributes["name"], link] }
      root_name = urdf_root_link_name(links_by_name, joints)
      base_transform = Transform.new(Quaternion.coerce(base_orientation), Vector3.coerce(base_position))
      link_transforms = { root_name => base_transform }
      link_body_ids = {
        root_name => create_urdf_link_body(links_by_name.fetch(root_name), base_transform, use_fixed_base, true, global_scaling, base_path)
      }
      joint_infos = []
      pending_joints = joints.dup

      until pending_joints.empty?
        progressed = false
        pending_joints.delete_if do |joint|
          parent_name = urdf_joint_parent(joint)
          child_name = urdf_joint_child(joint)
          next false unless link_body_ids[parent_name]

          origin = urdf_origin_transform(REXML::XPath.first(joint, "origin"), global_scaling)
          child_transform = link_transforms.fetch(parent_name) * origin
          link_transforms[child_name] = child_transform
          link_body_ids[child_name] = create_urdf_link_body(links_by_name.fetch(child_name), child_transform, false, false, global_scaling, base_path)
          joint_infos << create_urdf_joint(joint, link_body_ids.fetch(parent_name), link_body_ids.fetch(child_name), origin)
          progressed = true
        end

        raise ArgumentError, "URDF joint graph is disconnected or cyclic" unless progressed
      end

      base_id = link_body_ids.fetch(root_name)
      @models[base_id] = {
        links: link_body_ids.keys,
        body_ids: link_body_ids.values,
        joints: joint_infos
      }
      base_id
    end

    def urdf_root_link_name(links_by_name, joints)
      child_names = joints.map { |joint| urdf_joint_child(joint) }
      (links_by_name.keys - child_names).first || links_by_name.keys.first
    end

    def urdf_joint_parent(joint)
      REXML::XPath.first(joint, "parent").attributes["link"]
    end

    def urdf_joint_child(joint)
      REXML::XPath.first(joint, "child").attributes["link"]
    end

    def create_urdf_link_body(link, transform, use_fixed_base, root_link, global_scaling, base_path)
      shape = urdf_link_collision_shape(link, global_scaling, base_path)
      shape_id = register_shape(shape, urdf_shape_spec(link, global_scaling, base_path))
      body_id = create_rigid_body(
        mass: use_fixed_base && root_link ? 0.0 : urdf_link_mass(link),
        collision_shape: shape_id,
        position: transform.origin.to_a,
        orientation: transform.rotation.to_a
      )
      apply_urdf_contact(link, body_for(body_id))
      body_id
    end

    def urdf_link_collision_shape(link, global_scaling, base_path)
      urdf_collision_shape(link, global_scaling, base_path)
    rescue ArgumentError => error
      raise unless error.message.include?("no collision geometry")

      Shapes::SphereShape.new(0.001 * global_scaling)
    end

    def create_urdf_joint(joint, parent_body_id, child_body_id, origin)
      type = joint.attributes["type"].to_s
      axis = urdf_joint_axis(joint)
      limits = urdf_joint_limits(joint)
      constraint_id = case type
                      when "fixed"
                        create_constraint(:fixed, body_a: parent_body_id, body_b: child_body_id, frame_in_a: origin, frame_in_b: Transform.identity)
                      when "revolute", "continuous"
                        id = create_constraint(
                          :hinge,
                          body_a: parent_body_id,
                          body_b: child_body_id,
                          pivot_in_a: origin.origin.to_a,
                          pivot_in_b: [0, 0, 0],
                          axis_in_a: axis,
                          axis_in_b: axis
                        )
                        constraint(id).set_limit(limits[:lower], limits[:upper]) if type == "revolute" && limits[:lower] && limits[:upper]
                        id
                      when "prismatic"
                        id = create_constraint(:slider, body_a: parent_body_id, body_b: child_body_id, frame_in_a: origin, frame_in_b: Transform.identity)
                        constraint(id).lower_linear_limit = limits[:lower] if limits[:lower]
                        constraint(id).upper_linear_limit = limits[:upper] if limits[:upper]
                        id
                      else
                        create_constraint(:fixed, body_a: parent_body_id, body_b: child_body_id, frame_in_a: origin, frame_in_b: Transform.identity)
                      end

      {
        name: joint.attributes["name"].to_s,
        type: type.empty? ? "fixed" : type,
        parent_body: parent_body_id,
        child_body: child_body_id,
        constraint_id: constraint_id,
        axis: axis,
        lower_limit: limits[:lower],
        upper_limit: limits[:upper],
        target_velocity: 0.0,
        applied_torque: 0.0
      }
    end

    def urdf_joint_axis(joint)
      axis = REXML::XPath.first(joint, "axis")
      axis ? float_list(axis.attributes["xyz"]) : [1.0, 0.0, 0.0]
    end

    def urdf_joint_limits(joint)
      limit = REXML::XPath.first(joint, "limit")
      return {} unless limit

      {
        lower: limit.attributes["lower"] && Float(limit.attributes["lower"]),
        upper: limit.attributes["upper"] && Float(limit.attributes["upper"]),
        effort: limit.attributes["effort"] && Float(limit.attributes["effort"]),
        velocity: limit.attributes["velocity"] && Float(limit.attributes["velocity"])
      }
    end

    def joint_position(joint, constraint)
      return 0.0 unless constraint

      case joint[:type]
      when "revolute", "continuous"
        constraint.angle
      when "prismatic"
        constraint.linear_position
      else
        0.0
      end
    end

    def set_velocity_control(joint, constraint, target_velocity, force)
      joint[:target_velocity] = target_velocity
      joint[:applied_torque] = force
      case joint[:type]
      when "revolute", "continuous"
        constraint&.enable_angular_motor(true, target_velocity, force)
      when "prismatic"
        constraint.powered_linear_motor = true if constraint
        constraint.target_linear_motor_velocity = target_velocity if constraint
        constraint.max_linear_motor_force = force if constraint
      end
    end

    def set_position_control(joint, constraint, target_position, force)
      joint[:target_velocity] = 0.0
      joint[:applied_torque] = force
      case joint[:type]
      when "revolute", "continuous"
        constraint&.set_limit(target_position, target_position)
        constraint&.enable_angular_motor(true, 0.0, force)
      when "prismatic"
        if constraint
          constraint.lower_linear_limit = target_position
          constraint.upper_linear_limit = target_position
          constraint.max_linear_motor_force = force
        end
      end
    end

    def set_torque_control(joint, force)
      joint[:applied_torque] = force
      axis = joint[:axis]
      body_for(joint.fetch(:child_body)).apply_torque(axis.map { |component| component * force })
    end

    def load_sdf_model(model, base_position, base_orientation, global_scaling, base_path)
      model_transform = Transform.new(Quaternion.coerce(base_orientation), Vector3.coerce(base_position)) *
        sdf_pose_transform(REXML::XPath.first(model, "pose"), global_scaling)

      REXML::XPath.match(model, "link").map do |link|
        link_transform = model_transform * sdf_pose_transform(REXML::XPath.first(link, "pose"), global_scaling)
        shape = sdf_link_collision_shape(link, global_scaling, base_path)
        shape_id = register_shape(shape)
        create_rigid_body(
          mass: sdf_link_mass(link),
          collision_shape: shape_id,
          position: link_transform.origin.to_a,
          orientation: link_transform.rotation.to_a
        )
      end
    end

    def sdf_link_mass(link)
      mass = REXML::XPath.first(link, "inertial/mass")
      mass ? Float(mass.text) : 0.0
    end

    def sdf_link_collision_shape(link, global_scaling, base_path)
      collisions = REXML::XPath.match(link, "collision")
      raise ArgumentError, "SDF link has no collision geometry" if collisions.empty?

      shapes = collisions.map do |collision|
        geometry = REXML::XPath.first(collision, "geometry")
        [sdf_geometry_shape(geometry, global_scaling, base_path), sdf_pose_transform(REXML::XPath.first(collision, "pose"), global_scaling)]
      end
      return shapes.first.first if shapes.length == 1 && identity_transform?(shapes.first.last)

      compound = Shapes::CompoundShape.new
      shapes.each { |shape, transform| compound.add_child_shape(transform, shape) }
      compound
    end

    def sdf_geometry_shape(geometry, global_scaling, base_path)
      if (box = REXML::XPath.first(geometry, "box"))
        half_extents = float_list(REXML::XPath.first(box, "size").text).map { |value| value * global_scaling * 0.5 }
        return Shapes::BoxShape.new(Vector3.coerce(half_extents))
      end
      if (sphere = REXML::XPath.first(geometry, "sphere"))
        return Shapes::SphereShape.new(Float(REXML::XPath.first(sphere, "radius").text) * global_scaling)
      end
      if (cylinder = REXML::XPath.first(geometry, "cylinder"))
        radius = Float(REXML::XPath.first(cylinder, "radius").text) * global_scaling
        length = Float(REXML::XPath.first(cylinder, "length").text) * global_scaling
        return Shapes::CylinderShape.new(Vector3.new(radius, length * 0.5, radius))
      end
      if (plane = REXML::XPath.first(geometry, "plane"))
        normal = REXML::XPath.first(plane, "normal")&.text || "0 0 1"
        return Shapes::StaticPlaneShape.new(Vector3.coerce(float_list(normal)), 0.0)
      end
      if (mesh = REXML::XPath.first(geometry, "mesh"))
        uri = REXML::XPath.first(mesh, "uri").text
        scale = vector_scale(REXML::XPath.first(mesh, "scale")&.text, global_scaling)
        return Shapes::TriangleMeshShape.new(load_obj_triangles(resolve_mesh_filename(uri, base_path), scale))
      end

      raise ArgumentError, "unsupported SDF collision geometry"
    end

    def sdf_pose_transform(pose, global_scaling)
      return Transform.identity unless pose

      values = float_list(pose.text)
      xyz = [values[0] || 0.0, values[1] || 0.0, values[2] || 0.0].map { |value| value * global_scaling }
      rpy = [values[3] || 0.0, values[4] || 0.0, values[5] || 0.0]
      Transform.new(Quaternion.from_euler(*rpy), Vector3.coerce(xyz))
    end

    def load_mjcf_body(body_element, parent_transform, global_scaling, base_path)
      body_transform = parent_transform * mjcf_body_transform(body_element, global_scaling)
      body_ids = []
      geoms = REXML::XPath.match(body_element, "geom")
      unless geoms.empty?
        shape = mjcf_geoms_shape(geoms, global_scaling, base_path)
        shape_id = register_shape(shape)
        body_ids << create_rigid_body(
          mass: mjcf_body_mass(body_element, geoms),
          collision_shape: shape_id,
          position: body_transform.origin.to_a,
          orientation: body_transform.rotation.to_a
        )
      end

      REXML::XPath.match(body_element, "body").each do |child|
        body_ids.concat(load_mjcf_body(child, body_transform, global_scaling, base_path))
      end
      body_ids
    end

    def mjcf_body_transform(body_element, global_scaling)
      position = float_list(body_element.attributes["pos"] || "0 0 0").map { |value| value * global_scaling }
      orientation = body_element.attributes["quat"] ? Quaternion.coerce(float_list(body_element.attributes["quat"])) : Quaternion.identity
      Transform.new(orientation, Vector3.coerce(position))
    end

    def mjcf_body_mass(body_element, geoms)
      Float(body_element.attributes["mass"] || geoms.lazy.map { |geom| geom.attributes["mass"] }.find(&:itself) || 1.0)
    end

    def mjcf_geoms_shape(geoms, global_scaling, base_path)
      shapes = geoms.map do |geom|
        [mjcf_geom_shape(geom, global_scaling, base_path), mjcf_geom_transform(geom, global_scaling)]
      end
      return shapes.first.first if shapes.length == 1 && identity_transform?(shapes.first.last)

      compound = Shapes::CompoundShape.new
      shapes.each { |shape, transform| compound.add_child_shape(transform, shape) }
      compound
    end

    def mjcf_geom_shape(geom, global_scaling, base_path)
      type = geom.attributes["type"] || "sphere"
      size = float_list(geom.attributes["size"] || "1")
      case type
      when "box"
        Shapes::BoxShape.new(Vector3.coerce(size.first(3).map { |value| value * global_scaling }))
      when "sphere"
        Shapes::SphereShape.new(Float(size.fetch(0)) * global_scaling)
      when "capsule"
        Shapes::CapsuleShape.new(Float(size.fetch(0)) * global_scaling, Float(size.fetch(1, size.fetch(0))) * global_scaling * 2.0)
      when "cylinder"
        radius = Float(size.fetch(0)) * global_scaling
        half_height = Float(size.fetch(1, size.fetch(0))) * global_scaling
        Shapes::CylinderShape.new(Vector3.new(radius, half_height, radius))
      when "mesh"
        filename = geom.attributes["file"] || geom.attributes["mesh"]
        Shapes::TriangleMeshShape.new(load_obj_triangles(resolve_mesh_filename(filename, base_path), [global_scaling, global_scaling, global_scaling]))
      else
        raise ArgumentError, "unsupported MJCF geom type: #{type}"
      end
    end

    def mjcf_geom_transform(geom, global_scaling)
      position = float_list(geom.attributes["pos"] || "0 0 0").map { |value| value * global_scaling }
      orientation = geom.attributes["quat"] ? Quaternion.coerce(float_list(geom.attributes["quat"])) : Quaternion.identity
      Transform.new(orientation, Vector3.coerce(position))
    end

    def camera_basis(eye, target, up, fov, far)
      eye = vector_array(eye)
      target = vector_array(target)
      forward = normalize_vector(vector_subtract(target, eye))
      right = normalize_vector(cross_vector(forward, vector_array(up)))
      camera_up = normalize_vector(cross_vector(right, forward))
      {
        eye: eye,
        forward: forward,
        right: right,
        up: camera_up,
        scale: Math.tan(fov * Math::PI / 360.0),
        far: far
      }
    end

    def camera_ray_hit(camera, column, row, width, height)
      aspect = width.to_f / height
      x = ((column + 0.5) / width * 2.0 - 1.0) * aspect * camera.fetch(:scale)
      y = (1.0 - (row + 0.5) / height * 2.0) * camera.fetch(:scale)
      direction = normalize_vector(vector_add(camera.fetch(:forward), vector_add(vector_scale_array(camera.fetch(:right), x), vector_scale_array(camera.fetch(:up), y))))
      ray_to = vector_add(camera.fetch(:eye), vector_scale_array(direction, camera.fetch(:far)))
      ray_test(camera.fetch(:eye), ray_to)
    end

    def append_camera_pixel(hit, rgb, depth, segmentation, background_color, body_colors)
      if hit
        body_id = body_id_for(hit[:body]) || -1
        rgb.concat(body_colors.fetch(body_id) { body_color(body_id) })
        depth << hit[:fraction]
        segmentation << body_id
      else
        rgb.concat(background_color)
        depth << 1.0
        segmentation << -1
      end
    end

    def body_color(body_id)
      return [220, 220, 220, 255] if body_id.negative?

      @body_specs.fetch(body_id, nil)&.fetch(:rgba_color, nil) || [220, 220, 220, 255]
    end

    def draw_world_fallback(drawer)
      debug_mode = drawer.respond_to?(:debug_mode) ? drawer.debug_mode : DebugDraw::DRAW_AABB
      draw_aabbs(drawer) unless (debug_mode & DebugDraw::DRAW_AABB).zero?
      draw_contacts(drawer) unless (debug_mode & DebugDraw::DRAW_CONTACT_POINTS).zero?
      drawer
    end

    def draw_aabbs(drawer)
      @bodies.each_index do |body_id|
        next unless @bodies[body_id]

        draw_aabb(drawer, get_aabb(body_id), [1.0, 0.0, 0.0])
      end
    end

    def draw_aabb(drawer, aabb, color)
      min, max = aabb
      corners = [
        [min[0], min[1], min[2]], [max[0], min[1], min[2]],
        [max[0], max[1], min[2]], [min[0], max[1], min[2]],
        [min[0], min[1], max[2]], [max[0], min[1], max[2]],
        [max[0], max[1], max[2]], [min[0], max[1], max[2]]
      ]
      [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]].each do |from, to|
        drawer.draw_line(corners[from], corners[to], color)
      end
    end

    def draw_contacts(drawer)
      get_contact_points.each do |contact|
        drawer.draw_contact_point(
          contact.fetch(:position_world_on_b),
          contact.fetch(:normal_world_on_b),
          distance: contact.fetch(:distance),
          life_time: 0,
          color: [1.0, 1.0, 0.0]
        )
      end
    end

    def clear_importers
      @importers.each(&:delete_all_data)
      @importers.clear
      nil
    end

    def world_snapshot
      {
        shapes: @shape_specs.map { |spec| spec || raise(ArgumentError, "world contains an unserializable shape") },
        bodies: @body_specs.each_with_index.map { |spec, index| spec && body_snapshot(index, spec) },
        constraints: @constraint_specs
      }
    end

    def body_snapshot(index, spec)
      transform = body_for(index).world_transform
      spec.merge(position: transform.origin.to_a, orientation: transform.rotation.to_a)
    end

    def serializable_options(value)
      case value
      when Hash
        value.to_h { |key, child| [key, serializable_options(child)] }
      when Array
        value.map { |child| serializable_options(child) }
      when Transform
        { position: value.origin.to_a, orientation: value.rotation.to_a }
      when Vector3, Quaternion
        value.to_a
      when Symbol
        value.to_s
      else
        value
      end
    end

    def symbolize_hash(value)
      case value
      when Hash
        value.to_h { |key, child| [key.to_sym, symbolize_hash(child)] }
      when Array
        value.map { |child| symbolize_hash(child) }
      else
        value
      end
    end

    def vector_array(value)
      Vector3.coerce(value).to_a
    end

    def vector_add(left, right)
      left.zip(right).map { |a, b| a + b }
    end

    def vector_subtract(left, right)
      left.zip(right).map { |a, b| a - b }
    end

    def vector_scale_array(vector, scalar)
      vector.map { |component| component * scalar }
    end

    def cross_vector(left, right)
      [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0]
      ]
    end

    def normalize_vector(vector)
      length = Math.sqrt(vector.sum { |component| component * component })
      raise ArgumentError, "cannot normalize zero-length vector" if length.zero?

      vector.map { |component| component / length }
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

    def urdf_shape_spec(link, global_scaling, base_path)
      collisions = REXML::XPath.match(link, "collision")
      return nil unless collisions.length == 1

      collision = collisions.first
      return nil unless identity_transform?(urdf_origin_transform(REXML::XPath.first(collision, "origin"), global_scaling))

      urdf_geometry_shape_spec(REXML::XPath.first(collision, "geometry"), global_scaling, base_path)
    end

    def urdf_geometry_shape_spec(geometry, global_scaling, base_path)
      if (box = REXML::XPath.first(geometry, "box"))
        return { type: :box, options: { half_extents: float_list(box.attributes["size"]).map { |value| value * global_scaling * 0.5 } } }
      end
      if (sphere = REXML::XPath.first(geometry, "sphere"))
        return { type: :sphere, options: { radius: Float(sphere.attributes["radius"]) * global_scaling } }
      end
      if (cylinder = REXML::XPath.first(geometry, "cylinder"))
        radius = Float(cylinder.attributes["radius"]) * global_scaling
        length = Float(cylinder.attributes["length"]) * global_scaling
        return { type: :cylinder, options: { half_extents: [radius, length * 0.5, radius] } }
      end
      if (capsule = REXML::XPath.first(geometry, "capsule"))
        return { type: :capsule, options: { radius: Float(capsule.attributes["radius"]) * global_scaling, height: Float(capsule.attributes["length"]) * global_scaling } }
      end
      if (mesh = REXML::XPath.first(geometry, "mesh"))
        return { type: :mesh, options: { filename: resolve_mesh_filename(mesh.attributes["filename"], base_path), scale: vector_scale(mesh.attributes["scale"], global_scaling) } }
      end

      nil
    end

    def urdf_collision_shape(link, global_scaling, base_path)
      collisions = REXML::XPath.match(link, "collision")
      raise ArgumentError, "URDF link has no collision geometry" if collisions.empty?

      shapes_with_origins = collisions.map do |collision|
        geometry = REXML::XPath.first(collision, "geometry")
        raise ArgumentError, "URDF collision has no geometry" unless geometry

        [urdf_geometry_shape(geometry, global_scaling, base_path), urdf_origin_transform(REXML::XPath.first(collision, "origin"), global_scaling)]
      end

      return shapes_with_origins.first.first if shapes_with_origins.length == 1 && identity_transform?(shapes_with_origins.first.last)

      compound = Shapes::CompoundShape.new
      shapes_with_origins.each do |shape, transform|
        compound.add_child_shape(transform, shape)
      end
      compound
    end

    def urdf_geometry_shape(geometry, global_scaling, base_path)
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
      if mesh
        scale = vector_scale(mesh.attributes["scale"], global_scaling)
        return Shapes::TriangleMeshShape.new(load_obj_triangles(resolve_mesh_filename(mesh.attributes["filename"], base_path), scale))
      end

      raise ArgumentError, "unsupported URDF collision geometry"
    end

    def resolve_mesh_filename(filename, base_path)
      normalized = filename.to_s.sub(%r{\Afile://}, "").sub(%r{\Apackage://}, "")
      candidate = File.expand_path(normalized, base_path)
      return candidate if File.exist?(candidate)

      Data.find(normalized)
    end

    def vector_scale(value, global_scaling)
      parts = value ? float_list(value) : [1.0, 1.0, 1.0]
      raise ArgumentError, "URDF mesh scale must have 3 components" unless parts.length == 3

      parts.map { |component| component * global_scaling }
    end

    def load_obj_triangles(path, scale)
      vertices = []
      triangles = []

      File.foreach(path) do |line|
        parts = line.strip.split
        next if parts.empty? || parts.first.start_with?("#")

        case parts.first
        when "v"
          vertices << scaled_vertex(parts, scale)
        when "f"
          indices = face_indices(parts.drop(1), vertices.length)
          (1...(indices.length - 1)).each do |index|
            triangles << [vertices[indices.first], vertices[indices[index]], vertices[indices[index + 1]]]
          end
        end
      end

      raise ArgumentError, "OBJ mesh has no triangles: #{path}" if triangles.empty?

      triangles
    end

    def scaled_vertex(parts, scale)
      raise ArgumentError, "OBJ vertex must have 3 components" if parts.length < 4

      [
        Float(parts[1]) * scale[0],
        Float(parts[2]) * scale[1],
        Float(parts[3]) * scale[2]
      ]
    end

    def face_indices(tokens, vertex_count)
      indices = tokens.map do |token|
        index = Integer(token.split("/").first)
        index.negative? ? vertex_count + index : index - 1
      end
      raise ArgumentError, "OBJ face must have at least 3 vertices" if indices.length < 3

      indices
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
