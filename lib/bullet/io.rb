# frozen_string_literal: true

module Bullet
  module IO
    class Serializer
      def initialize(simulation = nil)
        @simulation = simulation
      end

      def serialize_world(simulation_or_world = nil)
        target = simulation_or_world || @simulation
        raise ArgumentError, "a simulation or dynamics world is required" unless target

        if native_serializer
          return native_serializer.serialize_world(extract_world(target))
        end

        raise Bullet::Error, "native extension is required for .bullet serialization"
      end

      def save_world(simulation_or_world, filename)
        if bullet_file?(filename)
          return save_bullet(simulation_or_world, filename)
        end

        simulation = extract_simulation(simulation_or_world)
        simulation.save_world(filename)
      end

      def load_world(simulation, filename)
        if bullet_file?(filename)
          return load_bullet(simulation, filename)
        end

        extract_simulation(simulation).load_world(filename)
      end

      def save_bullet(simulation_or_world, filename)
        raise Bullet::Error, "native extension is required for .bullet serialization" unless native_serializer

        native_serializer.save_world(extract_world(simulation_or_world), filename)
      end

      def load_bullet(simulation, filename)
        extract_simulation(simulation).load_bullet(filename)
      end

      private

      def native_serializer
        return nil unless IO.const_defined?(:DefaultSerializer, false)

        @native_serializer ||= DefaultSerializer.new
      end

      def extract_simulation(value)
        return value if value.is_a?(Simulation)

        raise ArgumentError, "expected Bullet::Simulation"
      end

      def extract_world(value)
        return value.world if value.is_a?(Simulation)
        return value if value.is_a?(DiscreteDynamicsWorld)

        raise ArgumentError, "expected Bullet::Simulation or Bullet::DiscreteDynamicsWorld"
      end

      def bullet_file?(filename)
        File.extname(filename.to_s) == ".bullet"
      end
    end

    class URDFImporter
      attr_reader :simulation

      def initialize(simulation = Simulation.new)
        @simulation = simulation
      end

      def load(filename, **options)
        simulation.load_urdf(filename, **options)
      end
    end

    class SDFImporter
      attr_reader :simulation

      def initialize(simulation = Simulation.new)
        @simulation = simulation
      end

      def load(filename, **options)
        simulation.load_sdf(filename, **options)
      end
    end

    class MJCFImporter
      attr_reader :simulation

      def initialize(simulation = Simulation.new)
        @simulation = simulation
      end

      def load(filename, **options)
        simulation.load_mjcf(filename, **options)
      end
    end
  end
end
