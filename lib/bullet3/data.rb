# frozen_string_literal: true

module Bullet3
  module Data
    DEFAULT_PATH = File.expand_path("../../data", __dir__)

    class << self
      def search_paths
        @search_paths ||= [DEFAULT_PATH]
      end

      def add_search_path(path)
        expanded = File.expand_path(path)
        search_paths << expanded unless search_paths.include?(expanded)
        expanded
      end

      def remove_search_path(path)
        search_paths.delete(File.expand_path(path))
      end

      def path(*parts)
        File.join(DEFAULT_PATH, *parts)
      end

      def find(filename)
        expanded = File.expand_path(filename)
        return expanded if File.exist?(expanded)

        search_paths.each do |search_path|
          candidate = File.expand_path(filename, search_path)
          return candidate if File.exist?(candidate)
        end

        raise Errno::ENOENT, filename
      end
    end
  end
end
