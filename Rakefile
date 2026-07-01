# frozen_string_literal: true

require "bundler/gem_tasks"
require "rake/extensiontask"
require "rspec/core/rake_task"

Rake::ExtensionTask.new("bullet_ruby") do |ext|
  ext.lib_dir = "lib/bullet_ruby"
end

RSpec::Core::RakeTask.new(:spec)

task default: :spec
