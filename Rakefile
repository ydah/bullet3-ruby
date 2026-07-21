# frozen_string_literal: true

require "bundler/gem_tasks"
require "rake/extensiontask"
require "rspec/core/rake_task"

Rake::ExtensionTask.new("bullet3") do |ext|
  ext.ext_dir = "ext/bullet3"
  ext.lib_dir = "lib/bullet3"
end

RSpec::Core::RakeTask.new(:spec)

task default: :spec
