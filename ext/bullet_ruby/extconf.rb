# frozen_string_literal: true

require "fileutils"
require "mkmf"
require "rbconfig"

extension_name = "bullet_ruby"
extension_suffix = RbConfig::CONFIG.fetch("DLEXT")
source_dir = __dir__
build_dir = File.expand_path("build", source_dir)
output_dir = File.expand_path("../../lib/bullet_ruby", source_dir)
staging_extension = File.expand_path("#{extension_name}.#{extension_suffix}", Dir.pwd)

cmake_args = [
  "-S", source_dir,
  "-B", build_dir,
  "-DCMAKE_BUILD_TYPE=#{ENV.fetch("BULLET_RUBY_BUILD_TYPE", "Release")}",
  "-DRUBY_EXECUTABLE=#{RbConfig.ruby}",
  "-DRUBY_EXTENSION_SUFFIX=.#{extension_suffix}"
]

if ENV["BULLET_RUBY_SKIP_NATIVE"] == "1"
  File.write("Makefile", <<~MAKEFILE)
    all:

    install:

    clean:

  MAKEFILE
  exit
end

abort "CMake is required to build bullet_ruby" unless find_executable("cmake")

FileUtils.mkdir_p(build_dir)
system("cmake", *cmake_args) || abort("cmake configure failed")
system(
  "cmake",
  "--build", build_dir,
  "--target", extension_name,
  "--config", ENV.fetch("BULLET_RUBY_BUILD_TYPE", "Release")
) || abort("cmake build failed")

FileUtils.mkdir_p(output_dir)
built_extension = File.join(build_dir, "#{extension_name}.#{extension_suffix}")
FileUtils.cp(built_extension, staging_extension)
FileUtils.cp(built_extension, File.join(output_dir, "#{extension_name}.#{extension_suffix}"))

File.write("Makefile", <<~MAKEFILE)
  all:

  install:

  clean:

MAKEFILE
