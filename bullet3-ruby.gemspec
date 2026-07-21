# frozen_string_literal: true

require_relative "lib/bullet3/version"

Gem::Specification.new do |spec|
  spec.name = "bullet3-ruby"
  spec.version = Bullet3::VERSION
  spec.authors = ["Yudai Takada"]
  spec.email = ["t.yudai92@gmail.com"]

  spec.summary = "Ruby bindings for the Bullet Physics SDK."
  spec.description = "Ruby bindings for bullet3 with a low-level API and a Ruby-friendly simulation layer."
  spec.homepage = "https://github.com/ydah/bullet3-ruby"
  spec.license = "MIT"
  spec.required_ruby_version = ">= 3.1.0"
  spec.metadata["homepage_uri"] = spec.homepage
  spec.metadata["source_code_uri"] = spec.homepage

  gemspec = File.basename(__FILE__)
  file_candidates = IO.popen(%w[git ls-files -z --cached --modified --others --exclude-standard], chdir: __dir__, err: IO::NULL) do |ls|
    ls.readlines("\x0", chomp: true)
  end
  package_paths = %w[LICENSE.txt README.md ext/ lib/ sig/]
  spec.files = file_candidates.uniq.select do |f|
    path = File.join(__dir__, f)
    File.file?(path) &&
      f != gemspec &&
      package_paths.any? { |package_path| f == package_path || f.start_with?(package_path) }
  end
  spec.bindir = "exe"
  spec.executables = spec.files.grep(%r{\Aexe/}) { |f| File.basename(f) }
  spec.require_paths = ["lib"]
  spec.extensions = ["ext/bullet3/extconf.rb"]

  spec.add_dependency "rice", "~> 4.0"
  spec.add_dependency "rexml", "~> 3.4"
end
