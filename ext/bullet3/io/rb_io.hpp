#pragma once

#include <memory>
#include <string>

#include <rice/rice.hpp>
#include <rice/stl.hpp>

#include "../dynamics/rb_dynamics.hpp"

class btBulletWorldImporter;

namespace bullet3 {
class DefaultSerializer {
public:
  VALUE serialize_world(DiscreteDynamicsWorld& world) const;
  std::string save_world(DiscreteDynamicsWorld& world, const std::string& filename) const;
};

#ifdef BULLET3_WITH_WORLD_IMPORTER
class BulletWorldImporter {
public:
  explicit BulletWorldImporter(DiscreteDynamicsWorld& world);
  ~BulletWorldImporter();

  bool load_file(const std::string& filename);
  void delete_all_data();
  int verbose_mode() const;
  int set_verbose_mode(int verbose_mode);
  int num_collision_shapes() const;
  int num_rigid_bodies() const;
  int num_constraints() const;

private:
  std::unique_ptr<btBulletWorldImporter> importer_;
};
#endif
} // namespace bullet3

void Init_IO(Rice::Module rb_mBullet);
