#include "rb_io.hpp"

#include <fstream>
#include <stdexcept>

#include <LinearMath/btSerializer.h>

#ifdef BULLET3_WITH_WORLD_IMPORTER
#include <btBulletWorldImporter.h>
#endif

namespace bullet3 {
VALUE DefaultSerializer::serialize_world(DiscreteDynamicsWorld& world) const
{
  btDefaultSerializer serializer;
  world.get()->serialize(&serializer);
  return rb_str_new(
    reinterpret_cast<const char*>(serializer.getBufferPointer()),
    serializer.getCurrentBufferSize());
}

std::string DefaultSerializer::save_world(DiscreteDynamicsWorld& world, const std::string& filename) const
{
  VALUE serialized = serialize_world(world);
  std::ofstream output(filename, std::ios::binary);
  if (!output) {
    throw std::runtime_error("failed to open .bullet file for writing: " + filename);
  }

  output.write(RSTRING_PTR(serialized), RSTRING_LEN(serialized));
  if (!output) {
    throw std::runtime_error("failed to write .bullet file: " + filename);
  }

  return filename;
}

#ifdef BULLET3_WITH_WORLD_IMPORTER
BulletWorldImporter::BulletWorldImporter(DiscreteDynamicsWorld& world)
  : importer_(std::make_unique<btBulletWorldImporter>(world.get()))
{
}

BulletWorldImporter::~BulletWorldImporter() = default;

bool BulletWorldImporter::load_file(const std::string& filename)
{
  return importer_->loadFile(filename.c_str());
}

void BulletWorldImporter::delete_all_data()
{
  importer_->deleteAllData();
}

int BulletWorldImporter::verbose_mode() const
{
  return importer_->getVerboseMode();
}

int BulletWorldImporter::set_verbose_mode(int verbose_mode)
{
  importer_->setVerboseMode(verbose_mode);
  return verbose_mode;
}

int BulletWorldImporter::num_collision_shapes() const
{
  return importer_->getNumCollisionShapes();
}

int BulletWorldImporter::num_rigid_bodies() const
{
  return importer_->getNumRigidBodies();
}

int BulletWorldImporter::num_constraints() const
{
  return importer_->getNumConstraints();
}
#endif
} // namespace bullet3

void Init_IO(Rice::Module rb_mBullet)
{
  Rice::Module rb_mIO = Rice::define_module_under(rb_mBullet, "IO");

  Rice::define_class_under<bullet3::DefaultSerializer>(rb_mIO, "DefaultSerializer")
    .define_constructor(Rice::Constructor<bullet3::DefaultSerializer>())
    .define_method("serialize_world", &bullet3::DefaultSerializer::serialize_world,
      Rice::Arg("world").keepAlive())
    .define_method("save_world", &bullet3::DefaultSerializer::save_world,
      Rice::Arg("world").keepAlive(),
      Rice::Arg("filename"));

#ifdef BULLET3_WITH_WORLD_IMPORTER
  Rice::define_class_under<bullet3::BulletWorldImporter>(rb_mIO, "BulletWorldImporter")
    .define_constructor(Rice::Constructor<bullet3::BulletWorldImporter, bullet3::DiscreteDynamicsWorld&>(),
      Rice::Arg("world").keepAlive())
    .define_method("load_file", &bullet3::BulletWorldImporter::load_file)
    .define_method("delete_all_data", &bullet3::BulletWorldImporter::delete_all_data)
    .define_method("verbose_mode", &bullet3::BulletWorldImporter::verbose_mode)
    .define_method("verbose_mode=", &bullet3::BulletWorldImporter::set_verbose_mode)
    .define_method("num_collision_shapes", &bullet3::BulletWorldImporter::num_collision_shapes)
    .define_method("num_rigid_bodies", &bullet3::BulletWorldImporter::num_rigid_bodies)
    .define_method("num_constraints", &bullet3::BulletWorldImporter::num_constraints);
#endif
}
