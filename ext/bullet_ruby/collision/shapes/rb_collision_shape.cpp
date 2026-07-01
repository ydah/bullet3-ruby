#include "rb_collision_shape.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>

#include <btBulletDynamicsCommon.h>
#include <rice/rice.hpp>

namespace {
btVector3 vector_from_array(Rice::Object object)
{
  Rice::Array array(object);
  if (array.size() != 3) {
    throw std::invalid_argument("expected a 3-element Array");
  }

  return btVector3(
    Rice::detail::From_Ruby<btScalar>().convert(array[0]),
    Rice::detail::From_Ruby<btScalar>().convert(array[1]),
    Rice::detail::From_Ruby<btScalar>().convert(array[2])
  );
}

btVector3 coerce_vector(Rice::Object object)
{
  if (rb_obj_is_kind_of(object.value(), rb_cArray)) {
    return vector_from_array(object);
  }

  return *Rice::detail::From_Ruby<btVector3*>().convert(object);
}

Rice::Symbol shape_type(const btCollisionShape& shape)
{
  std::string name(shape.getName());
  std::transform(name.begin(), name.end(), name.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });

  if (name == "box") {
    return Rice::Symbol("box");
  }
  if (name == "sphere") {
    return Rice::Symbol("sphere");
  }
  if (name == "capsule" || name == "capsuleshape") {
    return Rice::Symbol("capsule");
  }
  if (name.rfind("cylinder", 0) == 0) {
    return Rice::Symbol("cylinder");
  }
  if (name.rfind("cone", 0) == 0) {
    return Rice::Symbol("cone");
  }
  if (name == "staticplane") {
    return Rice::Symbol("static_plane");
  }

  return Rice::Symbol(name);
}

btVector3 calculate_local_inertia(const btCollisionShape& shape, btScalar mass)
{
  btVector3 inertia(0, 0, 0);
  shape.calculateLocalInertia(mass, inertia);
  return inertia;
}

Rice::Array bounding_sphere(const btCollisionShape& shape)
{
  btVector3 center(0, 0, 0);
  btScalar radius = 0;
  shape.getBoundingSphere(center, radius);

  Rice::Array result;
  result.push(center);
  result.push(radius);
  return result;
}
} // namespace

void Init_Shapes(Rice::Module rb_mBullet)
{
  Rice::Module rb_mShapes = Rice::define_module_under(rb_mBullet, "Shapes");

  Rice::define_class_under<btCollisionShape>(rb_mShapes, "CollisionShape")
    .define_method("shape_type", &shape_type)
    .define_method("local_scaling", [](const btCollisionShape& self) {
      return btVector3(self.getLocalScaling());
    })
    .define_method("local_scaling=", [](btCollisionShape& self, Rice::Object scaling) {
      btVector3 vector = coerce_vector(scaling);
      self.setLocalScaling(vector);
      return vector;
    })
    .define_method("margin", &btCollisionShape::getMargin)
    .define_method("margin=", [](btCollisionShape& self, btScalar margin) {
      self.setMargin(margin);
      return margin;
    })
    .define_method("calculate_local_inertia", &calculate_local_inertia)
    .define_method("bounding_sphere", &bounding_sphere);

  Rice::define_class_under<btBoxShape, btCollisionShape>(rb_mShapes, "BoxShape")
    .define_constructor(Rice::Constructor<btBoxShape, const btVector3&>());

  Rice::define_class_under<btSphereShape, btCollisionShape>(rb_mShapes, "SphereShape")
    .define_constructor(Rice::Constructor<btSphereShape, btScalar>());

  Rice::define_class_under<btCapsuleShape, btCollisionShape>(rb_mShapes, "CapsuleShape")
    .define_constructor(Rice::Constructor<btCapsuleShape, btScalar, btScalar>());

  Rice::define_class_under<btCylinderShape, btCollisionShape>(rb_mShapes, "CylinderShape")
    .define_constructor(Rice::Constructor<btCylinderShape, const btVector3&>());

  Rice::define_class_under<btConeShape, btCollisionShape>(rb_mShapes, "ConeShape")
    .define_constructor(Rice::Constructor<btConeShape, btScalar, btScalar>());

  Rice::define_class_under<btStaticPlaneShape, btCollisionShape>(rb_mShapes, "StaticPlaneShape")
    .define_constructor(Rice::Constructor<btStaticPlaneShape, const btVector3&, btScalar>());
}
