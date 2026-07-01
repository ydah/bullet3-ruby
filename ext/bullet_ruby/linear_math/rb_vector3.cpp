#include "rb_vector3.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

#include <LinearMath/btVector3.h>
#include <rice/rice.hpp>
#include <rice/stl.hpp>

namespace {
constexpr btScalar kEpsilon = btScalar(1e-6);

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

btScalar component_at(const btVector3& vector, int index)
{
  if (index < 0 || index > 2) {
    throw std::out_of_range("index outside vector");
  }

  return vector[index];
}

void set_component_at(btVector3& vector, int index, btScalar value)
{
  if (index < 0 || index > 2) {
    throw std::out_of_range("index outside vector");
  }

  vector[index] = value;
}

Rice::Array to_array(const btVector3& vector)
{
  Rice::Array array;
  array.push(vector.getX());
  array.push(vector.getY());
  array.push(vector.getZ());
  return array;
}
} // namespace

void Init_Vector3(Rice::Module rb_mBullet)
{
  Rice::define_class_under<btVector3>(rb_mBullet, "Vector3")
    .define_constructor(Rice::Constructor<btVector3, btScalar, btScalar, btScalar>(),
      Rice::Arg("x") = 0.0,
      Rice::Arg("y") = 0.0,
      Rice::Arg("z") = 0.0)
    .define_singleton_method("zero", [](Rice::Object) { return btVector3(0, 0, 0); })
    .define_method("x", &btVector3::getX)
    .define_method("y", &btVector3::getY)
    .define_method("z", &btVector3::getZ)
    .define_method("x=", &btVector3::setX)
    .define_method("y=", &btVector3::setY)
    .define_method("z=", &btVector3::setZ)
    .define_method("[]", &component_at)
    .define_method("[]=", &set_component_at)
    .define_method("+", [](const btVector3& self, Rice::Object other) { return self + coerce_vector(other); })
    .define_method("-", [](const btVector3& self, Rice::Object other) { return self - coerce_vector(other); })
    .define_method("-@", [](const btVector3& self) { return -self; })
    .define_method("*", [](const btVector3& self, btScalar scalar) { return self * scalar; })
    .define_method("/", [](const btVector3& self, btScalar scalar) {
      if (scalar == btScalar(0)) {
        throw std::domain_error("divided by 0");
      }
      return self / scalar;
    })
    .define_method("==", [](const btVector3& self, Rice::Object other) {
      return (self - coerce_vector(other)).length2() <= kEpsilon * kEpsilon;
    })
    .define_method("length", &btVector3::length)
    .define_method("length2", &btVector3::length2)
    .define_method("normalize", [](btVector3& self) -> btVector3& {
      self.normalize();
      return self;
    })
    .define_method("normalize!", [](btVector3& self) -> btVector3& {
      self.normalize();
      return self;
    })
    .define_method("normalized", &btVector3::normalized)
    .define_method("dot", [](const btVector3& self, Rice::Object other) { return self.dot(coerce_vector(other)); })
    .define_method("cross", [](const btVector3& self, Rice::Object other) { return self.cross(coerce_vector(other)); })
    .define_method("distance", [](const btVector3& self, Rice::Object other) { return self.distance(coerce_vector(other)); })
    .define_method("distance2", [](const btVector3& self, Rice::Object other) { return self.distance2(coerce_vector(other)); })
    .define_method("lerp", [](const btVector3& self, Rice::Object other, btScalar t) { return self.lerp(coerce_vector(other), t); })
    .define_method("angle", [](const btVector3& self, Rice::Object other) { return self.angle(coerce_vector(other)); })
    .define_method("absolute", &btVector3::absolute)
    .define_method("to_a", &to_array)
    .define_method("to_s", [](const btVector3& self) {
      std::ostringstream stream;
      stream << "(" << self.getX() << ", " << self.getY() << ", " << self.getZ() << ")";
      return stream.str();
    })
    .define_method("inspect", [](const btVector3& self) {
      std::ostringstream stream;
      stream << "#<Bullet::Vector3 (" << self.getX() << ", " << self.getY() << ", " << self.getZ() << ")>";
      return stream.str();
    });
}
