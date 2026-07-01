#include "rb_quaternion.hpp"

#include <cmath>
#include <stdexcept>

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>
#include <rice/rice.hpp>

namespace {
constexpr btScalar kEpsilon = btScalar(1e-6);

template <typename Value_T>
btScalar scalar_from_object(Value_T object)
{
  return Rice::detail::From_Ruby<btScalar>().convert(object);
}

btVector3 vector_from_array(Rice::Object object)
{
  Rice::Array array(object);
  if (array.size() != 3) {
    throw std::invalid_argument("expected a 3-element Array");
  }

  return btVector3(
    scalar_from_object(array[0]),
    scalar_from_object(array[1]),
    scalar_from_object(array[2])
  );
}

btVector3 coerce_vector(Rice::Object object)
{
  if (rb_obj_is_kind_of(object.value(), rb_cArray)) {
    return vector_from_array(object);
  }

  return *Rice::detail::From_Ruby<btVector3*>().convert(object);
}

btQuaternion quaternion_from_array(Rice::Object object)
{
  Rice::Array array(object);
  if (array.size() != 4) {
    throw std::invalid_argument("expected a 4-element Array");
  }

  return btQuaternion(
    scalar_from_object(array[0]),
    scalar_from_object(array[1]),
    scalar_from_object(array[2]),
    scalar_from_object(array[3])
  );
}

btQuaternion coerce_quaternion(Rice::Object object)
{
  if (rb_obj_is_kind_of(object.value(), rb_cArray)) {
    return quaternion_from_array(object);
  }

  return *Rice::detail::From_Ruby<btQuaternion*>().convert(object);
}

Rice::Array quaternion_to_array(const btQuaternion& quaternion)
{
  Rice::Array array;
  array.push(quaternion.getX());
  array.push(quaternion.getY());
  array.push(quaternion.getZ());
  array.push(quaternion.getW());
  return array;
}
} // namespace

void Init_Quaternion(Rice::Module rb_mBullet)
{
  Rice::define_class_under<btQuaternion>(rb_mBullet, "Quaternion")
    .define_constructor(Rice::Constructor<btQuaternion, btScalar, btScalar, btScalar, btScalar>(),
      Rice::Arg("x") = 0.0,
      Rice::Arg("y") = 0.0,
      Rice::Arg("z") = 0.0,
      Rice::Arg("w") = 1.0)
    .define_singleton_method("identity", [](Rice::Object) { return btQuaternion(0, 0, 0, 1); })
    .define_singleton_method("from_axis_angle", [](Rice::Object, Rice::Object axis, btScalar angle) {
      return btQuaternion(coerce_vector(axis), angle);
    })
    .define_singleton_method("from_euler", [](Rice::Object, btScalar roll, btScalar pitch, btScalar yaw) {
      btQuaternion quaternion;
      quaternion.setEulerZYX(yaw, pitch, roll);
      return quaternion;
    })
    .define_method("x", &btQuaternion::getX)
    .define_method("y", &btQuaternion::getY)
    .define_method("z", &btQuaternion::getZ)
    .define_method("w", &btQuaternion::getW)
    .define_method("x=", &btQuaternion::setX)
    .define_method("y=", &btQuaternion::setY)
    .define_method("z=", &btQuaternion::setZ)
    .define_method("w=", &btQuaternion::setW)
    .define_method("*", [](const btQuaternion& self, Rice::Object other) {
      return self * coerce_quaternion(other);
    })
    .define_method("==", [](const btQuaternion& self, Rice::Object other) {
      btQuaternion quaternion = coerce_quaternion(other);
      return std::fabs(self.getX() - quaternion.getX()) <= kEpsilon &&
             std::fabs(self.getY() - quaternion.getY()) <= kEpsilon &&
             std::fabs(self.getZ() - quaternion.getZ()) <= kEpsilon &&
             std::fabs(self.getW() - quaternion.getW()) <= kEpsilon;
    })
    .define_method("inverse", &btQuaternion::inverse)
    .define_method("angle", &btQuaternion::getAngle)
    .define_method("axis", &btQuaternion::getAxis)
    .define_method("slerp", [](const btQuaternion& self, Rice::Object other, btScalar t) {
      return self.slerp(coerce_quaternion(other), t);
    })
    .define_method("to_euler", [](const btQuaternion& self) {
      btScalar yaw;
      btScalar pitch;
      btScalar roll;
      self.getEulerZYX(yaw, pitch, roll);

      Rice::Array array;
      array.push(roll);
      array.push(pitch);
      array.push(yaw);
      return array;
    })
    .define_method("to_matrix", [](const btQuaternion& self) { return btMatrix3x3(self); })
    .define_method("normalized", &btQuaternion::normalized)
    .define_method("to_a", &quaternion_to_array);
}
