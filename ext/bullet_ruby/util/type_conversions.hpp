#pragma once

#include <stdexcept>

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>
#include <rice/rice.hpp>

namespace bullet_ruby {
template <typename Value_T>
inline btScalar scalar_from_object(Value_T object)
{
  return Rice::detail::From_Ruby<btScalar>().convert(object);
}

inline bool object_responds_to(Rice::Object object, const char* method_name)
{
  return rb_respond_to(object.value(), rb_intern(method_name)) != 0;
}

inline Rice::Array array_from_object(Rice::Object object)
{
  if (rb_obj_is_kind_of(object.value(), rb_cArray)) {
    return Rice::Array(object);
  }

  if (object_responds_to(object, "to_a")) {
    return Rice::Array(object.call("to_a"));
  }

  throw std::invalid_argument("expected an Array-like object");
}

inline btVector3 vector_from_array(Rice::Object object)
{
  Rice::Array array = array_from_object(object);
  if (array.size() != 3) {
    throw std::invalid_argument("expected a 3-element Array");
  }

  return btVector3(
    scalar_from_object(array[0]),
    scalar_from_object(array[1]),
    scalar_from_object(array[2])
  );
}

inline btVector3 coerce_vector(Rice::Object object)
{
  if (object.is_nil()) {
    throw std::invalid_argument("expected Bullet::Vector3 or a 3-element Array");
  }

  if (rb_obj_is_kind_of(object.value(), rb_cArray) || object_responds_to(object, "to_a")) {
    return vector_from_array(object);
  }

  return *Rice::detail::From_Ruby<btVector3*>().convert(object);
}

inline btQuaternion quaternion_from_array(Rice::Object object)
{
  Rice::Array array = array_from_object(object);
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

inline btQuaternion coerce_quaternion(Rice::Object object)
{
  if (object.is_nil()) {
    throw std::invalid_argument("expected Bullet::Quaternion or a 4-element Array");
  }

  if (rb_obj_is_kind_of(object.value(), rb_cArray) || object_responds_to(object, "to_a")) {
    return quaternion_from_array(object);
  }

  return *Rice::detail::From_Ruby<btQuaternion*>().convert(object);
}

inline Rice::Array vector_to_array(const btVector3& vector)
{
  Rice::Array array;
  array.push(vector.getX());
  array.push(vector.getY());
  array.push(vector.getZ());
  return array;
}

inline Rice::Array quaternion_to_array(const btQuaternion& quaternion)
{
  Rice::Array array;
  array.push(quaternion.getX());
  array.push(quaternion.getY());
  array.push(quaternion.getZ());
  array.push(quaternion.getW());
  return array;
}
} // namespace bullet_ruby
