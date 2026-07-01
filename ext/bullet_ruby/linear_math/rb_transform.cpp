#include "rb_transform.hpp"

#include <stdexcept>

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>
#include <rice/rice.hpp>

#include "../util/type_conversions.hpp"

namespace bullet_ruby {
Transform::Transform()
{
  transform_.setIdentity();
}

Transform::Transform(VALUE rotation, VALUE origin)
{
  transform_.setIdentity();
  transform_.setRotation(coerce_quaternion(Rice::Object(rotation)));
  transform_.setOrigin(coerce_vector(Rice::Object(origin)));
}

Transform::Transform(const btTransform& transform)
  : transform_(transform)
{
}

const btTransform& Transform::value() const
{
  return transform_;
}

btTransform& Transform::value()
{
  return transform_;
}

btVector3 Transform::origin() const
{
  return transform_.getOrigin();
}

btVector3 Transform::set_origin(Rice::Object origin)
{
  btVector3 vector = coerce_vector(origin);
  transform_.setOrigin(vector);
  return vector;
}

btQuaternion Transform::rotation() const
{
  return transform_.getRotation();
}

btQuaternion Transform::set_rotation(Rice::Object rotation)
{
  btQuaternion quaternion = coerce_quaternion(rotation);
  transform_.setRotation(quaternion);
  return quaternion;
}

btMatrix3x3 Transform::basis() const
{
  return transform_.getBasis();
}

Rice::Object Transform::multiply(Rice::Object other) const
{
  if (Rice::Data_Type<Transform>::is_bound() &&
      rb_obj_is_kind_of(other.value(), Rice::Data_Type<Transform>::klass())) {
    Transform* transform = Rice::detail::From_Ruby<Transform*>().convert(other);
    return wrap_transform(transform_ * transform->value());
  }

  btVector3 vector = coerce_vector(other);
  return Rice::Object(Rice::detail::To_Ruby<btVector3>().convert(transform_ * vector));
}

Transform Transform::inverse() const
{
  return Transform(transform_.inverse());
}

Transform Transform::inverse_times(Rice::Object other) const
{
  return Transform(transform_.inverseTimes(coerce_transform(other)));
}

Rice::Array Transform::to_array() const
{
  Rice::Array array;
  array.push(vector_to_array(transform_.getOrigin()));
  array.push(quaternion_to_array(transform_.getRotation()));
  return array;
}

btTransform coerce_transform(Rice::Object object)
{
  if (object.is_nil()) {
    throw std::invalid_argument("expected Bullet::Transform");
  }

  if (Rice::Data_Type<Transform>::is_bound() &&
      rb_obj_is_kind_of(object.value(), Rice::Data_Type<Transform>::klass())) {
    return Rice::detail::From_Ruby<Transform*>().convert(object)->value();
  }

  if (object_responds_to(object, "origin") && object_responds_to(object, "rotation")) {
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(coerce_vector(object.call("origin")));
    transform.setRotation(coerce_quaternion(object.call("rotation")));
    return transform;
  }

  if (rb_obj_is_kind_of(object.value(), rb_cArray)) {
    Rice::Array array(object);
    if (array.size() != 2) {
      throw std::invalid_argument("expected [origin, rotation] transform array");
    }

    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(coerce_vector(Rice::Object(array[0].value())));
    transform.setRotation(coerce_quaternion(Rice::Object(array[1].value())));
    return transform;
  }

  throw std::invalid_argument("expected Bullet::Transform");
}

Rice::Object wrap_transform(const btTransform& transform)
{
  return Rice::Object(Rice::detail::To_Ruby<Transform>().convert(Transform(transform)));
}
} // namespace bullet_ruby

void Init_Transform(Rice::Module rb_mBullet)
{
  Rice::define_class_under<bullet_ruby::Transform>(rb_mBullet, "Transform")
    .define_constructor(Rice::Constructor<bullet_ruby::Transform>())
    .define_constructor(Rice::Constructor<bullet_ruby::Transform, VALUE, VALUE>(),
      Rice::Arg("rotation").setValue(),
      Rice::Arg("origin").setValue())
    .define_singleton_method("identity", [](Rice::Object) { return bullet_ruby::Transform(); })
    .define_method("origin", &bullet_ruby::Transform::origin)
    .define_method("origin=", &bullet_ruby::Transform::set_origin)
    .define_method("rotation", &bullet_ruby::Transform::rotation)
    .define_method("rotation=", &bullet_ruby::Transform::set_rotation)
    .define_method("basis", &bullet_ruby::Transform::basis)
    .define_method("*", &bullet_ruby::Transform::multiply)
    .define_method("inverse", &bullet_ruby::Transform::inverse)
    .define_method("inverse_times", &bullet_ruby::Transform::inverse_times)
    .define_method("to_a", &bullet_ruby::Transform::to_array);
}
