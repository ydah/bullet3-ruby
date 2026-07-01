#pragma once

#include <LinearMath/btTransform.h>
#include <rice/rice.hpp>

namespace bullet_ruby {
class Transform {
public:
  Transform();
  Transform(VALUE rotation, VALUE origin);
  explicit Transform(const btTransform& transform);

  const btTransform& value() const;
  btTransform& value();

  btVector3 origin() const;
  btVector3 set_origin(Rice::Object origin);
  btQuaternion rotation() const;
  btQuaternion set_rotation(Rice::Object rotation);
  btMatrix3x3 basis() const;
  Rice::Object multiply(Rice::Object other) const;
  Transform inverse() const;
  Transform inverse_times(Rice::Object other) const;
  Rice::Array to_array() const;

private:
  btTransform transform_;
};

btTransform coerce_transform(Rice::Object object);
Rice::Object wrap_transform(const btTransform& transform);
} // namespace bullet_ruby

void Init_Transform(Rice::Module rb_mBullet);
