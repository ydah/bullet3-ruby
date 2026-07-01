#include "rb_matrix3x3.hpp"

#include <stdexcept>

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>
#include <rice/rice.hpp>

namespace {
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

btQuaternion coerce_quaternion(Rice::Object object)
{
  if (rb_obj_is_kind_of(object.value(), rb_cArray)) {
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

  return *Rice::detail::From_Ruby<btQuaternion*>().convert(object);
}

Rice::Array matrix_to_array(const btMatrix3x3& matrix)
{
  Rice::Array rows;
  for (int row_index = 0; row_index < 3; ++row_index) {
    Rice::Array row;
    const btVector3& vector = matrix.getRow(row_index);
    row.push(vector.getX());
    row.push(vector.getY());
    row.push(vector.getZ());
    rows.push(row);
  }
  return rows;
}
} // namespace

void Init_Matrix3x3(Rice::Module rb_mBullet)
{
  Rice::define_class_under<btMatrix3x3>(rb_mBullet, "Matrix3x3")
    .define_constructor(Rice::Constructor<btMatrix3x3,
      btScalar, btScalar, btScalar,
      btScalar, btScalar, btScalar,
      btScalar, btScalar, btScalar>(),
      Rice::Arg("xx") = 1.0, Rice::Arg("xy") = 0.0, Rice::Arg("xz") = 0.0,
      Rice::Arg("yx") = 0.0, Rice::Arg("yy") = 1.0, Rice::Arg("yz") = 0.0,
      Rice::Arg("zx") = 0.0, Rice::Arg("zy") = 0.0, Rice::Arg("zz") = 1.0)
    .define_singleton_method("identity", [](Rice::Object) { return btMatrix3x3::getIdentity(); })
    .define_singleton_method("from_quaternion", [](Rice::Object, Rice::Object quaternion) {
      return btMatrix3x3(coerce_quaternion(quaternion));
    })
    .define_method("*", [](const btMatrix3x3& self, Rice::Object vector) {
      return self * coerce_vector(vector);
    })
    .define_method("transpose", &btMatrix3x3::transpose)
    .define_method("to_a", &matrix_to_array);
}
