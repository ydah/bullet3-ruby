#include "rb_collision_shape.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <stdexcept>
#include <string>
#include <vector>

#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <BulletCollision/CollisionShapes/btMultiSphereShape.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <btBulletDynamicsCommon.h>
#include <rice/rice.hpp>

#include "../../linear_math/rb_transform.hpp"

namespace {
Rice::Object array_item(const Rice::Array& array, long index)
{
  return Rice::Object(array[index].value());
}

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

  if (rb_respond_to(object.value(), rb_intern("to_a")) != 0) {
    return vector_from_array(object.call("to_a"));
  }

  return *Rice::detail::From_Ruby<btVector3*>().convert(object);
}

btCollisionShape* coerce_collision_shape(Rice::Object object)
{
  btCollisionShape* shape = Rice::detail::From_Ruby<btCollisionShape*>().convert(object);
  if (shape == nullptr) {
    throw std::invalid_argument("expected Bullet3::Shapes::CollisionShape");
  }
  return shape;
}

std::vector<btVector3> vector_list_from_value(VALUE value)
{
  Rice::Array array{Rice::Object(value)};
  std::vector<btVector3> vectors;
  vectors.reserve(array.size());

  for (long index = 0; index < array.size(); ++index) {
    vectors.push_back(coerce_vector(array_item(array, index)));
  }

  return vectors;
}

std::vector<btScalar> scalar_list_from_value(VALUE value)
{
  Rice::Array array{Rice::Object(value)};
  std::vector<btScalar> scalars;
  scalars.reserve(array.size());

  for (long index = 0; index < array.size(); ++index) {
    scalars.push_back(Rice::detail::From_Ruby<btScalar>().convert(array[index]));
  }

  return scalars;
}

std::vector<float> float_list_from_value(VALUE value)
{
  Rice::Array array{Rice::Object(value)};
  std::vector<float> floats;
  floats.reserve(array.size());

  for (long index = 0; index < array.size(); ++index) {
    floats.push_back(Rice::detail::From_Ruby<float>().convert(array[index]));
  }

  return floats;
}

std::array<btVector3, 3> triangle_from_object(Rice::Object object)
{
  Rice::Array array(object);
  if (array.size() != 3) {
    throw std::invalid_argument("expected a triangle with 3 vertices");
  }

  return {
    coerce_vector(array_item(array, 0)),
    coerce_vector(array_item(array, 1)),
    coerce_vector(array_item(array, 2))
  };
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
  if (name == "compound") {
    return Rice::Symbol("compound");
  }
  if (name == "convex") {
    return Rice::Symbol("convex_hull");
  }
  if (name == "bvhtrianglemesh") {
    return Rice::Symbol("triangle_mesh");
  }
  if (name == "convextrimesh") {
    return Rice::Symbol("convex_triangle_mesh");
  }
  if (name == "multisphere") {
    return Rice::Symbol("multi_sphere");
  }
  if (name == "terrain") {
    return Rice::Symbol("heightfield");
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
  result.push(btVector3(center), true);
  result.push(radius);
  return result;
}

Rice::Array shape_aabb(const btCollisionShape& shape, const btTransform& transform)
{
  btVector3 min(0, 0, 0);
  btVector3 max(0, 0, 0);
  shape.getAabb(transform, min, max);

  Rice::Array result;
  result.push(btVector3(min), true);
  result.push(btVector3(max), true);
  return result;
}

class RubyConvexHullShape : public btConvexHullShape {
public:
  RubyConvexHullShape() = default;

  explicit RubyConvexHullShape(VALUE points)
  {
    for (const btVector3& point : vector_list_from_value(points)) {
      addPoint(point, false);
    }
    recalcLocalAabb();
  }
};

class TriangleMeshStorage {
public:
  explicit TriangleMeshStorage(VALUE triangles)
    : mesh_(true, true),
      triangle_count_(0)
  {
    Rice::Array array{Rice::Object(triangles)};
    for (long index = 0; index < array.size(); ++index) {
      std::array<btVector3, 3> triangle = triangle_from_object(array_item(array, index));
      mesh_.addTriangle(triangle[0], triangle[1], triangle[2]);
      ++triangle_count_;
    }
  }

  int triangle_count() const
  {
    return triangle_count_;
  }

protected:
  btTriangleMesh mesh_;
  int triangle_count_;
};

class RubyTriangleMeshShape : private TriangleMeshStorage, public btBvhTriangleMeshShape {
public:
  explicit RubyTriangleMeshShape(VALUE triangles)
    : TriangleMeshStorage(triangles),
      btBvhTriangleMeshShape(&mesh_, true)
  {
  }

  int num_triangles() const
  {
    return triangle_count();
  }
};

class RubyConvexTriangleMeshShape : private TriangleMeshStorage, public btConvexTriangleMeshShape {
public:
  explicit RubyConvexTriangleMeshShape(VALUE triangles)
    : TriangleMeshStorage(triangles),
      btConvexTriangleMeshShape(&mesh_, true)
  {
  }

  int num_triangles() const
  {
    return triangle_count();
  }
};

class MultiSphereStorage {
public:
  MultiSphereStorage(VALUE positions, VALUE radii)
    : positions_(vector_list_from_value(positions)),
      radii_(scalar_list_from_value(radii))
  {
    if (positions_.empty()) {
      throw std::invalid_argument("expected at least one sphere");
    }
    if (positions_.size() != radii_.size()) {
      throw std::invalid_argument("positions and radii must have the same length");
    }
  }

protected:
  std::vector<btVector3> positions_;
  std::vector<btScalar> radii_;
};

class RubyMultiSphereShape : private MultiSphereStorage, public btMultiSphereShape {
public:
  RubyMultiSphereShape(VALUE positions, VALUE radii)
    : MultiSphereStorage(positions, radii),
      btMultiSphereShape(positions_.data(), radii_.data(), static_cast<int>(positions_.size()))
  {
  }
};

class HeightfieldStorage {
public:
  HeightfieldStorage(int width, int length, VALUE heights)
    : width_(width),
      length_(length),
      heights_(float_list_from_value(heights))
  {
    if (width_ <= 0 || length_ <= 0) {
      throw std::invalid_argument("heightfield dimensions must be positive");
    }
    if (heights_.size() != static_cast<std::size_t>(width_ * length_)) {
      throw std::invalid_argument("heightfield data size must equal width * length");
    }
  }

  float height_at(int x, int y) const
  {
    if (x < 0 || x >= width_ || y < 0 || y >= length_) {
      throw std::out_of_range("heightfield index outside bounds");
    }
    return heights_[static_cast<std::size_t>(y * width_ + x)];
  }

protected:
  int width_;
  int length_;
  std::vector<float> heights_;
};

class RubyHeightfieldShape : private HeightfieldStorage, public btHeightfieldTerrainShape {
public:
  RubyHeightfieldShape(int width,
                       int length,
                       VALUE heights,
                       btScalar min_height,
                       btScalar max_height,
                       int up_axis,
                       bool flip_quad_edges)
    : HeightfieldStorage(width, length, heights),
      btHeightfieldTerrainShape(width_, length_, heights_.data(), min_height, max_height, up_axis, flip_quad_edges)
  {
  }

  int width() const
  {
    return width_;
  }

  int length() const
  {
    return length_;
  }

  float height(int x, int y) const
  {
    return height_at(x, y);
  }

  btVector3 vertex(int x, int y) const
  {
    btVector3 result(0, 0, 0);
    getVertex(x, y, result);
    return result;
  }
};
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
    .define_method("bounding_sphere", &bounding_sphere)
    .define_method("aabb", [](const btCollisionShape& self) {
      return shape_aabb(self, btTransform::getIdentity());
    })
    .define_method("aabb", [](const btCollisionShape& self, VALUE transform) {
      return shape_aabb(self, bullet3::coerce_transform(Rice::Object(transform)));
    }, Rice::Arg("transform").setValue());

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

  Rice::define_class_under<btCompoundShape, btCollisionShape>(rb_mShapes, "CompoundShape")
    .define_constructor(Rice::Constructor<btCompoundShape, bool, int>(),
      Rice::Arg("enable_dynamic_aabb_tree") = true,
      Rice::Arg("initial_child_capacity") = 0)
    .define_method("add_child_shape", [](btCompoundShape& self, Rice::Object transform, Rice::Object shape) {
      self.addChildShape(bullet3::coerce_transform(transform), coerce_collision_shape(shape));
    }, Rice::Arg("transform"), Rice::Arg("shape").keepAlive())
    .define_method("remove_child_shape", [](btCompoundShape& self, Rice::Object shape) {
      self.removeChildShape(coerce_collision_shape(shape));
    })
    .define_method("remove_child_shape_by_index", &btCompoundShape::removeChildShapeByIndex)
    .define_method("num_child_shapes", &btCompoundShape::getNumChildShapes)
    .define_method("child_shape", [](btCompoundShape& self, int index) {
      return self.getChildShape(index);
    })
    .define_method("child_transform", [](const btCompoundShape& self, int index) {
      return bullet3::Transform(self.getChildTransform(index));
    })
    .define_method("update_child_transform", [](btCompoundShape& self, int index, Rice::Object transform, bool recalculate) {
      self.updateChildTransform(index, bullet3::coerce_transform(transform), recalculate);
    }, Rice::Arg("index"), Rice::Arg("transform"), Rice::Arg("recalculate") = true)
    .define_method("recalculate_local_aabb", &btCompoundShape::recalculateLocalAabb);

  Rice::define_class_under<RubyConvexHullShape, btCollisionShape>(rb_mShapes, "ConvexHullShape")
    .define_constructor(Rice::Constructor<RubyConvexHullShape>())
    .define_constructor(Rice::Constructor<RubyConvexHullShape, VALUE>(),
      Rice::Arg("points").setValue())
    .define_method("add_point", [](RubyConvexHullShape& self, Rice::Object point, bool recalculate) {
      self.addPoint(coerce_vector(point), recalculate);
    }, Rice::Arg("point"), Rice::Arg("recalculate") = true)
    .define_method("num_points", &RubyConvexHullShape::getNumPoints)
    .define_method("point", [](const RubyConvexHullShape& self, int index) {
      if (index < 0 || index >= self.getNumPoints()) {
        throw std::out_of_range("point index outside convex hull");
      }
      return self.getUnscaledPoints()[index];
    })
    .define_method("optimize_convex_hull", &RubyConvexHullShape::optimizeConvexHull);

  Rice::define_class_under<RubyTriangleMeshShape, btCollisionShape>(rb_mShapes, "TriangleMeshShape")
    .define_constructor(Rice::Constructor<RubyTriangleMeshShape, VALUE>(),
      Rice::Arg("triangles").setValue())
    .define_method("num_triangles", &RubyTriangleMeshShape::num_triangles)
    .define_method("uses_quantized_aabb_compression", &RubyTriangleMeshShape::usesQuantizedAabbCompression);

  Rice::define_class_under<RubyConvexTriangleMeshShape, btCollisionShape>(rb_mShapes, "ConvexTriangleMeshShape")
    .define_constructor(Rice::Constructor<RubyConvexTriangleMeshShape, VALUE>(),
      Rice::Arg("triangles").setValue())
    .define_method("num_triangles", &RubyConvexTriangleMeshShape::num_triangles);

  Rice::define_class_under<RubyMultiSphereShape, btCollisionShape>(rb_mShapes, "MultiSphereShape")
    .define_constructor(Rice::Constructor<RubyMultiSphereShape, VALUE, VALUE>(),
      Rice::Arg("positions").setValue(),
      Rice::Arg("radii").setValue())
    .define_method("sphere_count", &RubyMultiSphereShape::getSphereCount)
    .define_method("sphere_position", &RubyMultiSphereShape::getSpherePosition)
    .define_method("sphere_radius", &RubyMultiSphereShape::getSphereRadius);

  Rice::define_class_under<RubyHeightfieldShape, btCollisionShape>(rb_mShapes, "HeightfieldShape")
    .define_constructor(Rice::Constructor<RubyHeightfieldShape, int, int, VALUE, btScalar, btScalar, int, bool>(),
      Rice::Arg("width"),
      Rice::Arg("length"),
      Rice::Arg("heights").setValue(),
      Rice::Arg("min_height"),
      Rice::Arg("max_height"),
      Rice::Arg("up_axis") = 1,
      Rice::Arg("flip_quad_edges") = false)
    .define_method("width", &RubyHeightfieldShape::width)
    .define_method("length", &RubyHeightfieldShape::length)
    .define_method("height", &RubyHeightfieldShape::height)
    .define_method("up_axis", &RubyHeightfieldShape::getUpAxis)
    .define_method("vertex", &RubyHeightfieldShape::vertex)
    .define_method("use_diamond_subdivision=", [](RubyHeightfieldShape& self, bool enabled) {
      self.setUseDiamondSubdivision(enabled);
      return enabled;
    })
    .define_method("use_zigzag_subdivision=", [](RubyHeightfieldShape& self, bool enabled) {
      self.setUseZigzagSubdivision(enabled);
      return enabled;
    })
    .define_method("flip_triangle_winding=", [](RubyHeightfieldShape& self, bool enabled) {
      self.setFlipTriangleWinding(enabled);
      return enabled;
    });
}
