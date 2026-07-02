#pragma once

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <BulletCollision/BroadphaseCollision/btAxisSweep3.h>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <btBulletDynamicsCommon.h>
#include <rice/rice.hpp>

#include "../linear_math/rb_transform.hpp"

namespace bullet_ruby {
class CollisionConfiguration;
class CollisionDispatcher;
class DbvtBroadphase;

class AxisSweep3 {
public:
  AxisSweep3(VALUE world_aabb_min, VALUE world_aabb_max, int max_handles);

  btAxisSweep3* get();

private:
  std::unique_ptr<btAxisSweep3> broadphase_;
};

class CollisionObject {
public:
  CollisionObject();
  explicit CollisionObject(VALUE collision_shape);

  btCollisionObject* get();
  const btCollisionObject* get() const;
  Transform world_transform() const;
  Transform set_world_transform(Rice::Object transform);
  btCollisionShape* collision_shape() const;
  void set_collision_shape(VALUE collision_shape);
  btScalar friction() const;
  btScalar set_friction(btScalar friction);
  btScalar restitution() const;
  btScalar set_restitution(btScalar restitution);
  int collision_flags() const;
  int set_collision_flags(int flags);
  bool active() const;
  void activate();
  Rice::Array aabb() const;

private:
  std::unique_ptr<btCollisionObject> collision_object_;
};

class CollisionWorld {
public:
  CollisionWorld();
  CollisionWorld(CollisionDispatcher& dispatcher, DbvtBroadphase& broadphase, CollisionConfiguration& configuration);
  CollisionWorld(CollisionDispatcher& dispatcher, AxisSweep3& broadphase, CollisionConfiguration& configuration);
  ~CollisionWorld();

  btCollisionWorld* get();
  void add_collision_object(VALUE collision_object, int group, int mask);
  void remove_collision_object(VALUE collision_object);
  int num_collision_objects() const;
  void update_aabbs();
  void compute_overlapping_pairs();
  Rice::Object ray_test_closest(Rice::Object from, Rice::Object to) const;
  Rice::Array ray_test_all(Rice::Object from, Rice::Object to) const;
  Rice::Array contact_test(VALUE collision_object) const;
  Rice::Array contact_pair_test(VALUE collision_object_a, VALUE collision_object_b) const;
  Rice::Array closest_points(VALUE collision_object_a, VALUE collision_object_b, btScalar distance_threshold) const;
  void mark() const;

private:
  void build_owned_world();
  VALUE ruby_object_for(const btCollisionObject* collision_object) const;

  std::unique_ptr<btDefaultCollisionConfiguration> owned_configuration_;
  std::unique_ptr<btCollisionDispatcher> owned_dispatcher_;
  std::unique_ptr<btDbvtBroadphase> owned_broadphase_;
  std::unique_ptr<btCollisionWorld> world_;
  std::unordered_set<btCollisionObject*> collision_objects_;
  std::unordered_map<const btCollisionObject*, VALUE> collision_object_values_;
};
} // namespace bullet_ruby

void Init_Collision(Rice::Module rb_mBullet);
