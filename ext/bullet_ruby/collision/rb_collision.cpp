#include "rb_collision.hpp"

#include <functional>
#include <stdexcept>
#include <utility>

#include "../dynamics/rb_dynamics.hpp"
#include "../util/type_conversions.hpp"

namespace {
btCollisionShape* collision_shape_from_value(VALUE value)
{
  btCollisionShape* shape = Rice::detail::From_Ruby<btCollisionShape*>().convert(value);
  if (shape == nullptr) {
    throw std::invalid_argument("expected Bullet::Shapes::CollisionShape");
  }
  return shape;
}

bullet_ruby::CollisionObject* collision_object_from_value(VALUE value)
{
  bullet_ruby::CollisionObject* object = Rice::detail::From_Ruby<bullet_ruby::CollisionObject*>().convert(value);
  if (object == nullptr) {
    throw std::invalid_argument("expected Bullet::CollisionObject");
  }
  return object;
}

void set_vector_hash_value(Rice::Hash& hash, const char* key, const btVector3& value)
{
  hash[Rice::Symbol(key)] = Rice::Object(Rice::detail::To_Ruby<btVector3>().convert(btVector3(value)));
}

Rice::Array aabb_array(const btVector3& aabb_min, const btVector3& aabb_max)
{
  Rice::Array array;
  array.push(Rice::Object(Rice::detail::To_Ruby<btVector3>().convert(btVector3(aabb_min))));
  array.push(Rice::Object(Rice::detail::To_Ruby<btVector3>().convert(btVector3(aabb_max))));
  return array;
}

class ContactCollector : public btCollisionWorld::ContactResultCallback {
public:
  explicit ContactCollector(std::function<VALUE(const btCollisionObject*)> ruby_object_for)
    : ruby_object_for_(std::move(ruby_object_for))
  {
  }

  btScalar addSingleResult(btManifoldPoint& contact_point,
                           const btCollisionObjectWrapper* object0,
                           int part_id0,
                           int index0,
                           const btCollisionObjectWrapper* object1,
                           int part_id1,
                           int index1) override
  {
    Rice::Hash contact;
    contact[Rice::Symbol("body0")] = Rice::Object(ruby_object_for_(object0->getCollisionObject()));
    contact[Rice::Symbol("body1")] = Rice::Object(ruby_object_for_(object1->getCollisionObject()));
    set_vector_hash_value(contact, "position_world_on_a", contact_point.getPositionWorldOnA());
    set_vector_hash_value(contact, "position_world_on_b", contact_point.getPositionWorldOnB());
    set_vector_hash_value(contact, "normal_world_on_b", contact_point.m_normalWorldOnB);
    contact[Rice::Symbol("distance")] = contact_point.getDistance();
    contact[Rice::Symbol("part_id0")] = part_id0;
    contact[Rice::Symbol("index0")] = index0;
    contact[Rice::Symbol("part_id1")] = part_id1;
    contact[Rice::Symbol("index1")] = index1;
    contacts.push(contact);
    return btScalar(0);
  }

  Rice::Array contacts;

private:
  std::function<VALUE(const btCollisionObject*)> ruby_object_for_;
};
} // namespace

namespace bullet_ruby {
AxisSweep3::AxisSweep3(VALUE world_aabb_min, VALUE world_aabb_max, int max_handles)
  : broadphase_(std::make_unique<btAxisSweep3>(
      coerce_vector(Rice::Object(world_aabb_min)),
      coerce_vector(Rice::Object(world_aabb_max)),
      static_cast<unsigned short>(max_handles)))
{
}

btAxisSweep3* AxisSweep3::get()
{
  return broadphase_.get();
}

CollisionObject::CollisionObject()
  : collision_object_(std::make_unique<btCollisionObject>())
{
}

CollisionObject::CollisionObject(VALUE collision_shape)
  : CollisionObject()
{
  set_collision_shape(collision_shape);
}

btCollisionObject* CollisionObject::get()
{
  return collision_object_.get();
}

const btCollisionObject* CollisionObject::get() const
{
  return collision_object_.get();
}

Transform CollisionObject::world_transform() const
{
  return Transform(collision_object_->getWorldTransform());
}

Transform CollisionObject::set_world_transform(Rice::Object transform)
{
  btTransform value = coerce_transform(transform);
  collision_object_->setWorldTransform(value);
  return Transform(value);
}

btCollisionShape* CollisionObject::collision_shape() const
{
  return collision_object_->getCollisionShape();
}

void CollisionObject::set_collision_shape(VALUE collision_shape)
{
  collision_object_->setCollisionShape(collision_shape_from_value(collision_shape));
}

btScalar CollisionObject::friction() const
{
  return collision_object_->getFriction();
}

btScalar CollisionObject::set_friction(btScalar friction)
{
  collision_object_->setFriction(friction);
  return friction;
}

btScalar CollisionObject::restitution() const
{
  return collision_object_->getRestitution();
}

btScalar CollisionObject::set_restitution(btScalar restitution)
{
  collision_object_->setRestitution(restitution);
  return restitution;
}

int CollisionObject::collision_flags() const
{
  return collision_object_->getCollisionFlags();
}

int CollisionObject::set_collision_flags(int flags)
{
  collision_object_->setCollisionFlags(flags);
  return flags;
}

bool CollisionObject::active() const
{
  return collision_object_->isActive();
}

void CollisionObject::activate()
{
  collision_object_->activate(true);
}

Rice::Array CollisionObject::aabb() const
{
  btCollisionShape* shape = collision_object_->getCollisionShape();
  if (shape == nullptr) {
    throw std::runtime_error("collision object has no shape");
  }

  btVector3 aabb_min;
  btVector3 aabb_max;
  shape->getAabb(collision_object_->getWorldTransform(), aabb_min, aabb_max);
  return aabb_array(aabb_min, aabb_max);
}

CollisionWorld::CollisionWorld()
{
  build_owned_world();
}

CollisionWorld::CollisionWorld(CollisionDispatcher& dispatcher, DbvtBroadphase& broadphase, CollisionConfiguration& configuration)
  : world_(std::make_unique<btCollisionWorld>(dispatcher.get(), broadphase.get(), configuration.get()))
{
}

CollisionWorld::CollisionWorld(CollisionDispatcher& dispatcher, AxisSweep3& broadphase, CollisionConfiguration& configuration)
  : world_(std::make_unique<btCollisionWorld>(dispatcher.get(), broadphase.get(), configuration.get()))
{
}

CollisionWorld::~CollisionWorld()
{
  collision_objects_.clear();
  collision_object_values_.clear();
}

void CollisionWorld::build_owned_world()
{
  owned_configuration_ = std::make_unique<btDefaultCollisionConfiguration>();
  owned_dispatcher_ = std::make_unique<btCollisionDispatcher>(owned_configuration_.get());
  owned_broadphase_ = std::make_unique<btDbvtBroadphase>();
  world_ = std::make_unique<btCollisionWorld>(owned_dispatcher_.get(), owned_broadphase_.get(), owned_configuration_.get());
}

btCollisionWorld* CollisionWorld::get()
{
  return world_.get();
}

void CollisionWorld::add_collision_object(VALUE collision_object, int group, int mask)
{
  CollisionObject* object = collision_object_from_value(collision_object);
  btCollisionObject* bullet_object = object->get();
  if (collision_objects_.insert(bullet_object).second) {
    world_->addCollisionObject(bullet_object, group, mask);
    collision_object_values_[bullet_object] = collision_object;
  }
}

void CollisionWorld::remove_collision_object(VALUE collision_object)
{
  CollisionObject* object = collision_object_from_value(collision_object);
  btCollisionObject* bullet_object = object->get();
  auto iterator = collision_objects_.find(bullet_object);
  if (iterator != collision_objects_.end()) {
    world_->removeCollisionObject(bullet_object);
    collision_objects_.erase(iterator);
  }
  collision_object_values_.erase(bullet_object);
}

int CollisionWorld::num_collision_objects() const
{
  return world_->getNumCollisionObjects();
}

void CollisionWorld::update_aabbs()
{
  world_->updateAabbs();
}

void CollisionWorld::compute_overlapping_pairs()
{
  world_->computeOverlappingPairs();
}

VALUE CollisionWorld::ruby_object_for(const btCollisionObject* collision_object) const
{
  auto iterator = collision_object_values_.find(collision_object);
  if (iterator == collision_object_values_.end()) {
    return Qnil;
  }
  return iterator->second;
}

Rice::Object CollisionWorld::ray_test_closest(Rice::Object from, Rice::Object to) const
{
  btVector3 ray_from = coerce_vector(from);
  btVector3 ray_to = coerce_vector(to);
  btCollisionWorld::ClosestRayResultCallback callback(ray_from, ray_to);
  world_->rayTest(ray_from, ray_to, callback);

  if (!callback.hasHit()) {
    return Rice::Object(Qnil);
  }

  Rice::Hash hit;
  hit[Rice::Symbol("body")] = Rice::Object(ruby_object_for(callback.m_collisionObject));
  hit[Rice::Symbol("fraction")] = callback.m_closestHitFraction;
  set_vector_hash_value(hit, "point", callback.m_hitPointWorld);
  set_vector_hash_value(hit, "normal", callback.m_hitNormalWorld);
  return hit;
}

Rice::Array CollisionWorld::ray_test_all(Rice::Object from, Rice::Object to) const
{
  btVector3 ray_from = coerce_vector(from);
  btVector3 ray_to = coerce_vector(to);
  btCollisionWorld::AllHitsRayResultCallback callback(ray_from, ray_to);
  world_->rayTest(ray_from, ray_to, callback);

  Rice::Array hits;
  for (int index = 0; index < callback.m_collisionObjects.size(); ++index) {
    Rice::Hash hit;
    hit[Rice::Symbol("body")] = Rice::Object(ruby_object_for(callback.m_collisionObjects[index]));
    hit[Rice::Symbol("fraction")] = callback.m_hitFractions[index];
    set_vector_hash_value(hit, "point", callback.m_hitPointWorld[index]);
    set_vector_hash_value(hit, "normal", callback.m_hitNormalWorld[index]);
    hits.push(hit);
  }
  return hits;
}

Rice::Array CollisionWorld::contact_test(VALUE collision_object) const
{
  CollisionObject* object = collision_object_from_value(collision_object);
  ContactCollector callback([this](const btCollisionObject* bullet_object) {
    return ruby_object_for(bullet_object);
  });
  world_->contactTest(object->get(), callback);
  return callback.contacts;
}

Rice::Array CollisionWorld::contact_pair_test(VALUE collision_object_a, VALUE collision_object_b) const
{
  CollisionObject* object_a = collision_object_from_value(collision_object_a);
  CollisionObject* object_b = collision_object_from_value(collision_object_b);
  ContactCollector callback([this](const btCollisionObject* bullet_object) {
    return ruby_object_for(bullet_object);
  });
  world_->contactPairTest(object_a->get(), object_b->get(), callback);
  return callback.contacts;
}

Rice::Array CollisionWorld::closest_points(VALUE collision_object_a, VALUE collision_object_b, btScalar distance_threshold) const
{
  CollisionObject* object_a = collision_object_from_value(collision_object_a);
  CollisionObject* object_b = collision_object_from_value(collision_object_b);
  ContactCollector callback([this](const btCollisionObject* bullet_object) {
    return ruby_object_for(bullet_object);
  });
  callback.m_closestDistanceThreshold = distance_threshold;
  world_->contactPairTest(object_a->get(), object_b->get(), callback);
  return callback.contacts;
}

void CollisionWorld::mark() const
{
  for (const auto& entry : collision_object_values_) {
    rb_gc_mark(entry.second);
  }
}
} // namespace bullet_ruby

namespace Rice {
template <>
void ruby_mark<bullet_ruby::CollisionWorld>(bullet_ruby::CollisionWorld* world)
{
  if (world != nullptr) {
    world->mark();
  }
}
} // namespace Rice

void Init_Collision(Rice::Module rb_mBullet)
{
  Rice::define_class_under<bullet_ruby::AxisSweep3>(rb_mBullet, "AxisSweep3")
    .define_constructor(Rice::Constructor<bullet_ruby::AxisSweep3, VALUE, VALUE, int>(),
      Rice::Arg("world_aabb_min").setValue(),
      Rice::Arg("world_aabb_max").setValue(),
      Rice::Arg("max_handles") = 16384);

  Rice::define_class_under<bullet_ruby::CollisionObject>(rb_mBullet, "CollisionObject")
    .define_constructor(Rice::Constructor<bullet_ruby::CollisionObject>())
    .define_constructor(Rice::Constructor<bullet_ruby::CollisionObject, VALUE>(),
      Rice::Arg("collision_shape").setValue().keepAlive())
    .define_method("world_transform", &bullet_ruby::CollisionObject::world_transform)
    .define_method("world_transform=", &bullet_ruby::CollisionObject::set_world_transform)
    .define_method("collision_shape", &bullet_ruby::CollisionObject::collision_shape)
    .define_method("collision_shape=", &bullet_ruby::CollisionObject::set_collision_shape,
      Rice::Arg("collision_shape").setValue().keepAlive())
    .define_method("friction", &bullet_ruby::CollisionObject::friction)
    .define_method("friction=", &bullet_ruby::CollisionObject::set_friction)
    .define_method("restitution", &bullet_ruby::CollisionObject::restitution)
    .define_method("restitution=", &bullet_ruby::CollisionObject::set_restitution)
    .define_method("collision_flags", &bullet_ruby::CollisionObject::collision_flags)
    .define_method("collision_flags=", &bullet_ruby::CollisionObject::set_collision_flags)
    .define_method("active?", &bullet_ruby::CollisionObject::active)
    .define_method("activate", &bullet_ruby::CollisionObject::activate)
    .define_method("aabb", &bullet_ruby::CollisionObject::aabb);

  Rice::define_class_under<bullet_ruby::CollisionWorld>(rb_mBullet, "CollisionWorld")
    .define_constructor(Rice::Constructor<bullet_ruby::CollisionWorld>())
    .define_constructor(Rice::Constructor<bullet_ruby::CollisionWorld,
        bullet_ruby::CollisionDispatcher&,
        bullet_ruby::DbvtBroadphase&,
        bullet_ruby::CollisionConfiguration&>(),
      Rice::Arg("dispatcher").keepAlive(),
      Rice::Arg("broadphase").keepAlive(),
      Rice::Arg("configuration").keepAlive())
    .define_constructor(Rice::Constructor<bullet_ruby::CollisionWorld,
        bullet_ruby::CollisionDispatcher&,
        bullet_ruby::AxisSweep3&,
        bullet_ruby::CollisionConfiguration&>(),
      Rice::Arg("dispatcher").keepAlive(),
      Rice::Arg("broadphase").keepAlive(),
      Rice::Arg("configuration").keepAlive())
    .define_singleton_method("create", [](Rice::Object) -> bullet_ruby::CollisionWorld* {
      return new bullet_ruby::CollisionWorld();
    }, Rice::Return().takeOwnership())
    .define_method("add_collision_object", &bullet_ruby::CollisionWorld::add_collision_object,
      Rice::Arg("collision_object").setValue().keepAlive(),
      Rice::Arg("group") = int(btBroadphaseProxy::DefaultFilter),
      Rice::Arg("mask") = int(btBroadphaseProxy::AllFilter))
    .define_method("remove_collision_object", &bullet_ruby::CollisionWorld::remove_collision_object,
      Rice::Arg("collision_object").setValue())
    .define_method("num_collision_objects", &bullet_ruby::CollisionWorld::num_collision_objects)
    .define_method("update_aabbs", &bullet_ruby::CollisionWorld::update_aabbs)
    .define_method("compute_overlapping_pairs", &bullet_ruby::CollisionWorld::compute_overlapping_pairs)
    .define_method("ray_test_closest", &bullet_ruby::CollisionWorld::ray_test_closest)
    .define_method("ray_test", &bullet_ruby::CollisionWorld::ray_test_closest)
    .define_method("ray_test_all", &bullet_ruby::CollisionWorld::ray_test_all)
    .define_method("contact_test", &bullet_ruby::CollisionWorld::contact_test,
      Rice::Arg("collision_object").setValue())
    .define_method("contact_pair_test", &bullet_ruby::CollisionWorld::contact_pair_test,
      Rice::Arg("collision_object_a").setValue(),
      Rice::Arg("collision_object_b").setValue())
    .define_method("closest_points", &bullet_ruby::CollisionWorld::closest_points,
      Rice::Arg("collision_object_a").setValue(),
      Rice::Arg("collision_object_b").setValue(),
      Rice::Arg("distance_threshold") = btScalar(0.0));
}
