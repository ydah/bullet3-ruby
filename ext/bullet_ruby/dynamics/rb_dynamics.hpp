#pragma once

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <btBulletDynamicsCommon.h>
#include <rice/rice.hpp>

#include "../linear_math/rb_transform.hpp"
#include "../util/rb_debug_draw.hpp"

namespace bullet_ruby {
class RaycastVehicle;

class CollisionConfiguration {
public:
  CollisionConfiguration();

  btDefaultCollisionConfiguration* get();

private:
  std::unique_ptr<btDefaultCollisionConfiguration> configuration_;
};

class CollisionDispatcher {
public:
  explicit CollisionDispatcher(CollisionConfiguration& configuration);

  btCollisionDispatcher* get();

private:
  std::unique_ptr<btCollisionDispatcher> dispatcher_;
};

class DbvtBroadphase {
public:
  DbvtBroadphase();

  btDbvtBroadphase* get();

private:
  std::unique_ptr<btDbvtBroadphase> broadphase_;
};

class SequentialImpulseConstraintSolver {
public:
  SequentialImpulseConstraintSolver();

  btSequentialImpulseConstraintSolver* get();

private:
  std::unique_ptr<btSequentialImpulseConstraintSolver> solver_;
};

class MotionState {
public:
  MotionState();
  explicit MotionState(VALUE start_transform);

  btMotionState* get();
  Transform world_transform() const;
  Transform set_world_transform(Rice::Object transform);

private:
  std::unique_ptr<btDefaultMotionState> motion_state_;
};

class RigidBodyConstructionInfo {
public:
  RigidBodyConstructionInfo(btScalar mass, MotionState& motion_state, VALUE collision_shape);
  RigidBodyConstructionInfo(btScalar mass, MotionState& motion_state, VALUE collision_shape, VALUE local_inertia);

  btRigidBody::btRigidBodyConstructionInfo& get();
  btScalar mass() const;
  btVector3 local_inertia() const;
  btScalar friction() const;
  btScalar set_friction(btScalar friction);
  btScalar restitution() const;
  btScalar set_restitution(btScalar restitution);

private:
  btVector3 local_inertia_;
  std::unique_ptr<btRigidBody::btRigidBodyConstructionInfo> info_;
};

class RigidBody {
public:
  explicit RigidBody(RigidBodyConstructionInfo& construction_info);

  btRigidBody* get();
  const btRigidBody* get() const;

  btScalar mass() const;
  btCollisionShape* collision_shape() const;
  Transform world_transform() const;
  Transform set_world_transform(Rice::Object transform);
  btVector3 linear_velocity() const;
  btVector3 set_linear_velocity(Rice::Object velocity);
  btVector3 angular_velocity() const;
  btVector3 set_angular_velocity(Rice::Object velocity);
  btVector3 total_force() const;
  btVector3 total_torque() const;
  btScalar friction() const;
  btScalar set_friction(btScalar friction);
  btScalar restitution() const;
  btScalar set_restitution(btScalar restitution);
  Rice::Array damping() const;
  Rice::Array set_damping(btScalar linear_damping, btScalar angular_damping);
  void apply_central_force(Rice::Object force);
  void apply_force(Rice::Object force, Rice::Object relative_position);
  void apply_torque(Rice::Object torque);
  void apply_central_impulse(Rice::Object impulse);
  void apply_impulse(Rice::Object impulse, Rice::Object relative_position);
  void clear_forces();
  void activate();
  bool active() const;
  bool static_object() const;
  bool kinematic_object() const;

private:
  std::unique_ptr<btRigidBody> rigid_body_;
};

class DiscreteDynamicsWorld {
public:
  DiscreteDynamicsWorld();
  DiscreteDynamicsWorld(CollisionDispatcher& dispatcher,
                        DbvtBroadphase& broadphase,
                        SequentialImpulseConstraintSolver& solver,
                        CollisionConfiguration& configuration);
  ~DiscreteDynamicsWorld();

  btDiscreteDynamicsWorld* get();
  btVector3 gravity() const;
  btVector3 set_gravity(Rice::Object gravity);
  void add_rigid_body(RigidBody& rigid_body);
  void add_rigid_body_object(VALUE rigid_body);
  void remove_rigid_body(RigidBody& rigid_body);
  void remove_rigid_body_object(VALUE rigid_body);
  void add_constraint_object(VALUE constraint, bool disable_collisions_between_linked_bodies);
  void remove_constraint_object(VALUE constraint);
  void add_vehicle_object(VALUE vehicle);
  void remove_vehicle_object(VALUE vehicle);
  int step_simulation(btScalar time_step, int max_sub_steps, btScalar fixed_time_step);
  int num_collision_objects() const;
  int num_constraints() const;
  void clear_forces();
  void synchronize_motion_states();
  Rice::Object ray_test_closest(Rice::Object from, Rice::Object to) const;
  Rice::Array ray_test_all(Rice::Object from, Rice::Object to) const;
  Rice::Array contact_test(VALUE rigid_body) const;
  Rice::Array contact_pair_test(VALUE rigid_body_a, VALUE rigid_body_b) const;
  Rice::Array closest_points(VALUE rigid_body_a, VALUE rigid_body_b, btScalar distance_threshold) const;
  Rice::Array contact_manifolds() const;
  void set_debug_drawer(VALUE debug_drawer);
  VALUE debug_drawer() const;
  void debug_draw_world();
  void mark() const;

private:
  void build_owned_world();
  VALUE ruby_object_for(const btCollisionObject* collision_object) const;

  std::unique_ptr<btDefaultCollisionConfiguration> owned_configuration_;
  std::unique_ptr<btCollisionDispatcher> owned_dispatcher_;
  std::unique_ptr<btDbvtBroadphase> owned_broadphase_;
  std::unique_ptr<btSequentialImpulseConstraintSolver> owned_solver_;
  std::unique_ptr<btDiscreteDynamicsWorld> world_;
  std::unordered_set<btRigidBody*> rigid_bodies_;
  std::unordered_set<btTypedConstraint*> constraints_;
  std::unordered_set<btActionInterface*> actions_;
  std::unordered_map<const btCollisionObject*, VALUE> collision_object_values_;
  std::unordered_map<const btTypedConstraint*, VALUE> constraint_values_;
  std::unordered_map<const btActionInterface*, VALUE> action_values_;
  VALUE debug_drawer_value_ = Qnil;
};
} // namespace bullet_ruby

void Init_Dynamics(Rice::Module rb_mBullet);
