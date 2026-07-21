#pragma once

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointMotor.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyPoint2Point.h>
#include <rice/rice.hpp>

#include "../dynamics/rb_dynamics.hpp"
#include "../linear_math/rb_transform.hpp"

namespace bullet3 {
class MultiBodyLinkCollider;

class MultiBody {
public:
  MultiBody(int link_count, btScalar base_mass, Rice::Object base_inertia, bool fixed_base, bool can_sleep, bool deprecated_multi_dof);

  btMultiBody* get();
  const btMultiBody* get() const;
  int num_links() const;
  int num_dofs() const;
  int num_pos_vars() const;
  btScalar base_mass() const;
  btScalar set_base_mass(btScalar mass);
  btVector3 base_inertia() const;
  btVector3 set_base_inertia(Rice::Object inertia);
  btVector3 base_position() const;
  btVector3 set_base_position(Rice::Object position);
  btVector3 base_velocity() const;
  btVector3 set_base_velocity(Rice::Object velocity);
  btVector3 base_omega() const;
  btVector3 set_base_omega(Rice::Object omega);
  btQuaternion world_to_base_rotation() const;
  btQuaternion set_world_to_base_rotation(Rice::Object rotation);
  Transform base_world_transform() const;
  Transform set_base_world_transform(Rice::Object transform);
  bool fixed_base() const;
  bool can_sleep() const;
  bool set_can_sleep(bool can_sleep);
  bool awake() const;
  void wake_up();
  void go_to_sleep();
  btScalar linear_damping() const;
  btScalar set_linear_damping(btScalar damping);
  btScalar angular_damping() const;
  btScalar set_angular_damping(btScalar damping);
  bool self_collision() const;
  bool set_self_collision(bool enabled);
  btScalar joint_position(int link) const;
  btScalar set_joint_position(int link, btScalar position);
  btScalar joint_velocity(int link) const;
  btScalar set_joint_velocity(int link, btScalar velocity);
  void setup_fixed(int link, btScalar mass, Rice::Object inertia, int parent, Rice::Object parent_to_this_rotation, Rice::Object parent_pivot, Rice::Object child_pivot, bool disable_parent_collision);
  void setup_revolute(int link, btScalar mass, Rice::Object inertia, int parent, Rice::Object parent_to_this_rotation, Rice::Object joint_axis, Rice::Object parent_pivot, Rice::Object child_pivot, bool disable_parent_collision);
  void setup_prismatic(int link, btScalar mass, Rice::Object inertia, int parent, Rice::Object parent_to_this_rotation, Rice::Object joint_axis, Rice::Object parent_pivot, Rice::Object child_pivot, bool disable_parent_collision);
  void finalize_multi_dof();
  void clear_forces_and_torques();
  void clear_velocities();
  void add_base_force(Rice::Object force);
  void add_base_torque(Rice::Object torque);
  void add_link_force(int link, Rice::Object force);
  void add_link_torque(int link, Rice::Object torque);
  void add_joint_torque(int link, btScalar torque);
  int parent(int link) const;
  void set_base_collider(MultiBodyLinkCollider& collider);
  void set_link_collider(int link, MultiBodyLinkCollider& collider);

private:
  int validate_link_index(int link) const;

  std::unique_ptr<btMultiBody> multi_body_;
};

class MultiBodyLinkCollider {
public:
  MultiBodyLinkCollider(MultiBody& multi_body, int link);

  btMultiBodyLinkCollider* get();
  const btMultiBodyLinkCollider* get() const;
  int link() const;
  Transform world_transform() const;
  Transform set_world_transform(Rice::Object transform);
  void set_collision_shape(VALUE collision_shape);
  btCollisionShape* collision_shape() const;
  int collision_flags() const;
  int set_collision_flags(int flags);
  bool kinematic() const;
  bool static_or_kinematic() const;

private:
  std::unique_ptr<btMultiBodyLinkCollider> collider_;
};

class MultiBodyConstraint {
public:
  virtual ~MultiBodyConstraint() = default;

  btMultiBodyConstraint* get();
  const btMultiBodyConstraint* get() const;
  Rice::Symbol constraint_type() const;
  int num_rows() const;
  btScalar max_applied_impulse() const;
  btScalar set_max_applied_impulse(btScalar impulse);
  btScalar applied_impulse(int row);
  btScalar position(int row) const;

protected:
  explicit MultiBodyConstraint(std::unique_ptr<btMultiBodyConstraint> constraint);

private:
  std::unique_ptr<btMultiBodyConstraint> constraint_;
};

class MultiBodyJointMotor : public MultiBodyConstraint {
public:
  MultiBodyJointMotor(VALUE multi_body, int link, btScalar desired_velocity, btScalar max_motor_impulse);
  MultiBodyJointMotor(VALUE multi_body, int link, int link_dof, btScalar desired_velocity, btScalar max_motor_impulse);

  void set_velocity_target(btScalar velocity, btScalar kd);
  void set_position_target(btScalar position, btScalar kp);
  btScalar erp() const;
  btScalar set_erp(btScalar erp);
  void set_rhs_clamp(btScalar clamp);
};

class MultiBodyJointLimitConstraint : public MultiBodyConstraint {
public:
  MultiBodyJointLimitConstraint(VALUE multi_body, int link, btScalar lower, btScalar upper);

  btScalar lower_bound() const;
  btScalar set_lower_bound(btScalar lower);
  btScalar upper_bound() const;
  btScalar set_upper_bound(btScalar upper);
};

class MultiBodyPoint2Point : public MultiBodyConstraint {
public:
  MultiBodyPoint2Point(VALUE multi_body, int link, VALUE rigid_body, VALUE pivot_in_a, VALUE pivot_in_b);
  MultiBodyPoint2Point(VALUE multi_body_a, int link_a, VALUE multi_body_b, int link_b, VALUE pivot_in_a, VALUE pivot_in_b);

  btVector3 pivot_in_b() const;
  btVector3 set_pivot_in_b(Rice::Object pivot);
};

class MultiBodyDynamicsWorld {
public:
  MultiBodyDynamicsWorld();
  ~MultiBodyDynamicsWorld();

  btMultiBodyDynamicsWorld* get();
  btVector3 gravity() const;
  btVector3 set_gravity(Rice::Object gravity);
  void add_multi_body_object(VALUE multi_body, int group, int mask);
  void remove_multi_body_object(VALUE multi_body);
  void add_constraint_object(VALUE constraint);
  void remove_constraint_object(VALUE constraint);
  void add_link_collider_object(VALUE collider, int group, int mask);
  void remove_link_collider_object(VALUE collider);
  int step_simulation(btScalar time_step, int max_sub_steps, btScalar fixed_time_step);
  int num_multi_bodies() const;
  int num_multi_body_constraints() const;
  int num_collision_objects() const;
  void forward_kinematics();
  void clear_forces();
  void clear_multi_body_forces();
  void mark() const;

private:
  std::unique_ptr<btDefaultCollisionConfiguration> configuration_;
  std::unique_ptr<btCollisionDispatcher> dispatcher_;
  std::unique_ptr<btDbvtBroadphase> broadphase_;
  std::unique_ptr<btMultiBodyConstraintSolver> solver_;
  std::unique_ptr<btMultiBodyDynamicsWorld> world_;
  std::unordered_set<btMultiBody*> multi_bodies_;
  std::unordered_set<btMultiBodyConstraint*> constraints_;
  std::unordered_set<btMultiBodyLinkCollider*> colliders_;
  std::unordered_map<const btMultiBody*, VALUE> multi_body_values_;
  std::unordered_map<const btMultiBodyConstraint*, VALUE> constraint_values_;
  std::unordered_map<const btCollisionObject*, VALUE> collider_values_;
};
} // namespace bullet3

void Init_MultiBody(Rice::Module rb_mBullet);
