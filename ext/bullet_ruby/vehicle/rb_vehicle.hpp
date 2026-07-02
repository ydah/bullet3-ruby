#pragma once

#include <memory>

#include <BulletDynamics/Vehicle/btRaycastVehicle.h>
#include <rice/rice.hpp>

#include "../dynamics/rb_dynamics.hpp"
#include "../linear_math/rb_transform.hpp"

namespace bullet_ruby {
class VehicleTuning {
public:
  VehicleTuning();

  btRaycastVehicle::btVehicleTuning& get();
  const btRaycastVehicle::btVehicleTuning& get() const;
  btScalar suspension_stiffness() const;
  btScalar set_suspension_stiffness(btScalar value);
  btScalar suspension_compression() const;
  btScalar set_suspension_compression(btScalar value);
  btScalar suspension_damping() const;
  btScalar set_suspension_damping(btScalar value);
  btScalar max_suspension_travel_cm() const;
  btScalar set_max_suspension_travel_cm(btScalar value);
  btScalar friction_slip() const;
  btScalar set_friction_slip(btScalar value);
  btScalar max_suspension_force() const;
  btScalar set_max_suspension_force(btScalar value);

private:
  btRaycastVehicle::btVehicleTuning tuning_;
};

class WheelInfo {
public:
  WheelInfo(btRaycastVehicle* vehicle, int index);

  int index() const;
  bool front_wheel() const;
  Transform world_transform() const;
  btVector3 chassis_connection_point() const;
  btVector3 wheel_direction() const;
  btVector3 wheel_axle() const;
  btScalar suspension_rest_length() const;
  btScalar radius() const;
  btScalar suspension_stiffness() const;
  btScalar set_suspension_stiffness(btScalar value);
  btScalar damping_compression() const;
  btScalar set_damping_compression(btScalar value);
  btScalar damping_relaxation() const;
  btScalar set_damping_relaxation(btScalar value);
  btScalar friction_slip() const;
  btScalar set_friction_slip(btScalar value);
  btScalar steering() const;
  btScalar rotation() const;
  btScalar delta_rotation() const;
  btScalar roll_influence() const;
  btScalar set_roll_influence(btScalar value);
  btScalar engine_force() const;
  btScalar brake() const;
  btScalar suspension_force() const;
  btScalar skid_info() const;
  bool contact() const;
  btVector3 contact_point() const;
  btVector3 contact_normal() const;

private:
  btWheelInfo& get();
  const btWheelInfo& get() const;

  btRaycastVehicle* vehicle_;
  int index_;
};

class RaycastVehicle {
public:
  RaycastVehicle(VehicleTuning& tuning, DiscreteDynamicsWorld& world, RigidBody& chassis);

  btRaycastVehicle* get();
  const btRaycastVehicle* get() const;
  WheelInfo add_wheel(Rice::Object connection_point,
                      Rice::Object wheel_direction,
                      Rice::Object wheel_axle,
                      btScalar suspension_rest_length,
                      btScalar wheel_radius,
                      VehicleTuning& tuning,
                      bool front_wheel);
  int num_wheels() const;
  WheelInfo wheel_info(int index);
  Transform wheel_transform(int index) const;
  void update_wheel_transform(int index, bool interpolated_transform);
  Transform chassis_world_transform() const;
  btScalar steering_value(int wheel) const;
  void set_steering_value(btScalar steering, int wheel);
  void apply_engine_force(btScalar force, int wheel);
  void set_brake(btScalar brake, int wheel);
  void reset_suspension();
  btScalar current_speed_km_hour() const;
  btVector3 forward_vector() const;
  void set_pitch_control(btScalar pitch);
  void set_coordinate_system(int right_index, int up_index, int forward_index);
  Rice::Array coordinate_system() const;
  void update_vehicle(btScalar step);

private:
  int validate_wheel_index(int index) const;

  btRaycastVehicle::btVehicleTuning tuning_;
  std::unique_ptr<btDefaultVehicleRaycaster> raycaster_;
  std::unique_ptr<btRaycastVehicle> vehicle_;
};
} // namespace bullet_ruby

void Init_Vehicle(Rice::Module rb_mBullet);
