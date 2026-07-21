#include "rb_vehicle.hpp"

#include <stdexcept>

#include "../util/type_conversions.hpp"

namespace bullet3 {
VehicleTuning::VehicleTuning() = default;

btRaycastVehicle::btVehicleTuning& VehicleTuning::get()
{
  return tuning_;
}

const btRaycastVehicle::btVehicleTuning& VehicleTuning::get() const
{
  return tuning_;
}

btScalar VehicleTuning::suspension_stiffness() const
{
  return tuning_.m_suspensionStiffness;
}

btScalar VehicleTuning::set_suspension_stiffness(btScalar value)
{
  tuning_.m_suspensionStiffness = value;
  return value;
}

btScalar VehicleTuning::suspension_compression() const
{
  return tuning_.m_suspensionCompression;
}

btScalar VehicleTuning::set_suspension_compression(btScalar value)
{
  tuning_.m_suspensionCompression = value;
  return value;
}

btScalar VehicleTuning::suspension_damping() const
{
  return tuning_.m_suspensionDamping;
}

btScalar VehicleTuning::set_suspension_damping(btScalar value)
{
  tuning_.m_suspensionDamping = value;
  return value;
}

btScalar VehicleTuning::max_suspension_travel_cm() const
{
  return tuning_.m_maxSuspensionTravelCm;
}

btScalar VehicleTuning::set_max_suspension_travel_cm(btScalar value)
{
  tuning_.m_maxSuspensionTravelCm = value;
  return value;
}

btScalar VehicleTuning::friction_slip() const
{
  return tuning_.m_frictionSlip;
}

btScalar VehicleTuning::set_friction_slip(btScalar value)
{
  tuning_.m_frictionSlip = value;
  return value;
}

btScalar VehicleTuning::max_suspension_force() const
{
  return tuning_.m_maxSuspensionForce;
}

btScalar VehicleTuning::set_max_suspension_force(btScalar value)
{
  tuning_.m_maxSuspensionForce = value;
  return value;
}

WheelInfo::WheelInfo(btRaycastVehicle* vehicle, int index)
  : vehicle_(vehicle),
    index_(index)
{
  if (vehicle_ == nullptr) {
    throw std::invalid_argument("expected a RaycastVehicle");
  }
  get();
}

int WheelInfo::index() const
{
  return index_;
}

btWheelInfo& WheelInfo::get()
{
  if (index_ < 0 || index_ >= vehicle_->getNumWheels()) {
    throw std::out_of_range("wheel index is out of range");
  }
  return vehicle_->getWheelInfo(index_);
}

const btWheelInfo& WheelInfo::get() const
{
  if (index_ < 0 || index_ >= vehicle_->getNumWheels()) {
    throw std::out_of_range("wheel index is out of range");
  }
  return vehicle_->getWheelInfo(index_);
}

bool WheelInfo::front_wheel() const
{
  return get().m_bIsFrontWheel;
}

Transform WheelInfo::world_transform() const
{
  return Transform(get().m_worldTransform);
}

btVector3 WheelInfo::chassis_connection_point() const
{
  return get().m_chassisConnectionPointCS;
}

btVector3 WheelInfo::wheel_direction() const
{
  return get().m_wheelDirectionCS;
}

btVector3 WheelInfo::wheel_axle() const
{
  return get().m_wheelAxleCS;
}

btScalar WheelInfo::suspension_rest_length() const
{
  return get().getSuspensionRestLength();
}

btScalar WheelInfo::radius() const
{
  return get().m_wheelsRadius;
}

btScalar WheelInfo::suspension_stiffness() const
{
  return get().m_suspensionStiffness;
}

btScalar WheelInfo::set_suspension_stiffness(btScalar value)
{
  get().m_suspensionStiffness = value;
  return value;
}

btScalar WheelInfo::damping_compression() const
{
  return get().m_wheelsDampingCompression;
}

btScalar WheelInfo::set_damping_compression(btScalar value)
{
  get().m_wheelsDampingCompression = value;
  return value;
}

btScalar WheelInfo::damping_relaxation() const
{
  return get().m_wheelsDampingRelaxation;
}

btScalar WheelInfo::set_damping_relaxation(btScalar value)
{
  get().m_wheelsDampingRelaxation = value;
  return value;
}

btScalar WheelInfo::friction_slip() const
{
  return get().m_frictionSlip;
}

btScalar WheelInfo::set_friction_slip(btScalar value)
{
  get().m_frictionSlip = value;
  return value;
}

btScalar WheelInfo::steering() const
{
  return get().m_steering;
}

btScalar WheelInfo::rotation() const
{
  return get().m_rotation;
}

btScalar WheelInfo::delta_rotation() const
{
  return get().m_deltaRotation;
}

btScalar WheelInfo::roll_influence() const
{
  return get().m_rollInfluence;
}

btScalar WheelInfo::set_roll_influence(btScalar value)
{
  get().m_rollInfluence = value;
  return value;
}

btScalar WheelInfo::engine_force() const
{
  return get().m_engineForce;
}

btScalar WheelInfo::brake() const
{
  return get().m_brake;
}

btScalar WheelInfo::suspension_force() const
{
  return get().m_wheelsSuspensionForce;
}

btScalar WheelInfo::skid_info() const
{
  return get().m_skidInfo;
}

bool WheelInfo::contact() const
{
  return get().m_raycastInfo.m_isInContact;
}

btVector3 WheelInfo::contact_point() const
{
  return get().m_raycastInfo.m_contactPointWS;
}

btVector3 WheelInfo::contact_normal() const
{
  return get().m_raycastInfo.m_contactNormalWS;
}

RaycastVehicle::RaycastVehicle(VehicleTuning& tuning, DiscreteDynamicsWorld& world, RigidBody& chassis)
  : tuning_(tuning.get()),
    raycaster_(std::make_unique<btDefaultVehicleRaycaster>(world.get())),
    vehicle_(std::make_unique<btRaycastVehicle>(tuning_, chassis.get(), raycaster_.get()))
{
}

btRaycastVehicle* RaycastVehicle::get()
{
  return vehicle_.get();
}

const btRaycastVehicle* RaycastVehicle::get() const
{
  return vehicle_.get();
}

int RaycastVehicle::validate_wheel_index(int index) const
{
  if (index < 0 || index >= vehicle_->getNumWheels()) {
    throw std::out_of_range("wheel index is out of range");
  }
  return index;
}

WheelInfo RaycastVehicle::add_wheel(Rice::Object connection_point,
                                    Rice::Object wheel_direction,
                                    Rice::Object wheel_axle,
                                    btScalar suspension_rest_length,
                                    btScalar wheel_radius,
                                    VehicleTuning& tuning,
                                    bool front_wheel)
{
  vehicle_->addWheel(
    coerce_vector(connection_point),
    coerce_vector(wheel_direction),
    coerce_vector(wheel_axle),
    suspension_rest_length,
    wheel_radius,
    tuning.get(),
    front_wheel);
  return WheelInfo(vehicle_.get(), vehicle_->getNumWheels() - 1);
}

int RaycastVehicle::num_wheels() const
{
  return vehicle_->getNumWheels();
}

WheelInfo RaycastVehicle::wheel_info(int index)
{
  return WheelInfo(vehicle_.get(), validate_wheel_index(index));
}

Transform RaycastVehicle::wheel_transform(int index) const
{
  return Transform(vehicle_->getWheelTransformWS(validate_wheel_index(index)));
}

void RaycastVehicle::update_wheel_transform(int index, bool interpolated_transform)
{
  vehicle_->updateWheelTransform(validate_wheel_index(index), interpolated_transform);
}

Transform RaycastVehicle::chassis_world_transform() const
{
  return Transform(vehicle_->getChassisWorldTransform());
}

btScalar RaycastVehicle::steering_value(int wheel) const
{
  return vehicle_->getSteeringValue(validate_wheel_index(wheel));
}

void RaycastVehicle::set_steering_value(btScalar steering, int wheel)
{
  vehicle_->setSteeringValue(steering, validate_wheel_index(wheel));
}

void RaycastVehicle::apply_engine_force(btScalar force, int wheel)
{
  vehicle_->applyEngineForce(force, validate_wheel_index(wheel));
}

void RaycastVehicle::set_brake(btScalar brake, int wheel)
{
  vehicle_->setBrake(brake, validate_wheel_index(wheel));
}

void RaycastVehicle::reset_suspension()
{
  vehicle_->resetSuspension();
}

btScalar RaycastVehicle::current_speed_km_hour() const
{
  return vehicle_->getCurrentSpeedKmHour();
}

btVector3 RaycastVehicle::forward_vector() const
{
  return vehicle_->getForwardVector();
}

void RaycastVehicle::set_pitch_control(btScalar pitch)
{
  vehicle_->setPitchControl(pitch);
}

void RaycastVehicle::set_coordinate_system(int right_index, int up_index, int forward_index)
{
  vehicle_->setCoordinateSystem(right_index, up_index, forward_index);
}

Rice::Array RaycastVehicle::coordinate_system() const
{
  Rice::Array array;
  array.push(vehicle_->getRightAxis());
  array.push(vehicle_->getUpAxis());
  array.push(vehicle_->getForwardAxis());
  return array;
}

void RaycastVehicle::update_vehicle(btScalar step)
{
  vehicle_->updateVehicle(step);
}
} // namespace bullet3

void Init_Vehicle(Rice::Module rb_mBullet)
{
  Rice::define_class_under<bullet3::VehicleTuning>(rb_mBullet, "VehicleTuning")
    .define_constructor(Rice::Constructor<bullet3::VehicleTuning>())
    .define_method("suspension_stiffness", &bullet3::VehicleTuning::suspension_stiffness)
    .define_method("suspension_stiffness=", &bullet3::VehicleTuning::set_suspension_stiffness)
    .define_method("suspension_compression", &bullet3::VehicleTuning::suspension_compression)
    .define_method("suspension_compression=", &bullet3::VehicleTuning::set_suspension_compression)
    .define_method("suspension_damping", &bullet3::VehicleTuning::suspension_damping)
    .define_method("suspension_damping=", &bullet3::VehicleTuning::set_suspension_damping)
    .define_method("max_suspension_travel_cm", &bullet3::VehicleTuning::max_suspension_travel_cm)
    .define_method("max_suspension_travel_cm=", &bullet3::VehicleTuning::set_max_suspension_travel_cm)
    .define_method("friction_slip", &bullet3::VehicleTuning::friction_slip)
    .define_method("friction_slip=", &bullet3::VehicleTuning::set_friction_slip)
    .define_method("max_suspension_force", &bullet3::VehicleTuning::max_suspension_force)
    .define_method("max_suspension_force=", &bullet3::VehicleTuning::set_max_suspension_force);

  Rice::define_class_under<bullet3::WheelInfo>(rb_mBullet, "WheelInfo")
    .define_method("index", &bullet3::WheelInfo::index)
    .define_method("front_wheel?", &bullet3::WheelInfo::front_wheel)
    .define_method("world_transform", &bullet3::WheelInfo::world_transform)
    .define_method("chassis_connection_point", &bullet3::WheelInfo::chassis_connection_point)
    .define_method("wheel_direction", &bullet3::WheelInfo::wheel_direction)
    .define_method("wheel_axle", &bullet3::WheelInfo::wheel_axle)
    .define_method("suspension_rest_length", &bullet3::WheelInfo::suspension_rest_length)
    .define_method("radius", &bullet3::WheelInfo::radius)
    .define_method("suspension_stiffness", &bullet3::WheelInfo::suspension_stiffness)
    .define_method("suspension_stiffness=", &bullet3::WheelInfo::set_suspension_stiffness)
    .define_method("damping_compression", &bullet3::WheelInfo::damping_compression)
    .define_method("damping_compression=", &bullet3::WheelInfo::set_damping_compression)
    .define_method("damping_relaxation", &bullet3::WheelInfo::damping_relaxation)
    .define_method("damping_relaxation=", &bullet3::WheelInfo::set_damping_relaxation)
    .define_method("friction_slip", &bullet3::WheelInfo::friction_slip)
    .define_method("friction_slip=", &bullet3::WheelInfo::set_friction_slip)
    .define_method("steering", &bullet3::WheelInfo::steering)
    .define_method("rotation", &bullet3::WheelInfo::rotation)
    .define_method("delta_rotation", &bullet3::WheelInfo::delta_rotation)
    .define_method("roll_influence", &bullet3::WheelInfo::roll_influence)
    .define_method("roll_influence=", &bullet3::WheelInfo::set_roll_influence)
    .define_method("engine_force", &bullet3::WheelInfo::engine_force)
    .define_method("brake", &bullet3::WheelInfo::brake)
    .define_method("suspension_force", &bullet3::WheelInfo::suspension_force)
    .define_method("skid_info", &bullet3::WheelInfo::skid_info)
    .define_method("contact?", &bullet3::WheelInfo::contact)
    .define_method("contact_point", &bullet3::WheelInfo::contact_point)
    .define_method("contact_normal", &bullet3::WheelInfo::contact_normal);

  Rice::define_class_under<bullet3::RaycastVehicle>(rb_mBullet, "RaycastVehicle")
    .define_constructor(Rice::Constructor<bullet3::RaycastVehicle,
        bullet3::VehicleTuning&,
        bullet3::DiscreteDynamicsWorld&,
        bullet3::RigidBody&>(),
      Rice::Arg("tuning").keepAlive(),
      Rice::Arg("world").keepAlive(),
      Rice::Arg("chassis").keepAlive())
    .define_method("add_wheel", &bullet3::RaycastVehicle::add_wheel,
      Rice::Arg("connection_point").setValue(),
      Rice::Arg("wheel_direction").setValue(),
      Rice::Arg("wheel_axle").setValue(),
      Rice::Arg("suspension_rest_length"),
      Rice::Arg("wheel_radius"),
      Rice::Arg("tuning").keepAlive(),
      Rice::Arg("front_wheel"))
    .define_method("num_wheels", &bullet3::RaycastVehicle::num_wheels)
    .define_method("wheel_info", &bullet3::RaycastVehicle::wheel_info)
    .define_method("wheel_transform", &bullet3::RaycastVehicle::wheel_transform)
    .define_method("update_wheel_transform", &bullet3::RaycastVehicle::update_wheel_transform,
      Rice::Arg("index"),
      Rice::Arg("interpolated_transform") = true)
    .define_method("chassis_world_transform", &bullet3::RaycastVehicle::chassis_world_transform)
    .define_method("steering_value", &bullet3::RaycastVehicle::steering_value)
    .define_method("set_steering_value", &bullet3::RaycastVehicle::set_steering_value)
    .define_method("apply_engine_force", &bullet3::RaycastVehicle::apply_engine_force)
    .define_method("set_brake", &bullet3::RaycastVehicle::set_brake)
    .define_method("reset_suspension", &bullet3::RaycastVehicle::reset_suspension)
    .define_method("current_speed_km_hour", &bullet3::RaycastVehicle::current_speed_km_hour)
    .define_method("forward_vector", &bullet3::RaycastVehicle::forward_vector)
    .define_method("pitch_control=", &bullet3::RaycastVehicle::set_pitch_control)
    .define_method("set_coordinate_system", &bullet3::RaycastVehicle::set_coordinate_system)
    .define_method("coordinate_system", &bullet3::RaycastVehicle::coordinate_system)
    .define_method("update_vehicle", &bullet3::RaycastVehicle::update_vehicle);
}
