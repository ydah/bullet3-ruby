#pragma once

#include <LinearMath/btIDebugDraw.h>
#include <rice/rice.hpp>

namespace bullet_ruby {
class DebugDraw : public btIDebugDraw {
public:
  DebugDraw();
  explicit DebugDraw(VALUE target);

  btIDebugDraw* get();
  Rice::Array lines() const;
  Rice::Array contact_points() const;
  Rice::Array warnings() const;
  Rice::Array texts() const;
  void clear();
  void draw_line_object(VALUE from, VALUE to, VALUE color);
  void draw_contact_point_object(VALUE point, VALUE normal, btScalar distance, int life_time, VALUE color);
  void report_error_warning_object(VALUE warning);
  void draw_3d_text_object(VALUE location, VALUE text);
  void mark() const;

  void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override;
  void drawContactPoint(const btVector3& point_on_b,
                        const btVector3& normal_on_b,
                        btScalar distance,
                        int life_time,
                        const btVector3& color) override;
  void reportErrorWarning(const char* warning_string) override;
  void draw3dText(const btVector3& location, const char* text_string) override;
  void setDebugMode(int debug_mode) override;
  int getDebugMode() const override;

private:
  void call_target(const char* method_name, int argc, const VALUE* argv) const;

  VALUE target_;
  VALUE lines_;
  VALUE contact_points_;
  VALUE warnings_;
  VALUE texts_;
  int debug_mode_;
};
} // namespace bullet_ruby

void Init_DebugDraw(Rice::Module rb_mBullet);
