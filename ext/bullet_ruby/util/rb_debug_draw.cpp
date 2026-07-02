#include "rb_debug_draw.hpp"

#include <ruby.h>

#include "type_conversions.hpp"

namespace {
VALUE array_from_vector(const btVector3& vector)
{
  return rb_ary_new_from_args(3,
    DBL2NUM(vector.getX()),
    DBL2NUM(vector.getY()),
    DBL2NUM(vector.getZ()));
}

VALUE line_hash(const btVector3& from, const btVector3& to, const btVector3& color)
{
  VALUE hash = rb_hash_new();
  rb_hash_aset(hash, ID2SYM(rb_intern("from")), array_from_vector(from));
  rb_hash_aset(hash, ID2SYM(rb_intern("to")), array_from_vector(to));
  rb_hash_aset(hash, ID2SYM(rb_intern("color")), array_from_vector(color));
  return hash;
}

VALUE contact_hash(const btVector3& point,
                   const btVector3& normal,
                   btScalar distance,
                   int life_time,
                   const btVector3& color)
{
  VALUE hash = rb_hash_new();
  rb_hash_aset(hash, ID2SYM(rb_intern("point")), array_from_vector(point));
  rb_hash_aset(hash, ID2SYM(rb_intern("normal")), array_from_vector(normal));
  rb_hash_aset(hash, ID2SYM(rb_intern("distance")), DBL2NUM(distance));
  rb_hash_aset(hash, ID2SYM(rb_intern("life_time")), INT2NUM(life_time));
  rb_hash_aset(hash, ID2SYM(rb_intern("color")), array_from_vector(color));
  return hash;
}

VALUE text_hash(const btVector3& location, const char* text)
{
  VALUE hash = rb_hash_new();
  rb_hash_aset(hash, ID2SYM(rb_intern("location")), array_from_vector(location));
  rb_hash_aset(hash, ID2SYM(rb_intern("text")), rb_str_new_cstr(text));
  return hash;
}
} // namespace

namespace bullet_ruby {
DebugDraw::DebugDraw()
  : DebugDraw(Qnil)
{
}

DebugDraw::DebugDraw(VALUE target)
  : target_(target),
    lines_(rb_ary_new()),
    contact_points_(rb_ary_new()),
    warnings_(rb_ary_new()),
    texts_(rb_ary_new()),
    debug_mode_(DBG_DrawWireframe | DBG_DrawAabb | DBG_DrawConstraints | DBG_DrawConstraintLimits)
{
}

btIDebugDraw* DebugDraw::get()
{
  return this;
}

Rice::Array DebugDraw::lines() const
{
  return Rice::Array(Rice::Object(lines_));
}

Rice::Array DebugDraw::contact_points() const
{
  return Rice::Array(Rice::Object(contact_points_));
}

Rice::Array DebugDraw::warnings() const
{
  return Rice::Array(Rice::Object(warnings_));
}

Rice::Array DebugDraw::texts() const
{
  return Rice::Array(Rice::Object(texts_));
}

void DebugDraw::clear()
{
  rb_ary_clear(lines_);
  rb_ary_clear(contact_points_);
  rb_ary_clear(warnings_);
  rb_ary_clear(texts_);
}

void DebugDraw::draw_line_object(VALUE from, VALUE to, VALUE color)
{
  drawLine(
    coerce_vector(Rice::Object(from)),
    coerce_vector(Rice::Object(to)),
    coerce_vector(Rice::Object(color)));
}

void DebugDraw::draw_contact_point_object(VALUE point, VALUE normal, btScalar distance, int life_time, VALUE color)
{
  drawContactPoint(
    coerce_vector(Rice::Object(point)),
    coerce_vector(Rice::Object(normal)),
    distance,
    life_time,
    coerce_vector(Rice::Object(color)));
}

void DebugDraw::report_error_warning_object(VALUE warning)
{
  reportErrorWarning(StringValueCStr(warning));
}

void DebugDraw::draw_3d_text_object(VALUE location, VALUE text)
{
  draw3dText(coerce_vector(Rice::Object(location)), StringValueCStr(text));
}

void DebugDraw::drawLine(const btVector3& from, const btVector3& to, const btVector3& color)
{
  VALUE entry = line_hash(from, to, color);
  rb_ary_push(lines_, entry);

  VALUE args[] = {
    rb_hash_aref(entry, ID2SYM(rb_intern("from"))),
    rb_hash_aref(entry, ID2SYM(rb_intern("to"))),
    rb_hash_aref(entry, ID2SYM(rb_intern("color")))
  };
  call_target("draw_line", 3, args);
}

void DebugDraw::drawContactPoint(const btVector3& point_on_b,
                                 const btVector3& normal_on_b,
                                 btScalar distance,
                                 int life_time,
                                 const btVector3& color)
{
  VALUE entry = contact_hash(point_on_b, normal_on_b, distance, life_time, color);
  rb_ary_push(contact_points_, entry);
  VALUE args[] = { entry };
  call_target("draw_contact_point", 1, args);
}

void DebugDraw::reportErrorWarning(const char* warning_string)
{
  VALUE warning = rb_str_new_cstr(warning_string);
  rb_ary_push(warnings_, warning);
  VALUE args[] = { warning };
  call_target("report_error_warning", 1, args);
}

void DebugDraw::draw3dText(const btVector3& location, const char* text_string)
{
  VALUE entry = text_hash(location, text_string);
  rb_ary_push(texts_, entry);
  VALUE args[] = {
    rb_hash_aref(entry, ID2SYM(rb_intern("location"))),
    rb_hash_aref(entry, ID2SYM(rb_intern("text")))
  };
  call_target("draw_3d_text", 2, args);
}

void DebugDraw::setDebugMode(int debug_mode)
{
  debug_mode_ = debug_mode;
}

int DebugDraw::getDebugMode() const
{
  return debug_mode_;
}

void DebugDraw::call_target(const char* method_name, int argc, const VALUE* argv) const
{
  if (NIL_P(target_)) {
    return;
  }

  ID method_id = rb_intern(method_name);
  if (!rb_respond_to(target_, method_id)) {
    return;
  }

  rb_funcallv(target_, method_id, argc, argv);
}

void DebugDraw::mark() const
{
  rb_gc_mark(target_);
  rb_gc_mark(lines_);
  rb_gc_mark(contact_points_);
  rb_gc_mark(warnings_);
  rb_gc_mark(texts_);
}
} // namespace bullet_ruby

namespace Rice {
template <>
void ruby_mark<bullet_ruby::DebugDraw>(bullet_ruby::DebugDraw* debug_draw)
{
  if (debug_draw != nullptr) {
    debug_draw->mark();
  }
}
} // namespace Rice

void Init_DebugDraw(Rice::Module rb_mBullet)
{
  Rice::Class rb_cDebugDraw = Rice::define_class_under<bullet_ruby::DebugDraw>(rb_mBullet, "DebugDraw")
    .define_constructor(Rice::Constructor<bullet_ruby::DebugDraw>())
    .define_constructor(Rice::Constructor<bullet_ruby::DebugDraw, VALUE>(),
      Rice::Arg("target").setValue().keepAlive())
    .define_method("lines", &bullet_ruby::DebugDraw::lines)
    .define_method("contact_points", &bullet_ruby::DebugDraw::contact_points)
    .define_method("warnings", &bullet_ruby::DebugDraw::warnings)
    .define_method("texts", &bullet_ruby::DebugDraw::texts)
    .define_method("clear", &bullet_ruby::DebugDraw::clear)
    .define_method("draw_line", &bullet_ruby::DebugDraw::draw_line_object,
      Rice::Arg("from").setValue(),
      Rice::Arg("to").setValue(),
      Rice::Arg("color").setValue())
    .define_method("draw_contact_point", &bullet_ruby::DebugDraw::draw_contact_point_object,
      Rice::Arg("point").setValue(),
      Rice::Arg("normal").setValue(),
      Rice::Arg("distance"),
      Rice::Arg("life_time"),
      Rice::Arg("color").setValue())
    .define_method("report_error_warning", &bullet_ruby::DebugDraw::report_error_warning_object,
      Rice::Arg("warning").setValue())
    .define_method("draw_3d_text", &bullet_ruby::DebugDraw::draw_3d_text_object,
      Rice::Arg("location").setValue(),
      Rice::Arg("text").setValue())
    .define_method("debug_mode", &bullet_ruby::DebugDraw::getDebugMode)
    .define_method("debug_mode=", &bullet_ruby::DebugDraw::setDebugMode);

  rb_cDebugDraw.define_constant("NO_DEBUG", static_cast<int>(btIDebugDraw::DBG_NoDebug));
  rb_cDebugDraw.define_constant("DRAW_WIREFRAME", static_cast<int>(btIDebugDraw::DBG_DrawWireframe));
  rb_cDebugDraw.define_constant("DRAW_AABB", static_cast<int>(btIDebugDraw::DBG_DrawAabb));
  rb_cDebugDraw.define_constant("DRAW_CONTACT_POINTS", static_cast<int>(btIDebugDraw::DBG_DrawContactPoints));
  rb_cDebugDraw.define_constant("DRAW_CONSTRAINTS", static_cast<int>(btIDebugDraw::DBG_DrawConstraints));
  rb_cDebugDraw.define_constant("DRAW_CONSTRAINT_LIMITS", static_cast<int>(btIDebugDraw::DBG_DrawConstraintLimits));
  rb_cDebugDraw.define_constant("DRAW_FRAMES", static_cast<int>(btIDebugDraw::DBG_DrawFrames));
}
