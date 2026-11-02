#include "Copter.h"

#if LOWALT_LIMIT_ENABLED

// an implausible below-minimum rangefinder reading is held silently for
// this long before it is reported as a lost sensor
#define LOWALT_BELOW_MIN_REPORT_MS 1000

const AP_Param::GroupInfo LowAltLimit::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Low altitude limits enable
    // @Description: Reduces the pilot's maximum descent rate, horizontal speed and lean angle in Loiter when the vehicle is close to the ground. Requires a downward facing rangefinder; without one this feature does nothing except warn the pilot when Loiter is entered. The full limits apply at or below LOWALT_HGT and blend back to the normal limits by twice LOWALT_HGT. If the rangefinder becomes unhealthy or out of range the limits in force are held until it recovers, the mode is re-entered, or the vehicle is disarmed. A missing or unhealthy rangefinder never prevents flight.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, LowAltLimit, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: HGT
    // @DisplayName: Low altitude limits height
    // @Description: Rangefinder height above ground at or below which the full LOWALT limits apply. The limits begin to apply at twice this height, which should be well below the rangefinder's maximum range so the limits can release.
    // @Units: m
    // @Range: 0.5 20
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("HGT", 2, LowAltLimit, _height_m, 5.0f),

    // @Param: SPD_DN
    // @DisplayName: Low altitude descent speed
    // @Description: Maximum pilot descent speed at or below LOWALT_HGT. Never raises the descent speed above the normal pilot limit. Set to zero to leave the descent speed unlimited.
    // @Units: cm/s
    // @Range: 0 200
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("SPD_DN", 3, LowAltLimit, _speed_dn_cms, 50.0f),

    // @Param: SPD_XY
    // @DisplayName: Low altitude horizontal speed
    // @Description: Maximum horizontal speed in Loiter at or below LOWALT_HGT. Never raises the speed above LOIT_SPEED. Values below 20 cm/s behave as 20 cm/s. Set to zero to leave the horizontal speed unlimited.
    // @Units: cm/s
    // @Range: 0 500
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("SPD_XY", 4, LowAltLimit, _speed_xy_cms, 100.0f),

    // @Param: ANG_MAX
    // @DisplayName: Low altitude lean angle
    // @Description: Maximum pilot commanded lean angle in Loiter at or below LOWALT_HGT. Never raises the angle above the normal Loiter limit. The position controller keeps its full authority to hold position. Set to zero to leave the lean angle unlimited.
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ANG_MAX", 5, LowAltLimit, _angle_max_deg, 10.0f),

    // index 6 was LOWALT_SRC in a development build; do not reuse

    AP_GROUPEND
};

LowAltLimit::LowAltLimit()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void LowAltLimit::reset_state()
{
    _height_valid = false;
    _lost_reported = false;
    _below_min_start_ms = 0;
    _blend = 1.0f;
}

// a missing or silent rangefinder is reported, never enforced
void LowAltLimit::check_rangefinder()
{
    if (!copter.rangefinder_state.enabled) {
        warn("no downward rangefinder, limits inactive");
        _lost_reported = true;
        return;
    }

    switch (copter.rangefinder.status_orient(ROTATION_PITCH_270)) {
    case RangeFinder::Status::NotConnected:
    case RangeFinder::Status::NoData:
        warn("rangefinder unhealthy, limits inactive");
        _lost_reported = true;
        break;
    case RangeFinder::Status::OutOfRangeLow:
    case RangeFinder::Status::OutOfRangeHigh:
    case RangeFinder::Status::Good:
        break;
    }

    // the limits cannot release above the sensor's reach
    const float max_m = copter.rangefinder.max_distance_cm_orient(ROTATION_PITCH_270) * 0.01f;
    if (max_m < 2.0f * _height_m) {
        gcs().send_text(MAV_SEVERITY_WARNING, "LowAlt: RNGFND max %.1fm below 2x LOWALT_HGT", (double)max_m);
    }
}

void LowAltLimit::init()
{
    if (enabled() && limiting() && _lost_reported) {
        // the pilot chose a mode change while a limit was held; say so
        warn("held limits released by mode change");
    }
    reset_state();
    _was_armed = copter.motors->armed();
    if (enabled()) {
        check_rangefinder();
    }
}

bool LowAltLimit::get_rangefinder_height(float &height_cm, bool &transient)
{
    transient = false;
    if (!copter.rangefinder_state.enabled) {
        return false;
    }

    // accept a height only from a healthy, non-glitching reading
    int32_t alt_cm;
    if (copter.rangefinder_state.glitch_count == 0 &&
        copter.get_rangefinder_height_interpolated_cm(alt_cm)) {
        height_cm = MAX(alt_cm, 0);
        _below_min_start_ms = 0;
        return true;
    }

    const RangeFinder::Status status = copter.rangefinder.status_orient(ROTATION_PITCH_270);
    if (status == RangeFinder::Status::OutOfRangeLow) {
        // the ground is closer than the sensor can measure.  Believed
        // only if the vehicle was already within the limited band; a
        // lost echo at altitude must not force the full limits
        if (!_height_valid || _height_cm < 2.0f * _height_m * 100.0f) {
            height_cm = 0.0f;
            _below_min_start_ms = 0;
            return true;
        }
        const uint32_t now_ms = AP_HAL::millis();
        if (_below_min_start_ms == 0) {
            _below_min_start_ms = now_ms;
        }
        transient = (now_ms - _below_min_start_ms) < LOWALT_BELOW_MIN_REPORT_MS;
        return false;
    }
    _below_min_start_ms = 0;

    // a Good status that is not yet trusted (glitch, or too few
    // consecutive samples) clears within a few samples
    transient = (status == RangeFinder::Status::Good);
    return false;
}

void LowAltLimit::update()
{
    if (!enabled()) {
        reset_state();
        return;
    }

    // nothing held carries across arming.  Arming re-reports a sensor
    // that is still unusable; disarming simply forgets the flight
    const bool armed = copter.motors->armed();
    if (armed != _was_armed) {
        _was_armed = armed;
        reset_state();
    }

    float height_cm = 0.0f;
    bool transient = false;
    if (get_rangefinder_height(height_cm, transient)) {
        _height_cm = height_cm;
        _height_valid = true;
        if (_lost_reported) {
            _lost_reported = false;
            gcs().send_text(MAV_SEVERITY_INFO, "LowAlt: height available");
        }
    } else if (!transient && !_lost_reported) {
        // the last accepted height, and so the limits, are held
        _lost_reported = true;
        warn(limiting() ? "height lost, limits held" : "height unavailable, limits inactive");
    }

    update_blend();
    log();
}

// 0 at or below LOWALT_HGT, 1 at or above twice LOWALT_HGT, linear between
void LowAltLimit::update_blend()
{
    if (!_height_valid || !is_positive(_height_m)) {
        _blend = 1.0f;
        return;
    }
    const float hgt_cm = _height_m * 100.0f;
    _blend = constrain_float((_height_cm - hgt_cm) / hgt_cm, 0.0f, 1.0f);
}

float LowAltLimit::get_max_descent_cms(float pilot_speed_dn_cms) const
{
    if (!limiting() || !is_positive(_speed_dn_cms)) {
        return pilot_speed_dn_cms;
    }
    return MIN(pilot_speed_dn_cms, linear_interpolate(_speed_dn_cms, pilot_speed_dn_cms, _blend, 0.0f, 1.0f));
}

float LowAltLimit::get_max_speed_xy_cms(float loiter_speed_cms) const
{
    if (!limiting() || !is_positive(_speed_xy_cms)) {
        return loiter_speed_cms;
    }
    return MIN(loiter_speed_cms, linear_interpolate(_speed_xy_cms, loiter_speed_cms, _blend, 0.0f, 1.0f));
}

float LowAltLimit::get_angle_max_cd(float angle_max_cd) const
{
    if (!limiting() || !is_positive(_angle_max_deg)) {
        return angle_max_cd;
    }
    return MIN(angle_max_cd, linear_interpolate(_angle_max_deg * 100.0f, angle_max_cd, _blend, 0.0f, 1.0f));
}

void LowAltLimit::warn(const char *msg) const
{
    gcs().send_text(MAV_SEVERITY_WARNING, "LowAlt: %s", msg);
}

// @LoggerMessage: LALT
// @Description: Low altitude limit state
// @Field: TimeUS: Time since system startup
// @Field: Hgt: Last accepted rangefinder height above ground, -1 if none
// @Field: Lost: 1 while the rangefinder is reported lost and the height is held
// @Field: Blnd: 0 at full limits, 1 at normal limits
void LowAltLimit::log()
{
#if HAL_LOGGING_ENABLED
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_log_ms < 100) {
        return;
    }
    _last_log_ms = now_ms;
    if (!copter.should_log(MASK_LOG_CTUN)) {
        return;
    }
    AP::logger().WriteStreaming("LALT", "TimeUS,Hgt,Lost,Blnd", "sm--", "F0--", "Qfbf",
                                AP_HAL::micros64(),
                                _height_valid ? _height_cm * 0.01f : -1.0f,
                                (int8_t)_lost_reported,
                                _blend);
#endif
}

#endif // LOWALT_LIMIT_ENABLED
