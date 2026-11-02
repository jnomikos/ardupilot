#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

/*
  Low altitude limits

  Reduces the pilot's maximum descent rate, horizontal speed and lean
  angle in Loiter when the downward rangefinder shows the vehicle is
  close to the ground.  The full limits apply at or below LOWALT_HGT
  and blend linearly back to the normal limits by twice LOWALT_HGT, so
  there is no step change and no hysteresis state.

  The rangefinder is the only height source.  While it cannot provide a
  trusted height the last accepted height is held, so a limit in force
  when the sensor fails stays in force until the sensor recovers, the
  mode is re-entered, or the vehicle is disarmed.  A missing or
  unhealthy rangefinder is reported to the pilot; it never raises a
  limit and never prevents flight.
 */
class LowAltLimit {
public:
    LowAltLimit();

    /* Do not allow copies */
    CLASS_NO_COPY(LowAltLimit);

    static const struct AP_Param::GroupInfo var_info[];

    bool enabled() const { return _enable != 0; }

    // called when a mode that uses the limits is entered.  Releases any
    // held height and warns the pilot if the rangefinder is not usable
    void init();

    // called once per main loop while a mode that uses the limits is running
    void update();

    // each returns the normal value when no limit applies
    float get_max_descent_cms(float pilot_speed_dn_cms) const;
    float get_max_speed_xy_cms(float loiter_speed_cms) const;
    float get_angle_max_cd(float angle_max_cd) const;

    // true when at least one limit may be below its normal value
    bool limiting() const { return _blend < 1.0f; }

private:
    // parameters
    AP_Int8  _enable;
    AP_Float _height_m;       // full limits at or below this height, none at twice this height
    AP_Float _speed_dn_cms;   // descent speed at the full limit, zero leaves descent unlimited
    AP_Float _speed_xy_cms;   // horizontal speed at the full limit, zero leaves it unlimited
    AP_Float _angle_max_deg;  // pilot lean angle at the full limit, zero leaves it unlimited

    // 0 = full limits, 1 = no limits.  Recomputed from the last accepted height
    float _blend = 1.0f;

    // last accepted height above ground in cm, retained while the rangefinder is unusable
    float _height_cm = 0.0f;
    bool _height_valid = false;

    // true after the pilot has been told the rangefinder is lost
    bool _lost_reported = false;

    // time an implausible below-minimum reading was first seen, 0 when none
    uint32_t _below_min_start_ms = 0;

    bool _was_armed = false;
    uint32_t _last_log_ms = 0;

    // clear the held height and the warning state
    void reset_state();

    // warn the pilot if the rangefinder is missing, unhealthy or too short ranged
    void check_rangefinder();

    // returns true and fills height_cm when the rangefinder can provide a trusted
    // height now.  transient is set when a failure is expected to clear within
    // a few samples and so should not be reported
    bool get_rangefinder_height(float &height_cm, bool &transient);

    void update_blend();
    void warn(const char *msg) const;
    void log();
};
