#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <Wire.h>

/*
 * AS5600 12-bit magnetic angle sensor (I2C address 0x36)
 *
 * Gives you:
 *   - angleDeg()     absolute angle 0..360 (after zero offset + invert)
 *   - velocityDps()  rotation speed in deg/s (+ = angle increasing)
 *   - direction()    CW (angle increasing) / CCW (angle decreasing) / STOPPED
 *
 * It reads the RAW ANGLE register (0x0C), so the chip's internal
 * ZPOS/MPOS/MANG settings can never limit the range.
 */

#ifndef AS5600_DIR_THRESHOLD_DPS
#define AS5600_DIR_THRESHOLD_DPS 5.0f   // slower than this -> STOPPED
#endif

static const uint8_t  AS5600_I2C_ADDR       = 0x36;
static const uint8_t  AS5600_REG_STATUS     = 0x0B;
static const uint8_t  AS5600_REG_RAW_H      = 0x0C;
static const uint8_t  AS5600_STATUS_MD      = 0x20;   // magnet detected
static const uint8_t  AS5600_MAX_FAILS      = 5;      // consecutive I2C errors before "unhealthy"
static const uint32_t AS5600_VEL_INTERVAL_MS = 20;    // speed is computed every 20 ms

class AS5600
{
public:
    enum Direction { CCW = -1, STOPPED = 0, CW = 1 };

    AS5600(TwoWire &wire, float zero_deg = 0.0f, bool invert = false)
        : wire_(wire), zero_deg_(zero_deg), invert_(invert) {}

    // true if the sensor answers AND a magnet is detected
    bool begin(uint32_t clock_hz = 400000)
    {
        wire_.begin();
        wire_.setClock(clock_hz);

        if (!readBytes(AS5600_REG_STATUS, &status_, 1)) return false;
        if (!(status_ & AS5600_STATUS_MD)) return false;
        if (!readAngle()) return false;

        fail_count_   = 0;
        vel_prev_deg_ = angle_deg_;
        vel_time_     = millis();
        return true;
    }

    // Call this every loop. Returns false if the I2C read failed.
    bool update()
    {
        if (!readAngle())
        {
            if (fail_count_ < 255) fail_count_++;
            return false;
        }
        fail_count_ = 0;

        uint32_t now = millis();
        if (now - vel_time_ >= AS5600_VEL_INTERVAL_MS)
        {
            float dt    = (now - vel_time_) / 1000.0f;
            float delta = wrap180(angle_deg_ - vel_prev_deg_);

            // light smoothing so 1-count flicker doesn't flip the direction
            vel_dps_ = 0.6f * vel_dps_ + 0.4f * (delta / dt);

            vel_prev_deg_ = angle_deg_;
            vel_time_     = now;

            if (vel_dps_ > AS5600_DIR_THRESHOLD_DPS)       dir_ = CW;
            else if (vel_dps_ < -AS5600_DIR_THRESHOLD_DPS) dir_ = CCW;
            else                                           dir_ = STOPPED;
        }
        return true;
    }

    float angleDeg() const    { return angle_deg_; }   // 0..360, zero offset applied
    float rawDeg() const      { return raw_deg_; }     // 0..360, invert applied, NO zero offset
    float velocityDps() const { return vel_dps_; }
    Direction direction() const { return dir_; }
    const char *directionStr() const
    {
        return dir_ == CW ? "CW" : (dir_ == CCW ? "CCW" : "STOP");
    }
    uint8_t status() const { return status_; }
    bool healthy() const   { return fail_count_ < AS5600_MAX_FAILS; }

private:
    bool readBytes(uint8_t reg, uint8_t *buf, uint8_t len)
    {
        wire_.beginTransmission(AS5600_I2C_ADDR);
        wire_.write(reg);
        if (wire_.endTransmission(false) != 0) return false;
        if (wire_.requestFrom(AS5600_I2C_ADDR, len) != len) return false;
        for (uint8_t i = 0; i < len; i++) buf[i] = wire_.read();
        return true;
    }

    bool readAngle()
    {
        uint8_t b[2];
        if (!readBytes(AS5600_REG_RAW_H, b, 2)) return false;

        uint16_t raw = (((uint16_t)b[0] << 8) | b[1]) & 0x0FFF;   // 0..4095
        float deg = raw * (360.0f / 4096.0f);
        if (invert_) deg = 360.0f - deg;

        raw_deg_   = wrap360(deg);
        angle_deg_ = wrap360(raw_deg_ - zero_deg_);
        return true;
    }

    static float wrap360(float x)
    {
        x = fmodf(x, 360.0f);
        if (x < 0) x += 360.0f;
        return x;
    }
    static float wrap180(float x) { return wrap360(x + 180.0f) - 180.0f; }

    TwoWire &wire_;
    float    zero_deg_;
    bool     invert_;

    uint8_t  status_     = 0;
    uint8_t  fail_count_ = 0;
    float    raw_deg_    = 0;
    float    angle_deg_  = 0;
    float    vel_dps_    = 0;
    float    vel_prev_deg_ = 0;
    uint32_t vel_time_   = 0;
    Direction dir_       = STOPPED;
};

#endif
