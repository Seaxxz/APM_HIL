/*
 * Aircraft.h
 *
 *  Created on: 2018-3-23
 *      Author: Sea
 */

#pragma once
#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
//#include <AP_Terrain/AP_Terrain.h>
#define	RAND_MAX	0x7FFF

class Aircraft {
public:
    Location home1;
    float mass;                     // kg
    float frame_height;             //m
    Vector3f accel_body;            // m/s/s NED, body frame
    Matrix3f dcm;                   // rotation matrix, APM conventions, from body to earth
    struct sitl_input {
        uint16_t servos[16];
        struct {
            float speed;      // m/s
            float direction;  // degrees 0..360
            float turbulence;
        } wind;
    };
    struct sitl_fdm {
        // this is the structure passed between FDM models and the main SITL code
        uint64_t timestamp_us;
        Location home;
        double latitude, longitude; // degrees
        double altitude;  // MSL
        double heading;   // degrees
        double speedN, speedE, speedD; // m/s
        double xAccel, yAccel, zAccel;       // m/s/s in body frame
        double rollRate, pitchRate, yawRate; // degrees/s/s in body frame
        double rollDeg, pitchDeg, yawDeg;    // euler angles, degrees
        Quaternion quaternion;
        double range;           // rangefinder value
        Vector3f bodyMagField;  // Truth XYZ magnetic field vector in body-frame. Includes motor interference. Units are milli-Gauss.
        Vector3f angAccel; // Angular acceleration in degrees/s/s about the XYZ body axes
    };
    void update_wind(const struct sitl_input &input);
    void update_dynamics(const Vector3f &rot_accel);
    void fill_fdm(struct sitl_fdm &fdm);
    void smooth_sensors(void);
    void time_advance(void);
    void setup_frame_time(float new_rate, float new_speedup);
    void adjust_frame_time(float new_rate);
    void set_speedup(float speedup);
    void update_position(void);
    const Vector3f &get_gyro(void) const {
        return gyro;
    }
    const Vector3f &get_velocity_air_ef(void) const {
        return velocity_air_ef;
    }
    const Matrix3f &get_dcm(void) const {
        return dcm;
    }

    /* return normal distribution random numbers */
    static double rand_normal(double mean, double stddev);

protected:
    bool on_ground();
    float hagl();
    // Wind Turbulence simulated Data
    float turbulence_azimuth = 0.0f;
    float turbulence_horizontal_speed = 0.0f;  // m/s
    float turbulence_vertical_speed = 0.0f;    // m/s


    uint64_t frame_time_us;

    Vector3f gyro;                  // rad/s
    Vector3f gyro_prev = Vector3f(0,0,0);             // rad/s
    Vector3f ang_accel;             // rad/s/s
    Vector3f velocity_ef;           // m/s, earth frame
    Vector3f wind_ef;               // m/s, earth frame
    Vector3f velocity_air_ef;       // velocity relative to airmass, earth frame
    Vector3f velocity_air_bf;       // velocity relative to airmass, body frame
    Vector3f position;              // meters, NED from origin
    float airspeed;                 // m/s, apparent airspeed
    float airspeed_pitot;           // m/s, apparent airspeed, as seen by fwd pitot tube

    uint64_t time_now_us;
    float rate_hz = 1200.0f;
    float target_speedup;
    float last_speedup = -1.0f;
    bool use_smoothing = false;

    Location location;

    float ground_level = 0.0f;
private:
    uint32_t last_ground_contact_ms;
    uint64_t last_time_us = 0;
    struct {
        bool enabled;
        Vector3f accel_body;
        Vector3f gyro;
        Matrix3f rotation_b2e;
        Vector3f position;
        Vector3f velocity_ef;
        uint64_t last_update_us;
        Location location;
    } smoothing;
};
