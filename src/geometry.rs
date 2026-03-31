//! Geometric helper types and conversions.

use nalgebra::{Vector2, Vector3};

/// 2D position on the field (x, y) in meters.
pub type Vec2 = Vector2<f64>;

/// 3D position (x, y, z) in meters.
pub type Vec3 = Vector3<f64>;

/// Convert degrees to radians.
#[inline]
pub fn deg2rad(deg: f64) -> f64 {
    deg * std::f64::consts::PI / 180.0
}

/// Convert radians to degrees.
#[inline]
pub fn rad2deg(rad: f64) -> f64 {
    rad * 180.0 / std::f64::consts::PI
}

/// Normalize angle to [-180, 180] degrees.
#[inline]
pub fn normalize_angle_deg(a: f64) -> f64 {
    let mut a = a % 360.0;
    if a > 180.0 {
        a -= 360.0;
    }
    if a < -180.0 {
        a += 360.0;
    }
    a
}

/// Normalize angle to [-pi, pi] radians.
#[inline]
pub fn normalize_angle_rad(a: f64) -> f64 {
    let mut a = a % (2.0 * std::f64::consts::PI);
    if a > std::f64::consts::PI {
        a -= 2.0 * std::f64::consts::PI;
    }
    if a < -std::f64::consts::PI {
        a += 2.0 * std::f64::consts::PI;
    }
    a
}
