#include <Matrices.h>
#include <Util.h>

/// <summary>
/// Snaps the matrix to the closest angle
/// </summary>
/// <param name="degrees">Degree interval</param>
/// <returns>The snapped matrix</returns>
inline void SnapMatrix(Matrix4* mat, const float degrees) {
    // Convert to the 3x4 matrix.
    vr::HmdMatrix34_t transform = mat->toOpenVR34();
    Vector3 translation = mat->getTranslation();

    // Clamp the values that will be used as input to asin and atan2
    // For pitch extraction, we need to make sure transform.m[1][2] stays in [-1, 1]
    float clamped_m12 = clamp(transform.m[1][2], -1.0f, 1.0f);
    // For yaw extraction, we'll clamp transform.m[0][2]
    float clamped_m02 = clamp(transform.m[0][2], -1.0f, 1.0f);

    // Extract Euler angles (assuming ZYX rotation order)
    // Compute the pitch (rotation about the X-axis)
    float x = std::asin(-clamped_m12);

    // Compute the roll (rotation about the Z-axis)
    float z = std::atan2(transform.m[1][0], transform.m[1][1]);

    // Compute the yaw (rotation about the Y-axis) using the clamped value
    float y = std::atan2(clamped_m02, transform.m[2][2]);

    // Constants for converting between radians and degrees.
    constexpr float radToDeg = 180.0f / 3.14159265358979323846f;
    constexpr float degToRad = 3.14159265358979323846f / 180.0f;

    // Convert to degrees.
    x *= radToDeg;
    y *= radToDeg;
    z *= radToDeg;

    //// Print original Euler angles.
    //char buffer[256];
    //sprintf_s(buffer, "Euler Angles: x = %.1f, y = %.1f, z = %.1f\n", x, y, z);
    //OutputDebugStringA(buffer);

    // Snap angles to the nearest multiple of 'degrees'
    auto snapToNearest = [degrees](float angle) {
        return round(angle / degrees) * degrees;
        };

    x = snapToNearest(x);
    y = snapToNearest(y);
    z = snapToNearest(z);

    //// Print snapped angles.
    //sprintf_s(buffer, "Snapped Angles: x = %.1f, y = %.1f, z = %.1f\n", x, y, z);
    //OutputDebugStringA(buffer);

    // Convert angles back to radians.
    x *= degToRad;
    y *= degToRad;
    z *= degToRad;

    float cy = std::cos(y);
    float sy = std::sin(y);
    float cp = std::cos(x);
    float sp = std::sin(x);
    float cr = std::cos(z);
    float sr = std::sin(z);

    // Fill the rotation portion of the matrix.
    // Row 0
    transform.m[0][0] = cy * cr + sy * sp * sr;
    transform.m[0][1] = -cy * sr + sy * sp * cr;
    transform.m[0][2] = sy * cp;
    transform.m[0][3] = 0.0f; // Translation x (0 if not used)

    // Row 1
    transform.m[1][0] = cp * sr;
    transform.m[1][1] = cp * cr;
    transform.m[1][2] = -sp;
    transform.m[1][3] = 0.0f; // Translation y (0 if not used)

    // Row 2
    transform.m[2][0] = cy * sp * sr - sy * cr;
    transform.m[2][1] = sy * sr + cy * sp * cr;
    transform.m[2][2] = cy * cp;
    transform.m[2][3] = 0.0f; // Translation z (0 if not used)

    mat->set(Matrix4(transform));
    mat->setTranslation(translation);
}
