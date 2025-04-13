#include <Matrices.h>
#include <Util.h>

/// <summary>
/// Snaps the matrix to the closest angle on selected axes.
/// </summary>
/// <param name="mat">Pointer to the matrix to be snapped.</param>
/// <param name="degrees">Degree interval for snapping.</param>
/// <param name="snapX">If true, snap the rotation about the X-axis.</param>
/// <param name="snapY">If true, snap the rotation about the Y-axis.</param>
/// <param name="snapZ">If true, snap the rotation about the Z-axis.</param>
/// <returns>The snapped matrix (modified in place).</returns>
inline void SnapMatrix(Matrix4* mat, const float degrees, bool snapX, bool snapY, bool snapZ) {
    // Convert to the 3x4 matrix.
    vr::HmdMatrix34_t transform = mat->toOpenVR34();
    Vector3 translation = mat->getTranslation();

    // Clamp the values that will be used as input to asin and atan2.
    float clamped_m12 = clamp(transform.m[1][2], -1.0f, 1.0f);
    float clamped_m02 = clamp(transform.m[0][2], -1.0f, 1.0f);

    // Extract Euler angles (assuming ZYX rotation order).
    float x = std::asin(-clamped_m12);                      // Pitch: rotation about the X-axis.
    float z = std::atan2(transform.m[1][0], transform.m[1][1]); // Roll: rotation about the Z-axis.
    float y = std::atan2(clamped_m02, transform.m[2][2]);     // Yaw: rotation about the Y-axis.

    // Convert radians to degrees.
    constexpr float radToDeg = 180.0f / 3.14159265358979323846f;
    constexpr float degToRad = 3.14159265358979323846f / 180.0f;
    x *= radToDeg;
    y *= radToDeg;
    z *= radToDeg;

    // Define a lambda to snap an angle to the nearest multiple of 'degrees'.
    auto snapToNearest = [degrees](float angle) {
        return round(angle / degrees) * degrees;
        };

    // Conditionally snap the angles based on the provided parameters.
    if (snapX) { x = snapToNearest(x); }
    if (snapY) { y = snapToNearest(y); }
    if (snapZ) { z = snapToNearest(z); }

    // Convert angles back to radians.
    x *= degToRad;
    y *= degToRad;
    z *= degToRad;

    // Compute cosine and sine for each angle.
    float cy = std::cos(y);
    float sy = std::sin(y);
    float cp = std::cos(x);
    float sp = std::sin(x);
    float cr = std::cos(z);
    float sr = std::sin(z);

    // Refill the rotation portion of the matrix.
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

    // Update the matrix.
    mat->set(Matrix4(transform));
    mat->setTranslation(translation);
}
