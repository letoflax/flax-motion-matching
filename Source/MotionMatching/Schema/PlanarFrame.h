#pragma once

#include "Engine/Core/Math/Math.h"
#include "Engine/Core/Math/Vector2.h"
#include "Engine/Core/Math/Vector3.h"

/// <summary>
/// Ground plane of the model space, used where the plugin reads root motion
/// in model space (bake yaw-rate channel, clip turn angles, clip travel
/// speed). The plane normal ("planar up") is the model-space image of the
/// schema RootUpAxis, derived at bake time from the root's bind pose and
/// stored in the database. A Y-up model (planar up exactly +Y) keeps the
/// original (x, z) projection bit for bit; any other up axis projects onto
/// an orthonormal basis of its plane with the same handedness.
/// </summary>
namespace PlanarFrame
{
    /// <summary>The Y-up model frame: planar up is exactly (0, 1, 0).</summary>
    inline bool IsYUp(const Float3& up)
    {
        return up.X == 0.0f && up.Y == 1.0f && up.Z == 0.0f;
    }

    /// <summary>
    /// Normalizes an axis and snaps it to the nearest cardinal axis when it
    /// lies within ~2.5 degrees of it (bind poses carry float noise from
    /// import conversions). Returns +Y for a degenerate axis.
    /// </summary>
    inline Float3 SnapAxis(const Float3& axis)
    {
        Float3 n = axis;
        const float lengthSquared = n.LengthSquared();
        if (!(lengthSquared > 1e-12f))
            return Float3(0.0f, 1.0f, 0.0f);
        n /= Math::Sqrt(lengthSquared);
        const Float3 cardinals[6] = {
            Float3(1.0f, 0.0f, 0.0f), Float3(-1.0f, 0.0f, 0.0f),
            Float3(0.0f, 1.0f, 0.0f), Float3(0.0f, -1.0f, 0.0f),
            Float3(0.0f, 0.0f, 1.0f), Float3(0.0f, 0.0f, -1.0f),
        };
        for (const Float3& cardinal : cardinals)
        {
            if (Float3::Dot(n, cardinal) > 0.999f)
                return cardinal;
        }
        return n;
    }

    /// <summary>
    /// Projects a model-space vector onto the ground plane: (side, forward)
    /// coordinates. For Y-up this is exactly (v.X, v.Z). Otherwise the
    /// forward basis is the model Z (or X when up is near Z) made orthogonal
    /// to up, and side = up x forward (Y x Z = X, matching the Y-up case).
    /// </summary>
    inline Float2 Project(const Float3& v, const Float3& up)
    {
        if (IsYUp(up))
            return Float2(v.X, v.Z);
        const Float3 reference = Math::Abs(up.Z) < 0.9f ? Float3(0.0f, 0.0f, 1.0f) : Float3(1.0f, 0.0f, 0.0f);
        Float3 forward = reference - up * Float3::Dot(reference, up);
        forward.Normalize();
        const Float3 side = Float3::Cross(up, forward);
        return Float2(Float3::Dot(v, side), Float3::Dot(v, forward));
    }
}
