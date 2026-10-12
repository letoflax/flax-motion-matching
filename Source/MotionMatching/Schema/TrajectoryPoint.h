#pragma once

#include "Engine/Core/Math/Vector3.h"
#include "Engine/Scripting/ScriptingType.h"

/// <summary>
/// Data-only description of one trajectory point in the pose-search schema.
/// </summary>
API_STRUCT(Namespace="MotionMatching")
struct MOTIONMATCHING_API TrajectoryPoint
{
    DECLARE_SCRIPTING_TYPE_MINIMAL(TrajectoryPoint);

    // Planar position in local space (X = Right, Z = Forward)
    API_FIELD() Float3 Position = Float3::Zero;

    // Planar facing direction in local space (defaults to forward)
    API_FIELD() Float3 Facing = Float3::Forward;
};

template<>
struct TIsPODType<TrajectoryPoint>
{
    enum { Value = true };
};