// This code was auto-generated. Do not modify it.

#include "Engine/Scripting/BinaryModule.h"
#include "MotionMatching.Gen.h"

StaticallyLinkedBinaryModuleInitializer StaticallyLinkedBinaryModuleMotionMatching(GetBinaryModuleMotionMatching);

extern "C" BinaryModule* GetBinaryModuleMotionMatching()
{
    static NativeBinaryModule module("MotionMatching");
    return &module;
}
