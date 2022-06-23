#pragma once

#include <vector>
#include <cmath>
#include "math.h"
#include "frame.h"
#include "ray.h"
#include "scene.h"
#include "utils.h"

class Material
{
public:
    Material()
    {
        Reset();
    }

    void Reset()
    {
        mDiffuseReflectance = Vec3f(0);
        mPhongReflectance   = Vec3f(0);
        mPhongExponent      = 1.f;
        mMirrorReflectance  = Vec3f(0);
        mIOR = -1.f;
    }

    // diffuse is simply added to the others
    Vec3f mDiffuseReflectance;
    // Phong is simply added to the others
    Vec3f mPhongReflectance;
    float mPhongExponent;

    // mirror can be either simply added, or mixed using Fresnel term
    // this is governed by mIOR, if it is >= 0, fresnel is used, otherwise
    // it is not
    Vec3f mMirrorReflectance;

    // When mIOR >= 0, we also transmit (just clear glass)
    float mIOR;
};
