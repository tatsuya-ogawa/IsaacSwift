//
//  IsaacSwiftPhysics.mm
//  IsaacSwift
//
//  Jolt Physics implementation of an ANYmal-class quadruped on a ground
//  plane. The articulation has a floating base + 4 legs × 3 hinges (HAA / HFE
//  / KFE) driven by PD position motors. Gravity, ground contact, and joint
//  drives are configured to roughly match Isaac Sim's flat-terrain ANYmal
//  setup so a policy → physics → observation loop can run headlessly.
//

#import "IsaacSwiftPhysics.h"

#include <Jolt/Jolt.h>

#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SpringSettings.h>

#include <array>
#include <memory>
#include <cmath>
#include <algorithm>

JPH_SUPPRESS_WARNINGS

namespace {

// MARK: - Layer setup
//
// Two object layers: ground in NON_MOVING, robot links in MOVING. MOVING ↔
// MOVING is disabled, matching Isaac Sim's `self_collision: false`.

namespace Layers {
    static constexpr JPH::ObjectLayer kNonMoving = 0;
    static constexpr JPH::ObjectLayer kMoving    = 1;
    static constexpr JPH::ObjectLayer kInternal  = 2;
}

namespace BPLayers {
    static constexpr JPH::BroadPhaseLayer kNonMoving(0);
    static constexpr JPH::BroadPhaseLayer kMoving(1);
    static constexpr JPH::uint            kCount = 2;
}

class BroadPhaseLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    JPH::uint GetNumBroadPhaseLayers() const override { return BPLayers::kCount; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override {
        return inLayer == Layers::kNonMoving ? BPLayers::kNonMoving : BPLayers::kMoving;
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char *GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override {
        return inLayer == BPLayers::kNonMoving ? "NonMoving" : "Moving";
    }
#endif
};

class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer inObj, JPH::BroadPhaseLayer inBP) const override {
        if (inObj == Layers::kNonMoving) return inBP == BPLayers::kMoving;
        if (inObj == Layers::kMoving)    return inBP == BPLayers::kNonMoving;
        if (inObj == Layers::kInternal)  return false;
        return false;
    }
};

class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override {
        if (a == Layers::kInternal || b == Layers::kInternal) return false;
        if (a == Layers::kMoving && b == Layers::kMoving) return false;
        return a != b;
    }
};

static void InitializeJoltOnce() {
    static dispatch_once_t sToken;
    dispatch_once(&sToken, ^{
        JPH::RegisterDefaultAllocator();
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
    });
}

// MARK: - Robot config (per-kind dimensions, masses, gains)
//
// ANYmal-C, Spot, and Go2 share the same articulation topology
// (floating base + 4 legs × 3 hinges in HAA/HFE/KFE order). Only the
// dimensions, masses, gains, and default standing pose change.

static constexpr int kQuadrupedJointCount = 12;
static constexpr int kH1JointCount = 19;
static constexpr int kMaxJointCount = 19;

struct LegConfig {
    float hipX;
    float hipY;
    int   baseIdx;
};

struct RobotConfig {
    int jointCount;

    // Base
    float baseHalfX;
    float baseHalfY;
    float baseHalfZ;
    float baseMass;
    float baseFriction;

    // Hip link (HAA pivot)
    float hipOffsetX;
    float hipOffsetY;
    float hipLinkHalf;
    float hipLinkMass;
    float hipFriction;
    float hfeOffsetX;
    float hfeOffsetY;
    float hfeOffsetZ;

    // Thigh
    float thighLength;
    float thighRadius;
    float thighMass;
    float thighFriction;
    float kfeOffsetX;

    // Shank / foot
    float shankLength;
    float shankRadius;
    float shankMass;
    float footRadius;
    float footMass;
    float footOffsetX;
    float footOffsetY;
    float footOffsetZ;
    float shankFriction;

    // PD + actuator
    float kp;
    float kd;
    float maxTorque;
    float actionScale;
    float physicsTimeStep;

    // Spawn height (m)
    float spawnZ;

    // Default joint positions in `[LF_HAA, LF_HFE, LF_KFE,
    //                              RF_HAA, RF_HFE, RF_KFE,
    //                              LH_HAA, LH_HFE, LH_KFE,
    //                              RH_HAA, RH_HFE, RH_KFE]` order.
    std::array<float, kMaxJointCount> defaults;
};

// ANYmal-C config — Isaac Lab `LocomotionVelocityRoughEnv` defaults.
// The ANYmal runtime path below uses USD-derived body origins and joint frames;
// when `AnymalActuatorRunner` is installed Jolt hinge motors are disabled and
// the LSTM effort is applied directly as a parent/child torque pair.
static const RobotConfig kAnymalConfig = {
    /*jointCount=*/kQuadrupedJointCount,
    /*baseHalfX=*/0.31f, /*baseHalfY=*/0.15f, /*baseHalfZ=*/0.075f,
    /*baseMass=*/25.0f, /*baseFriction=*/0.6f,
    /*hipOffsetX=*/0.28f, /*hipOffsetY=*/0.105f,
    /*hipLinkHalf=*/0.025f, /*hipLinkMass=*/0.6f, /*hipFriction=*/0.5f,
    /*hfeOffsetX=*/0.0f, /*hfeOffsetY=*/0.0f, /*hfeOffsetZ=*/0.0f,
    /*thighLength=*/0.285f, /*thighRadius=*/0.04f,
    /*thighMass=*/1.5f, /*thighFriction=*/0.6f,
    /*kfeOffsetX=*/0.0f,
    /*shankLength=*/0.285f, /*shankRadius=*/0.03f,
    /*shankMass=*/0.5f, /*footRadius=*/0.04f,
    /*footMass=*/0.25f, /*footOffsetX=*/0.0f,
    /*footOffsetY=*/0.0f, /*footOffsetZ=*/-0.285f,
    /*shankFriction=*/1.2f,
    /*kp=*/240.0f, /*kd=*/12.0f, /*maxTorque=*/120.0f, /*actionScale=*/0.5f,
    /*physicsTimeStep=*/1.0f / 200.0f,
    /*spawnZ=*/0.600f,
    /*defaults=*/{{
        0.0f,  0.4f, -0.8f,
        0.0f,  0.4f, -0.8f,
        0.0f, -0.4f,  0.8f,
        0.0f, -0.4f,  0.8f,
        0, 0, 0, 0, 0, 0, 0,
    }},
};

// Spot config — Isaac Lab `Spot-Flat-v0`-style defaults (Boston Dynamics Spot).
// Approximate masses/dimensions; default pose matches the policy training.
// Joint names are `[fl_hx, fl_hy, fl_kn, fr_hx, fr_hy, fr_kn,
//                    hl_hx, hl_hy, hl_kn, hr_hx, hr_hy, hr_kn]` (leg-major).
// Note that the bundled CoreML policy expects PhysX `dof_names` order
// (type-grouped, FL→FR→HL→HR). The remap is applied by
// `DemoPolicyActionProvider` via `IsaacPolicyRuntimeConfiguration.spotFlat
// .simToPolicyJointPermutation`. Gains/torque/action_scale below are the
// values picked by `spotPolicyJointPermutationAndGainSweep` and match
// Isaac Lab's `spot._action_scale = 0.2`.
static const RobotConfig kSpotConfig = {
    /*jointCount=*/kQuadrupedJointCount,
    /*baseHalfX=*/0.45f, /*baseHalfY=*/0.18f, /*baseHalfZ=*/0.075f,
    /*baseMass=*/30.0f, /*baseFriction=*/0.6f,
    /*hipOffsetX=*/0.29f, /*hipOffsetY=*/0.07f,
    /*hipLinkHalf=*/0.03f, /*hipLinkMass=*/1.0f, /*hipFriction=*/0.5f,
    /*hfeOffsetX=*/0.0f, /*hfeOffsetY=*/0.0f, /*hfeOffsetZ=*/0.0f,
    /*thighLength=*/0.32f, /*thighRadius=*/0.04f,
    /*thighMass=*/2.0f, /*thighFriction=*/0.6f,
    /*kfeOffsetX=*/0.0f,
    /*shankLength=*/0.32f, /*shankRadius=*/0.03f,
    /*shankMass=*/0.7f, /*footRadius=*/0.04f,
    /*footMass=*/0.0f, /*footOffsetX=*/0.0f,
    /*footOffsetY=*/0.0f, /*footOffsetZ=*/0.0f,
    /*shankFriction=*/1.2f,
    /*kp=*/240.0f, /*kd=*/12.0f, /*maxTorque=*/240.0f, /*actionScale=*/0.2f,
    /*physicsTimeStep=*/1.0f / 500.0f,
    /*spawnZ=*/0.500f,
    /*defaults=*/{{
        +0.1f,  0.9f, -1.5f,   // FL
        -0.1f,  0.9f, -1.5f,   // FR
        +0.1f,  1.1f, -1.5f,   // HL
        -0.1f,  1.1f, -1.5f,   // HR
        0, 0, 0, 0, 0, 0, 0,
    }},
};

// Unitree Go2 config — seeded from Isaac Sim 5.1 `Unitree/Go2/go2.usd`.
// The policy is currently a placeholder, so the PD gains are conservative
// Jolt-local values while effort limits and dimensions follow the USD.
static const RobotConfig kGo2Config = {
    /*jointCount=*/kQuadrupedJointCount,
    /*baseHalfX=*/0.1881f, /*baseHalfY=*/0.04675f, /*baseHalfZ=*/0.057f,
    /*baseMass=*/6.921f, /*baseFriction=*/0.6f,
    /*hipOffsetX=*/0.1934f, /*hipOffsetY=*/0.0465f,
    /*hipLinkHalf=*/0.035f, /*hipLinkMass=*/0.678f, /*hipFriction=*/0.5f,
    /*hfeOffsetX=*/0.0f, /*hfeOffsetY=*/0.0f, /*hfeOffsetZ=*/0.0f,
    /*thighLength=*/0.213f, /*thighRadius=*/0.026f,
    /*thighMass=*/1.152f, /*thighFriction=*/0.6f,
    /*kfeOffsetX=*/0.0f,
    /*shankLength=*/0.213f, /*shankRadius=*/0.018f,
    /*shankMass=*/0.154f, /*footRadius=*/0.022f,
    /*footMass=*/0.0f, /*footOffsetX=*/0.0f,
    /*footOffsetY=*/0.0f, /*footOffsetZ=*/0.0f,
    /*shankFriction=*/1.2f,
    /*kp=*/120.0f, /*kd=*/6.0f, /*maxTorque=*/45.43f, /*actionScale=*/0.25f,
    /*physicsTimeStep=*/1.0f / 200.0f,
    /*spawnZ=*/0.42f,
    /*defaults=*/{{
        +0.1f, 0.8f, -1.5f,   // FL
        -0.1f, 0.8f, -1.5f,   // FR
        +0.1f, 1.0f, -1.5f,   // RL
        -0.1f, 1.0f, -1.5f,   // RR
        0, 0, 0, 0, 0, 0, 0,
    }},
};

// Unitree H1 config — Isaac Sim `H1FlatTerrainPolicy` defaults. The local
// Jolt hinge order follows the USD file; policy I/O is remapped separately
// to Isaac Sim's PhysX traversal order in `IsaacPolicyRuntimeConfiguration`.
static const RobotConfig kH1Config = {
    /*jointCount=*/kH1JointCount,
    /*baseHalfX=*/0.12f, /*baseHalfY=*/0.09f, /*baseHalfZ=*/0.08f,
    /*baseMass=*/5.39f, /*baseFriction=*/0.8f,
    /*hipOffsetX=*/0.0f, /*hipOffsetY=*/0.0f,
    /*hipLinkHalf=*/0.04f, /*hipLinkMass=*/2.2f, /*hipFriction=*/0.8f,
    /*hfeOffsetX=*/0.0f, /*hfeOffsetY=*/0.0f, /*hfeOffsetZ=*/0.0f,
    /*thighLength=*/0.4f, /*thighRadius=*/0.045f,
    /*thighMass=*/4.152f, /*thighFriction=*/0.8f,
    /*kfeOffsetX=*/0.0f,
    /*shankLength=*/0.4f, /*shankRadius=*/0.035f,
    /*shankMass=*/1.721f, /*footRadius=*/0.05f,
    /*footMass=*/0.474f, /*footOffsetX=*/0.0f,
    /*footOffsetY=*/0.0f, /*footOffsetZ=*/0.0f,
    /*shankFriction=*/1.2f,
    /*kp=*/160.0f, /*kd=*/6.0f, /*maxTorque=*/300.0f, /*actionScale=*/0.5f,
    /*physicsTimeStep=*/1.0f / 200.0f,
    /*spawnZ=*/1.05f,
    /*defaults=*/{{
        0.0f,   // left_hip_yaw
        0.0f,   // right_hip_yaw
        0.0f,   // torso
        0.0f,   // left_hip_roll
       -0.28f,  // left_hip_pitch
        0.79f,  // left_knee
       -0.52f,  // left_ankle
        0.0f,   // right_hip_roll
       -0.28f,  // right_hip_pitch
        0.79f,  // right_knee
       -0.52f,  // right_ankle
        0.28f,  // left_shoulder_pitch
        0.28f,  // right_shoulder_pitch
        0.0f,   // left_shoulder_roll
        0.0f,   // left_shoulder_yaw
        0.52f,  // left_elbow
        0.0f,   // right_shoulder_roll
        0.0f,   // right_shoulder_yaw
        0.52f,  // right_elbow
    }},
};

static const RobotConfig &ConfigForKind(IsaacSwiftRobotKind kind) {
    switch (kind) {
        case IsaacSwiftRobotKindH1:      return kH1Config;
        case IsaacSwiftRobotKindGo2:     return kGo2Config;
        case IsaacSwiftRobotKindSpot:    return kSpotConfig;
        case IsaacSwiftRobotKindAnymalC:
        default:                         return kAnymalConfig;
    }
}

// Legs are positioned identically for both kinds — only the X/Y offsets vary
// via the active config. The simulator builds this lazily from the config.
static std::array<LegConfig, 4> MakeLegConfigs(const RobotConfig &cfg) {
    return {{
        LegConfig{ +cfg.hipOffsetX, +cfg.hipOffsetY, 0  },
        LegConfig{ +cfg.hipOffsetX, -cfg.hipOffsetY, 3  },
        LegConfig{ -cfg.hipOffsetX, +cfg.hipOffsetY, 6  },
        LegConfig{ -cfg.hipOffsetX, -cfg.hipOffsetY, 9  },
    }};
}

static JPH::Vec3 V(float x, float y, float z) { return JPH::Vec3(x, y, z); }
static JPH::Quat Q(float w, float x, float y, float z) { return JPH::Quat(x, y, z, w); }
static float Degrees(float degrees) { return degrees * JPH::JPH_PI / 180.0f; }

enum AnymalUsdBodyIndex : int {
    kUsdBase = 0,
    kUsdLFHip, kUsdLFThigh, kUsdLFShank, kUsdLFFoot,
    kUsdRFHip, kUsdRFThigh, kUsdRFShank, kUsdRFFoot,
    kUsdLHHip, kUsdLHThigh, kUsdLHShank, kUsdLHFoot,
    kUsdRHHip, kUsdRHThigh, kUsdRHShank, kUsdRHFoot,
    kAnymalUsdBodyCount
};

struct AnymalUsdBodyDef {
    const char *name;
    JPH::Vec3 localPosition;
    JPH::Quat localRotation;
    JPH::Vec3 inertialCom;
    float mass;
    float contactRadius;
    bool collides;
};

struct AnymalUsdJointDef {
    int parentBody;
    int childBody;
    int hingeIndex;  // -1 for fixed foot joints.
    int fixedLegIndex;
    JPH::Vec3 localPos0;
    JPH::Quat localRot0;
    JPH::Vec3 localPos1;
    JPH::Quat localRot1;
};

struct UsdBodyPose {
    JPH::RVec3 position;
    JPH::Quat rotation;
};

static const std::array<AnymalUsdBodyDef, kAnymalUsdBodyCount> kAnymalUsdBodies = {{
    {"base",     V(0.0f, 0.0f, 0.0f),                 Q(1, 0, 0, 0),                      V(-0.017793946f, -0.00017816867f, 0.0085043395f), 26.37317f, 0.0f,  true},
    {"LF_HIP",   V(0.2999f, 0.104f, 0.0f),            Q(0.2588190734f, 0.9659258127f, 0, 0), V(0.056660637f, -0.015293974f, -0.008297847f), 2.781f, 0.050f, false},
    {"LF_THIGH", V(0.3597999811f, 0.1878100038f, 0),  Q(0.7071067095f, 0, 0, 0.7071067095f), V(0.030814724f, 0.000046495177f, -0.24569571f), 3.0709999f, 0.055f, false},
    {"LF_SHANK", V(0.3597999811f, 0.288109988f, -0.2849998176f), Q(0.7071066499f, 0, 0, 0.7071066499f), V(0.006859668f, -0.034527674f, 0.0009771042f), 0.33841997f, 0.035f, false},
    {"LF_FOOT",  V(0.4477499425f, 0.3011600077f, -0.6229695082f), Q(0.9999998212f, 0, 0, 0), V(0.00948f, -0.00948f, 0.1468f), 0.25f, 0.040f, true},
    {"RF_HIP",   V(0.2999f, -0.104f, 0.0f),           Q(0.2588190734f, -0.9659258127f, 0, 0), V(0.056763325f, 0.015293974f, -0.008297847f), 2.781f, 0.050f, false},
    {"RF_THIGH", V(0.3597999811f, -0.1878100038f, 0), Q(0.7071067095f, 0, 0, -0.7071067095f), V(0.030814724f, 0.000046503843f, -0.24569571f), 3.0709999f, 0.055f, false},
    {"RF_SHANK", V(0.3597999811f, -0.288109988f, -0.2849998176f), Q(0.7071066499f, 0, 0, -0.7071066499f), V(0.006859668f, 0.034527674f, 0.0009771042f), 0.33841997f, 0.035f, false},
    {"RF_FOOT",  V(0.4477499425f, -0.3011600077f, -0.6229695082f), Q(0.9999998212f, 0, 0, 0), V(0.00948f, 0.00948f, 0.1468f), 0.25f, 0.040f, true},
    {"LH_HIP",   V(-0.2999f, 0.104f, 0.0f),           Q(-1.131334e-8f, 4.2221959e-8f, 0.9659258127f, -0.2588190734f), V(0.05676332f, 0.015293971f, -0.0082978485f), 2.781f, 0.050f, false},
    {"LH_THIGH", V(-0.3597999811f, 0.1878100038f, 0), Q(-0.7071067691f, -2.9802322e-8f, -3.6976328e-8f, -0.7071067691f), V(0.030814724f, 0.000046495177f, -0.24569571f), 3.0709999f, 0.055f, false},
    {"LH_SHANK", V(-0.3598000109f, 0.288109988f, -0.2849999368f), Q(-0.7071067095f, -2.9802321e-8f, -3.6976324e-8f, -0.7071067095f), V(0.0068596713f, 0.034527674f, 0.0009771042f), 0.33841997f, 0.035f, false},
    {"LH_FOOT",  V(-0.4477500319f, 0.3011599779f, -0.6229697466f), Q(-0.9999998808f, 5.0727866e-9f, -4.7219629e-8f, 0), V(-0.00948f, -0.00948f, 0.1468f), 0.25f, 0.040f, true},
    {"RH_HIP",   V(-0.2999f, -0.104f, 0.0f),          Q(-1.131334e-8f, -4.2221959e-8f, -0.9659258127f, -0.2588190734f), V(0.056660626f, -0.015293977f, -0.008297847f), 2.781f, 0.050f, false},
    {"RH_THIGH", V(-0.3597999811f, -0.1878100038f, 0), Q(-0.7071067095f, -2.9802322e-8f, 3.6976328e-8f, 0.7071067095f), V(0.030814724f, 0.000046503843f, -0.24569571f), 3.0709999f, 0.055f, false},
    {"RH_SHANK", V(-0.3597999513f, -0.288109988f, -0.2849998176f), Q(-0.7071066499f, -2.9802321e-8f, 3.6976324e-8f, 0.7071066499f), V(0.0068596713f, -0.034527674f, 0.0009771042f), 0.33841997f, 0.035f, false},
    {"RH_FOOT",  V(-0.4477499127f, -0.3011599481f, -0.6229695082f), Q(-0.9999998212f, 5.0727866e-9f, 4.7219629e-8f, 0), V(-0.00948f, 0.00948f, 0.1468f), 0.25f, 0.040f, true},
}};

static const std::array<AnymalUsdJointDef, 16> kAnymalUsdJoints = {{
    {kUsdBase,    kUsdLFHip,   0, -1, V(0.2999f, 0.104f, 0),                 Q(0.25881907f, 0.9659258f, 0, 0),                       V(0, 0, 0), Q(1, 0, 0, 0)},
    {kUsdLFHip,   kUsdLFThigh, 1, -1, V(0.059899993f, -0.07258159f, -0.041905005f), Q(0.18301272f, -0.68301266f, 0.68301266f, 0.18301272f), V(0, 0, 0), Q(1, 0, 0, 0)},
    {kUsdLFThigh, kUsdLFShank, 2, -1, V(0.10029999f, -5.978346e-9f, -0.28499994f), Q(0.99999994f, 0, 0, 0),                         V(0, 0, 0), Q(1, 0, 0, 0)},
    {kUsdLFShank, kUsdLFFoot, -1,  0, V(0.013049994f, -0.08795f, -0.33796993f), Q(0.70710677f, 0, 0, -0.70710677f),                 V(0, 0, 0), Q(1, 0, 0, 0)},
    {kUsdBase,    kUsdRFHip,   3, -1, V(0.2999f, -0.104f, 0),                Q(0.25881907f, -0.9659258f, 0, 0),                      V(0, 0, 0), Q(1, 0, 0, 0)},
    {kUsdRFHip,   kUsdRFThigh, 4, -1, V(0.059899993f, 0.07258159f, -0.041905005f), Q(-0.68301266f, 0.1830127f, 0.1830127f, 0.68301266f), V(0, 0, 0), Q(-4.371139e-8f, 0, 1, 0)},
    {kUsdRFThigh, kUsdRFShank, 5, -1, V(0.10029999f, 5.978346e-9f, -0.28499994f), Q(-4.3711385e-8f, 0, 0.99999994f, 0),             V(0, 0, 0), Q(-4.371139e-8f, 0, 1, 0)},
    {kUsdRFShank, kUsdRFFoot, -1,  1, V(0.013049994f, 0.08795f, -0.33796993f), Q(0.70710677f, 0, 0, 0.70710677f),                  V(0, 0, 0), Q(1, 0, 0, 0)},
    {kUsdBase,    kUsdLHHip,   6, -1, V(-0.2999f, 0.104f, 0),                Q(-0.9659258f, 0.25881907f, -5.3535302e-8f, 5.3535302e-8f), V(0, 0, 0), Q(-4.371139e-8f, 0, 1, 0)},
    {kUsdLHHip,   kUsdLHThigh, 7, -1, V(0.05990001f, 0.07258159f, -0.041905005f), Q(0.18301271f, 0.6830127f, 0.6830126f, -0.18301274f), V(0, 0, 0), Q(1, 0, 0, 0)},
    {kUsdLHThigh, kUsdLHShank, 8, -1, V(0.10029999f, -5.978346e-9f, -0.28499994f), Q(0.99999994f, 0, 0, 0),                        V(0, 0, 0), Q(1, 0, 0, 0)},
    {kUsdLHShank, kUsdLHFoot, -1,  2, V(0.013050005f, 0.08795f, -0.33796993f), Q(0.70710677f, 0, 0, -0.70710677f),                 V(0, 0, 0), Q(1, 0, 0, 0)},
    {kUsdBase,    kUsdRHHip,   9, -1, V(-0.2999f, -0.104f, 0),               Q(0.9659258f, 0.25881907f, 3.0908616e-8f, -3.0908616e-8f), V(0, 0, 0), Q(-4.371139e-8f, 0, 1, 0)},
    {kUsdRHHip,   kUsdRHThigh,10, -1, V(0.059899993f, -0.07258159f, -0.041905005f), Q(0.6830127f, 0.18301268f, -0.18301271f, 0.6830126f), V(0, 0, 0), Q(-4.371139e-8f, 0, 1, 0)},
    {kUsdRHThigh, kUsdRHShank,11, -1, V(0.10029999f, 5.978346e-9f, -0.28499994f), Q(-4.3711385e-8f, 0, 0.99999994f, 0),            V(0, 0, 0), Q(-4.371139e-8f, 0, 1, 0)},
    {kUsdRHShank, kUsdRHFoot, -1,  3, V(0.013050005f, -0.08795f, -0.33796993f), Q(0.70710677f, 0, 0, 0.70710677f),                 V(0, 0, 0), Q(1, 0, 0, 0)},
}};

static JPH::Vec3 AnymalUsdInertialHalfExtents(int bodyIndex) {
    switch (bodyIndex) {
        case kUsdLFHip: case kUsdRFHip: case kUsdLHHip: case kUsdRHHip:
            return JPH::Vec3(0.070f, 0.045f, 0.045f);
        case kUsdLFThigh: case kUsdRFThigh: case kUsdLHThigh: case kUsdRHThigh:
            return JPH::Vec3(0.055f, 0.045f, 0.170f);
        case kUsdLFShank: case kUsdRFShank: case kUsdLHShank: case kUsdRHShank:
            return JPH::Vec3(0.040f, 0.040f, 0.190f);
        default:
            return JPH::Vec3::sReplicate(0.050f);
    }
}

enum SpotUsdBodyIndex : int {
    kSpotUsdBase = 0,
    kSpotUsdFLHip, kSpotUsdFLUleg, kSpotUsdFLLleg, kSpotUsdFLFoot,
    kSpotUsdFRHip, kSpotUsdFRUleg, kSpotUsdFRLleg, kSpotUsdFRFoot,
    kSpotUsdHLHip, kSpotUsdHLUleg, kSpotUsdHLLleg, kSpotUsdHLFoot,
    kSpotUsdHRHip, kSpotUsdHRUleg, kSpotUsdHRLleg, kSpotUsdHRFoot,
    kSpotUsdBodyCount
};

struct SpotUsdBodyDef {
    const char *name;
    JPH::Vec3 inertialCom;
    float mass;
    float contactRadius;
    bool collides;
};

struct SpotUsdJointDef {
    int parentBody;
    int childBody;
    int hingeIndex;  // -1 for fixed foot joints.
    int fixedLegIndex;
    JPH::Vec3 localPos0;
    JPH::Vec3 localPos1;
    JPH::Vec3 axis;
};

struct SpotUsdLegDef {
    int hipBody;
    int ulegBody;
    int llegBody;
    int footBody;
    int baseJointIndex;
    JPH::Vec3 hipOffset;
    JPH::Vec3 ulegOffset;
};

static const std::array<SpotUsdBodyDef, kSpotUsdBodyCount> kSpotUsdBodies = {{
    {"body",    V(0.0f, 0.0f, -0.00496172f), 16.707651f, 0.0f,  true},
    {"fl_hip",  V(-0.01586739f, 0.00855842f, 0.00000903f), 1.1368834f, 0.050f, false},
    {"fl_uleg", V(0.00214442f, -0.01110184f, -0.07881204f), 2.2562037f, 0.055f, false},
    {"fl_lleg", V(0.0059736f, 0.0f, -0.17466427f), 0.33f, 0.035f, false},
    {"fl_foot", V(0.0f, 0.0f, 0.0f), 0.05f, 0.030f, true},
    {"fr_hip",  V(-0.01586739f, -0.00855842f, 0.00000903f), 1.1368834f, 0.050f, false},
    {"fr_uleg", V(0.00214442f, 0.01110184f, -0.07881204f), 2.2562037f, 0.055f, false},
    {"fr_lleg", V(0.0059736f, 0.0f, -0.17466427f), 0.33f, 0.035f, false},
    {"fr_foot", V(0.0f, 0.0f, 0.0f), 0.05f, 0.030f, true},
    {"hl_hip",  V(0.01586739f, 0.00855842f, 0.00000903f), 1.1368834f, 0.050f, false},
    {"hl_uleg", V(0.00214442f, -0.01110184f, -0.07881204f), 2.2562037f, 0.055f, false},
    {"hl_lleg", V(0.0059736f, 0.0f, -0.17466427f), 0.33f, 0.035f, false},
    {"hl_foot", V(0.0f, 0.0f, 0.0f), 0.05f, 0.030f, true},
    {"hr_hip",  V(0.01586739f, -0.00855842f, 0.00000903f), 1.1368834f, 0.050f, false},
    {"hr_uleg", V(0.00214442f, 0.01110184f, -0.07881204f), 2.2562037f, 0.055f, false},
    {"hr_lleg", V(0.0059736f, 0.0f, -0.17466427f), 0.33f, 0.035f, false},
    {"hr_foot", V(0.0f, 0.0f, 0.0f), 0.05f, 0.030f, true},
}};

static const std::array<SpotUsdLegDef, 4> kSpotUsdLegs = {{
    {kSpotUsdFLHip, kSpotUsdFLUleg, kSpotUsdFLLleg, kSpotUsdFLFoot, 0, V(0.29785f, 0.055f, 0.0f), V(0.0f, 0.110945f, 0.0f)},
    {kSpotUsdFRHip, kSpotUsdFRUleg, kSpotUsdFRLleg, kSpotUsdFRFoot, 3, V(0.29785f, -0.055f, 0.0f), V(0.0f, -0.110945f, 0.0f)},
    {kSpotUsdHLHip, kSpotUsdHLUleg, kSpotUsdHLLleg, kSpotUsdHLFoot, 6, V(-0.29785f, 0.055f, 0.0f), V(0.0f, 0.110945f, 0.0f)},
    {kSpotUsdHRHip, kSpotUsdHRUleg, kSpotUsdHRLleg, kSpotUsdHRFoot, 9, V(-0.29785f, -0.055f, 0.0f), V(0.0f, -0.110945f, 0.0f)},
}};

static const std::array<SpotUsdJointDef, 16> kSpotUsdJoints = {{
    {kSpotUsdBase,   kSpotUsdFLHip,  0, -1, V(0.29785f, 0.055f, 0.0f), V(0, 0, 0), JPH::Vec3::sAxisX()},
    {kSpotUsdFLHip,  kSpotUsdFLUleg, 1, -1, V(0.0f, 0.110945f, 0.0f), V(0, 0, 0), JPH::Vec3::sAxisY()},
    {kSpotUsdFLUleg, kSpotUsdFLLleg, 2, -1, V(0.025f, 0.0f, -0.3205f), V(0, 0, 0), JPH::Vec3::sAxisY()},
    {kSpotUsdFLLleg, kSpotUsdFLFoot,-1,  0, V(0.0f, 0.0f, -0.3365f), V(0, 0, 0), JPH::Vec3::sAxisX()},
    {kSpotUsdBase,   kSpotUsdFRHip,  3, -1, V(0.29785f, -0.055f, 0.0f), V(0, 0, 0), JPH::Vec3::sAxisX()},
    {kSpotUsdFRHip,  kSpotUsdFRUleg, 4, -1, V(0.0f, -0.110945f, 0.0f), V(0, 0, 0), JPH::Vec3::sAxisY()},
    {kSpotUsdFRUleg, kSpotUsdFRLleg, 5, -1, V(0.025f, 0.0f, -0.3205f), V(0, 0, 0), JPH::Vec3::sAxisY()},
    {kSpotUsdFRLleg, kSpotUsdFRFoot,-1,  1, V(0.0f, 0.0f, -0.3365f), V(0, 0, 0), JPH::Vec3::sAxisX()},
    {kSpotUsdBase,   kSpotUsdHLHip,  6, -1, V(-0.29785f, 0.055f, 0.0f), V(0, 0, 0), JPH::Vec3::sAxisX()},
    {kSpotUsdHLHip,  kSpotUsdHLUleg, 7, -1, V(0.0f, 0.110945f, 0.0f), V(0, 0, 0), JPH::Vec3::sAxisY()},
    {kSpotUsdHLUleg, kSpotUsdHLLleg, 8, -1, V(0.025f, 0.0f, -0.3205f), V(0, 0, 0), JPH::Vec3::sAxisY()},
    {kSpotUsdHLLleg, kSpotUsdHLFoot,-1,  2, V(0.0f, 0.0f, -0.3365f), V(0, 0, 0), JPH::Vec3::sAxisX()},
    {kSpotUsdBase,   kSpotUsdHRHip,  9, -1, V(-0.29785f, -0.055f, 0.0f), V(0, 0, 0), JPH::Vec3::sAxisX()},
    {kSpotUsdHRHip,  kSpotUsdHRUleg,10, -1, V(0.0f, -0.110945f, 0.0f), V(0, 0, 0), JPH::Vec3::sAxisY()},
    {kSpotUsdHRUleg, kSpotUsdHRLleg,11, -1, V(0.025f, 0.0f, -0.3205f), V(0, 0, 0), JPH::Vec3::sAxisY()},
    {kSpotUsdHRLleg, kSpotUsdHRFoot,-1,  3, V(0.0f, 0.0f, -0.3365f), V(0, 0, 0), JPH::Vec3::sAxisX()},
}};

static JPH::Vec3 SpotUsdInertialHalfExtents(int bodyIndex) {
    switch (bodyIndex) {
        case kSpotUsdFLHip: case kSpotUsdFRHip: case kSpotUsdHLHip: case kSpotUsdHRHip:
            return JPH::Vec3(0.055f, 0.045f, 0.045f);
        case kSpotUsdFLUleg: case kSpotUsdFRUleg: case kSpotUsdHLUleg: case kSpotUsdHRUleg:
            return JPH::Vec3(0.045f, 0.045f, 0.175f);
        case kSpotUsdFLLleg: case kSpotUsdFRLleg: case kSpotUsdHLLleg: case kSpotUsdHRLleg:
            return JPH::Vec3(0.035f, 0.035f, 0.185f);
        default:
            return JPH::Vec3::sReplicate(0.040f);
    }
}

static std::array<UsdBodyPose, kSpotUsdBodyCount> MakeSpotUsdBodyPoses(const std::array<float, kMaxJointCount> &defaults,
                                                                       JPH::RVec3 spawn) {
    std::array<UsdBodyPose, kSpotUsdBodyCount> poses{};
    poses[kSpotUsdBase] = UsdBodyPose{spawn, JPH::Quat::sIdentity()};

    for (const SpotUsdLegDef &leg : kSpotUsdLegs) {
        const float hx = defaults[leg.baseJointIndex + 0];
        const float hy = defaults[leg.baseJointIndex + 1];
        const float kn = defaults[leg.baseJointIndex + 2];

        const JPH::RVec3 hipPos = spawn + JPH::RVec3(leg.hipOffset);
        const JPH::Quat hipRot = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), hx);
        poses[leg.hipBody] = UsdBodyPose{hipPos, hipRot};

        const JPH::RVec3 ulegPos = hipPos + JPH::RVec3(hipRot * leg.ulegOffset);
        const JPH::Quat ulegRot = hipRot * JPH::Quat::sRotation(JPH::Vec3::sAxisY(), hy);
        poses[leg.ulegBody] = UsdBodyPose{ulegPos, ulegRot};

        const JPH::RVec3 llegPos = ulegPos + JPH::RVec3(ulegRot * JPH::Vec3(0.025f, 0.0f, -0.3205f));
        const JPH::Quat llegRot = ulegRot * JPH::Quat::sRotation(JPH::Vec3::sAxisY(), kn);
        poses[leg.llegBody] = UsdBodyPose{llegPos, llegRot};

        const JPH::RVec3 footPos = llegPos + JPH::RVec3(llegRot * JPH::Vec3(0.0f, 0.0f, -0.3365f));
        poses[leg.footBody] = UsdBodyPose{footPos, llegRot};
    }

    return poses;
}

enum H1BodyIndex : int {
    kH1Pelvis = 0,
    kH1LeftHipYaw, kH1RightHipYaw, kH1Torso,
    kH1LeftHipRoll, kH1LeftHipPitch, kH1LeftKnee, kH1LeftAnkle,
    kH1RightHipRoll, kH1RightHipPitch, kH1RightKnee, kH1RightAnkle,
    kH1LeftShoulderPitch, kH1RightShoulderPitch,
    kH1LeftShoulderRoll, kH1LeftShoulderYaw, kH1LeftElbow,
    kH1RightShoulderRoll, kH1RightShoulderYaw, kH1RightElbow,
    kH1BodyCount
};

struct H1BodyDef {
    const char *name;
    JPH::Vec3 inertialCom;
    float mass;
    JPH::Vec3 inertiaDiag;
    JPH::Quat principalAxes;
    JPH::Vec3 collisionCenter;
    JPH::Vec3 halfExtents;
    float friction;
    bool collides;
};

struct H1JointDef {
    int parentBody;
    int childBody;
    int hingeIndex;
    JPH::Vec3 localPos0;
    JPH::Quat localRot0;
    JPH::Vec3 localPos1;
    JPH::Quat localRot1;
    JPH::Vec3 axis;
};

static const std::array<H1BodyDef, kH1BodyCount> kH1Bodies = {{
    {"pelvis", V(-0.0002f, 0.00004f, -0.04522f), 5.39f, V(0.04458212f, 0.008246193f, 0.049021088f), Q(0.9999968f, -0.000047209065f, -0.0022397172f, 0.0011977674f), V(0, 0, 0), V(0.05f, 0.05f, 0.05f), 0.8f, true},
    {"left_hip_yaw_link", V(-0.04923f, 0.0001f, 0.0072f), 2.244f, V(0.0029688515f, 0.0030449356f, 0.0018920124f), Q(0.94657356f, -0.00996608f, 0.31995144f, -0.03911884f), V(-0.04923f, 0.0001f, 0.0072f), V(0.07f, 0.045f, 0.045f), 0.8f, false},
    {"right_hip_yaw_link", V(-0.04923f, -0.0001f, 0.0072f), 2.244f, V(0.0029688515f, 0.0030449356f, 0.0018920124f), Q(0.94657356f, 0.00996608f, 0.31995144f, 0.03911884f), V(-0.04923f, -0.0001f, 0.0072f), V(0.07f, 0.045f, 0.045f), 0.8f, false},
    {"torso_link", V(0.000489f, 0.002797f, 0.20484f), 17.789f, V(0.48731524f, 0.40962818f, 0.1278366f), Q(0.99998915f, -0.0013081023f, -0.0028228867f, -0.003491047f), V(0, 0, 0.15f), V(0.04f, 0.08f, 0.05f), 0.8f, true},
    {"left_hip_roll_link", V(-0.0058f, -0.00319f, -0.00009f), 2.232f, V(0.0020549176f, 0.002253245f, 0.002432637f), Q(0.9963683f, 0.020564958f, 0.0037783533f, -0.08254194f), V(-0.0058f, -0.00319f, -0.00009f), V(0.055f, 0.045f, 0.045f), 0.8f, false},
    {"left_hip_pitch_link", V(0.00746f, -0.02346f, -0.08193f), 4.152f, V(0.08295033f, 0.08214567f, 0.0051090913f), Q(0.979828f, 0.051352248f, -0.016985333f, -0.19238399f), V(0.00746f, -0.02346f, -0.08193f), V(0.06f, 0.055f, 0.20f), 0.8f, false},
    {"left_knee_link", V(-0.00136f, -0.00512f, -0.1384f), 1.721f, V(0.012310395f, 0.012523702f, 0.0019428049f), Q(0.99276686f, 0.0052330783f, -0.05363738f, 0.10728284f), V(-0.00136f, -0.00512f, -0.1384f), V(0.045f, 0.045f, 0.20f), 0.8f, false},
    {"left_ankle_link", V(0.042575f, -0.000001f, -0.044672f), 0.474f, V(0.00015216829f, 0.002900286f, 0.0028129376f), Q(0.9996474f, 0.000078506244f, 0.026554713f, -0.0000010414184f), V(0.05f, 0, -0.05f), V(0.14f, 0.015f, 0.012f), 0.8f, true},
    {"right_hip_roll_link", V(-0.0058f, 0.00319f, -0.00009f), 2.232f, V(0.0020549176f, 0.002253245f, 0.002432637f), Q(0.9963683f, -0.020564958f, 0.0037783533f, 0.08254194f), V(-0.0058f, 0.00319f, -0.00009f), V(0.055f, 0.045f, 0.045f), 0.8f, false},
    {"right_hip_pitch_link", V(0.00746f, 0.02346f, -0.08193f), 4.152f, V(0.08295033f, 0.08214567f, 0.0051090913f), Q(0.979828f, -0.051352248f, -0.016985333f, 0.19238399f), V(0.00746f, 0.02346f, -0.08193f), V(0.06f, 0.055f, 0.20f), 0.8f, false},
    {"right_knee_link", V(-0.00136f, 0.00512f, -0.1384f), 1.721f, V(0.012310395f, 0.012523702f, 0.0019428049f), Q(0.99276686f, -0.0052330783f, -0.05363738f, -0.10728284f), V(-0.00136f, 0.00512f, -0.1384f), V(0.045f, 0.045f, 0.20f), 0.8f, false},
    {"right_ankle_link", V(0.042575f, 0.000001f, -0.044672f), 0.474f, V(0.00015216829f, 0.002900286f, 0.0028129376f), Q(0.9996474f, -0.000078506244f, 0.026554713f, 0.0000010414184f), V(0.05f, 0, -0.05f), V(0.14f, 0.015f, 0.012f), 0.8f, true},
    {"left_shoulder_pitch_link", V(0.005045f, 0.053657f, -0.015715f), 1.033f, V(0.0012993595f, 0.00085819757f, 0.000987113f), Q(0.9857733f, -0.16661035f, -0.0075959545f, -0.020839892f), V(0.005045f, 0.053657f, -0.015715f), V(0.055f, 0.045f, 0.08f), 0.6f, false},
    {"right_shoulder_pitch_link", V(0.005045f, -0.053657f, -0.015715f), 1.033f, V(0.0012993595f, 0.00085819757f, 0.000987113f), Q(0.9857733f, 0.16661035f, -0.0075959545f, 0.020839892f), V(0.005045f, -0.053657f, -0.015715f), V(0.055f, 0.045f, 0.08f), 0.6f, false},
    {"left_shoulder_roll_link", V(0.000679f, 0.00115f, -0.094076f), 0.793f, V(0.0015825586f, 0.0017038822f, 0.0010033592f), Q(0.99622494f, -0.047725365f, 0.060696196f, -0.039674755f), V(0.000679f, 0.00115f, -0.094076f), V(0.045f, 0.04f, 0.13f), 0.6f, false},
    {"left_shoulder_yaw_link", V(0.01365f, 0.002767f, -0.16266f), 0.839f, V(0.0037036669f, 0.004080375f, 0.0006226872f), Q(0.9983215f, 0.010050264f, -0.056893215f, 0.0040697176f), V(0.01365f, 0.002767f, -0.16266f), V(0.045f, 0.04f, 0.18f), 0.6f, false},
    {"left_elbow_link", V(0.164862f, 0.000118f, -0.015734f), 0.723f, V(0.00040830488f, 0.0060057878f, 0.0060182875f), Q(0.99308085f, -0.11443613f, 0.025614899f, 0.006214754f), V(0.164862f, 0.000118f, -0.015734f), V(0.18f, 0.035f, 0.04f), 0.6f, false},
    {"right_shoulder_roll_link", V(0.000679f, -0.00115f, -0.094076f), 0.793f, V(0.0015825586f, 0.0017038822f, 0.0010033592f), Q(0.99622494f, 0.047725365f, 0.060696196f, 0.039674755f), V(0.000679f, -0.00115f, -0.094076f), V(0.045f, 0.04f, 0.13f), 0.6f, false},
    {"right_shoulder_yaw_link", V(0.01365f, -0.002767f, -0.16266f), 0.839f, V(0.0037036669f, 0.004080375f, 0.0006226872f), Q(0.9983215f, -0.010050264f, -0.056893215f, -0.0040697176f), V(0.01365f, -0.002767f, -0.16266f), V(0.045f, 0.04f, 0.18f), 0.6f, false},
    {"right_elbow_link", V(0.164862f, -0.000118f, -0.015734f), 0.723f, V(0.00040830488f, 0.0060057878f, 0.0060182875f), Q(0.99308085f, 0.11443613f, 0.025614899f, -0.006214754f), V(0.164862f, -0.000118f, -0.015734f), V(0.18f, 0.035f, 0.04f), 0.6f, false},
}};

static const std::array<H1JointDef, kH1JointCount> kH1Joints = {{
    {kH1Pelvis, kH1LeftHipYaw, 0, V(0, 0.0875f, -0.1742f), Q(1, 0, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisZ()},
    {kH1Pelvis, kH1RightHipYaw, 1, V(0, -0.0875f, -0.1742f), Q(1, 0, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisZ()},
    {kH1Pelvis, kH1Torso, 2, V(0, 0, 0), Q(1, 0, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisZ()},
    {kH1LeftHipYaw, kH1LeftHipRoll, 3, V(0.039468f, 0, 0), Q(1, 0, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisX()},
    {kH1LeftHipRoll, kH1LeftHipPitch, 4, V(0, 0.11536f, 0), Q(1, 0, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisY()},
    {kH1LeftHipPitch, kH1LeftKnee, 5, V(0, 0, -0.4f), Q(1, 0, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisY()},
    {kH1LeftKnee, kH1LeftAnkle, 6, V(0, 0, -0.4f), Q(1, 0, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisY()},
    {kH1RightHipYaw, kH1RightHipRoll, 7, V(0.039468f, 0, 0), Q(1, 0, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisX()},
    {kH1RightHipRoll, kH1RightHipPitch, 8, V(0, -0.11536f, 0), Q(1, 0, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisY()},
    {kH1RightHipPitch, kH1RightKnee, 9, V(0, 0, -0.4f), Q(1, 0, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisY()},
    {kH1RightKnee, kH1RightAnkle, 10, V(0, 0, -0.4f), Q(1, 0, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisY()},
    {kH1Torso, kH1LeftShoulderPitch, 11, V(0.0055f, 0.15535f, 0.42999f), Q(0.97629625f, 0.21643849f, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisY()},
    {kH1Torso, kH1RightShoulderPitch, 12, V(0.0055f, -0.15535f, 0.42999f), Q(0.97629625f, -0.21643849f, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisY()},
    {kH1LeftShoulderPitch, kH1LeftShoulderRoll, 13, V(-0.0055f, 0.0565f, -0.0165f), Q(0.97629625f, -0.21643849f, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisX()},
    {kH1LeftShoulderRoll, kH1LeftShoulderYaw, 14, V(0, 0, -0.1343f), Q(1, 0, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisZ()},
    {kH1LeftShoulderYaw, kH1LeftElbow, 15, V(0.0185f, 0, -0.198f), Q(1, 0, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisY()},
    {kH1RightShoulderPitch, kH1RightShoulderRoll, 16, V(-0.0055f, -0.0565f, -0.0165f), Q(0.97629625f, 0.21643849f, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisX()},
    {kH1RightShoulderRoll, kH1RightShoulderYaw, 17, V(0, 0, -0.1343f), Q(1, 0, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisZ()},
    {kH1RightShoulderYaw, kH1RightElbow, 18, V(0.0185f, 0, -0.198f), Q(1, 0, 0, 0), V(0, 0, 0), Q(1, 0, 0, 0), JPH::Vec3::sAxisY()},
}};

// H1's Isaac Lab environment overrides the USD's generic drives with
// actuator groups: strong legs/torso, compliant ankles, and damped arms.
// Keeping one global PD drive across all 19 joints makes the ankles and arms
// overreact to the flat-terrain policy and tips the humanoid immediately.
static const std::array<float, kH1JointCount> kH1JointStiffness = {{
    150.0f, 150.0f, 200.0f,
    150.0f, 200.0f, 200.0f, 20.0f,
    150.0f, 200.0f, 200.0f, 20.0f,
    40.0f, 40.0f, 40.0f, 40.0f, 40.0f,
    40.0f, 40.0f, 40.0f,
}};

static const std::array<float, kH1JointCount> kH1JointDamping = {{
    5.0f, 5.0f, 5.0f,
    5.0f, 5.0f, 5.0f, 4.0f,
    5.0f, 5.0f, 5.0f, 4.0f,
    10.0f, 10.0f, 10.0f, 10.0f, 10.0f,
    10.0f, 10.0f, 10.0f,
}};

static const std::array<float, kH1JointCount> kH1JointMaxTorque = {{
    300.0f, 300.0f, 300.0f,
    300.0f, 300.0f, 300.0f, 100.0f,
    300.0f, 300.0f, 300.0f, 100.0f,
    300.0f, 300.0f, 300.0f, 300.0f, 300.0f,
    300.0f, 300.0f, 300.0f,
}};

static const std::array<float, kH1JointCount> kH1JointLowerLimit = {{
    Degrees(-24.637184f),
    Degrees(-24.637184f),
    Degrees(-134.64507f),
    Degrees(-24.637184f),
    Degrees(-179.90874f),
    Degrees(-14.896901f),
    Degrees(-49.847324f),
    Degrees(-24.637184f),
    Degrees(-179.90874f),
    Degrees(-14.896901f),
    Degrees(-49.847324f),
    Degrees(-164.43887f),
    Degrees(-164.43887f),
    Degrees(-19.480564f),
    Degrees(-74.484505f),
    Degrees(-71.61972f),
    Degrees(-178.18987f),
    Degrees(-254.96619f),
    Degrees(-71.61972f),
}};

static const std::array<float, kH1JointCount> kH1JointUpperLimit = {{
    Degrees(24.637184f),
    Degrees(24.637184f),
    Degrees(134.64507f),
    Degrees(24.637184f),
    Degrees(144.95831f),
    Degrees(117.45634f),
    Degrees(29.793802f),
    Degrees(24.637184f),
    Degrees(144.95831f),
    Degrees(117.45634f),
    Degrees(29.793802f),
    Degrees(164.43887f),
    Degrees(164.43887f),
    Degrees(178.18987f),
    Degrees(254.96619f),
    Degrees(149.54198f),
    Degrees(19.480564f),
    Degrees(74.484505f),
    Degrees(149.54198f),
}};

static float JointStiffnessForKind(IsaacSwiftRobotKind kind, int jointIndex, float fallback) {
    if (kind == IsaacSwiftRobotKindH1 && jointIndex >= 0 && jointIndex < kH1JointCount) {
        return kH1JointStiffness[jointIndex];
    }
    return fallback;
}

static float JointDampingForKind(IsaacSwiftRobotKind kind, int jointIndex, float fallback) {
    if (kind == IsaacSwiftRobotKindH1 && jointIndex >= 0 && jointIndex < kH1JointCount) {
        return kH1JointDamping[jointIndex];
    }
    return fallback;
}

static float JointMaxTorqueForKind(IsaacSwiftRobotKind kind, int jointIndex, float fallback) {
    if (kind == IsaacSwiftRobotKindH1 && jointIndex >= 0 && jointIndex < kH1JointCount) {
        return kH1JointMaxTorque[jointIndex];
    }
    return fallback;
}

static float JointPolicyDirectionForKind(IsaacSwiftRobotKind kind, int jointIndex) {
    (void)kind;
    (void)jointIndex;
    return 1.0f;
}

static float ClampJoltHingeLimit(float limit) {
    return std::clamp(limit, -3.14f, 3.14f);
}

static float JointLowerDeltaLimitForKind(IsaacSwiftRobotKind kind, int jointIndex, float defaultAngle) {
    if (kind == IsaacSwiftRobotKindH1 && jointIndex >= 0 && jointIndex < kH1JointCount) {
        return ClampJoltHingeLimit(kH1JointLowerLimit[jointIndex]);
    }
    (void)defaultAngle;
    return -3.14f;
}

static float JointUpperDeltaLimitForKind(IsaacSwiftRobotKind kind, int jointIndex, float defaultAngle) {
    if (kind == IsaacSwiftRobotKindH1 && jointIndex >= 0 && jointIndex < kH1JointCount) {
        return ClampJoltHingeLimit(kH1JointUpperLimit[jointIndex]);
    }
    (void)defaultAngle;
    return 3.14f;
}

static float ClampJointTargetDeltaForKind(IsaacSwiftRobotKind kind, int jointIndex, float targetDelta, float defaultAngle) {
    const float lower = JointLowerDeltaLimitForKind(kind, jointIndex, defaultAngle);
    const float upper = JointUpperDeltaLimitForKind(kind, jointIndex, defaultAngle);
    return std::clamp(targetDelta, lower, upper);
}

static std::array<UsdBodyPose, kH1BodyCount> MakeH1BodyPoses(const std::array<float, kMaxJointCount> &defaults,
                                                             JPH::RVec3 spawn) {
    std::array<UsdBodyPose, kH1BodyCount> poses{};
    poses[kH1Pelvis] = UsdBodyPose{spawn, JPH::Quat::sIdentity()};

    for (const H1JointDef &joint : kH1Joints) {
        const UsdBodyPose &parent = poses[joint.parentBody];
        const JPH::RVec3 jointPos = parent.position + JPH::RVec3(parent.rotation * joint.localPos0);
        const JPH::Quat jointRot = parent.rotation * joint.localRot0;
        const JPH::Quat childFrameRot = jointRot * JPH::Quat::sRotation(joint.axis, defaults[joint.hingeIndex]);
        const JPH::Quat childRot = childFrameRot * joint.localRot1.Conjugated();
        const JPH::RVec3 childPos = jointPos - JPH::RVec3(childRot * joint.localPos1);
        poses[joint.childBody] = UsdBodyPose{childPos, childRot};
    }

    return poses;
}

static JPH::Vec3 HipToHfeOffset(const RobotConfig &cfg, const LegConfig &leg) {
    // IsaacSim's ANYmal USD places the HFE joint off the HAA pivot instead of
    // co-locating both hip axes. The lateral sign alternates by diagonal pair.
    const float side = (leg.hipX * leg.hipY > 0.0f) ? -1.0f : 1.0f;
    return JPH::Vec3(cfg.hfeOffsetX, side * cfg.hfeOffsetY, cfg.hfeOffsetZ);
}

static JPH::Vec3 ThighToKfeOffset(const RobotConfig &cfg) {
    return JPH::Vec3(cfg.kfeOffsetX, 0.0f, -cfg.thighLength);
}

static JPH::Vec3 ShankToFootOffset(const RobotConfig &cfg, const LegConfig &leg) {
    const float side = (leg.hipY >= 0.0f) ? 1.0f : -1.0f;
    return JPH::Vec3(cfg.footOffsetX, side * cfg.footOffsetY, cfg.footOffsetZ);
}

}  // namespace

// MARK: - Internal sim state

struct AnymalSimState {
    std::unique_ptr<JPH::TempAllocatorImpl>            tempAllocator;
    std::unique_ptr<JPH::JobSystemSingleThreaded>      jobSystem;
    std::unique_ptr<BroadPhaseLayerInterfaceImpl>      broadPhaseLayerInterface;
    std::unique_ptr<ObjectVsBroadPhaseLayerFilterImpl> objectVsBroadPhaseLayerFilter;
    std::unique_ptr<ObjectLayerPairFilterImpl>         objectLayerPairFilter;
    std::unique_ptr<JPH::PhysicsSystem>                physicsSystem;

    JPH::BodyID                                        ground;
    JPH::BodyID                                        base;
    std::array<JPH::BodyID, 4>                         hipLinks{};
    std::array<JPH::BodyID, 4>                         thighs{};
    std::array<JPH::BodyID, 4>                         shanks{};
    std::array<JPH::BodyID, 4>                         feet{};
    bool                                               usesUsdAnymal = false;
    std::array<JPH::BodyID, kAnymalUsdBodyCount>       usdBodies{};
    bool                                               usesUsdSpot = false;
    std::array<JPH::BodyID, kSpotUsdBodyCount>         spotUsdBodies{};
    bool                                               usesUsdH1 = false;
    std::array<JPH::BodyID, kH1BodyCount>              h1Bodies{};
    std::array<JPH::Ref<JPH::HingeConstraint>, kMaxJointCount> hinges{};
    std::array<JPH::Ref<JPH::FixedConstraint>, 4>       footFixedConstraints{};
    // For external-torque actuator path: per-hinge body-local hinge axis (in
    // body1's frame at construction time, when both bodies are at identity)
    // and the two body IDs the constraint connects. Used to apply equal/
    // opposite torque pairs through `BodyInterface::AddTorque`.
    std::array<JPH::Vec3,   kMaxJointCount> hingeAxisLocal{};
    std::array<JPH::BodyID, kMaxJointCount> hingeBodyA{};
    std::array<JPH::BodyID, kMaxJointCount> hingeBodyB{};
    // Previous joint angle, used to compute joint angular velocity per
    // substep for the actuator. Updated each substep.
    std::array<float, kMaxJointCount>       prevSubstepAngles{};

    RobotConfig                                        config;
    std::array<LegConfig, 4>                           legs{};
    std::array<float, kMaxJointCount>                  defaults{};
    // Smoothed motor targets (EMA filter to simulate SEA actuator delay).
    std::array<float, kMaxJointCount>                  smoothedTargets{};
    // Computed joint velocities from the most recent step.
    std::array<float, kMaxJointCount>                  jointVelocities{};
    double                                             accumulator = 0.0;
};

// MARK: - Observation

@interface IsaacSwiftAnymalObservation ()
- (instancetype)initWithJointPositions:(NSArray<NSNumber *> *)jp
                  jointPositionDeltas:(NSArray<NSNumber *> *)jpd
                       jointVelocities:(NSArray<NSNumber *> *)jv
                     basePositionWorld:(simd_float3)bp
              baseOrientationWorldXYZW:(simd_float4)bo
                baseLinearVelocityBody:(simd_float3)blv
               baseAngularVelocityBody:(simd_float3)bav
                   gravityDirectionBody:(simd_float3)gd;
@end

@implementation IsaacSwiftAnymalObservation

- (instancetype)initWithJointPositions:(NSArray<NSNumber *> *)jp
                  jointPositionDeltas:(NSArray<NSNumber *> *)jpd
                       jointVelocities:(NSArray<NSNumber *> *)jv
                     basePositionWorld:(simd_float3)bp
              baseOrientationWorldXYZW:(simd_float4)bo
                baseLinearVelocityBody:(simd_float3)blv
               baseAngularVelocityBody:(simd_float3)bav
                   gravityDirectionBody:(simd_float3)gd {
    self = [super init];
    if (self) {
        _jointPositions          = [jp copy];
        _jointPositionDeltas     = [jpd copy];
        _jointVelocities         = [jv copy];
        _basePositionWorld       = bp;
        _baseOrientationWorldXYZW= bo;
        _baseLinearVelocityBody  = blv;
        _baseAngularVelocityBody = bav;
        _gravityDirectionBody    = gd;
    }
    return self;
}

@end

// MARK: - Simulator

@implementation IsaacSwiftAnymalSimulator {
    std::unique_ptr<AnymalSimState> _state;
    simd_float3                     _spawnPosition;
    NSArray<NSNumber *>            *_defaultJointPositions;
    IsaacSwiftRobotKind             _robotKind;
    RobotConfig                     _config;
}

- (instancetype)init {
    return [self initWithRobotKind:IsaacSwiftRobotKindAnymalC physicsTimeStep:(1.0 / 200.0)];
}

- (instancetype)initWithPhysicsTimeStep:(double)physicsTimeStep {
    return [self initWithRobotKind:IsaacSwiftRobotKindAnymalC physicsTimeStep:physicsTimeStep];
}

- (instancetype)initWithRobotKind:(IsaacSwiftRobotKind)robotKind
                  physicsTimeStep:(double)physicsTimeStep {
    self = [super init];
    if (!self) return nil;

    InitializeJoltOnce();

    _robotKind       = robotKind;
    _config          = ConfigForKind(robotKind);
    _physicsTimeStep = physicsTimeStep > 0 ? physicsTimeStep : _config.physicsTimeStep;
    _jointStiffness  = _config.kp;
    _jointDamping    = _config.kd;
    _maxJointTorque  = _config.maxTorque;
    _motorTargetSmoothingTau = (robotKind == IsaacSwiftRobotKindH1)
        ? 0.0f
        : 0.008f;  // 8 ms — Spot/Isaac Lab default.
    _spawnPosition   = simd_make_float3(0.0f, 0.0f, _config.spawnZ);

    NSMutableArray<NSNumber *> *defaults = [NSMutableArray arrayWithCapacity:_config.jointCount];
    for (int i = 0; i < _config.jointCount; ++i) {
        [defaults addObject:@(_config.defaults[i])];
    }
    _defaultJointPositions = [defaults copy];

    [self buildScene];
    return self;
}

- (NSUInteger)jointCount { return (NSUInteger)_config.jointCount; }

- (float)recommendedActionScale { return _config.actionScale; }

- (double)recommendedPhysicsTimeStep { return _config.physicsTimeStep; }

- (NSArray<NSNumber *> *)defaultJointPositions { return _defaultJointPositions; }

- (void)setDefaultJointPositions:(NSArray<NSNumber *> *)defaults {
    NSAssert(defaults.count == _config.jointCount, @"default joint positions count must match robot joint count");
    _defaultJointPositions = [defaults copy];
    if (_state) {
        for (NSUInteger i = 0; i < (NSUInteger)_config.jointCount; ++i) {
            _state->defaults[i] = _defaultJointPositions[i].floatValue;
        }
    }
}

- (simd_float3)spawnPositionWorld { return _spawnPosition; }
- (void)setSpawnPositionWorld:(simd_float3)spawn { _spawnPosition = spawn; }

- (void)dealloc {
    if (!_state) return;
    JPH::BodyInterface &bi = _state->physicsSystem->GetBodyInterface();
    auto destroy = [&](JPH::BodyID &id) {
        if (!id.IsInvalid()) {
            bi.RemoveBody(id);
            bi.DestroyBody(id);
            id = JPH::BodyID();
        }
    };
    if (_state->usesUsdAnymal) {
        for (auto &id : _state->usdBodies) destroy(id);
    } else if (_state->usesUsdSpot) {
        for (auto &id : _state->spotUsdBodies) destroy(id);
    } else if (_state->usesUsdH1) {
        for (auto &id : _state->h1Bodies) destroy(id);
    } else {
        for (auto &id : _state->feet)     destroy(id);
        for (auto &id : _state->shanks)   destroy(id);
        for (auto &id : _state->thighs)   destroy(id);
        for (auto &id : _state->hipLinks) destroy(id);
        destroy(_state->base);
    }
    destroy(_state->ground);
}

// MARK: Scene construction

- (void)buildScene {
    auto state = std::make_unique<AnymalSimState>();
    state->config = _config;
    state->legs   = MakeLegConfigs(_config);
    for (int i = 0; i < _config.jointCount; ++i) {
        state->defaults[i] = _defaultJointPositions[i].floatValue;
    }

    constexpr JPH::uint cMaxBodies            = 1024;
    constexpr JPH::uint cMaxBodyPairs         = 1024;
    constexpr JPH::uint cMaxContactConstraints = 1024;

    state->tempAllocator                  = std::make_unique<JPH::TempAllocatorImpl>(8 * 1024 * 1024);
    state->jobSystem                      = std::make_unique<JPH::JobSystemSingleThreaded>(JPH::cMaxPhysicsJobs);
    state->broadPhaseLayerInterface       = std::make_unique<BroadPhaseLayerInterfaceImpl>();
    state->objectVsBroadPhaseLayerFilter  = std::make_unique<ObjectVsBroadPhaseLayerFilterImpl>();
    state->objectLayerPairFilter          = std::make_unique<ObjectLayerPairFilterImpl>();
    state->physicsSystem                  = std::make_unique<JPH::PhysicsSystem>();
    state->physicsSystem->Init(cMaxBodies, 0, cMaxBodyPairs, cMaxContactConstraints,
                               *state->broadPhaseLayerInterface,
                               *state->objectVsBroadPhaseLayerFilter,
                               *state->objectLayerPairFilter);
    state->physicsSystem->SetGravity(JPH::Vec3(0.0f, 0.0f, -9.81f));

    JPH::PhysicsSettings settings = state->physicsSystem->GetPhysicsSettings();
    settings.mNumVelocitySteps = (_robotKind == IsaacSwiftRobotKindH1) ? 16 : 10;
    settings.mNumPositionSteps = 4;
    if (_robotKind == IsaacSwiftRobotKindH1) {
        // H1's foot collision is only 2.4 cm thick. Jolt's default 2 cm
        // penetration slop is large enough to make the foot look and behave
        // partially buried compared with Isaac Sim / PhysX.
        settings.mPenetrationSlop = 0.002f;
        settings.mSpeculativeContactDistance = 0.010f;
    }
    state->physicsSystem->SetPhysicsSettings(settings);

    JPH::BodyInterface &bi = state->physicsSystem->GetBodyInterface();

    auto makeBody = [&](JPH::Shape *shape,
                        JPH::RVec3 pos, JPH::Quat rot,
                        JPH::EMotionType motion, JPH::ObjectLayer layer,
                        float mass, float friction, float linDamp, float angDamp,
                        bool allowSleep) -> JPH::Body * {
        JPH::BodyCreationSettings bcs(shape, pos, rot, motion, layer);
        if (motion != JPH::EMotionType::Static && mass > 0.0f) {
            bcs.mOverrideMassProperties      = JPH::EOverrideMassProperties::CalculateInertia;
            bcs.mMassPropertiesOverride.mMass = mass;
        }
        bcs.mFriction        = friction;
        bcs.mLinearDamping   = linDamp;
        bcs.mAngularDamping  = angDamp;
        bcs.mAllowSleeping   = allowSleep;
        if (_robotKind == IsaacSwiftRobotKindH1 && motion == JPH::EMotionType::Dynamic) {
            bcs.mNumVelocityStepsOverride = 16;
            bcs.mNumPositionStepsOverride = 4;
            if (layer == Layers::kMoving) {
                bcs.mMotionQuality = JPH::EMotionQuality::LinearCast;
            }
        }
        JPH::Body *body = bi.CreateBody(bcs);
        bi.AddBody(body->GetID(),
                   motion == JPH::EMotionType::Static ? JPH::EActivation::DontActivate
                                                       : JPH::EActivation::Activate);
        return body;
    };

    auto makeBodyWithMassProperties = [&](JPH::Shape *shape,
                                          JPH::RVec3 pos, JPH::Quat rot,
                                          JPH::EMotionType motion, JPH::ObjectLayer layer,
                                          float mass, JPH::Vec3 inertiaDiag, JPH::Quat principalAxes,
                                          float friction, float linDamp, float angDamp,
                                          bool allowSleep) -> JPH::Body * {
        JPH::BodyCreationSettings bcs(shape, pos, rot, motion, layer);
        if (motion != JPH::EMotionType::Static && mass > 0.0f) {
            bcs.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
            bcs.mMassPropertiesOverride.mMass = mass;
            const JPH::Mat44 axes = JPH::Mat44::sRotation(principalAxes);
            JPH::Mat44 inertia = axes * JPH::Mat44::sScale(inertiaDiag) * axes.Transposed();
            inertia(3, 3) = 1.0f;
            bcs.mMassPropertiesOverride.mInertia = inertia;
        }
        bcs.mFriction        = friction;
        bcs.mLinearDamping   = linDamp;
        bcs.mAngularDamping  = angDamp;
        bcs.mAllowSleeping   = allowSleep;
        if (_robotKind == IsaacSwiftRobotKindH1 && motion == JPH::EMotionType::Dynamic) {
            bcs.mNumVelocityStepsOverride = 16;
            bcs.mNumPositionStepsOverride = 4;
            if (layer == Layers::kMoving) {
                bcs.mMotionQuality = JPH::EMotionQuality::LinearCast;
            }
        }
        JPH::Body *body = bi.CreateBody(bcs);
        bi.AddBody(body->GetID(),
                   motion == JPH::EMotionType::Static ? JPH::EActivation::DontActivate
                                                       : JPH::EActivation::Activate);
        return body;
    };

    auto addHinge = [&](JPH::Body &a, JPH::Body &b,
                        JPH::RVec3 pivot, JPH::Vec3 axis,
                        int hingeIndex, float defaultAngle) {
        JPH::HingeConstraintSettings cs;
        cs.mSpace        = JPH::EConstraintSpace::WorldSpace;
        cs.mPoint1       = pivot;
        cs.mPoint2       = pivot;
        cs.mHingeAxis1   = axis;
        cs.mHingeAxis2   = axis;
        // Choose a normal axis perpendicular to the hinge axis.
        JPH::Vec3 normal = std::abs(axis.GetY()) > 0.5f ? JPH::Vec3(0, 0, 1) : JPH::Vec3(0, 1, 0);
        cs.mNormalAxis1  = normal;
        cs.mNormalAxis2  = normal;
        // Match Isaac Sim ANYmal joint limits (~±π for HFE/KFE, smaller for
        // HAA but close enough for stand/walk). A wider window keeps the PD
        // motor from chattering against the limit when the policy commands a
        // big swing.
        cs.mLimitsMin    = -3.14f;
        cs.mLimitsMax    =  3.14f;
        cs.mMotorSettings.mSpringSettings = JPH::SpringSettings(JPH::ESpringMode::StiffnessAndDamping,
                                                                _jointStiffness, _jointDamping);
        cs.mMotorSettings.mMinTorqueLimit = -_maxJointTorque;
        cs.mMotorSettings.mMaxTorqueLimit =  _maxJointTorque;

        JPH::Constraint *raw = cs.Create(a, b);
        JPH::Ref<JPH::HingeConstraint> hinge = static_cast<JPH::HingeConstraint *>(raw);
        hinge->SetMotorState(JPH::EMotorState::Position);
        // Jolt's hinge angle is measured relative to the spawn configuration,
        // which here equals the standing-pose default. So a target of 0 means
        // "hold the default joint angle". Targets are added later by `step:`.
        (void)defaultAngle;
        hinge->SetTargetAngle(0.0f);
        hinge->SetTargetAngularVelocity(0.0f);
        state->physicsSystem->AddConstraint(hinge);
        state->hinges[hingeIndex]   = hinge;
        // Store the hinge axis in body A's local frame for the external
        // effort path; the world-space axis changes as the parent link moves.
        state->hingeAxisLocal[hingeIndex] = (a.GetRotation().Conjugated() * axis).Normalized();
        state->hingeBodyA[hingeIndex]     = a.GetID();
        state->hingeBodyB[hingeIndex]     = b.GetID();
    };

    auto addFixed = [&](JPH::Body &a, JPH::Body &b,
                        JPH::RVec3 pivot, int legIdx) {
        JPH::FixedConstraintSettings cs;
        cs.mSpace = JPH::EConstraintSpace::WorldSpace;
        cs.mAutoDetectPoint = false;
        cs.mPoint1 = pivot;
        cs.mPoint2 = pivot;
        cs.mAxisX1 = JPH::Vec3::sAxisX();
        cs.mAxisY1 = JPH::Vec3::sAxisY();
        cs.mAxisX2 = JPH::Vec3::sAxisX();
        cs.mAxisY2 = JPH::Vec3::sAxisY();
        JPH::Constraint *raw = cs.Create(a, b);
        JPH::Ref<JPH::FixedConstraint> fixed = static_cast<JPH::FixedConstraint *>(raw);
        state->physicsSystem->AddConstraint(fixed);
        state->footFixedConstraints[legIdx] = fixed;
    };

    auto addUsdHinge = [&](JPH::Body &a, JPH::Body &b,
                           JPH::RVec3 pivotA, JPH::RVec3 pivotB,
                           JPH::Vec3 axisA, JPH::Vec3 axisB,
                           JPH::Vec3 normalA, JPH::Vec3 normalB,
                           int hingeIndex) {
        JPH::HingeConstraintSettings cs;
        cs.mSpace        = JPH::EConstraintSpace::WorldSpace;
        cs.mPoint1       = pivotA;
        cs.mPoint2       = pivotB;
        cs.mHingeAxis1   = axisA.Normalized();
        cs.mHingeAxis2   = axisB.Normalized();
        cs.mNormalAxis1  = normalA.Normalized();
        cs.mNormalAxis2  = normalB.Normalized();
        const float defaultAngle = state->defaults[hingeIndex];
        cs.mLimitsMin    = JointLowerDeltaLimitForKind(_robotKind, hingeIndex, defaultAngle);
        cs.mLimitsMax    = JointUpperDeltaLimitForKind(_robotKind, hingeIndex, defaultAngle);
        cs.mMotorSettings.mSpringSettings = JPH::SpringSettings(JPH::ESpringMode::StiffnessAndDamping,
                                                                JointStiffnessForKind(_robotKind, hingeIndex, _jointStiffness),
                                                                JointDampingForKind(_robotKind, hingeIndex, _jointDamping));
        const float maxTorque = JointMaxTorqueForKind(_robotKind, hingeIndex, _maxJointTorque);
        cs.mMotorSettings.mMinTorqueLimit = -maxTorque;
        cs.mMotorSettings.mMaxTorqueLimit =  maxTorque;

        JPH::Constraint *raw = cs.Create(a, b);
        JPH::Ref<JPH::HingeConstraint> hinge = static_cast<JPH::HingeConstraint *>(raw);
        hinge->SetMotorState(JPH::EMotorState::Position);
        hinge->SetTargetAngle(0.0f);
        hinge->SetTargetAngularVelocity(0.0f);
        state->physicsSystem->AddConstraint(hinge);
        state->hinges[hingeIndex]       = hinge;
        state->hingeAxisLocal[hingeIndex] = (a.GetRotation().Conjugated() * cs.mHingeAxis1).Normalized();
        state->hingeBodyA[hingeIndex]   = a.GetID();
        state->hingeBodyB[hingeIndex]   = b.GetID();
    };

    auto addUsdFixed = [&](JPH::Body &a, JPH::Body &b,
                           JPH::RVec3 pivotA, JPH::RVec3 pivotB,
                           JPH::Vec3 axisXA, JPH::Vec3 axisYA,
                           JPH::Vec3 axisXB, JPH::Vec3 axisYB,
                           int legIdx) {
        JPH::FixedConstraintSettings cs;
        cs.mSpace = JPH::EConstraintSpace::WorldSpace;
        cs.mAutoDetectPoint = false;
        cs.mPoint1 = pivotA;
        cs.mPoint2 = pivotB;
        cs.mAxisX1 = axisXA.Normalized();
        cs.mAxisY1 = axisYA.Normalized();
        cs.mAxisX2 = axisXB.Normalized();
        cs.mAxisY2 = axisYB.Normalized();
        JPH::Constraint *raw = cs.Create(a, b);
        JPH::Ref<JPH::FixedConstraint> fixed = static_cast<JPH::FixedConstraint *>(raw);
        state->physicsSystem->AddConstraint(fixed);
        state->footFixedConstraints[legIdx] = fixed;
    };

    // ---------- Ground
    {
        JPH::Ref<JPH::BoxShape> shape = new JPH::BoxShape(JPH::Vec3(50.0f, 50.0f, 0.1f));
        shape->SetEmbedded();
        JPH::Body *body = makeBody(shape, JPH::RVec3(0, 0, -0.1f), JPH::Quat::sIdentity(),
                                   JPH::EMotionType::Static, Layers::kNonMoving,
                                   0.0f, 1.0f, 0.0f, 0.0f, false);
        state->ground = body->GetID();
    }

    // ---------- Base
    const JPH::RVec3 spawn(_spawnPosition.x, _spawnPosition.y, _spawnPosition.z);
    const RobotConfig &cfg = state->config;

    if (_robotKind == IsaacSwiftRobotKindAnymalC) {
        state->usesUsdAnymal = true;
        std::array<JPH::Body *, kAnymalUsdBodyCount> bodies{};

        auto originFor = [&](int idx) -> JPH::RVec3 {
            const JPH::Vec3 local = kAnymalUsdBodies[idx].localPosition;
            return spawn + JPH::RVec3(local);
        };

        for (int i = 0; i < kAnymalUsdBodyCount; ++i) {
            const AnymalUsdBodyDef &def = kAnymalUsdBodies[i];
            JPH::Ref<JPH::Shape> shape;
            if (i == kUsdBase) {
                shape = new JPH::BoxShape(JPH::Vec3(cfg.baseHalfX, cfg.baseHalfY, cfg.baseHalfZ));
            } else if (def.collides) {
                JPH::Ref<JPH::SphereShape> foot = new JPH::SphereShape(def.contactRadius);
                shape = new JPH::RotatedTranslatedShape(JPH::Vec3(0.0f, 0.0f, 0.063f),
                                                        JPH::Quat::sIdentity(),
                                                        foot);
            } else {
                JPH::Ref<JPH::BoxShape> inertial = new JPH::BoxShape(AnymalUsdInertialHalfExtents(i));
                shape = new JPH::RotatedTranslatedShape(def.inertialCom,
                                                        JPH::Quat::sIdentity(),
                                                        inertial);
            }
            shape->SetEmbedded();
            bodies[i] = makeBody(shape,
                                 originFor(i),
                                 def.localRotation,
                                 JPH::EMotionType::Dynamic,
                                 def.collides ? Layers::kMoving : Layers::kInternal,
                                 def.mass,
                                 def.collides ? cfg.shankFriction : cfg.hipFriction,
                                 0.0f,
                                 0.0f,
                                 false);
            state->usdBodies[i] = bodies[i]->GetID();
        }

        state->base = state->usdBodies[kUsdBase];
        state->hipLinks = {{ state->usdBodies[kUsdLFHip], state->usdBodies[kUsdRFHip],
                             state->usdBodies[kUsdLHHip], state->usdBodies[kUsdRHHip] }};
        state->thighs = {{ state->usdBodies[kUsdLFThigh], state->usdBodies[kUsdRFThigh],
                           state->usdBodies[kUsdLHThigh], state->usdBodies[kUsdRHThigh] }};
        state->shanks = {{ state->usdBodies[kUsdLFShank], state->usdBodies[kUsdRFShank],
                           state->usdBodies[kUsdLHShank], state->usdBodies[kUsdRHShank] }};
        state->feet = {{ state->usdBodies[kUsdLFFoot], state->usdBodies[kUsdRFFoot],
                         state->usdBodies[kUsdLHFoot], state->usdBodies[kUsdRHFoot] }};

        for (const AnymalUsdJointDef &joint : kAnymalUsdJoints) {
            const int p = joint.parentBody;
            const int c = joint.childBody;
            const JPH::Quat parentRot = kAnymalUsdBodies[p].localRotation;
            const JPH::Quat childRot  = kAnymalUsdBodies[c].localRotation;
            const JPH::Quat frame0 = parentRot * joint.localRot0;
            const JPH::Quat frame1 = childRot  * joint.localRot1;
            const JPH::RVec3 pivot0 = originFor(p) + JPH::RVec3(parentRot * joint.localPos0);
            const JPH::RVec3 pivot1 = originFor(c) + JPH::RVec3(childRot  * joint.localPos1);

            if (joint.hingeIndex >= 0) {
                addUsdHinge(*bodies[p], *bodies[c],
                            pivot0, pivot1,
                            frame0 * JPH::Vec3::sAxisX(),
                            frame1 * JPH::Vec3::sAxisX(),
                            frame0 * JPH::Vec3::sAxisY(),
                            frame1 * JPH::Vec3::sAxisY(),
                            joint.hingeIndex);
            } else {
                addUsdFixed(*bodies[p], *bodies[c],
                            pivot0, pivot1,
                            frame0 * JPH::Vec3::sAxisX(),
                            frame0 * JPH::Vec3::sAxisY(),
                            frame1 * JPH::Vec3::sAxisX(),
                            frame1 * JPH::Vec3::sAxisY(),
                            joint.fixedLegIndex);
            }
        }

        state->physicsSystem->OptimizeBroadPhase();
        _state = std::move(state);
        return;
    }

    if (_robotKind == IsaacSwiftRobotKindSpot) {
        state->usesUsdSpot = true;
        std::array<JPH::Body *, kSpotUsdBodyCount> bodies{};
        const std::array<UsdBodyPose, kSpotUsdBodyCount> poses = MakeSpotUsdBodyPoses(state->defaults, spawn);

        for (int i = 0; i < kSpotUsdBodyCount; ++i) {
            const SpotUsdBodyDef &def = kSpotUsdBodies[i];
            JPH::Ref<JPH::Shape> shape;
            if (i == kSpotUsdBase) {
                shape = new JPH::BoxShape(JPH::Vec3(cfg.baseHalfX, cfg.baseHalfY, cfg.baseHalfZ));
            } else if (def.collides) {
                shape = new JPH::SphereShape(def.contactRadius);
            } else {
                JPH::Ref<JPH::BoxShape> inertial = new JPH::BoxShape(SpotUsdInertialHalfExtents(i));
                shape = new JPH::RotatedTranslatedShape(def.inertialCom,
                                                        JPH::Quat::sIdentity(),
                                                        inertial);
            }
            shape->SetEmbedded();
            bodies[i] = makeBody(shape,
                                 poses[i].position,
                                 poses[i].rotation,
                                 JPH::EMotionType::Dynamic,
                                 def.collides ? Layers::kMoving : Layers::kInternal,
                                 def.mass,
                                 def.collides ? cfg.shankFriction : cfg.hipFriction,
                                 0.0f,
                                 0.0f,
                                 false);
            state->spotUsdBodies[i] = bodies[i]->GetID();
        }

        state->base = state->spotUsdBodies[kSpotUsdBase];
        state->hipLinks = {{ state->spotUsdBodies[kSpotUsdFLHip], state->spotUsdBodies[kSpotUsdFRHip],
                             state->spotUsdBodies[kSpotUsdHLHip], state->spotUsdBodies[kSpotUsdHRHip] }};
        state->thighs = {{ state->spotUsdBodies[kSpotUsdFLUleg], state->spotUsdBodies[kSpotUsdFRUleg],
                           state->spotUsdBodies[kSpotUsdHLUleg], state->spotUsdBodies[kSpotUsdHRUleg] }};
        state->shanks = {{ state->spotUsdBodies[kSpotUsdFLLleg], state->spotUsdBodies[kSpotUsdFRLleg],
                           state->spotUsdBodies[kSpotUsdHLLleg], state->spotUsdBodies[kSpotUsdHRLleg] }};
        state->feet = {{ state->spotUsdBodies[kSpotUsdFLFoot], state->spotUsdBodies[kSpotUsdFRFoot],
                         state->spotUsdBodies[kSpotUsdHLFoot], state->spotUsdBodies[kSpotUsdHRFoot] }};

        for (const SpotUsdJointDef &joint : kSpotUsdJoints) {
            const int p = joint.parentBody;
            const int c = joint.childBody;
            const JPH::Quat frame0 = poses[p].rotation;
            const JPH::Quat frame1 = poses[c].rotation;
            const JPH::RVec3 pivot0 = poses[p].position + JPH::RVec3(poses[p].rotation * joint.localPos0);
            const JPH::RVec3 pivot1 = poses[c].position + JPH::RVec3(poses[c].rotation * joint.localPos1);
            const JPH::Vec3 normal = std::abs(joint.axis.GetY()) > 0.5f
                ? JPH::Vec3::sAxisZ()
                : JPH::Vec3::sAxisY();

            if (joint.hingeIndex >= 0) {
                addUsdHinge(*bodies[p], *bodies[c],
                            pivot0, pivot1,
                            frame0 * joint.axis,
                            frame1 * joint.axis,
                            frame0 * normal,
                            frame0 * normal,
                            joint.hingeIndex);
            } else {
                addUsdFixed(*bodies[p], *bodies[c],
                            pivot0, pivot1,
                            frame0 * JPH::Vec3::sAxisX(),
                            frame0 * JPH::Vec3::sAxisY(),
                            frame1 * JPH::Vec3::sAxisX(),
                            frame1 * JPH::Vec3::sAxisY(),
                            joint.fixedLegIndex);
            }
        }

        state->physicsSystem->OptimizeBroadPhase();
        _state = std::move(state);
        return;
    }

    if (_robotKind == IsaacSwiftRobotKindH1) {
        state->usesUsdH1 = true;
        std::array<JPH::Body *, kH1BodyCount> bodies{};
        const std::array<float, kMaxJointCount> zeroJointAngles{};
        const std::array<UsdBodyPose, kH1BodyCount> poses = MakeH1BodyPoses(zeroJointAngles, spawn);

        for (int i = 0; i < kH1BodyCount; ++i) {
            const H1BodyDef &def = kH1Bodies[i];
            JPH::Ref<JPH::BoxShape> collision = new JPH::BoxShape(def.halfExtents, 0.0f);
            JPH::Ref<JPH::Shape> collisionAtUsdLocal = new JPH::RotatedTranslatedShape(def.collisionCenter,
                                                                                       JPH::Quat::sIdentity(),
                                                                                       collision);
            JPH::Ref<JPH::Shape> shape = new JPH::OffsetCenterOfMassShape(collisionAtUsdLocal,
                                                                          def.inertialCom - def.collisionCenter);
            shape->SetEmbedded();
            bodies[i] = makeBodyWithMassProperties(shape,
                                                    poses[i].position,
                                                    poses[i].rotation,
                                                    JPH::EMotionType::Dynamic,
                                                    def.collides ? Layers::kMoving : Layers::kInternal,
                                                    def.mass,
                                                    def.inertiaDiag,
                                                    def.principalAxes,
                                                    def.friction,
                                                    0.0f,
                                                    0.0f,
                                                    false);
            state->h1Bodies[i] = bodies[i]->GetID();
        }

        state->base = state->h1Bodies[kH1Pelvis];

        for (const H1JointDef &joint : kH1Joints) {
            const int p = joint.parentBody;
            const int c = joint.childBody;
            const JPH::Quat frame0 = poses[p].rotation * joint.localRot0;
            const JPH::Quat frame1 = poses[c].rotation * joint.localRot1;
            const JPH::RVec3 pivot0 = poses[p].position + JPH::RVec3(poses[p].rotation * joint.localPos0);
            const JPH::RVec3 pivot1 = poses[c].position + JPH::RVec3(poses[c].rotation * joint.localPos1);
            const JPH::Vec3 normal = std::abs(joint.axis.GetZ()) > 0.5f
                ? JPH::Vec3::sAxisX()
                : JPH::Vec3::sAxisZ();

            addUsdHinge(*bodies[p], *bodies[c],
                        pivot0, pivot1,
                        frame0 * joint.axis,
                        frame1 * joint.axis,
                        frame0 * normal,
                        frame0 * normal,
                        joint.hingeIndex);
        }

        state->physicsSystem->OptimizeBroadPhase();
        _state = std::move(state);
        return;
    }

    JPH::Body *baseBody;
    {
        JPH::Ref<JPH::BoxShape> shape = new JPH::BoxShape(JPH::Vec3(cfg.baseHalfX, cfg.baseHalfY, cfg.baseHalfZ));
        shape->SetEmbedded();
        baseBody = makeBody(shape, spawn, JPH::Quat::sIdentity(),
                            JPH::EMotionType::Dynamic, Layers::kMoving,
                            cfg.baseMass, cfg.baseFriction, 0.0f, 0.0f, false);
        state->base = baseBody->GetID();
    }

    // ---------- Legs
    for (int legIdx = 0; legIdx < 4; ++legIdx) {
        const LegConfig &leg = state->legs[legIdx];
        const float haaDef = state->defaults[leg.baseIdx + 0];
        const float hfeDef = state->defaults[leg.baseIdx + 1];
        const float kfeDef = state->defaults[leg.baseIdx + 2];

        const JPH::RVec3 hipPivot(spawn.GetX() + leg.hipX,
                                  spawn.GetY() + leg.hipY,
                                  spawn.GetZ());

        // Hip-link rotated by HAA around world +X.
        JPH::Quat hipRot = JPH::Quat::sRotation(JPH::Vec3(1, 0, 0), haaDef);
        JPH::Body *hipLinkBody;
        {
            JPH::Ref<JPH::BoxShape> shape = new JPH::BoxShape(JPH::Vec3::sReplicate(cfg.hipLinkHalf));
            shape->SetEmbedded();
            hipLinkBody = makeBody(shape, hipPivot, hipRot,
                                   JPH::EMotionType::Dynamic, Layers::kMoving,
                                   cfg.hipLinkMass, cfg.hipFriction, 0.0f, 0.0f, false);
            state->hipLinks[legIdx] = hipLinkBody->GetID();
        }

        const JPH::Quat haaQ = JPH::Quat::sRotation(JPH::Vec3(1, 0, 0), haaDef);
        const JPH::Vec3 hipToHfe = HipToHfeOffset(cfg, leg);
        const JPH::RVec3 hfePivot = hipPivot + JPH::RVec3(haaQ * hipToHfe);
        const JPH::Quat hfeQ = JPH::Quat::sRotation(JPH::Vec3(0, 1, 0), hfeDef);
        const JPH::Quat thighRot = haaQ * hfeQ;
        const JPH::Vec3 thighToKfe = ThighToKfeOffset(cfg);
        const JPH::RVec3 thighCom = hfePivot + JPH::RVec3(thighRot * (thighToKfe * 0.5f));
        const JPH::RVec3 kfePivot = hfePivot + JPH::RVec3(thighRot * thighToKfe);

        JPH::Body *thighBody;
        {
            JPH::Ref<JPH::BoxShape> shape = new JPH::BoxShape(JPH::Vec3(cfg.thighRadius, cfg.thighRadius,
                                                                       cfg.thighLength * 0.5f));
            shape->SetEmbedded();
            thighBody = makeBody(shape, thighCom, thighRot,
                                 JPH::EMotionType::Dynamic, Layers::kMoving,
                                 cfg.thighMass, cfg.thighFriction, 0.0f, 0.0f, false);
            state->thighs[legIdx] = thighBody->GetID();
        }

        const JPH::Quat kfeQ = JPH::Quat::sRotation(JPH::Vec3(0, 1, 0), kfeDef);
        const JPH::Quat shankRot = thighRot * kfeQ;
        const JPH::Vec3 shankDir = shankRot * JPH::Vec3(0, 0, -1);
        const JPH::RVec3 shankCom = kfePivot + JPH::RVec3(shankDir * (cfg.shankLength * 0.5f));
        const JPH::Vec3 shankToFoot = ShankToFootOffset(cfg, leg);
        const JPH::RVec3 footCom = (cfg.footMass > 0.0f)
            ? kfePivot + JPH::RVec3(shankRot * shankToFoot)
            : shankCom + JPH::RVec3(shankDir * (cfg.shankLength * 0.5f));

        JPH::Body *shankBody;
        {
            JPH::Ref<JPH::CapsuleShape> capsule = new JPH::CapsuleShape(cfg.shankLength * 0.5f,
                                                                         cfg.footRadius);
            JPH::Ref<JPH::RotatedTranslatedShape> shape =
                new JPH::RotatedTranslatedShape(JPH::Vec3::sZero(),
                                                JPH::Quat::sRotation(JPH::Vec3(1, 0, 0),
                                                                     0.5f * JPH::JPH_PI),
                                                capsule);
            shape->SetEmbedded();
            shankBody = makeBody(shape, shankCom, shankRot,
                                 JPH::EMotionType::Dynamic,
                                 cfg.footMass > 0.0f ? Layers::kInternal : Layers::kMoving,
                                 cfg.shankMass, cfg.shankFriction, 0.0f, 0.0f, false);
            state->shanks[legIdx] = shankBody->GetID();
        }

        JPH::Body *footBody = nullptr;
        if (cfg.footMass > 0.0f) {
            JPH::Ref<JPH::SphereShape> shape = new JPH::SphereShape(cfg.footRadius);
            shape->SetEmbedded();
            footBody = makeBody(shape, footCom, shankRot,
                                JPH::EMotionType::Dynamic, Layers::kMoving,
                                cfg.footMass, cfg.shankFriction, 0.0f, 0.0f, false);
            state->feet[legIdx] = footBody->GetID();
        }

        addHinge(*baseBody,    *hipLinkBody, hipPivot, JPH::Vec3(1, 0, 0), leg.baseIdx + 0, haaDef);
        addHinge(*hipLinkBody, *thighBody,   hfePivot, JPH::Vec3(0, 1, 0), leg.baseIdx + 1, hfeDef);
        addHinge(*thighBody,   *shankBody,   kfePivot, JPH::Vec3(0, 1, 0), leg.baseIdx + 2, kfeDef);
        if (footBody != nullptr) {
            addFixed(*shankBody, *footBody, footCom, legIdx);
        }
    }

    state->physicsSystem->OptimizeBroadPhase();
    _state = std::move(state);
}

// MARK: Reset

- (void)reset {
    if (!_state) return;
    JPH::BodyInterface &bi = _state->physicsSystem->GetBodyInterface();
    JPH::Vec3 zero = JPH::Vec3::sZero();
    auto zeroVel = [&](JPH::BodyID id) {
        if (!id.IsInvalid()) bi.SetLinearAndAngularVelocity(id, zero, zero);
    };

    const JPH::RVec3 spawn(_spawnPosition.x, _spawnPosition.y, _spawnPosition.z);
    if (_state->usesUsdAnymal) {
        for (int i = 0; i < kAnymalUsdBodyCount; ++i) {
            const AnymalUsdBodyDef &def = kAnymalUsdBodies[i];
            const JPH::RVec3 origin = spawn + JPH::RVec3(def.localPosition);
            bi.SetPositionAndRotation(_state->usdBodies[i], origin, def.localRotation, JPH::EActivation::Activate);
            zeroVel(_state->usdBodies[i]);
        }
        for (int i = 0; i < _config.jointCount; ++i) {
            _state->hinges[i]->SetTargetAngle(0.0f);
            _state->hinges[i]->SetTargetAngularVelocity(0.0f);
            _state->prevSubstepAngles[i] = _state->hinges[i]->GetCurrentAngle();
        }
        _state->accumulator = 0.0;
        _state->jointVelocities.fill(0.0f);
        _state->smoothedTargets.fill(0.0f);
        [_jointActuator resetState];
        return;
    }

    if (_state->usesUsdSpot) {
        const std::array<UsdBodyPose, kSpotUsdBodyCount> poses = MakeSpotUsdBodyPoses(_state->defaults, spawn);
        for (int i = 0; i < kSpotUsdBodyCount; ++i) {
            bi.SetPositionAndRotation(_state->spotUsdBodies[i],
                                      poses[i].position,
                                      poses[i].rotation,
                                      JPH::EActivation::Activate);
            zeroVel(_state->spotUsdBodies[i]);
        }
        for (int i = 0; i < _config.jointCount; ++i) {
            _state->hinges[i]->SetTargetAngle(0.0f);
            _state->hinges[i]->SetTargetAngularVelocity(0.0f);
            _state->prevSubstepAngles[i] = _state->hinges[i]->GetCurrentAngle();
        }
        _state->accumulator = 0.0;
        _state->jointVelocities.fill(0.0f);
        _state->smoothedTargets.fill(0.0f);
        [_jointActuator resetState];
        return;
    }

    if (_state->usesUsdH1) {
        const std::array<float, kMaxJointCount> zeroJointAngles{};
        const std::array<UsdBodyPose, kH1BodyCount> poses = MakeH1BodyPoses(zeroJointAngles, spawn);
        for (int i = 0; i < kH1BodyCount; ++i) {
            bi.SetPositionAndRotation(_state->h1Bodies[i],
                                      poses[i].position,
                                      poses[i].rotation,
                                      JPH::EActivation::Activate);
            zeroVel(_state->h1Bodies[i]);
        }
        for (int i = 0; i < _config.jointCount; ++i) {
            _state->hinges[i]->SetTargetAngle(0.0f);
            _state->hinges[i]->SetTargetAngularVelocity(0.0f);
            _state->prevSubstepAngles[i] = _state->hinges[i]->GetCurrentAngle();
        }
        _state->accumulator = 0.0;
        _state->jointVelocities.fill(0.0f);
        _state->smoothedTargets.fill(0.0f);
        [_jointActuator resetState];
        return;
    }

    bi.SetPositionAndRotation(_state->base, spawn, JPH::Quat::sIdentity(), JPH::EActivation::Activate);
    zeroVel(_state->base);

    for (int legIdx = 0; legIdx < 4; ++legIdx) {
        const LegConfig &leg = _state->legs[legIdx];
        const RobotConfig &cfg = _state->config;
        const float haaDef = _state->defaults[leg.baseIdx + 0];
        const float hfeDef = _state->defaults[leg.baseIdx + 1];
        const float kfeDef = _state->defaults[leg.baseIdx + 2];

        const JPH::RVec3 hipPivot(spawn.GetX() + leg.hipX, spawn.GetY() + leg.hipY, spawn.GetZ());
        const JPH::Quat haaQ = JPH::Quat::sRotation(JPH::Vec3(1, 0, 0), haaDef);
        const JPH::Vec3 hipToHfe = HipToHfeOffset(cfg, leg);
        const JPH::RVec3 hfePivot = hipPivot + JPH::RVec3(haaQ * hipToHfe);
        const JPH::Quat hfeQ = JPH::Quat::sRotation(JPH::Vec3(0, 1, 0), hfeDef);
        const JPH::Quat kfeQ = JPH::Quat::sRotation(JPH::Vec3(0, 1, 0), kfeDef);
        const JPH::Quat thighRot = haaQ * hfeQ;
        const JPH::Quat shankRot = thighRot * kfeQ;
        const JPH::Vec3 thighToKfe = ThighToKfeOffset(cfg);
        const JPH::RVec3 thighCom = hfePivot + JPH::RVec3(thighRot * (thighToKfe * 0.5f));
        const JPH::RVec3 kfePivot = hfePivot + JPH::RVec3(thighRot * thighToKfe);
        const JPH::Vec3 shankDir = shankRot * JPH::Vec3(0, 0, -1);
        const JPH::RVec3 shankCom = kfePivot + JPH::RVec3(shankDir * (cfg.shankLength * 0.5f));
        const JPH::Vec3 shankToFoot = ShankToFootOffset(cfg, leg);
        const JPH::RVec3 footCom = (cfg.footMass > 0.0f)
            ? kfePivot + JPH::RVec3(shankRot * shankToFoot)
            : shankCom + JPH::RVec3(shankDir * (cfg.shankLength * 0.5f));

        bi.SetPositionAndRotation(_state->hipLinks[legIdx], hipPivot, haaQ,    JPH::EActivation::Activate);
        bi.SetPositionAndRotation(_state->thighs[legIdx],   thighCom, thighRot, JPH::EActivation::Activate);
        bi.SetPositionAndRotation(_state->shanks[legIdx],   shankCom, shankRot, JPH::EActivation::Activate);
        if (!_state->feet[legIdx].IsInvalid()) {
            bi.SetPositionAndRotation(_state->feet[legIdx], footCom, shankRot, JPH::EActivation::Activate);
            zeroVel(_state->feet[legIdx]);
        }
        zeroVel(_state->hipLinks[legIdx]);
        zeroVel(_state->thighs[legIdx]);
        zeroVel(_state->shanks[legIdx]);

        _state->hinges[leg.baseIdx + 0]->SetTargetAngle(0.0f);
        _state->hinges[leg.baseIdx + 0]->SetTargetAngularVelocity(0.0f);
        _state->hinges[leg.baseIdx + 1]->SetTargetAngle(0.0f);
        _state->hinges[leg.baseIdx + 1]->SetTargetAngularVelocity(0.0f);
        _state->hinges[leg.baseIdx + 2]->SetTargetAngle(0.0f);
        _state->hinges[leg.baseIdx + 2]->SetTargetAngularVelocity(0.0f);
        (void)haaDef; (void)hfeDef; (void)kfeDef;
    }
    _state->accumulator = 0.0;
    _state->jointVelocities.fill(0.0f);
    _state->smoothedTargets.fill(0.0f);
    _state->prevSubstepAngles.fill(0.0f);
    [_jointActuator resetState];
}

// MARK: Motor settings

- (void)refreshMotorSettings {
    if (!_state) return;
    for (int jointIndex = 0; jointIndex < _config.jointCount; ++jointIndex) {
        auto &hinge = _state->hinges[jointIndex];
        if (!hinge) continue;
        JPH::MotorSettings &m = hinge->GetMotorSettings();
        m.mSpringSettings = JPH::SpringSettings(JPH::ESpringMode::StiffnessAndDamping,
                                                JointStiffnessForKind(_robotKind, jointIndex, _jointStiffness),
                                                JointDampingForKind(_robotKind, jointIndex, _jointDamping));
        const float maxTorque = JointMaxTorqueForKind(_robotKind, jointIndex, _maxJointTorque);
        m.mMinTorqueLimit = -maxTorque;
        m.mMaxTorqueLimit =  maxTorque;
    }
}

- (void)setJointStiffness:(float)v { _jointStiffness = v; [self refreshMotorSettings]; }
- (void)setJointDamping:(float)v   { _jointDamping   = v; [self refreshMotorSettings]; }
- (void)setMaxJointTorque:(float)v { _maxJointTorque = v; [self refreshMotorSettings]; }

// MARK: Step

- (NSArray<NSNumber *> *)stepWithScaledActions:(NSArray<NSNumber *> *)scaledActions
                                  elapsedTime:(double)elapsedTime {
    if (!_state) return @[];

    const NSUInteger provided = scaledActions.count;
    // Optional exponential moving average on motor targets. Direct H1
    // position policies set tau to zero; actuator-backed quadrupeds may
    // smooth targets to match their training-time actuator dynamics.
    const int jointCount = _config.jointCount;
    std::array<float, kMaxJointCount> commandedTargets;
    for (int i = 0; i < jointCount; ++i) {
        const float targetDelta = (i < provided ? scaledActions[i].floatValue : 0.0f)
            * JointPolicyDirectionForKind(_robotKind, i);
        const float target = (_robotKind == IsaacSwiftRobotKindH1)
            ? _state->defaults[i] + targetDelta
            : targetDelta;
        commandedTargets[i] = ClampJointTargetDeltaForKind(_robotKind, i, target, _state->defaults[i]);
    }

    _state->accumulator += MAX(elapsedTime, 0.0);
    int substeps = 0;
    if (_physicsTimeStep > 0) {
        substeps = static_cast<int>(_state->accumulator / _physicsTimeStep);
        if (substeps > 0) {
            _state->accumulator -= substeps * _physicsTimeStep;
        }
    }
    if (substeps > 32) {
        substeps = 32;
        _state->accumulator = 0.0;
    }
    if (substeps == 0 && elapsedTime > 0.0) {
        substeps = 1;
    }

    JPH::PhysicsSystem &system = *_state->physicsSystem;

    // Capture angles before substeps to compute instantaneous velocity.
    std::array<float, kMaxJointCount> preAngles;
    for (int i = 0; i < jointCount; ++i) {
        preAngles[i] = _state->hinges[i]->GetCurrentAngle();
    }

    // EMA smoothing constant. H1 sets this to zero because Isaac Sim applies
    // each direct position target immediately at the 200 Hz physics rate.
    const float kTau = MAX(_motorTargetSmoothingTau, 0.0f);
    const float dt   = static_cast<float>(_physicsTimeStep);
    const float alpha = (kTau > 0.0f) ? dt / (kTau + dt) : 1.0f;

    id<IsaacSwiftJointActuator> actuator = _jointActuator;
    JPH::BodyInterface &bi = system.GetBodyInterface();
    // Direct position policies use Jolt's constraint-space position motor,
    // matching Isaac Sim's `ArticulationAction(joint_positions=...)` contract.
    // Learned actuator paths provide torque directly.
    const bool useExternalTorqueDrive = actuator != nil;
    const bool usePositionMotor = !useExternalTorqueDrive;

    for (int i = 0; i < jointCount; ++i) {
        _state->hinges[i]->SetMotorState(usePositionMotor
                                         ? JPH::EMotorState::Position
                                         : JPH::EMotorState::Off);
    }

    for (int s = 0; s < substeps; ++s) {
        for (int i = 0; i < jointCount; ++i) {
            _state->smoothedTargets[i] += alpha * (commandedTargets[i] - _state->smoothedTargets[i]);
            if (usePositionMotor) {
                _state->hinges[i]->SetTargetAngle(_state->smoothedTargets[i]);
            }
        }

        if (useExternalTorqueDrive) {
            NSMutableArray<NSNumber *> *posErr = [NSMutableArray arrayWithCapacity:jointCount];
            NSMutableArray<NSNumber *> *jvel   = [NSMutableArray arrayWithCapacity:jointCount];
            std::array<float, kMaxJointCount> currentAngles;
            std::array<float, kMaxJointCount> driveVelocities;
            for (int i = 0; i < jointCount; ++i) {
                const float current = _state->hinges[i]->GetCurrentAngle();
                const float velSub  = (dt > 0.0f) ? (current - _state->prevSubstepAngles[i]) / dt : 0.0f;
                _state->prevSubstepAngles[i] = current;
                currentAngles[i] = current;
                const float driveVelocityLimit = (_robotKind == IsaacSwiftRobotKindH1) ? 100.0f : 20.0f;
                driveVelocities[i] = std::clamp(velSub, -driveVelocityLimit, driveVelocityLimit);
                if (actuator != nil) {
                    const float actuatorInputVelocity = std::clamp(velSub, -20.0f, 20.0f);
                    [posErr addObject:@(_state->smoothedTargets[i] - current)];
                    [jvel   addObject:@(actuatorInputVelocity)];
                }
            }
            NSArray<NSNumber *> *torques = actuator != nil
                ? [actuator torquesForJointPositionErrors:posErr jointVelocities:jvel]
                : nil;
            const NSUInteger n = actuator != nil ? MIN((NSUInteger)jointCount, torques.count) : (NSUInteger)jointCount;
            // IsaacSim's ANYmal LSTM SEA wrapper clips actuator effort to
            // ±80 N·m. Keep that separate from the direct position-motor
            // fallback limit, which remains controlled by `maxJointTorque`.
            for (NSUInteger i = 0; i < n; ++i) {
                float tau;
                float torqueLimit;
                if (actuator != nil) {
                    tau = torques[i].floatValue;
                    torqueLimit = (_robotKind == IsaacSwiftRobotKindAnymalC) ? 80.0f : _maxJointTorque;
                } else {
                    const float positionError = _state->smoothedTargets[i] - currentAngles[i];
                    tau = JointStiffnessForKind(_robotKind, static_cast<int>(i), _jointStiffness) * positionError
                        - JointDampingForKind(_robotKind, static_cast<int>(i), _jointDamping) * driveVelocities[i];
                    torqueLimit = JointMaxTorqueForKind(_robotKind, static_cast<int>(i), _maxJointTorque);
                }
                const float actuatorTorqueLimit = torqueLimit;
                if (tau >  actuatorTorqueLimit) tau =  actuatorTorqueLimit;
                if (tau < -actuatorTorqueLimit) tau = -actuatorTorqueLimit;
                JPH::BodyID idA = _state->hingeBodyA[i];
                JPH::BodyID idB = _state->hingeBodyB[i];
                JPH::Quat rotA  = bi.GetRotation(idA);
                JPH::Vec3 worldAxis = (rotA * _state->hingeAxisLocal[i]).Normalized();
                JPH::Vec3 torqueVec = worldAxis * tau;
                bi.AddTorque(idB,  torqueVec);
                bi.AddTorque(idA, -torqueVec);
            }
        }

        system.Update(static_cast<float>(_physicsTimeStep), 1,
                      _state->tempAllocator.get(),
                      _state->jobSystem.get());
    }

    // Compute per-step joint velocities: Δangle / Δtime.
    const float totalDt = substeps * static_cast<float>(_physicsTimeStep);
    for (int i = 0; i < jointCount; ++i) {
        const float postAngle = _state->hinges[i]->GetCurrentAngle();
        const float direction = JointPolicyDirectionForKind(_robotKind, i);
        _state->jointVelocities[i] = (totalDt > 0)
            ? direction * (postAngle - preAngles[i]) / totalDt
            : 0.0f;
    }

    NSMutableArray<NSNumber *> *deltas = [NSMutableArray arrayWithCapacity:jointCount];
    for (int i = 0; i < jointCount; ++i) {
        const float direction = JointPolicyDirectionForKind(_robotKind, i);
        const float angle = direction * _state->hinges[i]->GetCurrentAngle();
        const float delta = (_robotKind == IsaacSwiftRobotKindH1)
            ? angle - _state->defaults[i]
            : angle;
        [deltas addObject:@(delta)];
    }
    return deltas;
}

// MARK: Observation

- (IsaacSwiftAnymalObservation *)currentObservation {
    const int jointCount = _config.jointCount;
    NSMutableArray<NSNumber *> *jp  = [NSMutableArray arrayWithCapacity:jointCount];
    NSMutableArray<NSNumber *> *jpd = [NSMutableArray arrayWithCapacity:jointCount];
    NSMutableArray<NSNumber *> *jv  = [NSMutableArray arrayWithCapacity:jointCount];

    if (!_state) {
        for (int i = 0; i < jointCount; ++i) {
            [jp addObject:@(0)]; [jpd addObject:@(0)]; [jv addObject:@(0)];
        }
        return [[IsaacSwiftAnymalObservation alloc]
                initWithJointPositions:jp
                  jointPositionDeltas:jpd
                       jointVelocities:jv
                     basePositionWorld:simd_make_float3(0,0,0)
              baseOrientationWorldXYZW:simd_make_float4(0,0,0,1)
                baseLinearVelocityBody:simd_make_float3(0,0,0)
               baseAngularVelocityBody:simd_make_float3(0,0,0)
                  gravityDirectionBody:simd_make_float3(0,0,-1)];
    }

    JPH::BodyInterface &bi = _state->physicsSystem->GetBodyInterface();

    for (int i = 0; i < jointCount; ++i) {
        const float direction = JointPolicyDirectionForKind(_robotKind, i);
        const float rawAngle = direction * _state->hinges[i]->GetCurrentAngle();
        const float delta = (_robotKind == IsaacSwiftRobotKindH1)
            ? rawAngle - _state->defaults[i]
            : rawAngle;
        const float angle = (_robotKind == IsaacSwiftRobotKindH1)
            ? rawAngle
            : _state->defaults[i] + delta;
        [jp  addObject:@(angle)];
        [jpd addObject:@(delta)];
        // Use the velocity computed during the last step call.
        [jv addObject:@(_state->jointVelocities[i])];
    }

    // Renderer articulation profiles are authored against USD link origins,
    // not inertial COM positions. H1 carries real USD COM offsets, so using
    // the COM here makes the visual pelvis appear a few centimeters too low.
    const JPH::RVec3 basePos     = bi.GetPosition(_state->base);
    const JPH::Quat  baseQuat    = bi.GetRotation(_state->base);
    const JPH::Vec3  baseLinVelW = bi.GetLinearVelocity(_state->base);
    const JPH::Vec3  baseAngVelW = bi.GetAngularVelocity(_state->base);

    JPH::Quat invQ = baseQuat.Inversed();
    JPH::Vec3 linVelB = invQ * baseLinVelW;
    JPH::Vec3 angVelB = invQ * baseAngVelW;
    JPH::Vec3 gravB   = invQ * JPH::Vec3(0, 0, -1);

    return [[IsaacSwiftAnymalObservation alloc]
            initWithJointPositions:jp
              jointPositionDeltas:jpd
                   jointVelocities:jv
                 basePositionWorld:simd_make_float3((float)basePos.GetX(),
                                                    (float)basePos.GetY(),
                                                    (float)basePos.GetZ())
          baseOrientationWorldXYZW:simd_make_float4(baseQuat.GetX(), baseQuat.GetY(),
                                                    baseQuat.GetZ(), baseQuat.GetW())
            baseLinearVelocityBody:simd_make_float3(linVelB.GetX(), linVelB.GetY(), linVelB.GetZ())
           baseAngularVelocityBody:simd_make_float3(angVelB.GetX(), angVelB.GetY(), angVelB.GetZ())
              gravityDirectionBody:simd_make_float3(gravB.GetX(), gravB.GetY(), gravB.GetZ())];
}

@end
