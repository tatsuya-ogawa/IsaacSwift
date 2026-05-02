//
//  IsaacSwiftPhysics.h
//  IsaacSwift
//
//  Headless Jolt Physics simulator for an ANYmal-class quadruped on a ground
//  plane. Mirrors the structure of Isaac Sim's `AnymalFlatTerrainPolicy` so a
//  policy → physics → observation loop can run without a renderer attached.
//

#ifndef IsaacSwiftPhysics_h
#define IsaacSwiftPhysics_h

#import <Foundation/Foundation.h>
#import <simd/simd.h>

NS_ASSUME_NONNULL_BEGIN

/// Selects which morphology + parameter set the simulator should
/// instantiate. The quadrupeds share a floating-base 12-DOF leg topology;
/// H1 uses its 19-DOF humanoid articulation.
typedef NS_ENUM(NSInteger, IsaacSwiftRobotKind) {
    IsaacSwiftRobotKindAnymalC = 0,
    IsaacSwiftRobotKindSpot    = 1,
    IsaacSwiftRobotKindGo2     = 2,
    IsaacSwiftRobotKindH1      = 3,
};

/// Snapshot of the simulator state in the conventions Isaac Sim's ANYmal policy
/// expects. Vectors expressed in the base body frame use right-handed axes with
/// +X forward, +Y left, +Z up.
@interface IsaacSwiftAnymalObservation : NSObject
@property (nonatomic, readonly) NSArray<NSNumber *> *jointPositions;       ///< 12 absolute joint angles [rad]
@property (nonatomic, readonly) NSArray<NSNumber *> *jointPositionDeltas;  ///< current - default [rad]
@property (nonatomic, readonly) NSArray<NSNumber *> *jointVelocities;      ///< [rad/s]
@property (nonatomic, readonly) simd_float3 basePositionWorld;             ///< [m]
@property (nonatomic, readonly) simd_float4 baseOrientationWorldXYZW;      ///< quaternion (x,y,z,w)
@property (nonatomic, readonly) simd_float3 baseLinearVelocityBody;        ///< [m/s] in base frame
@property (nonatomic, readonly) simd_float3 baseAngularVelocityBody;       ///< [rad/s] in base frame
@property (nonatomic, readonly) simd_float3 gravityDirectionBody;          ///< unit vector pointing in the direction gravity acts, expressed in base frame
@end

/// Per-joint torque generator. Implementations are expected to map
/// `(joint_pos_err, joint_vel)` for all 12 joints to per-joint torques in
/// N·m. Called once per physics substep when installed on a simulator.
///
/// `jointPositionErrors[i] = target_angle - current_angle` for joint i,
/// using the simulator's joint-order convention (leg-major).
/// `jointVelocities[i]` is the joint's instantaneous angular velocity [rad/s].
///
/// The actuator may keep internal state (e.g. an LSTM hidden state); call
/// `resetState` whenever the simulator itself is reset so the recurrent
/// state is purged in lockstep.
@protocol IsaacSwiftJointActuator <NSObject>
- (void)resetState;
- (NSArray<NSNumber *> *)torquesForJointPositionErrors:(NSArray<NSNumber *> *)jointPositionErrors
                                       jointVelocities:(NSArray<NSNumber *> *)jointVelocities;
@end

/// Headless ANYmal-like articulated body simulator. The model is intentionally
/// approximate (one floating base + four 3-DOF legs articulated through hinges
/// with PD position motors). Quadruped joint indexing matches Isaac Sim's
/// flat-terrain policy:
///
///   `[LF_HAA, LF_HFE, LF_KFE, RF_HAA, RF_HFE, RF_KFE,
///     LH_HAA, LH_HFE, LH_KFE, RH_HAA, RH_HFE, RH_KFE]`
///
/// H1 uses the local USD/Jolt hinge order; policy I/O is remapped separately
/// by `IsaacPolicyRuntimeConfiguration.h1Flat`:
///
///   `[left_hip_yaw, right_hip_yaw, torso, left_hip_roll, left_hip_pitch,
///     left_knee, left_ankle, right_hip_roll, right_hip_pitch, right_knee,
///     right_ankle, left_shoulder_pitch, right_shoulder_pitch,
///     left_shoulder_roll, left_shoulder_yaw, left_elbow,
///     right_shoulder_roll, right_shoulder_yaw, right_elbow]`
@interface IsaacSwiftAnymalSimulator : NSObject

@property (nonatomic, readonly) IsaacSwiftRobotKind robotKind;
@property (nonatomic, readonly) NSUInteger jointCount;
@property (nonatomic, readonly) double physicsTimeStep;
@property (nonatomic, assign)   float jointStiffness;     ///< Kp [N·m/rad]
@property (nonatomic, assign)   float jointDamping;       ///< Kd [N·m·s/rad]
@property (nonatomic, assign)   float maxJointTorque;     ///< per-joint clamp [N·m]
/// Time-constant (seconds) for the EMA low-pass on motor target angles. The
/// commanded target is filtered with `alpha = dt / (tau + dt)` per substep.
/// This approximates SEA / actuator-network smoothing of the policy output.
/// Default is 8 ms for quadruped PD paths and 0 ms for H1, matching Isaac
/// Sim's direct per-step `ArticulationAction(joint_positions=...)` path.
@property (nonatomic, assign)   float motorTargetSmoothingTau;
/// Recommended `action_scale` for this robot's local Jolt-backed policy loop.
/// The policy loop multiplies raw policy outputs by this value before treating
/// them as joint position deltas.
@property (nonatomic, readonly) float recommendedActionScale;
/// Recommended physics step for this robot's policy/simulator pairing.
/// ANYmal-C and Go2 use 1/200 s; Spot Flat uses Isaac Lab's 1/500 s.
@property (nonatomic, readonly) double recommendedPhysicsTimeStep;
/// Default joint positions used as the policy's reference pose. Length equals
/// `jointCount`.
@property (nonatomic, copy)     NSArray<NSNumber *> *defaultJointPositions;
/// World-space spawn pose for the base; written by `reset`.
@property (nonatomic, assign)   simd_float3 spawnPositionWorld;

/// Optional learned actuator (e.g. ANYmal's `sea_net_jit2.pt` ported to
/// CoreML). The commanded scaled action is treated as a target joint angle
/// delta; the simulator computes `pos_err = target - current` and
/// `joint_vel` and hands them to the actuator. In Isaac Sim this runs as
/// pure effort control; the local Jolt approximation keeps a small PD spring
/// at the default pose to stabilize the simplified articulation. Setting
/// this to `nil` reverts to the built-in PD target-tracking path.
@property (nonatomic, strong, nullable) id<IsaacSwiftJointActuator> jointActuator;

- (instancetype)initWithRobotKind:(IsaacSwiftRobotKind)robotKind
                  physicsTimeStep:(double)physicsTimeStep NS_DESIGNATED_INITIALIZER;
/// Convenience: ANYmal-C with the given physics step.
- (instancetype)initWithPhysicsTimeStep:(double)physicsTimeStep;
/// Convenience: ANYmal-C at 200 Hz.
- (instancetype)init;

/// Snap base back to spawn pose, joints to defaults, all velocities to zero.
- (void)reset;

/// Advance physics by `elapsedTime` seconds. The motor target for joint `i`
/// is `defaultJointPositions[i] + scaledActions[i]`; if the array is shorter
/// than 12, the missing entries are treated as zero. Returns the joint
/// position deltas after the step (current - default), 12 floats.
- (NSArray<NSNumber *> *)stepWithScaledActions:(NSArray<NSNumber *> *)scaledActions
                                  elapsedTime:(double)elapsedTime;

/// Latest observation without integrating.
- (IsaacSwiftAnymalObservation *)currentObservation;

@end

NS_ASSUME_NONNULL_END

#endif /* IsaacSwiftPhysics_h */
