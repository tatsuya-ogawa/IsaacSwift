//
//  PolicyPhysicsLoop.swift
//  IsaacSwift
//
//  Encapsulates the policy / physics tick that used to live inside Renderer.
//  Renderer-free: drives an `IsaacSwiftAnymalSimulator` from a
//  `PolicyActionProvider` so it can be exercised headlessly in tests.
//

import Foundation
import simd

/// Drives one Isaac-Lab-style policy step + N physics substeps per tick.
///
/// The contract mirrors Isaac Sim's ANYmal velocity controller:
/// * ANYmal-C: `physics_dt = 1/200`, `decimation = 4`
/// * Spot Flat: `physics_dt = 1/500`, `decimation = 10`
/// * policy runs at 50 Hz for both
/// * PD position drive on each hinge (`Kp`/`Kd` baked into the simulator)
final class PolicyPhysicsLoop {
    struct AnymalActuatorRuntimeTuning: Equatable {
        let jointStiffness: Float
        let jointDamping: Float
        let maxJointTorque: Float
        let motorTargetSmoothingTau: Float
        let spawnZ: Float
    }

    static let anymalActuatorTuning = AnymalActuatorRuntimeTuning(jointStiffness: 80,
                                                                  jointDamping: 6,
                                                                  maxJointTorque: 120,
                                                                  motorTargetSmoothingTau: 0,
                                                                  spawnZ: 0.58)

    /// Action scale for this loop's robot kind (Isaac Lab `action_scale`).
    var actionScale: Float { configuration.actionScale }

    let configuration: IsaacPolicyRuntimeConfiguration
    let simulator: IsaacSwiftAnymalSimulator
    private let provider: PolicyActionProvider?
    private(set) var lastJointDeltas: [Float]
    private(set) var lastRawActions: [Float]
    private var lastTime: TimeInterval?
    /// Monotonic logical time fed to the provider's update scheduler. Always
    /// advances by exactly `policyUpdateInterval` per internal tick, even
    /// when the wall-clock call rate differs (60 Hz / 120 Hz frames vs
    /// 50 Hz policy).
    private var logicalTime: TimeInterval = 0
    /// Wall-clock time owed to the policy/physics that has not yet been
    /// consumed by an internal tick.
    private var timeAccumulator: TimeInterval = 0

    /// When true, `step(at:)` returns the last known deltas without advancing
    /// the simulation. Used for step-by-step debugging.
    var paused: Bool = false

    var hasJointActuator: Bool {
        simulator.jointActuator != nil
    }

    /// Incremented each time the simulator actually advances (for UI sync).
    private(set) var stepCount: Int = 0

    init(simulator: IsaacSwiftAnymalSimulator = IsaacSwiftAnymalSimulator(robotKind: .anymalC,
                                                                          physicsTimeStep: IsaacPolicyRuntimeConfiguration.anymalC.physicsTimeStep),
         configuration: IsaacPolicyRuntimeConfiguration = .anymalC,
         provider: PolicyActionProvider?) {
        self.configuration = configuration
        self.simulator = simulator
        self.provider = provider
        let zero = Array(repeating: Float(0), count: Int(simulator.jointCount))
        self.lastJointDeltas = zero
        self.lastRawActions = zero
        Self.installDefaultActuator(on: simulator, configuration: configuration)
    }

    /// Convenience initializer that builds a fresh simulator with the runtime
    /// settings for the given robot kind.
    convenience init(robotKind: IsaacSwiftRobotKind,
                     provider: PolicyActionProvider?) {
        let configuration = IsaacPolicyRuntimeConfiguration.configuration(for: robotKind)
        let sim = IsaacSwiftAnymalSimulator(robotKind: robotKind,
                                            physicsTimeStep: configuration.physicsTimeStep)
        self.init(simulator: sim, configuration: configuration, provider: provider)
    }

    /// For ANYmal-C we always pair the simulator with the bundled ANYdrive
    /// `ActuatorNetLSTM` (`AnymalActuatorRunner`). The runner adds a learned
    /// torque on top of the hinge PD; together with a low PD anchored at
    /// the default pose this is what lets the bundled CoreML policy walk
    /// forward for the full eval window. The simulator's stock PD-only
    /// defaults (kp=80, kd=2, maxTorque=40) are tuned for the actuator-
    /// less path, so we override them here when the LSTM is installed.
    /// Other robot kinds use the plain PD-only path with their own defaults.
    private static func installDefaultActuator(on simulator: IsaacSwiftAnymalSimulator,
                                               configuration: IsaacPolicyRuntimeConfiguration) {
        guard configuration.robotKind == .anymalC, simulator.jointActuator == nil else { return }
        let actuator: AnymalActuatorRunner
        do {
            actuator = try AnymalActuatorRunner(bundle: .main)
        } catch {
            fatalError("ANYmal actuator warm-up failed: \(error)")
        }
        // Tuned by `anymalPolicyJointPermutationAndGainSweep`: keeps ANYmal
        // upright and walking when paired with the LSTM actuator. Tests read
        // this same value instead of keeping their own copy.
        let tuning = anymalActuatorTuning
        simulator.jointStiffness = tuning.jointStiffness
        simulator.jointDamping = tuning.jointDamping
        simulator.maxJointTorque = tuning.maxJointTorque
        simulator.motorTargetSmoothingTau = tuning.motorTargetSmoothingTau
        simulator.spawnPositionWorld = SIMD3<Float>(0, 0, tuning.spawnZ)
        simulator.jointActuator = actuator
        simulator.reset()
    }

    func reset() {
        simulator.reset()
        provider?.resetPolicyState()
        let zero = Array(repeating: Float(0), count: Int(simulator.jointCount))
        lastJointDeltas = zero
        lastRawActions = zero
        lastTime = nil
        logicalTime = 0
        timeAccumulator = 0
        stepCount = 0
    }

    /// Drive the policy + simulator forward to `time`. Returns the post-step
    /// joint angle deltas (relative to the standing default pose) so the
    /// renderer can map them onto USD bones.
    ///
    /// `time` is wall-clock (e.g. `CACurrentMediaTime()`) and the renderer
    /// calls this once per drawn frame, so the call rate is whatever the
    /// display happens to be (typically 60 Hz on iPhone, 120 Hz on ProMotion
    /// devices). The policy was trained at a fixed 50 Hz, so we **decouple**
    /// the call rate from the policy rate by accumulating elapsed wall time
    /// and running zero or more fixed-size policy/physics ticks per call.
    /// Each internal tick advances exactly `policyUpdateInterval` of physics
    /// and runs one inference, matching the training cadence regardless of
    /// frame timing.
    @discardableResult
    func step(at time: TimeInterval) -> [Float] {
        if paused { return lastJointDeltas }

        let wallElapsed: TimeInterval
        if let last = lastTime {
            wallElapsed = max(0, time - last)
        } else {
            // First call after reset: run one policy tick immediately so the
            // returned deltas reflect real motor targets, not the zero-init.
            wallElapsed = configuration.policyUpdateInterval
        }
        lastTime = time
        timeAccumulator += wallElapsed

        let policyDt = configuration.policyUpdateInterval
        // Cap how many catch-up ticks we run per frame so a single huge frame
        // gap (e.g. backgrounded then resumed) cannot explode into hundreds
        // of physics updates. 4 ticks/frame is enough headroom for 240 Hz
        // ProMotion vs 50 Hz policy.
        var ticks = 0
        while timeAccumulator + 1e-9 >= policyDt && ticks < 8 {
            advanceOneTick()
            timeAccumulator -= policyDt
            ticks += 1
        }
        if timeAccumulator < 0 { timeAccumulator = 0 }
        return lastJointDeltas
    }

    /// Advances exactly one fixed policy/physics tick, independent of the
    /// wall-clock scheduler used by the renderer. Debug controls use this so
    /// manual stepping remains deterministic even after `step(at:)` has been
    /// driven by `CACurrentMediaTime()`.
    @discardableResult
    func stepOnePolicyTick() -> [Float] {
        advanceOneTick()
        lastTime = nil
        timeAccumulator = 0
        return lastJointDeltas
    }

    /// One fixed-rate policy + physics tick. Should be called at exactly
    /// `policyUpdateInterval` intervals (caller manages the schedule).
    private func advanceOneTick() {
        let policyDt = configuration.policyUpdateInterval
        let observation = simulator.currentObservation()
        publishObservationFeedback(observation, at: logicalTime)

        let scaled: [Float]
        if let provider {
            let rawActions = provider.currentActions(at: logicalTime)
            if rawActions.isEmpty {
                scaled = Array(repeating: 0, count: Int(simulator.jointCount))
            } else {
                lastRawActions = rawActions
                scaled = rawActions.map { $0 * self.actionScale }
            }
        } else {
            scaled = Array(repeating: 0, count: Int(simulator.jointCount))
        }
        _ = advancePhysics(scaledActions: scaled, elapsed: policyDt)
        logicalTime += policyDt
        stepCount += 1
    }

    private func advancePhysics(scaledActions: [Float], elapsed: TimeInterval) -> [Float] {
        let nsScaled: [NSNumber] = scaledActions.map { NSNumber(value: $0) }
        let returned = simulator.step(withScaledActions: nsScaled, elapsedTime: elapsed)
        let deltas = returned.map { $0.floatValue }
        lastJointDeltas = deltas
        return deltas
    }

    private func publishObservationFeedback(_ observation: IsaacSwiftAnymalObservation,
                                            at time: TimeInterval) {
        guard let provider else { return }

        if let demo = provider as? DemoPolicyActionProvider {
            // Push all observation components directly from the simulator
            // to avoid re-computing velocity via finite difference.
            let positions = observation.jointPositionDeltas.map { $0.floatValue }
            let velocities = observation.jointVelocities.map { $0.floatValue }
            demo.updateJointState(positionDeltas: positions, velocities: velocities)
            demo.updateBaseFeedback(linearVelocityBody: observation.baseLinearVelocityBody,
                                    angularVelocityBody: observation.baseAngularVelocityBody,
                                    projectedGravityBody: observation.gravityDirectionBody)
        } else {
            let positions = observation.jointPositionDeltas.map { $0.floatValue }
            provider.updateJointFeedback(positions, at: time)
        }
    }
}
