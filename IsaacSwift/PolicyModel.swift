//
//  PolicyModel.swift
//  IsaacSwift
//
//  Created by Tatsuya Ogawa on 2026/04/29.
//

import CoreML
import Foundation
import simd

struct PolicyUpdateScheduler {
    let updateInterval: TimeInterval
    private(set) var lastUpdateTime: TimeInterval?

    init(updateInterval: TimeInterval, lastUpdateTime: TimeInterval? = nil) {
        self.updateInterval = updateInterval
        self.lastUpdateTime = lastUpdateTime
    }

    mutating func shouldUpdate(at time: TimeInterval) -> Bool {
        guard let lastUpdateTime else {
            self.lastUpdateTime = time
            return true
        }

        guard time - lastUpdateTime >= updateInterval else {
            return false
        }

        self.lastUpdateTime = time
        return true
    }
}

struct PolicyJointFeedbackSnapshot {
    let jointPositionDeltas: [Float]
    let jointVelocities: [Float]
    let previousRawActions: [Float]
    let currentRawActions: [Float]
    let lastFeedbackTime: TimeInterval?
}

enum PolicyObservationLayout: Equatable {
    case isaacLab
    case go2Backflip(maxEpisodeLength: Int)
}

struct IsaacPolicyRuntimeConfiguration: Equatable {
    let robotKind: IsaacSwiftRobotKind
    let physicsTimeStep: TimeInterval
    let policyDecimation: Int
    let actionScale: Float
    let defaultCommand: SIMD3<Float>
    /// Policy-specific simulator joint defaults in local simulator/render
    /// joint order. When present, physics initialization and renderer rest
    /// angles both read from this single source.
    let defaultJointPositions: [Float]?
    /// Permutation that maps local Jolt simulator joint indices to the order
    /// the bundled policy was trained with. PhysX `dof_names` can differ from
    /// USD file order: Spot/ANYmal policies use type-grouped leg joints, while
    /// H1 uses a breadth-first traversal across pelvis, legs, torso, and arms.
    let simToPolicyJointPermutation: [Int]
    let observationLayout: PolicyObservationLayout

    init(robotKind: IsaacSwiftRobotKind,
         physicsTimeStep: TimeInterval,
         policyDecimation: Int,
         actionScale: Float,
         defaultCommand: SIMD3<Float>,
         defaultJointPositions: [Float]? = nil,
         simToPolicyJointPermutation: [Int],
         observationLayout: PolicyObservationLayout = .isaacLab) {
        self.robotKind = robotKind
        self.physicsTimeStep = physicsTimeStep
        self.policyDecimation = policyDecimation
        self.actionScale = actionScale
        self.defaultCommand = defaultCommand
        self.defaultJointPositions = defaultJointPositions
        self.simToPolicyJointPermutation = simToPolicyJointPermutation
        self.observationLayout = observationLayout
    }

    var jointCount: Int {
        simToPolicyJointPermutation.count
    }

    var policyUpdateInterval: TimeInterval {
        physicsTimeStep * Double(policyDecimation)
    }

    var usesOneStepActionLatency: Bool {
        switch observationLayout {
        case .isaacLab:
            return false
        case .go2Backflip:
            return true
        }
    }
}

protocol PolicyActionProvider: AnyObject {
    func currentActions(at time: TimeInterval) -> [Float]
    func resetPolicyState()
    func updateJointFeedback(_ jointPositions: [Float], at time: TimeInterval)
    func jointFeedbackSnapshot() -> PolicyJointFeedbackSnapshot?
}

extension PolicyActionProvider {
    func resetPolicyState() {}

    func updateJointFeedback(_ jointPositions: [Float], at time: TimeInterval) {
        _ = jointPositions
        _ = time
    }

    func jointFeedbackSnapshot() -> PolicyJointFeedbackSnapshot? {
        nil
    }
}

final class BufferedPolicyActionProvider: PolicyActionProvider {
    private let lock = NSLock()
    private var actions: [Float]

    init(actions: [Float] = []) {
        self.actions = actions
    }

    func updateActions(_ actions: [Float]) {
        lock.lock()
        self.actions = actions
        lock.unlock()
    }

    func currentActions(at time: TimeInterval) -> [Float] {
        lock.lock()
        let currentActions = actions
        lock.unlock()
        return currentActions
    }

    func jointFeedbackSnapshot() -> PolicyJointFeedbackSnapshot? {
        lock.lock()
        let currentActions = actions
        lock.unlock()
        return PolicyJointFeedbackSnapshot(jointPositionDeltas: [],
                                           jointVelocities: [],
                                           previousRawActions: currentActions,
                                           currentRawActions: currentActions,
                                           lastFeedbackTime: nil)
    }
}

struct PolicyModelConfiguration: Equatable {
    let resourceName: String
    let resourceExtension: String
    let inputFeatureName: String?
    let outputFeatureName: String?
    let repositoryRelativePath: String?

    static let spot = PolicyModelConfiguration(resourceName: "spot_policy",
                                               resourceExtension: "mlmodelc",
                                               inputFeatureName: "observations",
                                               outputFeatureName: "actions",
                                               repositoryRelativePath: "PolicyModels/spot_policy.mlmodelc")

    static let anymal = PolicyModelConfiguration(resourceName: "anymal_policy",
                                                 resourceExtension: "mlmodelc",
                                                 inputFeatureName: "observations",
                                                 outputFeatureName: "actions",
                                                 repositoryRelativePath: "PolicyModels/anymal_policy.mlmodelc")

    static let anymalRough = PolicyModelConfiguration(resourceName: "anymal_rough_policy",
                                                      resourceExtension: "mlmodelc",
                                                      inputFeatureName: "observations",
                                                      outputFeatureName: "actions",
                                                      repositoryRelativePath: "PolicyModels/anymal_rough_policy.mlmodelc")

    static let h1 = PolicyModelConfiguration(resourceName: "h1_policy",
                                             resourceExtension: "mlmodelc",
                                             inputFeatureName: "observations",
                                             outputFeatureName: "actions",
                                             repositoryRelativePath: "PolicyModels/h1_policy.mlmodelc")

    static let go2Flat = PolicyModelConfiguration(resourceName: "go2_policy",
                                                  resourceExtension: "mlmodelc",
                                                  inputFeatureName: "observations",
                                                  outputFeatureName: "actions",
                                                  repositoryRelativePath: "PolicyModels/go2_policy.mlmodelc")

    static let go2Rough = PolicyModelConfiguration(resourceName: "go2_rough_policy",
                                                   resourceExtension: "mlmodelc",
                                                   inputFeatureName: "observations",
                                                   outputFeatureName: "actions",
                                                   repositoryRelativePath: "PolicyModels/go2_rough_policy.mlmodelc")

    static let go2Backflip = PolicyModelConfiguration(resourceName: "go2_backflip_policy",
                                                      resourceExtension: "mlmodelc",
                                                      inputFeatureName: "observations",
                                                      outputFeatureName: "actions",
                                                      repositoryRelativePath: "PolicyModels/go2_backflip_policy.mlmodelc")

    static let go2 = go2Rough

    static func configuration(for robotKind: IsaacSwiftRobotKind) -> PolicyModelConfiguration {
        robotKind.modelDefinition.policyModelConfiguration
    }
}

final class DemoPolicyActionProvider: PolicyActionProvider {
    private struct FeedbackState {
        var jointPositionDeltas: [Float]
        var jointVelocities: [Float]
        var previousRawActions: [Float]
        var currentRawActions: [Float]
        var lastPolicyActions: [Float]
        var lastFeedbackTime: TimeInterval?
        // Body-frame base state required for a full Isaac Lab observation.
        // Defaults assume the robot is standing upright at rest.
        var baseLinearVelocityBody: SIMD3<Float> = .zero
        var baseAngularVelocityBody: SIMD3<Float> = .zero
        var projectedGravityBody: SIMD3<Float> = SIMD3<Float>(0, 0, -1)
        var velocityCommand: SIMD3<Float> = SIMD3<Float>(0.4, 0, 0)
        var basePositionWorld: SIMD3<Float> = SIMD3<Float>(0, 0, 0.5)
    }

    static let isaacPolicyUpdateInterval = IsaacPolicyRuntimeConfiguration.spotFlat.policyUpdateInterval

    let configuration: IsaacPolicyRuntimeConfiguration
    private let runner: PolicyModelRunner
    private let lock = NSLock()
    private var feedbackState: FeedbackState
    private var policyUpdateScheduler = PolicyUpdateScheduler(updateInterval: isaacPolicyUpdateInterval)
    static let defaultCommand = IsaacPolicyRuntimeConfiguration.spotFlat.defaultCommand

    /// Permutation that maps a sim-side joint index to the corresponding
    /// policy-observation joint index. Isaac Sim/PhysX returns `dof_names`
    /// in articulation traversal order, which for Spot tends to be grouped
    /// by joint type (`[*_hx, *_hy, *_kn]`) rather than leg-major. The Jolt
    /// simulator here is built leg-major, so the policy → sim direction may
    /// need a permutation to match the order the network was trained with.
    /// Default is identity (no remap). Length is robot-specific.
    var simToPolicyJointPermutation: [Int] = Array(0..<12) {
        didSet {
            precondition(Set(simToPolicyJointPermutation).count == simToPolicyJointPermutation.count)
        }
    }

    /// Inverse of `simToPolicyJointPermutation`. Computed lazily.
    private var policyToSimJointPermutation: [Int] {
        var inv = Array(repeating: 0, count: simToPolicyJointPermutation.count)
        for (sim, policy) in simToPolicyJointPermutation.enumerated() {
            inv[policy] = sim
        }
        return inv
    }

    init(runner: PolicyModelRunner,
         configuration: IsaacPolicyRuntimeConfiguration = .spotFlat,
         command: SIMD3<Float>? = nil) {
        self.configuration = configuration
        self.runner = runner
        let zeroActions = Array(repeating: Float(0), count: configuration.jointCount)
        self.feedbackState = FeedbackState(jointPositionDeltas: zeroActions,
                                           jointVelocities: zeroActions,
                                           previousRawActions: zeroActions,
                                           currentRawActions: zeroActions,
                                           lastPolicyActions: zeroActions,
                                           lastFeedbackTime: nil,
                                           velocityCommand: command ?? configuration.defaultCommand)
        self.policyUpdateScheduler = PolicyUpdateScheduler(updateInterval: configuration.policyUpdateInterval)
        self.simToPolicyJointPermutation = configuration.simToPolicyJointPermutation
    }

    /// Push body-frame base state into the policy observation. Should be
    /// called once per policy tick, with values straight from the physics
    /// simulator (`IsaacSwiftAnymalSimulator.currentObservation()`).
    func updateBaseFeedback(linearVelocityBody: SIMD3<Float>,
                            angularVelocityBody: SIMD3<Float>,
                            projectedGravityBody: SIMD3<Float>,
                            basePositionWorld: SIMD3<Float>? = nil) {
        lock.lock()
        feedbackState.baseLinearVelocityBody = linearVelocityBody
        feedbackState.baseAngularVelocityBody = angularVelocityBody
        feedbackState.projectedGravityBody = projectedGravityBody
        if let basePositionWorld {
            feedbackState.basePositionWorld = basePositionWorld
        }
        lock.unlock()
    }

    /// Push joint state directly from the simulator (avoids re-computing
    /// velocity via finite difference at a lower rate). Input is in **sim**
    /// joint order; the provider remaps it to the policy's joint order
    /// internally so the network sees the order it was trained with.
    func updateJointState(positionDeltas: [Float], velocities: [Float]) {
        let jointCount = configuration.jointCount
        let posSim = Array(positionDeltas.prefix(jointCount))
        let velSim = Array(velocities.prefix(jointCount))
        let perm = simToPolicyJointPermutation
        var posPolicy = Array(repeating: Float(0), count: jointCount)
        var velPolicy = Array(repeating: Float(0), count: jointCount)
        for (simIdx, value) in posSim.enumerated() where simIdx < jointCount {
            posPolicy[perm[simIdx]] = value
        }
        for (simIdx, value) in velSim.enumerated() where simIdx < jointCount {
            velPolicy[perm[simIdx]] = value
        }
        lock.lock()
        feedbackState.jointPositionDeltas = posPolicy
        feedbackState.jointVelocities = velPolicy
        lock.unlock()
    }

    func setVelocityCommand(_ command: SIMD3<Float>) {
        lock.lock()
        feedbackState.velocityCommand = command
        lock.unlock()
    }

    func currentActions(at time: TimeInterval) -> [Float] {
        lock.lock()
        if !policyUpdateScheduler.shouldUpdate(at: time) {
            let currentRawActions = feedbackState.currentRawActions
            lock.unlock()
            return currentRawActions
        }
        lock.unlock()

        let observations = demoObservations(at: time)
        let actions: [Float]
        do {
            actions = try runner.predictActions(observations: observations)
        } catch {
            fatalError("Policy prediction failed for \(configuration.robotKind): \(error)")
        }

        // The CoreML model emits actions in policy joint order. The simulator
        // applies them in sim joint order, so permute on the way out. The
        // policy-order copy is kept as `previousRawActions` because that is
        // what the next observation's `previous_action` slot expects.
        let jointCount = configuration.jointCount
        var policyActions = Array(repeating: Float(0), count: jointCount)
        for (idx, value) in actions.prefix(jointCount).enumerated() {
            policyActions[idx] = value
        }
        let permPolicyToSim = policyToSimJointPermutation
        var simActions = Array(repeating: Float(0), count: jointCount)
        for (policyIdx, value) in policyActions.enumerated() where policyIdx < jointCount {
            simActions[permPolicyToSim[policyIdx]] = value
        }

        lock.lock()
        if configuration.usesOneStepActionLatency {
            // Go2 backflip's task env observes the fresh `_actions` but
            // executes `_last_actions` for one 50 Hz control tick.
            let delayedPolicyActions = feedbackState.previousRawActions
            var delayedSimActions = Array(repeating: Float(0), count: jointCount)
            for (policyIdx, value) in delayedPolicyActions.enumerated() where policyIdx < jointCount {
                delayedSimActions[permPolicyToSim[policyIdx]] = value
            }
            feedbackState.lastPolicyActions = delayedPolicyActions
            feedbackState.previousRawActions = policyActions
            feedbackState.currentRawActions = delayedSimActions
            lock.unlock()
            return delayedSimActions
        }
        feedbackState.lastPolicyActions = feedbackState.previousRawActions
        feedbackState.previousRawActions = policyActions
        feedbackState.currentRawActions = simActions
        lock.unlock()
        return simActions
    }

    func resetPolicyState() {
        lock.lock()
        let command = feedbackState.velocityCommand
        let zero = Array(repeating: Float(0), count: configuration.jointCount)
        feedbackState = FeedbackState(jointPositionDeltas: zero,
                                      jointVelocities: zero,
                                      previousRawActions: zero,
                                      currentRawActions: zero,
                                      lastPolicyActions: zero,
                                      lastFeedbackTime: nil,
                                      velocityCommand: command)
        policyUpdateScheduler = PolicyUpdateScheduler(updateInterval: configuration.policyUpdateInterval)
        lock.unlock()
    }

    func updateJointFeedback(_ jointPositions: [Float], at time: TimeInterval) {
        let jointCount = configuration.jointCount
        let limitedPositions = Array(jointPositions.prefix(jointCount))
        guard limitedPositions.count == jointCount else {
            return
        }

        lock.lock()
        let previousPositions = feedbackState.jointPositionDeltas
        let velocities: [Float]
        if let lastFeedbackTime = feedbackState.lastFeedbackTime {
            let dt = max(Float(time - lastFeedbackTime), 1.0 / 120.0)
            velocities = zip(limitedPositions, previousPositions).map { current, previous in
                (current - previous) / dt
            }
        } else {
            velocities = Array(repeating: 0, count: limitedPositions.count)
        }

        feedbackState.jointPositionDeltas = limitedPositions
        feedbackState.jointVelocities = velocities
        feedbackState.lastFeedbackTime = time
        lock.unlock()
    }

    func jointFeedbackSnapshot() -> PolicyJointFeedbackSnapshot? {
        lock.lock()
        let feedbackState = self.feedbackState
        lock.unlock()
        return PolicyJointFeedbackSnapshot(jointPositionDeltas: feedbackState.jointPositionDeltas,
                                           jointVelocities: feedbackState.jointVelocities,
                                           previousRawActions: feedbackState.previousRawActions,
                                           currentRawActions: feedbackState.currentRawActions,
                                           lastFeedbackTime: feedbackState.lastFeedbackTime)
    }

    private func demoObservations(at time: TimeInterval) -> [Float] {
        var observations = runner.zeroObservations()

        lock.lock()
        let feedbackState = self.feedbackState
        lock.unlock()

        if case .go2Backflip(let maxEpisodeLength) = configuration.observationLayout {
            writeGo2BackflipObservations(&observations,
                                         feedbackState: feedbackState,
                                         time: time,
                                         maxEpisodeLength: maxEpisodeLength)
            return observations
        }

        // Isaac Lab locomotion observation layout:
        //   [0:3]   base_lin_vel_b
        //   [3:6]   base_ang_vel_b
        //   [6:9]   projected_gravity_b
        //   [9:12]  velocity_command (vx, vy, wz)
        //   [12:12+N] joint_pos - default_joint_pos
        //   [12+N:12+2N] joint_vel
        // Flat policies then store previous raw actions. Rough policies add a
        // 17x11 height scan; Go2 stores previous_action before height_scan,
        // while ANYmal-C Direct rough stores height_scan before previous_action.
        func write(_ value: SIMD3<Float>, at offset: Int) {
            if observations.count > offset + 0 { observations[offset + 0] = value.x }
            if observations.count > offset + 1 { observations[offset + 1] = value.y }
            if observations.count > offset + 2 { observations[offset + 2] = value.z }
        }
        write(feedbackState.baseLinearVelocityBody, at: 0)
        write(feedbackState.baseAngularVelocityBody, at: 3)
        write(feedbackState.projectedGravityBody, at: 6)
        write(feedbackState.velocityCommand, at: 9)

        let jointCount = configuration.jointCount
        let jointPositionOffset = 12
        let jointVelocityOffset = jointPositionOffset + jointCount
        let jointPositionCount = Swift.min(feedbackState.jointPositionDeltas.count,
                                           Swift.min(max(observations.count - jointPositionOffset, 0), jointCount))
        for index in 0..<jointPositionCount {
            observations[jointPositionOffset + index] = feedbackState.jointPositionDeltas[index]
        }

        let jointVelocityCount = Swift.min(feedbackState.jointVelocities.count,
                                           Swift.min(max(observations.count - jointVelocityOffset, 0), jointCount))
        for index in 0..<jointVelocityCount {
            observations[jointVelocityOffset + index] = feedbackState.jointVelocities[index]
        }

        let stateEndOffset = jointVelocityOffset + jointCount
        let usesAnymalRoughLayout = configuration.robotKind == .anymalC && observations.count == 235
        let heightScanOffset = usesAnymalRoughLayout ? stateEndOffset : stateEndOffset + jointCount
        let previousActionOffset = usesAnymalRoughLayout ? 223 : stateEndOffset

        let previousActionCount = Swift.min(feedbackState.previousRawActions.count,
                                            Swift.min(max(observations.count - previousActionOffset, 0), jointCount))
        for index in 0..<previousActionCount {
            observations[previousActionOffset + index] = feedbackState.previousRawActions[index]
        }

        // Rough-terrain policies use a 17x11 ray-cast height scan. The
        // current simulator exposes a flat ground plane, so every sample has
        // the same relative height: base_z - terrain_z - Isaac Lab's default
        // 0.5 m offset, clipped to the rough-env observation range.
        if observations.count > heightScanOffset {
            let heightScanEnd = usesAnymalRoughLayout ? previousActionOffset : observations.count
            let flatGroundHeight = Float(0)
            let heightSample = Swift.min(Swift.max(feedbackState.basePositionWorld.z - flatGroundHeight - 0.5, -1), 1)
            for index in heightScanOffset..<heightScanEnd {
                observations[index] = heightSample
            }
        }

        return observations
    }

    private func writeGo2BackflipObservations(_ observations: inout [Float],
                                              feedbackState: FeedbackState,
                                              time: TimeInterval,
                                              maxEpisodeLength: Int) {
        func write(_ value: SIMD3<Float>, at offset: Int, scale: Float = 1) {
            if observations.count > offset + 0 { observations[offset + 0] = value.x * scale }
            if observations.count > offset + 1 { observations[offset + 1] = value.y * scale }
            if observations.count > offset + 2 { observations[offset + 2] = value.z * scale }
        }

        write(feedbackState.baseAngularVelocityBody, at: 0, scale: 0.25)
        write(feedbackState.projectedGravityBody, at: 3)

        let jointCount = configuration.jointCount
        let jointPositionOffset = 6
        let jointVelocityOffset = jointPositionOffset + jointCount
        let actionOffset = jointVelocityOffset + jointCount
        let lastActionOffset = actionOffset + jointCount
        let phaseOffset = lastActionOffset + jointCount

        let jointPositionCount = Swift.min(feedbackState.jointPositionDeltas.count,
                                           Swift.min(max(observations.count - jointPositionOffset, 0), jointCount))
        for index in 0..<jointPositionCount {
            observations[jointPositionOffset + index] = feedbackState.jointPositionDeltas[index]
        }

        let jointVelocityCount = Swift.min(feedbackState.jointVelocities.count,
                                           Swift.min(max(observations.count - jointVelocityOffset, 0), jointCount))
        for index in 0..<jointVelocityCount {
            observations[jointVelocityOffset + index] = feedbackState.jointVelocities[index] * 0.05
        }

        let actionCount = Swift.min(feedbackState.previousRawActions.count,
                                    Swift.min(max(observations.count - actionOffset, 0), jointCount))
        for index in 0..<actionCount {
            observations[actionOffset + index] = feedbackState.previousRawActions[index]
        }

        let lastActionCount = Swift.min(feedbackState.lastPolicyActions.count,
                                        Swift.min(max(observations.count - lastActionOffset, 0), jointCount))
        for index in 0..<lastActionCount {
            observations[lastActionOffset + index] = feedbackState.lastPolicyActions[index]
        }

        guard observations.count >= phaseOffset + 6 else { return }
        let episodeStep = max(0, Int((time / configuration.policyUpdateInterval).rounded(.down)))
        let phase = Float.pi * Float(episodeStep) / Float(max(maxEpisodeLength, 1))
        observations[phaseOffset + 0] = sin(phase)
        observations[phaseOffset + 1] = cos(phase)
        observations[phaseOffset + 2] = sin(phase / 2.0)
        observations[phaseOffset + 3] = cos(phase / 2.0)
        observations[phaseOffset + 4] = sin(phase / 4.0)
        observations[phaseOffset + 5] = cos(phase / 4.0)
    }
}

enum PolicyModelError: LocalizedError {
    case resourceNotFound(String)
    case inputFeatureNotFound(String)
    case outputFeatureNotFound(String)
    case invalidInputShape(String)
    case invalidObservationCount(expected: Int, actual: Int)
    case outputValueMissing(String)

    var errorDescription: String? {
        switch self {
        case .resourceNotFound(let description):
            return "Policy model not found: \(description)"
        case .inputFeatureNotFound(let featureName):
            return "Policy model input feature not found: \(featureName)"
        case .outputFeatureNotFound(let featureName):
            return "Policy model output feature not found: \(featureName)"
        case .invalidInputShape(let description):
            return "Policy model input shape is invalid: \(description)"
        case .invalidObservationCount(let expected, let actual):
            return "Policy model expected \(expected) observations but received \(actual)"
        case .outputValueMissing(let featureName):
            return "Policy model output value missing for feature: \(featureName)"
        }
    }
}

final class PolicyModelRunner {
    let configuration: PolicyModelConfiguration
    let model: MLModel
    let inputFeatureName: String
    let outputFeatureName: String
    let inputShape: [NSNumber]
    let observationCount: Int

    convenience init(bundle: Bundle = .main) throws {
        try self.init(bundle: bundle, configuration: .spot)
    }

    convenience init(robotKind: IsaacSwiftRobotKind,
                     bundle: Bundle = .main) throws {
        try self.init(bundle: bundle, configuration: .configuration(for: robotKind))
    }

    init(bundle: Bundle = .main,
         configuration: PolicyModelConfiguration) throws {
        let modelURL = try Self.resolveModelURL(bundle: bundle, configuration: configuration)
        let model = try MLModel(contentsOf: modelURL)
        let modelDescription = model.modelDescription

        let inputFeatureName = try Self.resolveFeatureName(preferredName: configuration.inputFeatureName,
                                                           available: modelDescription.inputDescriptionsByName.keys,
                                                           errorBuilder: { .inputFeatureNotFound($0) })
        let outputFeatureName = try Self.resolveFeatureName(preferredName: configuration.outputFeatureName,
                                                            available: modelDescription.outputDescriptionsByName.keys,
                                                            errorBuilder: { .outputFeatureNotFound($0) })

        guard let inputDescription = modelDescription.inputDescriptionsByName[inputFeatureName],
              let multiArrayConstraint = inputDescription.multiArrayConstraint else {
            throw PolicyModelError.invalidInputShape(inputFeatureName)
        }

        let inputShape = multiArrayConstraint.shape
        let observationCount = try Self.elementCount(from: inputShape, featureName: inputFeatureName)

        self.configuration = configuration
        self.model = model
        self.inputFeatureName = inputFeatureName
        self.outputFeatureName = outputFeatureName
        self.inputShape = inputShape
        self.observationCount = observationCount
    }

    func predictActions(observations: [Float]) throws -> [Float] {
        guard observations.count == observationCount else {
            throw PolicyModelError.invalidObservationCount(expected: observationCount,
                                                           actual: observations.count)
        }

        let multiArray = try MLMultiArray(shape: inputShape, dataType: .float32)
        for (index, value) in observations.enumerated() {
            multiArray[index] = NSNumber(value: value)
        }

        let inputProvider = try MLDictionaryFeatureProvider(dictionary: [
            inputFeatureName: MLFeatureValue(multiArray: multiArray),
        ])
        let output = try model.prediction(from: inputProvider)

        guard let actions = output.featureValue(for: outputFeatureName)?.multiArrayValue else {
            throw PolicyModelError.outputValueMissing(outputFeatureName)
        }

        return (0..<actions.count).map { actions[$0].floatValue }
    }

    func zeroObservations() -> [Float] {
        Array(repeating: 0, count: observationCount)
    }

    static func bundledModelURL(bundle: Bundle = .main,
                                configuration: PolicyModelConfiguration = .spot) -> URL? {
        bundle.url(forResource: configuration.resourceName, withExtension: configuration.resourceExtension)
    }

    static func repositoryModelURL(configuration: PolicyModelConfiguration) -> URL? {
        guard let repositoryRelativePath = configuration.repositoryRelativePath else {
            return nil
        }

        return URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
            .appendingPathComponent(repositoryRelativePath)
    }

    private static func resolveModelURL(bundle: Bundle,
                                        configuration: PolicyModelConfiguration) throws -> URL {
        if let bundledURL = bundledModelURL(bundle: bundle, configuration: configuration) {
            return bundledURL
        }

        if let repositoryURL = repositoryModelURL(configuration: configuration) {
            if FileManager.default.fileExists(atPath: repositoryURL.path) {
                return repositoryURL
            }
        }

        throw PolicyModelError.resourceNotFound("\(configuration.resourceName).\(configuration.resourceExtension)")
    }

    private static func resolveFeatureName(preferredName: String?,
                                           available: Dictionary<String, MLFeatureDescription>.Keys,
                                           errorBuilder: (String) -> PolicyModelError) throws -> String {
        let availableNames = Array(available)

        if let preferredName {
            guard availableNames.contains(preferredName) else {
                throw errorBuilder(preferredName)
            }
            return preferredName
        }

        guard let firstFeatureName = availableNames.sorted().first else {
            throw errorBuilder("<none>")
        }
        return firstFeatureName
    }

    private static func elementCount(from shape: [NSNumber],
                                     featureName: String) throws -> Int {
        guard !shape.isEmpty else {
            throw PolicyModelError.invalidInputShape(featureName)
        }

        return try shape.reduce(1) { partialResult, dimension in
            let value = dimension.intValue
            guard value > 0 else {
                throw PolicyModelError.invalidInputShape("\(featureName)=\(shape)")
            }
            return partialResult * value
        }
    }
}
