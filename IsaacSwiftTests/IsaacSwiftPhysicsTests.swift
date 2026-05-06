//
//  IsaacSwiftPhysicsTests.swift
//  IsaacSwiftTests
//
//  Headless convergence tests for the Jolt-backed ANYmal simulator and the
//  `PolicyPhysicsLoop` that wraps it. These tests deliberately bypass Metal
//  and Renderer so they can run on `My Mac (Designed for iPad)` without GPU.
//

import Foundation
import Testing
import simd
@testable import IsaacSwift

private final class IsaacSwiftPhysicsTestBundleLocator {}

private struct RolloutStats {
    var minBaseZ: Float = .greatestFiniteMagnitude
    var maxBaseZ: Float = -.greatestFiniteMagnitude
    var minBaseUprightZ: Float = .greatestFiniteMagnitude
    var minJointDeltas: [Float] = Array(repeating: .greatestFiniteMagnitude, count: 19)
    var maxJointDeltas: [Float] = Array(repeating: -.greatestFiniteMagnitude, count: 19)
    var maxAbsJointDelta: Float = 0
    var maxAbsJointVelocity: Float = 0
    var sawNaN: Bool = false
    var endBasePos: SIMD3<Float> = .zero
    var endBaseUprightZ: Float = 0
}

private struct Go2BackflipTraceSample {
    var frame: Int
    var time: Double
    var x: Float
    var z: Float
    var uprightZ: Float
}

private struct Go2BackflipTuningCandidate: CustomStringConvertible {
    var spawnZ: Float
    var kp: Float
    var kd: Float
    var maxTorque: Float
    var actionScale: Float

    var description: String {
        String(format: "spawnZ=%.3f kp=%.1f kd=%.1f T=%.2f scale=%.3f",
               Double(spawnZ), Double(kp), Double(kd),
               Double(maxTorque), Double(actionScale))
    }
}

private struct Go2BackflipTraceFit: CustomStringConvertible {
    var candidate: Go2BackflipTuningCandidate
    var rmsZ: Float
    var rmsUpright: Float
    var peakZ: Float
    var peakZFrame: Int
    var endZ: Float
    var endUpright: Float
    var minZ: Float
    var score: Float

    var description: String {
        String(format: "score=%.3f rmsZ=%.3f rmsUp=%.3f peakZ=%.3f@%d endZ=%.3f endUp=%.3f minZ=%.3f %@",
               Double(score), Double(rmsZ), Double(rmsUpright),
               Double(peakZ), peakZFrame, Double(endZ),
               Double(endUpright), Double(minZ), candidate.description)
    }
}

private struct SpotStabilityCandidate: CustomStringConvertible {
    var spawnZ: Float
    var kp: Float
    var kd: Float
    var maxTorque: Float
    var actionScale: Float
    var commandX: Float
    var haa: Float
    var hfeFront: Float
    var hfeHind: Float
    var kfe: Float

    var defaultJointPositions: [Float] {
        [
            +haa, hfeFront, kfe,
            -haa, hfeFront, kfe,
            +haa, hfeHind,  kfe,
            -haa, hfeHind,  kfe,
        ]
    }

    var description: String {
        String(format: "spawnZ=%.3f kp=%.1f kd=%.1f torque=%.1f actionScale=%.3f commandX=%.3f haa=%.3f hfeFront=%.3f hfeHind=%.3f kfe=%.3f",
               Double(spawnZ), Double(kp), Double(kd), Double(maxTorque),
               Double(actionScale), Double(commandX), Double(haa),
               Double(hfeFront), Double(hfeHind), Double(kfe))
    }
}

private struct SpotCandidateResult: CustomStringConvertible {
    var candidate: SpotStabilityCandidate
    var stats: RolloutStats
    var physicsSteps: Int
    var requiredPhysicsSteps: Int
    var score: Float

    var isNonRolling: Bool {
        physicsSteps >= requiredPhysicsSteps &&
        !stats.sawNaN &&
        stats.minBaseZ > 0.18 &&
        stats.minBaseUprightZ > 0.90 &&
        stats.maxAbsJointVelocity < 80.0 &&
        stats.maxAbsJointDelta < 1.2
    }

    var isForwardWalking: Bool {
        isNonRolling && stats.endBasePos.x > 0.10
    }

    var description: String {
        String(format: "score=%.3f steps=%d x=%.3f uprightMin=%.3f z=%.3f...%.3f jointDelta=%.3f jointVel=%.3f %@",
               Double(score), physicsSteps, Double(stats.endBasePos.x),
               Double(stats.minBaseUprightZ), Double(stats.minBaseZ), Double(stats.maxBaseZ),
               Double(stats.maxAbsJointDelta), Double(stats.maxAbsJointVelocity),
               candidate.description)
    }
}

private final class ConstantJointActuator: NSObject, IsaacSwiftJointActuator {
    let torques: [Float]

    init(torques: [Float]) {
        self.torques = torques
    }

    func resetState() {}

    func torques(forJointPositionErrors jointPositionErrors: [NSNumber],
                 jointVelocities: [NSNumber]) -> [NSNumber] {
        torques.map { NSNumber(value: $0) }
    }
}

@MainActor
struct IsaacSwiftPhysicsTests {

    private func observe(_ obs: IsaacSwiftAnymalObservation, into stats: inout RolloutStats) {
        stats.minBaseZ = min(stats.minBaseZ, obs.basePositionWorld.z)
        stats.maxBaseZ = max(stats.maxBaseZ, obs.basePositionWorld.z)
        let q = obs.baseOrientationWorldXYZW
        let uprightZ = 1 - 2 * (q.x * q.x + q.y * q.y)
        stats.minBaseUprightZ = min(stats.minBaseUprightZ, uprightZ)
        for (index, d) in obs.jointPositionDeltas.enumerated() {
            let v = abs(d.floatValue)
            if index < stats.minJointDeltas.count {
                stats.minJointDeltas[index] = min(stats.minJointDeltas[index], d.floatValue)
                stats.maxJointDeltas[index] = max(stats.maxJointDeltas[index], d.floatValue)
            }
            stats.maxAbsJointDelta = max(stats.maxAbsJointDelta, v)
            if !v.isFinite { stats.sawNaN = true }
        }
        for v in obs.jointVelocities {
            let f = abs(v.floatValue)
            stats.maxAbsJointVelocity = max(stats.maxAbsJointVelocity, f)
            if !f.isFinite { stats.sawNaN = true }
        }
        if !obs.basePositionWorld.x.isFinite ||
           !obs.basePositionWorld.y.isFinite ||
           !obs.basePositionWorld.z.isFinite {
            stats.sawNaN = true
        }
    }

    private func runSimulator(_ sim: IsaacSwiftAnymalSimulator,
                              seconds: Double,
                              controlInterval: Double = 1.0 / 50.0,
                              actions: (IsaacSwiftAnymalObservation, Double) -> [Float]) -> RolloutStats {
        var stats = RolloutStats()
        var t: Double = 0
        var obs = sim.currentObservation()
        let dt = controlInterval
        while t < seconds {
            let scaled = actions(obs, t)
            let nsScaled: [NSNumber] = scaled.map { NSNumber(value: $0) }
            _ = sim.step(withScaledActions: nsScaled, elapsedTime: dt)
            obs = sim.currentObservation()
            observe(obs, into: &stats)
            t += dt
        }
        stats.endBasePos = obs.basePositionWorld
        let q = obs.baseOrientationWorldXYZW
        stats.endBaseUprightZ = 1 - 2 * (q.x * q.x + q.y * q.y)
        return stats
    }

    private func runLoop(_ loop: PolicyPhysicsLoop,
                         seconds: Double,
                         controlInterval: Double = 1.0 / 50.0) -> RolloutStats {
        var stats = RolloutStats()
        var t: Double = 0
        let dt = controlInterval
        while t < seconds {
            _ = loop.step(at: t)
            let obs = loop.simulator.currentObservation()
            observe(obs, into: &stats)
            t += dt
        }
        let obs = loop.simulator.currentObservation()
        stats.endBasePos = obs.basePositionWorld
        let q = obs.baseOrientationWorldXYZW
        stats.endBaseUprightZ = 1 - 2 * (q.x * q.x + q.y * q.y)
        return stats
    }

    private func runGo2BackflipTrace(candidate: Go2BackflipTuningCandidate) throws -> [Go2BackflipTraceSample] {
        let runner = try PolicyModelRunner(configuration: .go2Backflip)
        let base = IsaacPolicyRuntimeConfiguration.go2Backflip
        let cfg = IsaacPolicyRuntimeConfiguration(robotKind: base.robotKind,
                                                  physicsTimeStep: base.physicsTimeStep,
                                                  policyDecimation: base.policyDecimation,
                                                  actionScale: candidate.actionScale,
                                                  defaultCommand: base.defaultCommand,
                                                  simToPolicyJointPermutation: base.simToPolicyJointPermutation,
                                                  observationLayout: base.observationLayout)
        let provider = DemoPolicyActionProvider(runner: runner,
                                                configuration: cfg)
        let sim = IsaacSwiftAnymalSimulator(robotKind: cfg.robotKind,
                                            physicsTimeStep: cfg.physicsTimeStep)
        let loop = PolicyPhysicsLoop(simulator: sim,
                                     configuration: cfg,
                                     provider: provider)

        sim.defaultJointPositions = [
            0.0, 0.8, -1.5,
            0.0, 0.8, -1.5,
            0.0, 1.0, -1.5,
            0.0, 1.0, -1.5,
        ].map { NSNumber(value: $0) }
        sim.spawnPositionWorld = SIMD3<Float>(0, 0, candidate.spawnZ)
        sim.jointStiffness = candidate.kp
        sim.jointDamping = candidate.kd
        sim.maxJointTorque = candidate.maxTorque
        sim.motorTargetSmoothingTau = 0
        loop.reset()

        let dt = cfg.policyUpdateInterval
        return (0..<100).map { frame in
            _ = loop.step(at: Double(frame) * dt)
            let obs = loop.simulator.currentObservation()
            let q = obs.baseOrientationWorldXYZW
            let upright = 1 - 2 * (q.x * q.x + q.y * q.y)
            return Go2BackflipTraceSample(frame: frame,
                                          time: Double(frame) * dt,
                                          x: obs.basePositionWorld.x,
                                          z: obs.basePositionWorld.z,
                                          uprightZ: upright)
        }
    }

    private func loadGo2BackflipPlayTrace() -> [Go2BackflipTraceSample]? {
        let env = ProcessInfo.processInfo.environment
        let bundleURL = Bundle(for: IsaacSwiftPhysicsTestBundleLocator.self)
            .url(forResource: "go2_backflip_play_trace", withExtension: "csv")
        let fallbackURL = repositoryRootURL()
            .appendingPathComponent("tmp/play_trace/play_trace.csv")
        let url = env["ISAACSWIFT_GO2_BACKFLIP_TRACE_CSV"].map { URL(fileURLWithPath: $0) }
            ?? bundleURL
            ?? fallbackURL
        guard let raw = try? String(contentsOf: url, encoding: .utf8) else {
            return nil
        }
        let lines = raw.components(separatedBy: .newlines).filter { !$0.isEmpty }
        guard lines.count > 1 else { return nil }

        let header = lines[0].components(separatedBy: ",")
        func index(_ name: String) -> Int? { header.firstIndex(of: name) }
        guard let frameIdx = index("frame"),
              let timeIdx = index("time_s"),
              let xIdx = index("post_root_pos_w_x"),
              let zIdx = index("post_root_pos_w_z"),
              let qxIdx = index("post_root_quat_w_x"),
              let qyIdx = index("post_root_quat_w_y") else {
            return nil
        }

        return lines.dropFirst().compactMap { line in
            let values = line.components(separatedBy: ",")
            guard values.count == header.count,
                  let frame = Int(values[frameIdx]),
                  let time = Double(values[timeIdx]),
                  let x = Float(values[xIdx]),
                  let z = Float(values[zIdx]),
                  let qx = Float(values[qxIdx]),
                  let qy = Float(values[qyIdx]) else {
                return nil
            }
            let upright = 1 - 2 * (qx * qx + qy * qy)
            return Go2BackflipTraceSample(frame: frame,
                                          time: time,
                                          x: x,
                                          z: z,
                                          uprightZ: upright)
        }
    }

    private func repositoryRootURL() -> URL {
        URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
    }

    private func shouldRunGo2BackflipTraceFit() -> Bool {
        let env = ProcessInfo.processInfo.environment
        if env["ISAACSWIFT_GO2_BACKFLIP_TRACE_FIT"] == "1" {
            return true
        }
        let marker = repositoryRootURL()
            .appendingPathComponent("tmp/.go2_backflip_trace_fit")
        return FileManager.default.fileExists(atPath: marker.path)
    }

    private func scoreGo2BackflipTrace(candidate: Go2BackflipTuningCandidate,
                                       actual: [Go2BackflipTraceSample],
                                       reference: [Go2BackflipTraceSample]) -> Go2BackflipTraceFit {
        let count = min(actual.count, reference.count)
        var sumZ: Float = 0
        var sumUpright: Float = 0
        var peakZ: Float = -.greatestFiniteMagnitude
        var peakZFrame = 0
        var minZ: Float = .greatestFiniteMagnitude

        for index in 0..<count {
            let dz = actual[index].z - reference[index].z
            let du = actual[index].uprightZ - reference[index].uprightZ
            sumZ += dz * dz
            sumUpright += du * du
            if actual[index].z > peakZ {
                peakZ = actual[index].z
                peakZFrame = actual[index].frame
            }
            minZ = min(minZ, actual[index].z)
        }

        let denom = Float(max(count, 1))
        let rmsZ = sqrt(sumZ / denom)
        let rmsUpright = sqrt(sumUpright / denom)
        let end = actual[min(max(count - 1, 0), actual.count - 1)]
        let refPeakFrame = reference.max(by: { $0.z < $1.z })?.frame ?? 46
        let refPeakZ = reference.map(\.z).max() ?? 1.0
        let refEndZ = reference.last?.z ?? 0.27
        let refEndUpright = reference.last?.uprightZ ?? 0.98
        let score = rmsZ * 2.0
            + rmsUpright * 0.6
            + abs(peakZ - refPeakZ) * 1.8
            + abs(Float(peakZFrame - refPeakFrame)) * 0.02
            + abs(end.z - refEndZ) * 1.0
            + abs(end.uprightZ - refEndUpright) * 0.3
            + max(0, 0.15 - minZ) * 2.0
            + max(0, 0.75 - end.uprightZ) * 1.5
        return Go2BackflipTraceFit(candidate: candidate,
                                   rmsZ: rmsZ,
                                   rmsUpright: rmsUpright,
                                   peakZ: peakZ,
                                   peakZFrame: peakZFrame,
                                   endZ: end.z,
                                   endUpright: end.uprightZ,
                                   minZ: minZ,
                                   score: score)
    }

    private func spotSearchCandidates(full: Bool) -> [SpotStabilityCandidate] {
        let gainProfiles: [(Float, Float, Float)] = full
            ? [(120, 8, 120), (180, 12, 180), (240, 18, 240), (320, 28, 320), (420, 40, 420)]
            : [(180, 12, 180), (240, 18, 240), (320, 28, 320)]
        let spawnZs: [Float] = full ? [0.485, 0.500, 0.515, 0.530] : [0.500, 0.515]
        let actionScales: [Float] = full ? [0.000, 0.025, 0.050, 0.075, 0.100, 0.150, 0.200] : [0.000, 0.050, 0.075, 0.100]
        let commandXs: [Float] = full ? [0.0, 0.4, 0.8, 1.2] : [0.4, 0.8, 1.2]
        let poses: [(Float, Float, Float, Float)] = full
            ? [
                (0.08, 0.80, 1.00, -1.40),
                (0.10, 0.90, 1.10, -1.50),
                (0.12, 0.90, 1.10, -1.60),
                (0.15, 0.80, 1.00, -1.50),
                (0.18, 0.75, 0.95, -1.45),
            ]
            : [
                (0.10, 0.90, 1.10, -1.50),
                (0.12, 0.90, 1.10, -1.60),
                (0.15, 0.80, 1.00, -1.50),
            ]

        var candidates: [SpotStabilityCandidate] = []
        for (kp, kd, torque) in gainProfiles {
            for spawnZ in spawnZs {
                for actionScale in actionScales {
                    for commandX in commandXs {
                        for (haa, hfeFront, hfeHind, kfe) in poses {
                            candidates.append(SpotStabilityCandidate(spawnZ: spawnZ,
                                                                     kp: kp,
                                                                     kd: kd,
                                                                     maxTorque: torque,
                                                                     actionScale: actionScale,
                                                                     commandX: commandX,
                                                                     haa: haa,
                                                                     hfeFront: hfeFront,
                                                                     hfeHind: hfeHind,
                                                                     kfe: kfe))
                        }
                    }
                }
            }
        }
        return candidates
    }

    private func evaluateSpotPolicyCandidate(_ candidate: SpotStabilityCandidate,
                                             runner: PolicyModelRunner,
                                             seconds: Double) -> SpotCandidateResult {
        let config = IsaacPolicyRuntimeConfiguration.spotFlat
        let sim = IsaacSwiftAnymalSimulator(robotKind: config.robotKind,
                                            physicsTimeStep: config.physicsTimeStep)
        sim.jointStiffness = candidate.kp
        sim.jointDamping = candidate.kd
        sim.maxJointTorque = candidate.maxTorque
        sim.spawnPositionWorld = SIMD3<Float>(0, 0, candidate.spawnZ)
        sim.defaultJointPositions = candidate.defaultJointPositions.map { NSNumber(value: $0) }
        sim.reset()

        let provider = DemoPolicyActionProvider(runner: runner,
                                                command: SIMD3<Float>(candidate.commandX, 0, 0))
        let policyDt = DemoPolicyActionProvider.isaacPolicyUpdateInterval
        var stats = RolloutStats()
        var physicsSteps = 0
        var t: Double = 0

        while t < seconds {
            var obs = sim.currentObservation()
            provider.updateJointState(positionDeltas: obs.jointPositionDeltas.map { $0.floatValue },
                                      velocities: obs.jointVelocities.map { $0.floatValue })
            provider.updateBaseFeedback(linearVelocityBody: obs.baseLinearVelocityBody,
                                        angularVelocityBody: obs.baseAngularVelocityBody,
                                        projectedGravityBody: obs.gravityDirectionBody)

            let rawActions = provider.currentActions(at: t)
            let scaled = (0..<Int(sim.jointCount)).map { index -> Float in
                index < rawActions.count ? rawActions[index] * candidate.actionScale : 0
            }
            _ = sim.step(withScaledActions: scaled.map { NSNumber(value: $0) },
                         elapsedTime: policyDt)
            physicsSteps += Int((policyDt / sim.physicsTimeStep).rounded())

            obs = sim.currentObservation()
            observe(obs, into: &stats)
            t += policyDt

            if stats.sawNaN ||
               stats.minBaseZ < 0.12 ||
               stats.minBaseUprightZ < 0.50 ||
               stats.maxAbsJointVelocity > 250.0 {
                break
            }
        }

        stats.endBasePos = sim.currentObservation().basePositionWorld
        stats.endBaseUprightZ = {
            let q = sim.currentObservation().baseOrientationWorldXYZW
            return 1 - 2 * (q.x * q.x + q.y * q.y)
        }()
        let survival = Float(physicsSteps) / 1000.0
        let uprightPenalty = max(0, 1 - stats.minBaseUprightZ) * 4
        let heightPenalty = max(0, 0.20 - stats.minBaseZ) * 8
        let velocityPenalty = min(stats.maxAbsJointVelocity / 120.0, 3.0)
        let deltaPenalty = min(stats.maxAbsJointDelta / 2.0, 2.0)
        let forwardReward = max(0, stats.endBasePos.x)
        let score = survival + forwardReward - uprightPenalty - heightPenalty - velocityPenalty - deltaPenalty
        return SpotCandidateResult(candidate: candidate,
                                   stats: stats,
                                   physicsSteps: physicsSteps,
                                   requiredPhysicsSteps: Int((seconds / sim.physicsTimeStep).rounded(.down)),
                                   score: score)
    }

    // MARK: - Static checks

    @Test func defaultJointPositionsAreTwelveAndStandingPose() {
        let sim = IsaacSwiftAnymalSimulator()
        #expect(sim.jointCount == 12)
        let defaults = sim.defaultJointPositions.map { $0.floatValue }
        #expect(defaults.count == 12)
        #expect(defaults[1] > 0)
        #expect(defaults[2] < 0)
        #expect(defaults[7] < 0)
        #expect(defaults[8] > 0)
    }

    @Test func freshObservationIsFinite() {
        let sim = IsaacSwiftAnymalSimulator()
        let obs = sim.currentObservation()
        #expect(obs.basePositionWorld.z.isFinite)
        #expect(obs.basePositionWorld.z > 0)
        for delta in obs.jointPositionDeltas {
            #expect(abs(delta.floatValue) <= 1e-3)
        }
    }

    @Test func h1FreshObservationHasNineteenFiniteJoints() {
        let config = IsaacPolicyRuntimeConfiguration.h1Flat
        let sim = IsaacSwiftAnymalSimulator(robotKind: config.robotKind,
                                            physicsTimeStep: config.physicsTimeStep)
        let obs = sim.currentObservation()
        #expect(sim.jointCount == 19)
        #expect(obs.jointPositions.count == 19)
        #expect(obs.jointPositionDeltas.count == 19)
        #expect(obs.jointVelocities.count == 19)
        #expect(obs.basePositionWorld.z.isFinite)
        #expect(abs(obs.basePositionWorld.z - sim.spawnPositionWorld.z) <= 0.002,
                "H1 base observation should report the USD pelvis origin, not COM")
        let defaults = sim.defaultJointPositions.map { $0.floatValue }
        for (index, position) in obs.jointPositions.enumerated() {
            #expect(abs(position.floatValue) <= 1e-3,
                    "H1 starts from the USD zero pose; joint \(index) was \(position)")
        }
        for (index, delta) in obs.jointPositionDeltas.enumerated() {
            #expect(abs(delta.floatValue + defaults[index]) <= 1e-3,
                    "H1 policy observation should report zero pose minus default; joint \(index) delta=\(delta)")
        }
    }

    // MARK: - Simulator convergence

    @Test func robotStandsStablyWithZeroAction() {
        let sim = IsaacSwiftAnymalSimulator()
        sim.reset()
        let stats = runSimulator(sim, seconds: 2.0) { _, _ in
            Array(repeating: Float(0), count: 12)
        }
        #expect(!stats.sawNaN)
        #expect(stats.minBaseZ > 0.15)
        #expect(stats.maxBaseZ < 0.9)
        #expect(stats.endBaseUprightZ > 0.6)
        #expect(stats.maxAbsJointDelta < 0.5)
    }

    @Test func spotDoesNotStaggerDuringFirstThousandPhysicsSteps() {
        let config = IsaacPolicyRuntimeConfiguration.spotFlat
        let sim = IsaacSwiftAnymalSimulator(robotKind: config.robotKind,
                                            physicsTimeStep: config.physicsTimeStep)
        sim.reset()
        let initialZ = sim.currentObservation().basePositionWorld.z
        let stats = runSimulator(sim, seconds: 5.0, controlInterval: config.physicsTimeStep) { _, _ in
            Array(repeating: Float(0), count: 12)
        }

        #expect(!stats.sawNaN)
        #expect(abs(initialZ - 0.500) <= 0.001)
        #expect(stats.maxBaseZ - stats.minBaseZ < 0.05,
                "base z range \(stats.minBaseZ)...\(stats.maxBaseZ) indicates a drop impact")
        #expect(stats.minBaseUprightZ > 0.98,
                "uprightZ dipped to \(stats.minBaseUprightZ)")
        #expect(stats.maxAbsJointDelta < 0.15)
        #expect(stats.maxAbsJointVelocity < 8.0)
    }

    @Test func anymalDoesNotStaggerDuringFirstThousandPhysicsSteps() {
        let sim = IsaacSwiftAnymalSimulator(robotKind: .anymalC, physicsTimeStep: 1.0 / 200.0)
        sim.reset()
        let initialZ = sim.currentObservation().basePositionWorld.z
        let stats = runSimulator(sim, seconds: 5.0, controlInterval: 1.0 / 200.0) { _, _ in
            Array(repeating: Float(0), count: 12)
        }

        #expect(!stats.sawNaN)
        #expect(abs(initialZ - 0.600) <= 0.001)
        #expect(stats.maxBaseZ - stats.minBaseZ < 0.05,
                "base z range \(stats.minBaseZ)...\(stats.maxBaseZ) indicates a drop impact")
        #expect(stats.minBaseUprightZ > 0.98,
                "uprightZ dipped to \(stats.minBaseUprightZ)")
        #expect(stats.maxAbsJointDelta < 0.15)
        #expect(stats.maxAbsJointVelocity < 8.0)
    }

    @Test func h1CoreMLPolicyStaysUprightWithZeroCommand() throws {
        let configuration = PolicyModelConfiguration.h1
        guard PolicyModelRunner.bundledModelURL(configuration: configuration) != nil ||
              PolicyModelRunner.repositoryModelURL(configuration: configuration).map({ FileManager.default.fileExists(atPath: $0.path) }) == true
        else { return }

        let runner = try PolicyModelRunner(configuration: configuration)
        let cfg = IsaacPolicyRuntimeConfiguration.h1Flat
        let provider = DemoPolicyActionProvider(runner: runner,
                                                configuration: cfg,
                                                command: SIMD3<Float>(0, 0, 0))
        let loop = PolicyPhysicsLoop(robotKind: cfg.robotKind,
                                     configuration: cfg,
                                     provider: provider)
        loop.reset()
        let stats = runLoop(loop, seconds: 3.0)

        #expect(!stats.sawNaN)
        #expect(stats.minBaseZ > 0.55,
                "H1 base dropped to z=\(stats.minBaseZ)")
        #expect(stats.minBaseUprightZ > 0.85,
                "H1 uprightZ dipped to \(stats.minBaseUprightZ)")
        #expect(stats.maxAbsJointVelocity < 80.0)
    }

    @Test func actuatorPositiveTorqueMatchesPositiveMotorTargetDirection() {
        // KFE's Jolt hinge angle convention is opposite to the ActuatorNet
        // effort convention; HAA/HFE must still match directly.
        for jointIndex in 0..<12 where jointIndex % 3 != 2 {
            let motorSim = IsaacSwiftAnymalSimulator(robotKind: .anymalC, physicsTimeStep: 1.0 / 200.0)
            motorSim.spawnPositionWorld = SIMD3<Float>(0, 0, 1.2)
            motorSim.reset()
            var motorActions = Array(repeating: Float(0), count: 12)
            motorActions[jointIndex] = 0.2
            _ = motorSim.step(withScaledActions: motorActions.map { NSNumber(value: $0) },
                              elapsedTime: 0.1)
            let motorDelta = motorSim.currentObservation().jointPositionDeltas[jointIndex].floatValue

            let torqueSim = IsaacSwiftAnymalSimulator(robotKind: .anymalC, physicsTimeStep: 1.0 / 200.0)
            torqueSim.spawnPositionWorld = SIMD3<Float>(0, 0, 1.2)
            var torques = Array(repeating: Float(0), count: 12)
            torques[jointIndex] = 20
            torqueSim.jointActuator = ConstantJointActuator(torques: torques)
            torqueSim.reset()
            _ = torqueSim.step(withScaledActions: Array(repeating: NSNumber(value: 0), count: 12),
                               elapsedTime: 0.1)
            let torqueDelta = torqueSim.currentObservation().jointPositionDeltas[jointIndex].floatValue
            #expect(motorDelta > 0, "joint \(jointIndex): positive motor target produced delta \(motorDelta)")
            #expect(torqueDelta > 0, "joint \(jointIndex): positive actuator torque produced delta \(torqueDelta), opposite to motor target direction")
        }
    }

    @Test func spotPolicyParameterSearchFindsNonRollingCandidate() throws {
        let env = ProcessInfo.processInfo.environment
        guard env["ISAACSWIFT_SPOT_PARAM_SEARCH"] == "1" ||
              env["ISAACSWIFT_SPOT_PARAM_SEARCH_FULL"] == "1"
        else { return }

        let configuration = PolicyModelConfiguration.spot
        guard PolicyModelRunner.bundledModelURL(configuration: configuration) != nil ||
              PolicyModelRunner.repositoryModelURL(configuration: configuration).map({ FileManager.default.fileExists(atPath: $0.path) }) == true
        else {
            #expect(Bool(false),
                    "Spot parameter search requires bundled or repository spot_policy.mlmodelc")
            return
        }

        let runner = try PolicyModelRunner(configuration: configuration)
        let fullSearch = env["ISAACSWIFT_SPOT_PARAM_SEARCH_FULL"] == "1"
        let seconds = Double(env["ISAACSWIFT_SPOT_PARAM_SEARCH_SECONDS"] ?? "") ?? (fullSearch ? 12.0 : 5.0)
        let candidates = spotSearchCandidates(full: fullSearch)
        var results: [SpotCandidateResult] = []
        results.reserveCapacity(candidates.count)

        for candidate in candidates {
            let result = evaluateSpotPolicyCandidate(candidate,
                                                     runner: runner,
                                                     seconds: seconds)
            results.append(result)
        }

        let ranked = results.sorted { lhs, rhs in
            if lhs.isForwardWalking != rhs.isForwardWalking {
                return lhs.isForwardWalking && !rhs.isForwardWalking
            }
            if lhs.isNonRolling != rhs.isNonRolling {
                return lhs.isNonRolling && !rhs.isNonRolling
            }
            let lhsMoves = lhs.candidate.actionScale > 0
            let rhsMoves = rhs.candidate.actionScale > 0
            if lhsMoves != rhsMoves {
                return lhsMoves && !rhsMoves
            }
            let lhsCommanded = lhs.candidate.commandX > 0
            let rhsCommanded = rhs.candidate.commandX > 0
            if lhsCommanded != rhsCommanded {
                return lhsCommanded && !rhsCommanded
            }
            return lhs.score > rhs.score
        }
        let top = ranked.prefix(10).map(\.description).joined(separator: "\n")
        print("Spot policy parameter search top candidates:\n\(top)")

        let bestMovingCandidate = ranked.first {
            $0.isForwardWalking && $0.candidate.actionScale > 0
        }
        #expect(bestMovingCandidate != nil,
                "No forward-walking Spot candidate with actionScale > 0 found. Run the broader search with ISAACSWIFT_SPOT_PARAM_SEARCH_FULL=1. Top candidates:\n\(top)")
    }

    @Test func robotStaysBoundedUnderSmallRandomActions() {
        let sim = IsaacSwiftAnymalSimulator()
        sim.reset()
        var rng = SystemRandomNumberGenerator()
        let stats = runSimulator(sim, seconds: 2.0) { _, _ in
            (0..<12).map { _ in Float.random(in: -0.15...0.15, using: &rng) }
        }
        #expect(!stats.sawNaN)
        #expect(stats.minBaseZ > 0.10)
        #expect(stats.maxBaseZ < 1.0)
        #expect(stats.maxAbsJointVelocity < 60.0)
    }

    // MARK: - PolicyPhysicsLoop

    @Test func policyPhysicsLoopWithoutProviderHoldsPose() {
        let loop = PolicyPhysicsLoop(provider: nil)
        loop.reset()
        let stats = runLoop(loop, seconds: 2.0)
        #expect(!stats.sawNaN)
        #expect(stats.minBaseZ > 0.15)
        #expect(stats.endBaseUprightZ > 0.6)
        #expect(stats.maxAbsJointDelta < 0.5)
    }

    @Test func policyPhysicsLoopWithCoreMLPolicyStaysBounded() throws {
        let configuration = PolicyModelConfiguration.spot
        guard PolicyModelRunner.bundledModelURL(configuration: configuration) != nil ||
              PolicyModelRunner.repositoryModelURL(configuration: configuration).map({ FileManager.default.fileExists(atPath: $0.path) }) == true
        else { return }

        let runner = try PolicyModelRunner(configuration: configuration)
        let config = IsaacPolicyRuntimeConfiguration.spotFlat
        let provider = DemoPolicyActionProvider(runner: runner,
                                                configuration: config,
                                                command: SIMD3<Float>(0.4, 0, 0))
        let loop = PolicyPhysicsLoop(robotKind: config.robotKind, provider: provider)
        loop.reset()

        let stats = runLoop(loop, seconds: 4.0)

        // The loop wires base lin/ang vel + projected gravity into the
        // policy, so the network now receives an Isaac-Lab-shaped 48-dim
        // input. With correct observations the robot should stay upright.
        #expect(!stats.sawNaN)
        #expect(stats.minBaseZ > 0.05)
        #expect(stats.maxBaseZ < 1.5)
        #expect(stats.endBaseUprightZ > 0.5,
                "uprightZ=\(stats.endBaseUprightZ) — robot fell over")
        #expect(stats.maxAbsJointDelta <= Float.pi + 0.1)
        #expect(stats.maxAbsJointVelocity < 1500.0)
    }

    @Test func spotCoreMLPolicyWalksForwardOnFlatGround() throws {
        let configuration = PolicyModelConfiguration.spot
        guard PolicyModelRunner.bundledModelURL(configuration: configuration) != nil ||
              PolicyModelRunner.repositoryModelURL(configuration: configuration).map({ FileManager.default.fileExists(atPath: $0.path) }) == true
        else { return }

        let runner = try PolicyModelRunner(configuration: configuration)
        let config = IsaacPolicyRuntimeConfiguration.spotFlat
        let provider = DemoPolicyActionProvider(runner: runner,
                                                configuration: config)
        let loop = PolicyPhysicsLoop(robotKind: config.robotKind, provider: provider)
        loop.reset()

        let stats = runLoop(loop, seconds: 4.0)

        // With the joint-permutation fix locked in by `IsaacPolicyRuntime
        // Configuration.spotFlat`, Spot must stay solidly upright and walk
        // close to the commanded forward velocity (0.8 m/s) over 4 s.
        #expect(!stats.sawNaN)
        #expect(stats.minBaseUprightZ > 0.95,
                "uprightZ dipped to \(stats.minBaseUprightZ) — Spot is tipping")
        #expect(stats.minBaseZ > 0.35,
                "base dropped to z=\(stats.minBaseZ) — Spot is collapsing")
        #expect(stats.endBasePos.x > 1.5,
                "x displacement was only \(stats.endBasePos.x)m for a 4 s rollout at 0.8 m/s")
    }

    @Test func go2FlatRuntimeMatchesIsaacLabActuatorSettings() {
        let sim = IsaacSwiftAnymalSimulator(robotKind: .go2, physicsTimeStep: 1.0 / 200.0)
        #expect(abs(sim.jointStiffness - 25.0) <= 0.001)
        #expect(abs(sim.jointDamping - 0.5) <= 0.001)
        #expect(abs(sim.maxJointTorque - 23.5) <= 0.001)
        #expect(abs(sim.recommendedActionScale - 0.25) <= 0.001)
        #expect(abs(sim.motorTargetSmoothingTau) <= 0.001)
        #expect(abs(IsaacPolicyRuntimeConfiguration.go2.defaultCommand.x - 0.8) <= 0.001)
    }

    @Test func go2BackflipRuntimeMatchesTunedDriveSettings() {
        let cfg = IsaacPolicyRuntimeConfiguration.go2Backflip
        let loop = PolicyPhysicsLoop(robotKind: cfg.robotKind,
                                     configuration: cfg,
                                     provider: nil)
        let defaults = loop.simulator.defaultJointPositions.map { $0.floatValue }
        #expect(!loop.hasJointActuator,
                "Go2 backflip currently uses the Jolt position motor approximation")
        #expect(cfg.observationLayout == .go2Backflip(maxEpisodeLength: 100))
        #expect(cfg.usesOneStepActionLatency)
        #expect(abs(loop.simulator.spawnPositionWorld.z - 0.34) <= 0.001)
        #expect(abs(loop.simulator.jointStiffness - 70.0) <= 0.001)
        #expect(abs(loop.simulator.jointDamping - 4.5) <= 0.001)
        #expect(abs(loop.simulator.maxJointTorque - 21.0) <= 0.001)
        #expect(abs(loop.actionScale - 0.5) <= 0.001)
        #expect(defaults == cfg.defaultJointPositions ?? [])
    }

    @Test func go2BackflipCoreMLPolicyCreatesLiftImpulse() throws {
        let configuration = PolicyModelConfiguration.go2Backflip
        guard PolicyModelRunner.bundledModelURL(configuration: configuration) != nil ||
              PolicyModelRunner.repositoryModelURL(configuration: configuration).map({ FileManager.default.fileExists(atPath: $0.path) }) == true
        else { return }

        let runner = try PolicyModelRunner(configuration: configuration)
        let cfg = IsaacPolicyRuntimeConfiguration.go2Backflip
        let provider = DemoPolicyActionProvider(runner: runner,
                                                configuration: cfg)
        let loop = PolicyPhysicsLoop(robotKind: cfg.robotKind,
                                     configuration: cfg,
                                     provider: provider)
        loop.reset()
        let initialZ = loop.simulator.currentObservation().basePositionWorld.z

        let stats = runLoop(loop, seconds: 2.0)

        #expect(!stats.sawNaN)
        #expect(stats.maxBaseZ > initialZ + 0.04,
                "max base z was \(stats.maxBaseZ), initial z was \(initialZ); backflip policy did not create lift")
        #expect(stats.maxAbsJointDelta > 0.10,
                "joint deltas barely moved during backflip rollout")
    }

    @Test func go2BackflipTraceFitDiagnostic() throws {
        guard shouldRunGo2BackflipTraceFit() else {
            return
        }
        guard let reference = loadGo2BackflipPlayTrace() else {
            Issue.record("tmp/play_trace/play_trace.csv is required for the Go2 backflip trace-fit diagnostic.")
            return
        }
        guard PolicyModelRunner.bundledModelURL(configuration: .go2Backflip) != nil ||
              PolicyModelRunner.repositoryModelURL(configuration: .go2Backflip).map({ FileManager.default.fileExists(atPath: $0.path) }) == true else {
            Issue.record("go2_backflip_policy.mlmodelc is required for trace-fit diagnostic.")
            return
        }

        var candidates: [Go2BackflipTuningCandidate] = [
            Go2BackflipTuningCandidate(spawnZ: 0.35, kp: 70, kd: 3.0, maxTorque: 35.55, actionScale: 0.5),
        ]
        for spawnZ in [Float(0.32), 0.34, 0.35] {
            for kp in [Float(45), 55, 65, 70] {
                for kd in [Float(2.0), 3.0, 4.5, 6.0] {
                    for maxTorque in [Float(21.0), 23.7, 28.0, 32.0, 35.55] {
                        candidates.append(Go2BackflipTuningCandidate(spawnZ: spawnZ,
                                                                      kp: kp,
                                                                      kd: kd,
                                                                      maxTorque: maxTorque,
                                                                      actionScale: 0.5))
                    }
                }
            }
        }

        let fits = try candidates.map { candidate in
            let actual = try runGo2BackflipTrace(candidate: candidate)
            return scoreGo2BackflipTrace(candidate: candidate,
                                         actual: actual,
                                         reference: reference)
        }.sorted { $0.score < $1.score }

        var lines: [String] = []
        lines.append("=== Go2 backflip trace-fit candidates (top 20 of \(fits.count)) ===")
        for fit in fits.prefix(20) {
            lines.append("  " + fit.description)
        }
        let outputPath = ProcessInfo.processInfo.environment["ISAACSWIFT_GO2_BACKFLIP_TRACE_FIT_OUTPUT"]
            ?? repositoryRootURL().appendingPathComponent("tmp/go2_backflip_trace_fit.txt").path
        let outputURL = URL(fileURLWithPath: outputPath)
        let output = lines.joined(separator: "\n")
        do {
            try output.write(to: outputURL, atomically: true, encoding: .utf8)
        } catch {
            let tempURL = URL(fileURLWithPath: NSTemporaryDirectory())
                .appendingPathComponent("go2_backflip_trace_fit.txt")
            try? output.write(to: tempURL, atomically: true, encoding: .utf8)
            print("Go2 backflip trace-fit output: \(tempURL.path)")
        }
        print(output)
    }

    @Test func go2FlatCoreMLPolicyWalksForwardOnFlatGround() throws {
        let configuration = PolicyModelConfiguration.go2Flat
        guard PolicyModelRunner.bundledModelURL(configuration: configuration) != nil ||
              PolicyModelRunner.repositoryModelURL(configuration: configuration).map({ FileManager.default.fileExists(atPath: $0.path) }) == true
        else { return }

        let runner = try PolicyModelRunner(configuration: configuration)
        let config = IsaacPolicyRuntimeConfiguration.go2
        let provider = DemoPolicyActionProvider(runner: runner,
                                                configuration: config,
                                                command: SIMD3<Float>(0.8, 0, 0))
        let loop = PolicyPhysicsLoop(robotKind: config.robotKind, provider: provider)
        loop.reset()

        let stats = runLoop(loop, seconds: 4.0)

        #expect(!stats.sawNaN)
        #expect(stats.minBaseUprightZ > 0.90,
                "uprightZ dipped to \(stats.minBaseUprightZ) — Go2 is tipping")
        #expect(stats.minBaseZ > 0.05,
                "base dropped to z=\(stats.minBaseZ) — Go2 is collapsing")
        #expect(stats.endBasePos.x > 1.5,
                "x displacement was only \(stats.endBasePos.x)m for a 4 s rollout at 0.8 m/s")
        #expect(stats.maxAbsJointVelocity < 120.0)
    }

    // MARK: - Spot joint-permutation + gain search
    //
    // The bundled CoreML policy was trained inside Isaac Sim, where the
    // articulation's `dof_names` are returned in PhysX traversal order.
    // For Spot that is normally grouped by joint *type*
    // (`[fl_hx, fr_hx, hl_hx, hr_hx, fl_hy, fr_hy, hl_hy, hr_hy, fl_kn, …]`),
    // not leg-major. The Jolt simulator here is built leg-major, so feeding
    // the policy with the wrong permutation will swap joints between legs and
    // make Spot fall over instantly. This sweep tries a few plausible
    // orderings × actuator gains × action_scale and prints the top results.

    private struct JointPermutationCandidate {
        let name: String
        /// `simToPolicy[sim_idx]` → policy obs index. Length 12.
        let simToPolicy: [Int]
    }

    private func spotPermutationCandidates() -> [JointPermutationCandidate] {
        // Sim joint index → joint name (leg-major):
        //   0 fl_hx 1 fl_hy 2 fl_kn  3 fr_hx 4 fr_hy 5 fr_kn
        //   6 hl_hx 7 hl_hy 8 hl_kn  9 hr_hx 10 hr_hy 11 hr_kn
        // Below we list policy-index → joint name; the simToPolicy mapping
        // is the inverse.
        func mapping(policyOrder: [String]) -> [Int] {
            let simNames = [
                "fl_hx","fl_hy","fl_kn","fr_hx","fr_hy","fr_kn",
                "hl_hx","hl_hy","hl_kn","hr_hx","hr_hy","hr_kn",
            ]
            return simNames.map { name in policyOrder.firstIndex(of: name)! }
        }
        return [
            JointPermutationCandidate(
                name: "identity-leg-major",
                simToPolicy: mapping(policyOrder: [
                    "fl_hx","fl_hy","fl_kn","fr_hx","fr_hy","fr_kn",
                    "hl_hx","hl_hy","hl_kn","hr_hx","hr_hy","hr_kn",
                ])),
            JointPermutationCandidate(
                name: "type-grouped FL-FR-HL-HR",
                simToPolicy: mapping(policyOrder: [
                    "fl_hx","fr_hx","hl_hx","hr_hx",
                    "fl_hy","fr_hy","hl_hy","hr_hy",
                    "fl_kn","fr_kn","hl_kn","hr_kn",
                ])),
            JointPermutationCandidate(
                name: "type-grouped FL-HL-FR-HR",
                simToPolicy: mapping(policyOrder: [
                    "fl_hx","hl_hx","fr_hx","hr_hx",
                    "fl_hy","hl_hy","fr_hy","hr_hy",
                    "fl_kn","hl_kn","fr_kn","hr_kn",
                ])),
            JointPermutationCandidate(
                name: "type-grouped FR-FL-HR-HL",
                simToPolicy: mapping(policyOrder: [
                    "fr_hx","fl_hx","hr_hx","hl_hx",
                    "fr_hy","fl_hy","hr_hy","hl_hy",
                    "fr_kn","fl_kn","hr_kn","hl_kn",
                ])),
            JointPermutationCandidate(
                name: "leg-major FR-FL-HR-HL",
                simToPolicy: mapping(policyOrder: [
                    "fr_hx","fr_hy","fr_kn","fl_hx","fl_hy","fl_kn",
                    "hr_hx","hr_hy","hr_kn","hl_hx","hl_hy","hl_kn",
                ])),
        ]
    }

    private struct SpotPermResult {
        var permName: String
        var kp: Float
        var kd: Float
        var maxTorque: Float
        var actionScale: Float
        var commandX: Float
        var stats: RolloutStats
        var fellOverAt: Double?

        var description: String {
            let fall = fellOverAt.map { String(format: "%.2fs", $0) } ?? "-"
            return String(format:
                "perm=%@ kp=%.0f kd=%.1f T=%.0f scale=%.3f cmdX=%.1f x=%+.3fm uprightMin=%.3f zMin=%.3f maxVel=%.1f fell=%@",
                permName, Double(kp), Double(kd), Double(maxTorque), Double(actionScale),
                Double(commandX),
                Double(stats.endBasePos.x), Double(stats.minBaseUprightZ),
                Double(stats.minBaseZ), Double(stats.maxAbsJointVelocity), fall)
        }

        var isWalking: Bool {
            stats.minBaseUprightZ > 0.90 &&
            stats.minBaseZ > 0.30 &&
            !stats.sawNaN &&
            fellOverAt == nil &&
            stats.endBasePos.x > 0.5
        }

        /// Higher = better. Reward forward x, penalize tilt and falls.
        var score: Float {
            let upright = stats.minBaseUprightZ
            if stats.sawNaN || fellOverAt != nil { return -10 }
            let forward = stats.endBasePos.x
            let tiltPenalty = max(0, 1 - upright) * 4
            let dropPenalty = max(0, 0.30 - stats.minBaseZ) * 6
            let chatterPenalty = min(stats.maxAbsJointVelocity / 100, 2.0)
            return forward - tiltPenalty - dropPenalty - chatterPenalty
        }
    }

    private func evaluateSpotPolicy(perm: JointPermutationCandidate,
                                    runner: PolicyModelRunner,
                                    kp: Float, kd: Float, maxTorque: Float,
                                    actionScale: Float, commandX: Float,
                                    seconds: Double) -> SpotPermResult {
        let config = IsaacPolicyRuntimeConfiguration.spotFlat
        let sim = IsaacSwiftAnymalSimulator(robotKind: config.robotKind,
                                            physicsTimeStep: config.physicsTimeStep)
        sim.jointStiffness = kp
        sim.jointDamping = kd
        sim.maxJointTorque = maxTorque
        sim.reset()

        let provider = DemoPolicyActionProvider(runner: runner,
                                                configuration: config,
                                                command: SIMD3<Float>(commandX, 0, 0))
        provider.simToPolicyJointPermutation = perm.simToPolicy

        let policyDt = config.policyUpdateInterval
        var stats = RolloutStats()
        var t: Double = 0
        var fellOverAt: Double? = nil

        while t < seconds {
            let obs = sim.currentObservation()
            provider.updateJointState(positionDeltas: obs.jointPositionDeltas.map { $0.floatValue },
                                      velocities: obs.jointVelocities.map { $0.floatValue })
            provider.updateBaseFeedback(linearVelocityBody: obs.baseLinearVelocityBody,
                                        angularVelocityBody: obs.baseAngularVelocityBody,
                                        projectedGravityBody: obs.gravityDirectionBody)
            // `currentActions` already returns sim-order (raw, unscaled).
            let raw = provider.currentActions(at: t)
            let scaled = (0..<Int(sim.jointCount)).map { i -> Float in
                i < raw.count ? raw[i] * actionScale : 0
            }
            _ = sim.step(withScaledActions: scaled.map { NSNumber(value: $0) },
                         elapsedTime: policyDt)
            let obs2 = sim.currentObservation()
            observe(obs2, into: &stats)
            t += policyDt

            if stats.sawNaN { fellOverAt = t; break }
            // Treat as fallen if the body drops below half spawn height OR
            // tilts past 60° from upright.
            let q = obs2.baseOrientationWorldXYZW
            let up = 1 - 2 * (q.x * q.x + q.y * q.y)
            if obs2.basePositionWorld.z < 0.20 || up < 0.50 {
                fellOverAt = t
                break
            }
        }

        let final = sim.currentObservation()
        stats.endBasePos = final.basePositionWorld
        let q = final.baseOrientationWorldXYZW
        stats.endBaseUprightZ = 1 - 2 * (q.x * q.x + q.y * q.y)

        return SpotPermResult(permName: perm.name, kp: kp, kd: kd,
                              maxTorque: maxTorque, actionScale: actionScale,
                              commandX: commandX, stats: stats,
                              fellOverAt: fellOverAt)
    }

    /// Sweeps joint-order permutations × gains × action_scale to identify the
    /// configuration that lets the bundled Spot CoreML policy actually walk
    /// forward. Always runs (not env-gated) so results show up in CI logs.
    @Test func spotPolicyJointPermutationAndGainSweep() throws {
        let configuration = PolicyModelConfiguration.spot
        guard PolicyModelRunner.bundledModelURL(configuration: configuration) != nil ||
              PolicyModelRunner.repositoryModelURL(configuration: configuration).map({ FileManager.default.fileExists(atPath: $0.path) }) == true
        else { return }

        let runner = try PolicyModelRunner(configuration: configuration)
        let env = ProcessInfo.processInfo.environment
        let full = env["ISAACSWIFT_SPOT_PERM_SWEEP_FULL"] == "1"
        let seconds = Double(env["ISAACSWIFT_SPOT_PERM_SWEEP_SECONDS"] ?? "") ?? 4.0

        let perms = spotPermutationCandidates()
        let gainGrid: [(Float, Float, Float)] = full
            ? [(60, 4, 60), (120, 6, 120), (180, 10, 180), (240, 12, 240), (320, 18, 320)]
            : [(120, 6, 120), (240, 12, 240)]
        let scales: [Float] = full ? [0.05, 0.1, 0.15, 0.2, 0.3] : [0.1, 0.2]
        let commands: [Float] = full ? [0.4, 0.8, 1.0] : [0.8]

        var results: [SpotPermResult] = []
        for perm in perms {
            for (kp, kd, t) in gainGrid {
                for scale in scales {
                    for cmd in commands {
                        let r = evaluateSpotPolicy(perm: perm, runner: runner,
                                                   kp: kp, kd: kd, maxTorque: t,
                                                   actionScale: scale, commandX: cmd,
                                                   seconds: seconds)
                        results.append(r)
                    }
                }
            }
        }

        let ranked = results.sorted { $0.score > $1.score }
        let topN = min(15, ranked.count)
        var lines: [String] = []
        lines.append("=== Spot policy joint-perm × gain sweep (top \(topN) of \(ranked.count)) ===")
        for r in ranked.prefix(topN) {
            lines.append("  " + r.description)
        }
        // Also print best per permutation so we can see ordering effects.
        lines.append("--- best per permutation ---")
        for perm in perms {
            if let best = ranked.first(where: { $0.permName == perm.name }) {
                lines.append("  " + best.description)
            }
        }
        print(lines.joined(separator: "\n"))

        let walking = ranked.first { $0.isWalking }
        let summary = lines.joined(separator: "\n")
        #expect(walking != nil,
                Comment(rawValue: "No (perm, kp, kd, scale, cmd) combination produced a walking Spot.\n\(summary)"))
    }

    // MARK: - ANYmal joint-permutation + gain search
    //
    // Same shape as the Spot sweep. Isaac Lab's anymal_env.yaml (bundled in
    // `isaac_policy_sources/Anymal_Policies/anymal_env.yaml`) ships an
    // ActuatorNetLSTM, not a raw PD drive — so we approximate the network's
    // effective stiffness/damping with a torque-limited PD and search for
    // the gains that let the bundled `anymal_policy.mlmodelc` walk forward
    // for ≥10 s without falling.

    private struct AnymalCandidate {
        let permName: String
        let simToPolicy: [Int]
        let kp: Float
        let kd: Float
        let maxTorque: Float
        let actionScale: Float
        let commandX: Float
        let spawnZ: Float
        let smoothingTau: Float
    }

    private struct AnymalResult {
        var candidate: AnymalCandidate
        var stats: RolloutStats
        var fellOverAt: Double?
        var survivedSeconds: Double

        var description: String {
            let fall = fellOverAt.map { String(format: "%.2fs", $0) } ?? "-"
            return String(format:
                "perm=%@ kp=%.0f kd=%.1f T=%.0f scale=%.3f cmdX=%.1f z0=%.2f tau=%.0fms survived=%.2fs x=%+.3fm uprightMin=%.3f zMin=%.3f maxVel=%.1f fell=%@",
                candidate.permName, Double(candidate.kp), Double(candidate.kd),
                Double(candidate.maxTorque), Double(candidate.actionScale),
                Double(candidate.commandX), Double(candidate.spawnZ),
                Double(candidate.smoothingTau * 1000),
                survivedSeconds,
                Double(stats.endBasePos.x), Double(stats.minBaseUprightZ),
                Double(stats.minBaseZ), Double(stats.maxAbsJointVelocity), fall)
        }

        /// 10 s upright + clearly visible forward progress. A lower threshold
        /// allowed a nearly-static shuffle to pass while the renderer still
        /// looked stationary, so the bar stays high enough to catch that drift.
        var isWalking: Bool {
            survivedSeconds >= 10.0 &&
            stats.minBaseUprightZ > 0.85 &&
            stats.minBaseZ > 0.30 &&
            !stats.sawNaN &&
            stats.endBasePos.x > 1.0
        }

        var score: Float {
            if stats.sawNaN { return -100 }
            // Survival is the dominant term — reaching 10 s is the requirement.
            let survival = Float(survivedSeconds)
            let forward = max(0, stats.endBasePos.x)
            let tiltPenalty = max(0, 1 - stats.minBaseUprightZ) * 5
            let dropPenalty = max(0, 0.30 - stats.minBaseZ) * 10
            let chatterPenalty = min(stats.maxAbsJointVelocity / 80, 3.0)
            return survival + forward - tiltPenalty - dropPenalty - chatterPenalty
        }
    }

    private func anymalPermutationCandidates() -> [(name: String, perm: [Int])] {
        // sim order (leg-major):
        //   0 LF_HAA 1 LF_HFE 2 LF_KFE  3 RF_HAA 4 RF_HFE 5 RF_KFE
        //   6 LH_HAA 7 LH_HFE 8 LH_KFE  9 RH_HAA 10 RH_HFE 11 RH_KFE
        func mapping(policyOrder: [String]) -> [Int] {
            let simNames = [
                "LF_HAA","LF_HFE","LF_KFE","RF_HAA","RF_HFE","RF_KFE",
                "LH_HAA","LH_HFE","LH_KFE","RH_HAA","RH_HFE","RH_KFE",
            ]
            return simNames.map { name in policyOrder.firstIndex(of: name)! }
        }
        return [
            ("identity-leg-major",
             mapping(policyOrder: [
                "LF_HAA","LF_HFE","LF_KFE","RF_HAA","RF_HFE","RF_KFE",
                "LH_HAA","LH_HFE","LH_KFE","RH_HAA","RH_HFE","RH_KFE",
             ])),
            // PhysX BFS order traditionally seen on ANYmal-C:
            // four HAAs first, then four HFEs, then four KFEs. Leg order
            // at each depth varies between ANYmal-B/C/D USDs, so try all.
            ("type-grouped LF-LH-RF-RH",
             mapping(policyOrder: [
                "LF_HAA","LH_HAA","RF_HAA","RH_HAA",
                "LF_HFE","LH_HFE","RF_HFE","RH_HFE",
                "LF_KFE","LH_KFE","RF_KFE","RH_KFE",
             ])),
            ("type-grouped LF-RF-LH-RH",
             mapping(policyOrder: [
                "LF_HAA","RF_HAA","LH_HAA","RH_HAA",
                "LF_HFE","RF_HFE","LH_HFE","RH_HFE",
                "LF_KFE","RF_KFE","LH_KFE","RH_KFE",
             ])),
            ("type-grouped RF-LF-RH-LH",
             mapping(policyOrder: [
                "RF_HAA","LF_HAA","RH_HAA","LH_HAA",
                "RF_HFE","LF_HFE","RH_HFE","LH_HFE",
                "RF_KFE","LF_KFE","RH_KFE","LH_KFE",
             ])),
        ]
    }

    private func evaluateAnymalCandidate(_ c: AnymalCandidate,
                                         runner: PolicyModelRunner,
                                         seconds: Double) -> AnymalResult {
        let config = IsaacPolicyRuntimeConfiguration.anymalC
        let sim = IsaacSwiftAnymalSimulator(robotKind: config.robotKind,
                                            physicsTimeStep: config.physicsTimeStep)
        sim.jointStiffness = c.kp
        sim.jointDamping = c.kd
        sim.maxJointTorque = c.maxTorque
        sim.motorTargetSmoothingTau = c.smoothingTau
        sim.spawnPositionWorld = SIMD3<Float>(0, 0, c.spawnZ)
        // Wire the ANYdrive actuator network into the simulator. With the
        // actuator installed the Jolt hinge motor is disabled and the LSTM
        // effort is applied directly, matching Isaac Lab's actuator path.
        // The runner falls back to the repository's
        // `PolicyModels/anymal_actuator.mlmodelc` when the test bundle does
        // not embed the resource.
        if let actuator = try? AnymalActuatorRunner(bundle: .main) {
            sim.jointActuator = actuator
        }
        sim.reset()

        let provider = DemoPolicyActionProvider(runner: runner,
                                                configuration: config,
                                                command: SIMD3<Float>(c.commandX, 0, 0))
        provider.simToPolicyJointPermutation = c.simToPolicy

        let policyDt = config.policyUpdateInterval
        var stats = RolloutStats()
        var t: Double = 0
        var fellOverAt: Double? = nil
        var survived: Double = 0

        while t < seconds {
            let obs = sim.currentObservation()
            provider.updateJointState(positionDeltas: obs.jointPositionDeltas.map { $0.floatValue },
                                      velocities: obs.jointVelocities.map { $0.floatValue })
            provider.updateBaseFeedback(linearVelocityBody: obs.baseLinearVelocityBody,
                                        angularVelocityBody: obs.baseAngularVelocityBody,
                                        projectedGravityBody: obs.gravityDirectionBody)
            let raw = provider.currentActions(at: t)
            let scaled = (0..<Int(sim.jointCount)).map { i -> Float in
                i < raw.count ? raw[i] * c.actionScale : 0
            }
            _ = sim.step(withScaledActions: scaled.map { NSNumber(value: $0) },
                         elapsedTime: policyDt)
            let obs2 = sim.currentObservation()
            observe(obs2, into: &stats)
            t += policyDt

            if stats.sawNaN { fellOverAt = t; break }
            let q = obs2.baseOrientationWorldXYZW
            let up = 1 - 2 * (q.x * q.x + q.y * q.y)
            // Only count "still standing" up to the moment of failure.
            if obs2.basePositionWorld.z < 0.20 || up < 0.50 {
                fellOverAt = t
                break
            }
            survived = t
        }

        let final = sim.currentObservation()
        stats.endBasePos = final.basePositionWorld
        let q = final.baseOrientationWorldXYZW
        stats.endBaseUprightZ = 1 - 2 * (q.x * q.x + q.y * q.y)
        return AnymalResult(candidate: c, stats: stats,
                            fellOverAt: fellOverAt, survivedSeconds: survived)
    }

    /// Verify the runtime ANYmal permutation/tuning stays upright and walks
    /// forward under the same policy cadence as IsaacSim.
    @Test func anymalPolicyJointPermutationAndGainSweep() throws {
        let configuration = PolicyModelConfiguration.anymal
        guard PolicyModelRunner.bundledModelURL(configuration: configuration) != nil ||
              PolicyModelRunner.repositoryModelURL(configuration: configuration).map({ FileManager.default.fileExists(atPath: $0.path) }) == true
        else { return }

        let runner = try PolicyModelRunner(configuration: configuration)
        let env = ProcessInfo.processInfo.environment
        let full = env["ISAACSWIFT_ANYMAL_PERM_SWEEP_FULL"] == "1"
        let seconds = Double(env["ISAACSWIFT_ANYMAL_PERM_SWEEP_SECONDS"] ?? "") ?? 10.0

        if !full {
            let provider = DemoPolicyActionProvider(runner: runner,
                                                    configuration: IsaacPolicyRuntimeConfiguration.anymalC)
            let loop = PolicyPhysicsLoop(robotKind: IsaacPolicyRuntimeConfiguration.anymalC.robotKind,
                                         provider: provider)
            loop.reset()
            #expect(loop.simulator.jointActuator != nil,
                    "ANYmal runtime loop did not install AnymalActuatorRunner")
            let stats = runLoop(loop, seconds: seconds)
            #expect(!stats.sawNaN)
            #expect(stats.minBaseUprightZ > 0.85,
                    "runtime uprightZ dipped to \(stats.minBaseUprightZ)")
            #expect(stats.minBaseZ > 0.30,
                    "runtime base dropped to z=\(stats.minBaseZ)")
            #expect(stats.endBasePos.x > 1.0,
                    "runtime x displacement was only \(stats.endBasePos.x)m over \(seconds) s")
            return
        }

        // Full mode keeps the old exploratory grid for diagnosing direct-PD
        // fallback regressions. The normal ActuatorNet path is checked above
        // through the actual runtime loop.
        let gainGrid: [(Float, Float, Float)] = [
            (80, 2, 80), (120, 6, 120), (160, 8, 120),
            (200, 10, 120), (240, 12, 120), (300, 15, 120),
        ]
        let scales: [Float] = [0.1, 0.15, 0.2, 0.3, 0.5, 0.7]
        let commands: [Float] = [0.0, 0.4, 0.8]
        let spawnZs: [Float] = [0.58, 0.60, 0.62]
        let taus: [Float] = [0.008, 0.025, 0.050, 0.080]
        let perms = [("runtime",
                      IsaacPolicyRuntimeConfiguration.anymalC.simToPolicyJointPermutation)]

        var results: [AnymalResult] = []
        for (pName, perm) in perms {
            for (kp, kd, t) in gainGrid {
                for scale in scales {
                    for cmd in commands {
                        for z in spawnZs {
                            for tau in taus {
                                let c = AnymalCandidate(permName: pName,
                                                        simToPolicy: perm,
                                                        kp: kp, kd: kd, maxTorque: t,
                                                        actionScale: scale,
                                                        commandX: cmd, spawnZ: z,
                                                        smoothingTau: tau)
                                results.append(evaluateAnymalCandidate(c,
                                                                       runner: runner,
                                                                       seconds: seconds))
                            }
                        }
                    }
                }
            }
        }

        let ranked = results.sorted { $0.score > $1.score }
        var lines: [String] = []
        lines.append("=== ANYmal policy joint-perm × gain sweep (top 15 of \(ranked.count)) ===")
        for r in ranked.prefix(15) { lines.append("  " + r.description) }
        lines.append("--- best per permutation ---")
        for (pName, _) in perms {
            if let best = ranked.first(where: { $0.candidate.permName == pName }) {
                lines.append("  " + best.description)
            }
        }
        print(lines.joined(separator: "\n"))

        let summary = lines.joined(separator: "\n")
        let walking = ranked.first { $0.isWalking }
        #expect(walking != nil,
                Comment(rawValue: "No (perm, kp, kd, scale, cmd, z) combination produced a 10s / 1m+ walking ANYmal.\n\(summary)"))
    }

    @Test func anymalPolicyOutputDiagnostic() throws {
        let configuration = PolicyModelConfiguration.anymal
        guard PolicyModelRunner.bundledModelURL(configuration: configuration) != nil ||
              PolicyModelRunner.repositoryModelURL(configuration: configuration).map({ FileManager.default.fileExists(atPath: $0.path) }) == true
        else { return }

        let runner = try PolicyModelRunner(configuration: configuration)
        let config = IsaacPolicyRuntimeConfiguration.anymalC
        let sim = IsaacSwiftAnymalSimulator(robotKind: config.robotKind,
                                            physicsTimeStep: config.physicsTimeStep)
        sim.jointStiffness = 120
        sim.jointDamping = 6
        sim.maxJointTorque = 120
        sim.spawnPositionWorld = SIMD3<Float>(0, 0, 0.60)
        sim.reset()

        let provider = DemoPolicyActionProvider(runner: runner,
                                                configuration: config,
                                                command: SIMD3<Float>(0.4, 0, 0))
        let policyDt = config.policyUpdateInterval
        var t: Double = 0
        var lines: [String] = ["=== ANYmal raw policy output diagnostic ==="]
        for step in 0..<25 {
            let obs = sim.currentObservation()
            provider.updateJointState(positionDeltas: obs.jointPositionDeltas.map { $0.floatValue },
                                      velocities: obs.jointVelocities.map { $0.floatValue })
            provider.updateBaseFeedback(linearVelocityBody: obs.baseLinearVelocityBody,
                                        angularVelocityBody: obs.baseAngularVelocityBody,
                                        projectedGravityBody: obs.gravityDirectionBody)
            let raw = provider.currentActions(at: t)
            let absMax = raw.map { abs($0) }.max() ?? 0
            let mean = raw.reduce(0, +) / Float(raw.count)
            if step < 5 || step % 5 == 0 {
                lines.append(String(format: "step=%2d t=%.3fs base_z=%.3f |max|=%.3f mean=%+.3f raw=%@",
                                   step, t, obs.basePositionWorld.z, absMax, mean,
                                   raw.map { String(format: "%+.2f", $0) }.joined(separator: ",")))
            }
            let scaled = raw.map { $0 * 0.3 }
            _ = sim.step(withScaledActions: scaled.map { NSNumber(value: $0) },
                         elapsedTime: policyDt)
            t += policyDt
        }
        print(lines.joined(separator: "\n"))
    }

    @Test func anymalCoreMLPolicyWalksForwardOnFlatGround() throws {
        let configuration = PolicyModelConfiguration.anymal
        guard PolicyModelRunner.bundledModelURL(configuration: configuration) != nil ||
              PolicyModelRunner.repositoryModelURL(configuration: configuration).map({ FileManager.default.fileExists(atPath: $0.path) }) == true
        else { return }

        let runner = try PolicyModelRunner(configuration: configuration)
        let cfg = IsaacPolicyRuntimeConfiguration.anymalC
        let provider = DemoPolicyActionProvider(runner: runner,
                                                configuration: cfg)
        let loop = PolicyPhysicsLoop(robotKind: cfg.robotKind, provider: provider)
        loop.reset()
        #expect(loop.simulator.jointActuator != nil,
                "ANYmal runtime loop did not install AnymalActuatorRunner")
        let result = runLoop(loop, seconds: 10.0)

        // Requirement: stay upright (`uprightZ > 0.85`) and walk forward
        // at least 1 m over 10 s (no fall before the end).
        #expect(!result.sawNaN)
        #expect(result.minBaseUprightZ > 0.85,
                "uprightZ dipped to \(result.minBaseUprightZ) — ANYmal is tipping")
        #expect(result.minBaseZ > 0.30,
                "base dropped to z=\(result.minBaseZ) — ANYmal is collapsing")
        #expect(result.endBasePos.x > 1.0,
                "x displacement was only \(result.endBasePos.x)m over 10 s — ANYmal is not advancing enough")

        let frontSwingIndices = [1, 2, 4, 5] // LF/RF HFE + KFE in sim/render order.
        for index in frontSwingIndices {
            let range = result.maxJointDeltas[index] - result.minJointDeltas[index]
            #expect(range > 0.04,
                    "front joint \(index) barely moved: range=\(range)")
        }
        #expect(min(result.minJointDeltas[2], result.minJointDeltas[5]) < -0.02,
                "front KFE joints never reached forward-swing deltas")
    }

    @Test func anymalRoughCoreMLPolicyWalksForwardOnFlatGround() throws {
        let configuration = PolicyModelConfiguration.anymalRough
        guard PolicyModelRunner.bundledModelURL(configuration: configuration) != nil ||
              PolicyModelRunner.repositoryModelURL(configuration: configuration).map({ FileManager.default.fileExists(atPath: $0.path) }) == true
        else { return }

        let runner = try PolicyModelRunner(configuration: configuration)
        let cfg = IsaacPolicyRuntimeConfiguration.anymalRough
        let provider = DemoPolicyActionProvider(runner: runner,
                                                configuration: cfg)
        let loop = PolicyPhysicsLoop(robotKind: cfg.robotKind,
                                     configuration: cfg,
                                     provider: provider)
        #expect(abs(loop.simulator.motorTargetSmoothingTau) <= 0.001)
        #expect(abs(loop.simulator.spawnPositionWorld.z - 0.70) <= 0.001)
        loop.reset()
        #expect(loop.simulator.jointActuator != nil,
                "ANYmal rough runtime loop did not install AnymalActuatorRunner")

        let result = runLoop(loop, seconds: 10.0)

        #expect(!result.sawNaN)
        #expect(result.minBaseUprightZ > 0.85,
                "uprightZ dipped to \(result.minBaseUprightZ) — ANYmal rough is tipping")
        #expect(result.minBaseZ > 0.30,
                "base dropped to z=\(result.minBaseZ) — ANYmal rough is collapsing")
        #expect(result.endBasePos.x > 1.0,
                "x displacement was only \(result.endBasePos.x)m over 10 s — ANYmal rough is not advancing enough")
    }

    @Test func h1CoreMLPolicyStaysUprightOnFlatGround() throws {
        let configuration = PolicyModelConfiguration.h1
        guard PolicyModelRunner.bundledModelURL(configuration: configuration) != nil ||
              PolicyModelRunner.repositoryModelURL(configuration: configuration).map({ FileManager.default.fileExists(atPath: $0.path) }) == true
        else { return }

        let runner = try PolicyModelRunner(configuration: configuration)
        let cfg = IsaacPolicyRuntimeConfiguration.h1Flat
        let provider = DemoPolicyActionProvider(runner: runner,
                                                configuration: cfg)
        let loop = PolicyPhysicsLoop(robotKind: cfg.robotKind, provider: provider)
        loop.reset()
        let result = runLoop(loop, seconds: 10.0)

        #expect(!result.sawNaN)
        #expect(result.minBaseUprightZ > 0.95,
                "uprightZ dipped to \(result.minBaseUprightZ) — H1 tilted")
        #expect(result.minBaseZ > 0.55,
                "base dropped to z=\(result.minBaseZ) — H1 collapsed")
        #expect(result.maxAbsJointDelta <= Float.pi + 0.1)
        #expect(result.endBasePos.x > 1.0,
                "x displacement was only \(result.endBasePos.x)m over 10 s")
        #expect(result.maxJointDeltas[6] - result.minJointDeltas[6] > 0.15,
                "left ankle barely moved during H1 policy rollout")
        #expect(result.maxJointDeltas[10] - result.minJointDeltas[10] > 0.15,
                "right ankle barely moved during H1 policy rollout")
    }

    @Test func h1RendererCadencePolicyLoopWalksForward() throws {
        let configuration = PolicyModelConfiguration.h1
        guard PolicyModelRunner.bundledModelURL(configuration: configuration) != nil ||
              PolicyModelRunner.repositoryModelURL(configuration: configuration).map({ FileManager.default.fileExists(atPath: $0.path) }) == true
        else { return }

        let runner = try PolicyModelRunner(configuration: configuration)
        let cfg = IsaacPolicyRuntimeConfiguration.h1Flat
        let provider = DemoPolicyActionProvider(runner: runner,
                                                configuration: cfg)
        let loop = PolicyPhysicsLoop(robotKind: cfg.robotKind, provider: provider)
        loop.reset()
        let result = runLoop(loop, seconds: 10.0, controlInterval: 1.0 / 60.0)

        #expect(!result.sawNaN)
        #expect(result.minBaseUprightZ > 0.95,
                "renderer-cadence uprightZ dipped to \(result.minBaseUprightZ), end=\(result.endBaseUprightZ), x=\(result.endBasePos.x)")
        #expect(result.minBaseZ > 0.55,
                "renderer-cadence base dropped to z=\(result.minBaseZ)")
        #expect(result.endBasePos.x > 1.0,
                "renderer-cadence x displacement was only \(result.endBasePos.x)m over 10 s")
    }

    @Test func anymalRendererCadencePolicyLoopWalksForward() throws {
        let configuration = PolicyModelConfiguration.anymal
        guard PolicyModelRunner.bundledModelURL(configuration: configuration) != nil ||
              PolicyModelRunner.repositoryModelURL(configuration: configuration).map({ FileManager.default.fileExists(atPath: $0.path) }) == true
        else { return }

        let runner = try PolicyModelRunner(configuration: configuration)
        let cfg = IsaacPolicyRuntimeConfiguration.anymalC
        let provider = DemoPolicyActionProvider(runner: runner,
                                                configuration: cfg)
        let loop = PolicyPhysicsLoop(robotKind: cfg.robotKind, provider: provider)
        loop.reset()

        let stats = runLoop(loop, seconds: 10.0, controlInterval: 1.0 / 60.0)
        #expect(!stats.sawNaN)
        #expect(stats.minBaseUprightZ > 0.85,
                "renderer-cadence uprightZ dipped to \(stats.minBaseUprightZ)")
        #expect(stats.minBaseZ > 0.30,
                "renderer-cadence base dropped to z=\(stats.minBaseZ)")
        #expect(stats.endBasePos.x > 1.0,
                "renderer-cadence x displacement was only \(stats.endBasePos.x)m over 10 s")
    }
}
