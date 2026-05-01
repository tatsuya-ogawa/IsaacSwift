//
//  AnymalActuatorRunner.swift
//  IsaacSwift
//
//  CoreML wrapper for ANYmal-C's `sea_net_jit2.pt` (an LSTM-based ANYdrive
//  actuator network) ported into a stateful per-substep torque generator.
//
//  The network maps a `(joint_pos_err, joint_vel)` pair to a torque per joint
//  while keeping a 2-layer × hidden=8 LSTM state per joint. We expose it via
//  the Objective-C `IsaacSwiftJointActuator` protocol so
//  `IsaacSwiftAnymalSimulator` can swap the built-in PD motor for the learned
//  actuator without touching the rest of the pipeline.
//
//  See `scripts/convert_anymal_actuator.py` for the conversion entry point.
//

import CoreML
import Foundation

@objc final class AnymalActuatorRunner: NSObject, IsaacSwiftJointActuator {

    static let configuration = PolicyModelConfiguration(
        resourceName: "anymal_actuator",
        resourceExtension: "mlmodelc",
        inputFeatureName: nil,
        outputFeatureName: nil,
        repositoryRelativePath: "PolicyModels/anymal_actuator.mlmodelc")

    static let jointCount   = 12
    static let numLayers    = 2
    static let hiddenSize   = 8

    private let model: MLModel
    // Hidden / cell state: shape [num_layers, batch=12, hidden=8].
    private var hState: MLMultiArray
    private var cState: MLMultiArray
    // Reusable input scratch buffers.
    private let xBuf:  MLMultiArray

    @objc convenience override init() {
        do {
            try self.init(bundle: .main)
        } catch {
            fatalError("Failed to load anymal_actuator.mlmodelc: \(error)")
        }
    }

    init(bundle: Bundle) throws {
        let modelURL = try Self.resolveModelURL(bundle: bundle)
        let cfg = MLModelConfiguration()
        // Keep the actuator on CPU for parity across tests and real devices.
        // This small stateful LSTM is latency-light, and CPU avoids silent
        // Neural Engine compatibility differences that leave ANYmal with no
        // actuator in the app while unit tests still pass on macOS.
        cfg.computeUnits = .cpuOnly
        self.model = try MLModel(contentsOf: modelURL, configuration: cfg)
        self.hState = try MLMultiArray(shape: [NSNumber(value: Self.numLayers),
                                               NSNumber(value: Self.jointCount),
                                               NSNumber(value: Self.hiddenSize)],
                                       dataType: .float32)
        self.cState = try MLMultiArray(shape: [NSNumber(value: Self.numLayers),
                                               NSNumber(value: Self.jointCount),
                                               NSNumber(value: Self.hiddenSize)],
                                       dataType: .float32)
        self.xBuf   = try MLMultiArray(shape: [NSNumber(value: Self.jointCount),
                                               NSNumber(value: 2)],
                                       dataType: .float32)
        super.init()
        Self.zero(hState)
        Self.zero(cState)
    }

    // MARK: - IsaacSwiftJointActuator

    @objc func resetState() {
        Self.zero(hState)
        Self.zero(cState)
    }

    @objc(torquesForJointPositionErrors:jointVelocities:)
    func torques(forJointPositionErrors jointPositionErrors: [NSNumber],
                 jointVelocities: [NSNumber]) -> [NSNumber] {
        let n = Swift.min(Self.jointCount, jointPositionErrors.count, jointVelocities.count)
        // Pack (pos_err, joint_vel) into the 12×2 input tensor.
        for i in 0..<Self.jointCount {
            let pe = i < n ? jointPositionErrors[i].floatValue : 0
            let rawVelocity = i < n ? jointVelocities[i].floatValue : 0
            let jv = min(max(rawVelocity, -20), 20)
            xBuf[2 * i]     = NSNumber(value: pe)
            xBuf[2 * i + 1] = NSNumber(value: jv)
        }
        do {
            let input = try MLDictionaryFeatureProvider(dictionary: [
                "x":  MLFeatureValue(multiArray: xBuf),
                "h0": MLFeatureValue(multiArray: hState),
                "c0": MLFeatureValue(multiArray: cState),
            ])
            let out = try model.prediction(from: input)
            guard let tau = out.featureValue(for: "tau")?.multiArrayValue,
                  let h1  = out.featureValue(for: "h1")?.multiArrayValue,
                  let c1  = out.featureValue(for: "c1")?.multiArrayValue else {
                fatalError("ANYmal actuator output is missing tau/h1/c1")
            }
            // Promote the new state to the rolling state buffers.
            Self.copy(h1, into: hState)
            Self.copy(c1, into: cState)
            return (0..<Self.jointCount).map { tau[$0] }
        } catch {
            fatalError("ANYmal actuator prediction failed: \(error)")
        }
    }

    // MARK: - Helpers

    private static func zero(_ array: MLMultiArray) {
        let count = array.count
        for i in 0..<count { array[i] = NSNumber(value: Float(0)) }
    }

    private static func copy(_ source: MLMultiArray, into destination: MLMultiArray) {
        let count = Swift.min(source.count, destination.count)
        for i in 0..<count { destination[i] = source[i] }
    }

    private static func resolveModelURL(bundle: Bundle) throws -> URL {
        if let bundled = bundle.url(forResource: configuration.resourceName,
                                    withExtension: configuration.resourceExtension) {
            return bundled
        }
        if let repo = repositoryModelURL() {
            if FileManager.default.fileExists(atPath: repo.path) {
                return repo
            }
        }
        throw PolicyModelError.resourceNotFound(
            "\(configuration.resourceName).\(configuration.resourceExtension)")
    }

    static func repositoryModelURL() -> URL? {
        guard let path = configuration.repositoryRelativePath else { return nil }
        return URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
            .appendingPathComponent(path)
    }
}
