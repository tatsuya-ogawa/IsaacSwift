//
//  RobotModelDefinitions.swift
//  IsaacSwift
//
//  Central registry for robot-specific Swift-side policy, asset, UI, and
//  renderer articulation metadata.
//

import Foundation
import simd

struct RobotModelDefinition {
    let kind: IsaacSwiftRobotKind
    let displayName: String
    let pickerLabel: String
    let assetCandidate: RobotAssetCandidate
    let fallbackAssetCandidates: [RobotAssetCandidate]
    let policyModelConfiguration: PolicyModelConfiguration
    let policyRuntimeConfiguration: IsaacPolicyRuntimeConfiguration
    let articulationProfile: RobotArticulationProfile
    let solidColorByNodePath: [String: SIMD4<UInt8>]

    init(kind: IsaacSwiftRobotKind,
         displayName: String,
         pickerLabel: String,
         assetCandidate: RobotAssetCandidate,
         fallbackAssetCandidates: [RobotAssetCandidate],
         policyModelConfiguration: PolicyModelConfiguration,
         policyRuntimeConfiguration: IsaacPolicyRuntimeConfiguration,
         articulationProfile: RobotArticulationProfile,
         solidColorByNodePath: [String: SIMD4<UInt8>] = [:]) {
        self.kind = kind
        self.displayName = displayName
        self.pickerLabel = pickerLabel
        self.assetCandidate = assetCandidate
        self.fallbackAssetCandidates = fallbackAssetCandidates
        self.policyModelConfiguration = policyModelConfiguration
        self.policyRuntimeConfiguration = policyRuntimeConfiguration
        self.articulationProfile = articulationProfile
        self.solidColorByNodePath = solidColorByNodePath
    }

    var assetCandidates: [RobotAssetCandidate] {
        [assetCandidate] + fallbackAssetCandidates
    }
}

enum RobotModelDefinitions {
    static let defaultKind: IsaacSwiftRobotKind = .spot

    static let all: [RobotModelDefinition] = [
        anymalC,
        spot,
        go2,
    ]

    static var selectable: [RobotModelDefinition] {
        all
    }

    static func definition(for kind: IsaacSwiftRobotKind) -> RobotModelDefinition {
        all.first { $0.kind == kind } ?? spot
    }

    private static let anymalCAsset = RobotAssetCandidate(resourceName: "anymal_c",
                                                          resourceExtension: "usdz",
                                                          subdirectory: "RobotAssets/anymal_c",
                                                          robotKind: .anymalC,
                                                          bundledFiles: [
                                                              BundledAssetFile(sourceName: "anymal_c.usdz",
                                                                               destinationRelativePath: "anymal_c.usdz"),
                                                          ])

    private static let spotAsset = RobotAssetCandidate(resourceName: "spot",
                                                       resourceExtension: "usdz",
                                                       subdirectory: "RobotAssets/spot",
                                                       robotKind: .spot)

    private static let go2Asset = RobotAssetCandidate(resourceName: "go2",
                                                      resourceExtension: "usdz",
                                                      subdirectory: "RobotAssets/go2",
                                                      robotKind: .go2)

    private static let anymalC = RobotModelDefinition(kind: .anymalC,
                                                      displayName: "ANYmal-C",
                                                      pickerLabel: "ANYmal",
                                                      assetCandidate: anymalCAsset,
                                                      fallbackAssetCandidates: [],
                                                      policyModelConfiguration: .anymal,
                                                      policyRuntimeConfiguration: .anymalC,
                                                      articulationProfile: anymalArticulationProfile)

    private static let spot = RobotModelDefinition(kind: .spot,
                                                   displayName: "Spot",
                                                   pickerLabel: "Spot",
                                                   assetCandidate: spotAsset,
                                                   fallbackAssetCandidates: [anymalCAsset],
                                                   policyModelConfiguration: .spot,
                                                   policyRuntimeConfiguration: .spotFlat,
                                                   articulationProfile: spotArticulationProfile)

    private static let go2 = RobotModelDefinition(kind: .go2,
                                                  displayName: "Go2",
                                                  pickerLabel: "Go2",
                                                  assetCandidate: go2Asset,
                                                  fallbackAssetCandidates: [anymalCAsset],
                                                  policyModelConfiguration: .spot,
                                                  policyRuntimeConfiguration: .go2,
                                                  articulationProfile: go2ArticulationProfile,
                                                  solidColorByNodePath: go2SolidColorByNodePath)
}

extension IsaacSwiftRobotKind {
    var modelDefinition: RobotModelDefinition {
        RobotModelDefinitions.definition(for: self)
    }
}

extension IsaacPolicyRuntimeConfiguration {
    static let anymalC = IsaacPolicyRuntimeConfiguration(robotKind: .anymalC,
                                                         physicsTimeStep: 1.0 / 200.0,
                                                         policyDecimation: 4,
                                                         actionScale: 0.5,
                                                         defaultCommand: SIMD3<Float>(1.0, 0, 0),
                                                         simToPolicyJointPermutation: [
                                                             0, 4, 8,   // LF: HAA, HFE, KFE
                                                             2, 6, 10,  // RF
                                                             1, 5, 9,   // LH
                                                             3, 7, 11,  // RH
                                                         ])

    static let spotFlat = IsaacPolicyRuntimeConfiguration(robotKind: .spot,
                                                          physicsTimeStep: 1.0 / 500.0,
                                                          policyDecimation: 10,
                                                          actionScale: 0.2,
                                                          defaultCommand: SIMD3<Float>(0.8, 0, 0),
                                                          simToPolicyJointPermutation: [
                                                              0, 4, 8,   // FL: hx, hy, kn
                                                              1, 5, 9,   // FR
                                                              2, 6, 10,  // HL
                                                              3, 7, 11,  // HR
                                                          ])

    // Go2 does not have a bundled policy yet. This runtime entry is a
    // placeholder matching Isaac Sim's Go2 USD joint metadata closely enough
    // for selection, visualization, and zero-action physics tests.
    static let go2 = IsaacPolicyRuntimeConfiguration(robotKind: .go2,
                                                     physicsTimeStep: 1.0 / 200.0,
                                                     policyDecimation: 4,
                                                     actionScale: 0.25,
                                                     defaultCommand: SIMD3<Float>(0.4, 0, 0),
                                                     simToPolicyJointPermutation: [
                                                         0, 4, 5,    // FL: hip, thigh, calf
                                                         1, 6, 7,    // FR
                                                         2, 8, 9,    // RL
                                                         3, 10, 11,  // RR
                                                     ])

    static func configuration(for robotKind: IsaacSwiftRobotKind) -> IsaacPolicyRuntimeConfiguration {
        robotKind.modelDefinition.policyRuntimeConfiguration
    }
}

extension Renderer {
    static func articulationProfile(for kind: IsaacSwiftRobotKind) -> RobotArticulationProfile {
        kind.modelDefinition.articulationProfile
    }
}

private let anymalPolicyJointBindings = [
    JointActionBinding(nodePath: "/anymal/LF_HIP", actionIndex: 0),
    JointActionBinding(nodePath: "/anymal/LF_THIGH", actionIndex: 1),
    JointActionBinding(nodePath: "/anymal/LF_SHANK", actionIndex: 2),
    JointActionBinding(nodePath: "/anymal/RF_HIP", actionIndex: 3),
    JointActionBinding(nodePath: "/anymal/RF_THIGH", actionIndex: 4),
    JointActionBinding(nodePath: "/anymal/RF_SHANK", actionIndex: 5),
    JointActionBinding(nodePath: "/anymal/LH_HIP", actionIndex: 6),
    JointActionBinding(nodePath: "/anymal/LH_THIGH", actionIndex: 7),
    JointActionBinding(nodePath: "/anymal/LH_SHANK", actionIndex: 8),
    JointActionBinding(nodePath: "/anymal/RH_HIP", actionIndex: 9),
    JointActionBinding(nodePath: "/anymal/RH_THIGH", actionIndex: 10),
    JointActionBinding(nodePath: "/anymal/RH_SHANK", actionIndex: 11),
]

private let anymalArticulationParentByNodePath: [String: String] = [
    "/anymal/LF_HIP": "/anymal/base",
    "/anymal/LF_THIGH": "/anymal/LF_HIP",
    "/anymal/LF_SHANK": "/anymal/LF_THIGH",
    "/anymal/LF_FOOT": "/anymal/LF_SHANK",
    "/anymal/LH_HIP": "/anymal/base",
    "/anymal/LH_THIGH": "/anymal/LH_HIP",
    "/anymal/LH_SHANK": "/anymal/LH_THIGH",
    "/anymal/LH_FOOT": "/anymal/LH_SHANK",
    "/anymal/RF_HIP": "/anymal/base",
    "/anymal/RF_THIGH": "/anymal/RF_HIP",
    "/anymal/RF_SHANK": "/anymal/RF_THIGH",
    "/anymal/RF_FOOT": "/anymal/RF_SHANK",
    "/anymal/RH_HIP": "/anymal/base",
    "/anymal/RH_THIGH": "/anymal/RH_HIP",
    "/anymal/RH_SHANK": "/anymal/RH_THIGH",
    "/anymal/RH_FOOT": "/anymal/RH_SHANK",
]

private let anymalArticulationTransformOverrides = [
    ArticulationTransformOverride(childPath: "/anymal/LF_HIP",
                                  parentPath: "/anymal/base",
                                  localPos0: SIMD3<Float>(0.2999, 0.104, 0),
                                  localRot0: usdQuaternion(real: 0.25881907, ix: 0.9659258, iy: 0, iz: 0),
                                  localPos1: .zero,
                                  localRot1: usdQuaternion(real: 1, ix: 0, iy: 0, iz: 0)),
    ArticulationTransformOverride(childPath: "/anymal/LH_HIP",
                                  parentPath: "/anymal/base",
                                  localPos0: SIMD3<Float>(-0.2999, 0.104, 0),
                                  localRot0: usdQuaternion(real: -0.9659258, ix: 0.25881907, iy: -5.3535302e-8, iz: 5.3535302e-8),
                                  localPos1: .zero,
                                  localRot1: usdQuaternion(real: -4.371139e-8, ix: 0, iy: 1, iz: 0)),
    ArticulationTransformOverride(childPath: "/anymal/RF_HIP",
                                  parentPath: "/anymal/base",
                                  localPos0: SIMD3<Float>(0.2999, -0.104, 0),
                                  localRot0: usdQuaternion(real: 0.25881907, ix: -0.9659258, iy: 0, iz: 0),
                                  localPos1: .zero,
                                  localRot1: usdQuaternion(real: 1, ix: 0, iy: 0, iz: 0)),
    ArticulationTransformOverride(childPath: "/anymal/RH_HIP",
                                  parentPath: "/anymal/base",
                                  localPos0: SIMD3<Float>(-0.2999, -0.104, 0),
                                  localRot0: usdQuaternion(real: 0.9659258, ix: 0.25881907, iy: 3.0908616e-8, iz: -3.0908616e-8),
                                  localPos1: .zero,
                                  localRot1: usdQuaternion(real: -4.371139e-8, ix: 0, iy: 1, iz: 0)),
    ArticulationTransformOverride(childPath: "/anymal/LF_THIGH",
                                  parentPath: "/anymal/LF_HIP",
                                  localPos0: SIMD3<Float>(0.059899993, -0.07258159, -0.041905005),
                                  localRot0: usdQuaternion(real: 0.18301272, ix: -0.68301266, iy: 0.68301266, iz: 0.18301272),
                                  localPos1: .zero,
                                  localRot1: usdQuaternion(real: 1, ix: 0, iy: 0, iz: 0)),
    ArticulationTransformOverride(childPath: "/anymal/LH_THIGH",
                                  parentPath: "/anymal/LH_HIP",
                                  localPos0: SIMD3<Float>(0.05990001, 0.07258159, -0.041905005),
                                  localRot0: usdQuaternion(real: 0.18301271, ix: 0.6830127, iy: 0.6830126, iz: -0.18301274),
                                  localPos1: .zero,
                                  localRot1: usdQuaternion(real: 1, ix: 0, iy: 0, iz: 0)),
    ArticulationTransformOverride(childPath: "/anymal/RF_THIGH",
                                  parentPath: "/anymal/RF_HIP",
                                  localPos0: SIMD3<Float>(0.059899993, 0.07258159, -0.041905005),
                                  localRot0: usdQuaternion(real: -0.68301266, ix: 0.1830127, iy: 0.1830127, iz: 0.68301266),
                                  localPos1: .zero,
                                  localRot1: usdQuaternion(real: -4.371139e-8, ix: 0, iy: 1, iz: 0)),
    ArticulationTransformOverride(childPath: "/anymal/RH_THIGH",
                                  parentPath: "/anymal/RH_HIP",
                                  localPos0: SIMD3<Float>(0.059899993, -0.07258159, -0.041905005),
                                  localRot0: usdQuaternion(real: 0.6830127, ix: 0.18301268, iy: -0.18301271, iz: 0.6830126),
                                  localPos1: .zero,
                                  localRot1: usdQuaternion(real: -4.371139e-8, ix: 0, iy: 1, iz: 0)),
    ArticulationTransformOverride(childPath: "/anymal/LF_SHANK",
                                  parentPath: "/anymal/LF_THIGH",
                                  localPos0: SIMD3<Float>(0.10029999, -5.978346e-9, -0.28499994),
                                  localRot0: usdQuaternion(real: 0.99999994, ix: 0, iy: 0, iz: 0),
                                  localPos1: .zero,
                                  localRot1: usdQuaternion(real: 1, ix: 0, iy: 0, iz: 0)),
    ArticulationTransformOverride(childPath: "/anymal/LH_SHANK",
                                  parentPath: "/anymal/LH_THIGH",
                                  localPos0: SIMD3<Float>(0.10029999, -5.978346e-9, -0.28499994),
                                  localRot0: usdQuaternion(real: 0.99999994, ix: 0, iy: 0, iz: 0),
                                  localPos1: .zero,
                                  localRot1: usdQuaternion(real: 1, ix: 0, iy: 0, iz: 0)),
    ArticulationTransformOverride(childPath: "/anymal/RF_SHANK",
                                  parentPath: "/anymal/RF_THIGH",
                                  localPos0: SIMD3<Float>(0.10029999, 5.978346e-9, -0.28499994),
                                  localRot0: usdQuaternion(real: -4.3711385e-8, ix: 0, iy: 0.99999994, iz: 0),
                                  localPos1: .zero,
                                  localRot1: usdQuaternion(real: -4.371139e-8, ix: 0, iy: 1, iz: 0)),
    ArticulationTransformOverride(childPath: "/anymal/RH_SHANK",
                                  parentPath: "/anymal/RH_THIGH",
                                  localPos0: SIMD3<Float>(0.10029999, 5.978346e-9, -0.28499994),
                                  localRot0: usdQuaternion(real: -4.3711385e-8, ix: 0, iy: 0.99999994, iz: 0),
                                  localPos1: .zero,
                                  localRot1: usdQuaternion(real: -4.371139e-8, ix: 0, iy: 1, iz: 0)),
    ArticulationTransformOverride(childPath: "/anymal/LF_FOOT",
                                  parentPath: "/anymal/LF_SHANK",
                                  localPos0: SIMD3<Float>(0.013049994, -0.08795, -0.33796993),
                                  localRot0: usdQuaternion(real: 0.70710677, ix: 0, iy: 0, iz: -0.70710677),
                                  localPos1: .zero,
                                  localRot1: usdQuaternion(real: 1, ix: 0, iy: 0, iz: 0)),
    ArticulationTransformOverride(childPath: "/anymal/LH_FOOT",
                                  parentPath: "/anymal/LH_SHANK",
                                  localPos0: SIMD3<Float>(0.013050005, 0.08795, -0.33796993),
                                  localRot0: usdQuaternion(real: 0.70710677, ix: 0, iy: 0, iz: -0.70710677),
                                  localPos1: .zero,
                                  localRot1: usdQuaternion(real: 1, ix: 0, iy: 0, iz: 0)),
    ArticulationTransformOverride(childPath: "/anymal/RF_FOOT",
                                  parentPath: "/anymal/RF_SHANK",
                                  localPos0: SIMD3<Float>(0.013049994, 0.08795, -0.33796993),
                                  localRot0: usdQuaternion(real: 0.70710677, ix: 0, iy: 0, iz: 0.70710677),
                                  localPos1: .zero,
                                  localRot1: usdQuaternion(real: 1, ix: 0, iy: 0, iz: 0)),
    ArticulationTransformOverride(childPath: "/anymal/RH_FOOT",
                                  parentPath: "/anymal/RH_SHANK",
                                  localPos0: SIMD3<Float>(0.013050005, -0.08795, -0.33796993),
                                  localRot0: usdQuaternion(real: 0.70710677, ix: 0, iy: 0, iz: 0.70710677),
                                  localPos1: .zero,
                                  localRot1: usdQuaternion(real: 1, ix: 0, iy: 0, iz: 0)),
]

private let anymalArticulationProfile = RobotArticulationProfile(
    baseNodePath: "/anymal/base",
    policyJointBindings: anymalPolicyJointBindings,
    articulationParents: anymalArticulationParentByNodePath,
    transformOverrides: anymalArticulationTransformOverrides)

private let spotLegOffsets: [(hx: Float, hy: Float, kn: Float, hipX: Float, hipY: Float, ulegY: Float)] = [
    (hx:  0.1, hy: 0.9, kn: -1.5, hipX:  0.29785, hipY:  0.055, ulegY:  0.110945),
    (hx: -0.1, hy: 0.9, kn: -1.5, hipX:  0.29785, hipY: -0.055, ulegY: -0.110945),
    (hx:  0.1, hy: 1.1, kn: -1.5, hipX: -0.29785, hipY:  0.055, ulegY:  0.110945),
    (hx: -0.1, hy: 1.1, kn: -1.5, hipX: -0.29785, hipY: -0.055, ulegY: -0.110945),
]

private let spotLegPrefixes = ["fl", "fr", "hl", "hr"]

private let spotPolicyJointBindings: [JointActionBinding] = {
    var bindings: [JointActionBinding] = []
    for (legIdx, prefix) in spotLegPrefixes.enumerated() {
        bindings.append(JointActionBinding(nodePath: "/spot/\(prefix)_hip", actionIndex: legIdx * 3 + 0))
        bindings.append(JointActionBinding(nodePath: "/spot/\(prefix)_uleg", actionIndex: legIdx * 3 + 1))
        bindings.append(JointActionBinding(nodePath: "/spot/\(prefix)_lleg", actionIndex: legIdx * 3 + 2))
    }
    return bindings
}()

private let spotArticulationParents: [String: String] = {
    var parents: [String: String] = [:]
    for prefix in spotLegPrefixes {
        parents["/spot/\(prefix)_hip"] = "/spot/body"
        parents["/spot/\(prefix)_uleg"] = "/spot/\(prefix)_hip"
        parents["/spot/\(prefix)_lleg"] = "/spot/\(prefix)_uleg"
        parents["/spot/\(prefix)_foot"] = "/spot/\(prefix)_lleg"
    }
    return parents
}()

private let spotArticulationTransformOverrides: [ArticulationTransformOverride] = {
    let identityQuat = usdQuaternion(real: 1, ix: 0, iy: 0, iz: 0)
    let xAxis = SIMD3<Float>(1, 0, 0)
    let yAxis = SIMD3<Float>(0, 1, 0)
    var overrides: [ArticulationTransformOverride] = []

    for (legIdx, prefix) in spotLegPrefixes.enumerated() {
        let leg = spotLegOffsets[legIdx]
        overrides.append(ArticulationTransformOverride(childPath: "/spot/\(prefix)_hip",
                                                       parentPath: "/spot/body",
                                                       localPos0: SIMD3<Float>(leg.hipX, leg.hipY, 0),
                                                       localRot0: identityQuat,
                                                       localPos1: .zero,
                                                       localRot1: identityQuat,
                                                       jointAxis: xAxis,
                                                       restAngleOffset: leg.hx))
        overrides.append(ArticulationTransformOverride(childPath: "/spot/\(prefix)_uleg",
                                                       parentPath: "/spot/\(prefix)_hip",
                                                       localPos0: SIMD3<Float>(0, leg.ulegY, 0),
                                                       localRot0: identityQuat,
                                                       localPos1: .zero,
                                                       localRot1: identityQuat,
                                                       jointAxis: yAxis,
                                                       restAngleOffset: leg.hy))
        overrides.append(ArticulationTransformOverride(childPath: "/spot/\(prefix)_lleg",
                                                       parentPath: "/spot/\(prefix)_uleg",
                                                       localPos0: SIMD3<Float>(0.025, 0, -0.3205),
                                                       localRot0: identityQuat,
                                                       localPos1: .zero,
                                                       localRot1: identityQuat,
                                                       jointAxis: yAxis,
                                                       restAngleOffset: leg.kn))
        overrides.append(ArticulationTransformOverride(childPath: "/spot/\(prefix)_foot",
                                                       parentPath: "/spot/\(prefix)_lleg",
                                                       localPos0: SIMD3<Float>(0, 0, -0.3365),
                                                       localRot0: identityQuat,
                                                       localPos1: .zero,
                                                       localRot1: identityQuat))
    }
    return overrides
}()

private let spotArticulationProfile = RobotArticulationProfile(
    baseNodePath: "/spot/body",
    policyJointBindings: spotPolicyJointBindings,
    articulationParents: spotArticulationParents,
    transformOverrides: spotArticulationTransformOverrides)

private let go2LegPrefixes = ["FL", "FR", "RL", "RR"]

private let go2PolicyJointBindings: [JointActionBinding] = {
    var bindings: [JointActionBinding] = []
    for (legIdx, prefix) in go2LegPrefixes.enumerated() {
        bindings.append(JointActionBinding(nodePath: "/go2_description/\(prefix)_hip", actionIndex: legIdx * 3 + 0))
        bindings.append(JointActionBinding(nodePath: "/go2_description/\(prefix)_thigh", actionIndex: legIdx * 3 + 1))
        bindings.append(JointActionBinding(nodePath: "/go2_description/\(prefix)_calf", actionIndex: legIdx * 3 + 2))
    }
    return bindings
}()

private let go2ArticulationParents: [String: String] = {
    var parents: [String: String] = [:]
    for prefix in go2LegPrefixes {
        parents["/go2_description/\(prefix)_hip"] = "/go2_description/base"
        parents["/go2_description/\(prefix)_thigh"] = "/go2_description/\(prefix)_hip"
        parents["/go2_description/\(prefix)_calf"] = "/go2_description/\(prefix)_thigh"
        parents["/go2_description/\(prefix)_foot"] = "/go2_description/\(prefix)_calf"
    }
    return parents
}()

private let go2ArticulationTransformOverrides: [ArticulationTransformOverride] = {
    let identityQuat = usdQuaternion(real: 1, ix: 0, iy: 0, iz: 0)
    let jointQuat = usdQuaternion(real: 0.70710677, ix: 0, iy: 0, iz: 0.70710677)
    let xAxis = SIMD3<Float>(1, 0, 0)
    let legOffsets: [(hipX: Float, hipY: Float, thighY: Float, hip: Float, thigh: Float, calf: Float)] = [
        (hipX:  0.1934, hipY:  0.0465, thighY:  0.0955, hip:  0.1, thigh: 0.8, calf: -1.5),
        (hipX:  0.1934, hipY: -0.0465, thighY: -0.0955, hip: -0.1, thigh: 0.8, calf: -1.5),
        (hipX: -0.1934, hipY:  0.0465, thighY:  0.0955, hip:  0.1, thigh: 1.0, calf: -1.5),
        (hipX: -0.1934, hipY: -0.0465, thighY: -0.0955, hip: -0.1, thigh: 1.0, calf: -1.5),
    ]
    var overrides: [ArticulationTransformOverride] = []

    for (legIdx, prefix) in go2LegPrefixes.enumerated() {
        let leg = legOffsets[legIdx]
        overrides.append(ArticulationTransformOverride(childPath: "/go2_description/\(prefix)_hip",
                                                       parentPath: "/go2_description/base",
                                                       localPos0: SIMD3<Float>(leg.hipX, leg.hipY, 0),
                                                       localRot0: identityQuat,
                                                       localPos1: .zero,
                                                       localRot1: identityQuat,
                                                       jointAxis: xAxis,
                                                       restAngleOffset: leg.hip))
        overrides.append(ArticulationTransformOverride(childPath: "/go2_description/\(prefix)_thigh",
                                                       parentPath: "/go2_description/\(prefix)_hip",
                                                       localPos0: SIMD3<Float>(0, leg.thighY, 0),
                                                       localRot0: jointQuat,
                                                       localPos1: .zero,
                                                       localRot1: jointQuat,
                                                       jointAxis: xAxis,
                                                       restAngleOffset: leg.thigh))
        overrides.append(ArticulationTransformOverride(childPath: "/go2_description/\(prefix)_calf",
                                                       parentPath: "/go2_description/\(prefix)_thigh",
                                                       localPos0: SIMD3<Float>(0, 0, -0.213),
                                                       localRot0: jointQuat,
                                                       localPos1: .zero,
                                                       localRot1: jointQuat,
                                                       jointAxis: xAxis,
                                                       restAngleOffset: leg.calf))
        overrides.append(ArticulationTransformOverride(childPath: "/go2_description/\(prefix)_foot",
                                                       parentPath: "/go2_description/\(prefix)_calf",
                                                       localPos0: SIMD3<Float>(0, 0, -0.213),
                                                       localRot0: usdQuaternion(real: 0.70710677, ix: 0, iy: 0.70710677, iz: 0),
                                                       localPos1: .zero,
                                                       localRot1: usdQuaternion(real: 0.70710677, ix: 0, iy: 0.70710677, iz: 0)))
    }
    return overrides
}()

private let go2ArticulationProfile = RobotArticulationProfile(
    baseNodePath: "/go2_description/base",
    policyJointBindings: go2PolicyJointBindings,
    articulationParents: go2ArticulationParents,
    transformOverrides: go2ArticulationTransformOverrides)

private let go2SolidColorByNodePath: [String: SIMD4<UInt8>] = {
    let mediumGray = SIMD4<UInt8>(77, 77, 77, 255)
    let lightGray = SIMD4<UInt8>(115, 115, 115, 255)
    let darkGray = SIMD4<UInt8>(13, 13, 13, 255)
    let white = SIMD4<UInt8>(255, 255, 255, 255)
    var colors: [String: SIMD4<UInt8>] = [
        "/go2_description/base/visuals": mediumGray,
        "/go2_description/base_black/visuals": darkGray,
        "/go2_description/base_white/visuals": white,
    ]

    for prefix in go2LegPrefixes {
        colors["/go2_description/\(prefix)_hip/visuals"] = lightGray
        colors["/go2_description/\(prefix)_hip_protector/visuals"] = mediumGray
        colors["/go2_description/\(prefix)_thigh/visuals"] = lightGray
        colors["/go2_description/\(prefix)_calf/visuals"] = lightGray
        colors["/go2_description/\(prefix)_foot/visuals"] = darkGray
        colors["/go2_description/\(prefix)_thigh_protector/visuals"] = mediumGray
    }

    return colors
}()
