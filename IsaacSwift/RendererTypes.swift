//
//  RendererTypes.swift
//  IsaacSwift
//
//  Shared renderer data types.
//

import Foundation
import Metal
import MetalKit
import simd

let alignedUniformsSize = (MemoryLayout<Uniforms>.size + 0xFF) & -0x100
let maxBuffersInFlight = 3

nonisolated enum RendererError: Error {
    case assetNotFound
    case assetPreparationFailed(String)
    case badVertexDescriptor
    case meshNotFound
}

struct RobotAssetCandidate {
    let resourceName: String
    let resourceExtension: String
    let subdirectory: String
    let robotKind: IsaacSwiftRobotKind
    let bundledFiles: [BundledAssetFile]

    init(resourceName: String,
         resourceExtension: String,
         subdirectory: String,
         robotKind: IsaacSwiftRobotKind,
         bundledFiles: [BundledAssetFile] = []) {
        self.resourceName = resourceName
        self.resourceExtension = resourceExtension
        self.subdirectory = subdirectory
        self.robotKind = robotKind
        self.bundledFiles = bundledFiles
    }
}

struct BundledAssetFile {
    let sourceName: String
    let destinationRelativePath: String
}

struct StoredZIPEntry {
    let path: String
    let payloadRange: Range<Int>
    let isDirectory: Bool
}

struct MaterialTextureContext {
    let device: MTLDevice
    let textureLoader: MTKTextureLoader
    let searchDirectories: [URL]
    let defaultTexture: MTLTexture
    let textureRelativePathsByMaterialName: [String: [String]]
    let solidColorByNodePath: [String: SIMD4<UInt8>]
}

enum BaseColorTextureSource: Hashable {
    case textureSampler
    case url(URL)
    case string(URL)
    case solidColor
    case fallback
}

struct BaseColorTextureResolution {
    let texture: MTLTexture
    let source: BaseColorTextureSource
}

struct SubmeshRenderData {
    let submesh: MTKSubmesh
    let baseColorTexture: MTLTexture
}

struct SceneNode {
    let path: String
    let name: String
    let localTransform: matrix_float4x4
    let parentIndex: Int?
}

struct JointActionBinding {
    let nodePath: String
    let actionIndex: Int
}

struct ArticulationTransformOverride {
    let childPath: String
    let parentPath: String
    let localPos0: SIMD3<Float>
    let localRot0: simd_quatf
    let localPos1: SIMD3<Float>
    let localRot1: simd_quatf
    /// Hinge axis expressed in the joint frame. Defaults to +X (ANYmal's
    /// existing convention where the per-joint orientation is baked into
    /// `localRot0`). Spot stores joint frames as identity and instead
    /// distinguishes axes per joint, so its overrides set Y where needed.
    let jointAxis: SIMD3<Float>
    /// Constant angle [rad] added to the policy delta before rotating around
    /// `jointAxis`. Used to render the standing-pose offset for robots whose
    /// USD rest geometry is the zero-joint pose (Spot). ANYmal's USD already
    /// bakes the standing pose into `localRot0`, so this is 0 for ANYmal.
    let restAngleOffset: Float

    init(childPath: String,
         parentPath: String,
         localPos0: SIMD3<Float>,
         localRot0: simd_quatf,
         localPos1: SIMD3<Float>,
         localRot1: simd_quatf,
         jointAxis: SIMD3<Float> = SIMD3<Float>(1, 0, 0),
         restAngleOffset: Float = 0) {
        self.childPath = childPath
        self.parentPath = parentPath
        self.localPos0 = localPos0
        self.localRot0 = localRot0
        self.localPos1 = localPos1
        self.localRot1 = localRot1
        self.jointAxis = jointAxis
        self.restAngleOffset = restAngleOffset
    }
}

struct RobotArticulationProfile {
    let baseNodePath: String
    let policyJointBindings: [JointActionBinding]
    let articulationParents: [String: String]
    let transformOverrides: [ArticulationTransformOverride]

    let policyJointBindingByNodePath: [String: JointActionBinding]
    let transformOverrideByNodePath: [String: ArticulationTransformOverride]

    init(baseNodePath: String,
         policyJointBindings: [JointActionBinding],
         articulationParents: [String: String],
         transformOverrides: [ArticulationTransformOverride]) {
        self.baseNodePath = baseNodePath
        self.policyJointBindings = policyJointBindings
        self.articulationParents = articulationParents
        self.transformOverrides = transformOverrides
        self.policyJointBindingByNodePath = Dictionary(uniqueKeysWithValues: policyJointBindings.map { ($0.nodePath, $0) })
        self.transformOverrideByNodePath = Dictionary(uniqueKeysWithValues: transformOverrides.map { ($0.childPath, $0) })
    }
}

struct MeshRenderData {
    let mesh: MTKMesh
    let nodeIndex: Int
    let submeshes: [SubmeshRenderData]
}

struct AssetScene {
    let meshes: [MeshRenderData]
    let nodes: [SceneNode]
    let boundsCenter: SIMD3<Float>
    let boundsRadius: Float
}

extension Array {
    subscript(safe index: Int) -> Element? {
        guard indices.contains(index) else {
            return nil
        }
        return self[index]
    }
}
