//
//  Renderer+Articulation.swift
//  IsaacSwift
//
//  Generic robot articulation transform resolution.
//

import Foundation
import ModelIO
import simd

extension Renderer {
    static func sceneNodes(from assetURL: URL) -> [SceneNode] {
        sceneNodes(from: assetURL,
                   profile: RobotModelDefinitions.definition(for: RobotModelDefinitions.defaultKind).articulationProfile)
    }

    static func sceneNodes(from assetURL: URL,
                           robotKind: IsaacSwiftRobotKind) -> [SceneNode] {
        sceneNodes(from: assetURL,
                   profile: robotKind.modelDefinition.articulationProfile)
    }

    static func sceneNodes(from assetURL: URL,
                           profile: RobotArticulationProfile) -> [SceneNode] {
        let asset = MDLAsset(url: assetURL)
        var nodes: [SceneNode] = []

        func walk(_ object: MDLObject,
                  parentIndex: Int?,
                  parentPath: String) {
            let localTransform = object.transform?.matrix ?? matrix_identity_float4x4
            let nodeName = object.name.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty
                ? String(describing: type(of: object))
                : object.name
            let nodePath = parentPath.isEmpty ? "/\(nodeName)" : "\(parentPath)/\(nodeName)"
            let nodeIndex = nodes.count
            nodes.append(SceneNode(path: nodePath,
                                   name: nodeName,
                                   localTransform: localTransform,
                                   parentIndex: parentIndex))

            for child in object.children.objects {
                walk(child,
                     parentIndex: nodeIndex,
                     parentPath: nodePath)
            }

            if let instance = object.instance {
                walk(instance,
                     parentIndex: nodeIndex,
                     parentPath: nodePath)
            }
        }

        for index in 0..<asset.count {
            walk(asset.object(at: index),
                 parentIndex: nil,
                 parentPath: "")
        }

        return articulationResolvedSceneNodes(from: nodes, profile: profile)
    }

    static func worldTransforms(for nodes: [SceneNode],
                                actions: [Float]) -> [matrix_float4x4] {
        worldTransforms(for: nodes,
                        actions: actions,
                        profile: RobotModelDefinitions.definition(for: RobotModelDefinitions.defaultKind).articulationProfile)
    }

    static func worldTransforms(for nodes: [SceneNode],
                                actions: [Float],
                                profile: RobotArticulationProfile) -> [matrix_float4x4] {
        guard !nodes.isEmpty else {
            return []
        }

        return accumulatedWorldTransforms(for: nodes) { index in
            let node = nodes[index]
            if let articulationOverride = profile.transformOverrideByNodePath[node.path] {
                return articulatedLocalTransform(for: articulationOverride,
                                                 actionBinding: profile.policyJointBindingByNodePath[node.path],
                                                 actions: actions)
            }

            var localTransform = node.localTransform
            if let binding = profile.policyJointBindingByNodePath[node.path],
               binding.actionIndex < actions.count {
                let jointRotation = matrix4x4_rotation(radians: actions[binding.actionIndex],
                                                       axis: SIMD3<Float>(1, 0, 0))
                localTransform = simd_mul(localTransform, jointRotation)
            }
            return localTransform
        }
    }

    private static func articulationResolvedSceneNodes(from nodes: [SceneNode]) -> [SceneNode] {
        articulationResolvedSceneNodes(from: nodes,
                                       profile: RobotModelDefinitions.definition(for: RobotModelDefinitions.defaultKind).articulationProfile)
    }

    static func articulationResolvedSceneNodes(from nodes: [SceneNode],
                                               profile: RobotArticulationProfile) -> [SceneNode] {
        guard !nodes.isEmpty else {
            return []
        }

        var nodeIndicesByPath: [String: Int] = [:]
        for (index, node) in nodes.enumerated() where nodeIndicesByPath[node.path] == nil {
            nodeIndicesByPath[node.path] = index
        }
        let restWorldTransforms = restWorldTransforms(for: nodes)
        var resolvedNodes = nodes

        for (index, node) in nodes.enumerated() {
            guard let parentPath = profile.articulationParents[node.path],
                  let parentIndex = nodeIndicesByPath[parentPath],
                  parentIndex != index else {
                continue
            }

            let localTransform: matrix_float4x4
            if let articulationOverride = profile.transformOverrideByNodePath[node.path] {
                localTransform = articulatedRestLocalTransform(for: articulationOverride)
            } else {
                let parentWorldTransform = restWorldTransforms[parentIndex]
                let nodeWorldTransform = restWorldTransforms[index]
                localTransform = simd_mul(simd_inverse(parentWorldTransform), nodeWorldTransform)
            }
            resolvedNodes[index] = SceneNode(path: node.path,
                                             name: node.name,
                                             localTransform: localTransform,
                                             parentIndex: parentIndex)
        }

        return resolvedNodes
    }

    private static func restWorldTransforms(for nodes: [SceneNode]) -> [matrix_float4x4] {
        guard !nodes.isEmpty else {
            return []
        }

        return accumulatedWorldTransforms(for: nodes) { index in
            nodes[index].localTransform
        }
    }

    private static func accumulatedWorldTransforms(for nodes: [SceneNode],
                                                   localTransformAtIndex: (Int) -> matrix_float4x4) -> [matrix_float4x4] {
        var worldTransforms = Array(repeating: matrix_identity_float4x4, count: nodes.count)
        var resolved = Array(repeating: false, count: nodes.count)

        func resolveWorldTransform(at index: Int) -> matrix_float4x4 {
            if resolved[index] {
                return worldTransforms[index]
            }

            let localTransform = localTransformAtIndex(index)
            if let parentIndex = nodes[index].parentIndex {
                worldTransforms[index] = simd_mul(resolveWorldTransform(at: parentIndex), localTransform)
            } else {
                worldTransforms[index] = localTransform
            }

            resolved[index] = true
            return worldTransforms[index]
        }

        for index in nodes.indices {
            _ = resolveWorldTransform(at: index)
        }

        return worldTransforms
    }

    private static func articulatedLocalTransform(for articulationOverride: ArticulationTransformOverride,
                                                  actionBinding: JointActionBinding?,
                                                  actions: [Float]) -> matrix_float4x4 {
        var jointTransform = articulatedRestLocalTransform(for: articulationOverride,
                                                           includeChildFrameInverse: false)
        var angle = articulationOverride.restAngleOffset
        if let actionBinding,
           actionBinding.actionIndex < actions.count {
            angle += actions[actionBinding.actionIndex]
        }
        if angle != 0 {
            let jointRotation = matrix4x4_rotation(radians: angle,
                                                   axis: articulationOverride.jointAxis)
            jointTransform = simd_mul(jointTransform, jointRotation)
        }

        return simd_mul(jointTransform,
                        articulatedChildFrameInverseTransform(for: articulationOverride))
    }

    private static func articulatedRestLocalTransform(for articulationOverride: ArticulationTransformOverride,
                                                      includeChildFrameInverse: Bool = true) -> matrix_float4x4 {
        let parentFrameTransform = matrix4x4_translation(articulationOverride.localPos0.x,
                                                         articulationOverride.localPos0.y,
                                                         articulationOverride.localPos0.z)
        let parentFrameRotation = matrix4x4_quaternion(articulationOverride.localRot0)
        let parentFrame = simd_mul(parentFrameTransform, parentFrameRotation)

        if includeChildFrameInverse {
            return simd_mul(parentFrame,
                            articulatedChildFrameInverseTransform(for: articulationOverride))
        }

        return parentFrame
    }

    private static func articulatedChildFrameInverseTransform(for articulationOverride: ArticulationTransformOverride) -> matrix_float4x4 {
        let childFrameTransform = matrix4x4_translation(articulationOverride.localPos1.x,
                                                        articulationOverride.localPos1.y,
                                                        articulationOverride.localPos1.z)
        let childFrameRotation = matrix4x4_quaternion(articulationOverride.localRot1)
        let childFrame = simd_mul(childFrameTransform, childFrameRotation)
        return simd_inverse(childFrame)
    }

    static func worldTransformsByNodePath(for assetURL: URL,
                                          actions: [Float]) -> [String: matrix_float4x4] {
        let nodes = sceneNodes(from: assetURL)
        let transforms = worldTransforms(for: nodes, actions: actions)
        var transformsByNodePath: [String: matrix_float4x4] = [:]

        for (index, node) in nodes.enumerated() where index < transforms.count {
            transformsByNodePath[node.path] = transforms[index]
        }

        return transformsByNodePath
    }

    static func worldTransformsByNodeName(for assetURL: URL,
                                          actions: [Float]) -> [String: matrix_float4x4] {
        let nodes = sceneNodes(from: assetURL)
        let transforms = worldTransforms(for: nodes, actions: actions)
        var transformsByNodeName: [String: matrix_float4x4] = [:]

        for (index, node) in nodes.enumerated() where index < transforms.count {
            if transformsByNodeName[node.name] == nil {
                transformsByNodeName[node.name] = transforms[index]
            }
        }

        return transformsByNodeName
    }

    static func visualizedPolicyActions(from rawActions: [Float],
                                        robotKind: IsaacSwiftRobotKind = RobotModelDefinitions.defaultKind) -> [Float] {
        let actionScale = IsaacPolicyRuntimeConfiguration.configuration(for: robotKind).actionScale
        return rawActions.map { $0 * actionScale }
    }
}
