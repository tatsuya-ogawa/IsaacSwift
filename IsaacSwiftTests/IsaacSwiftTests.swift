//
//  IsaacSwiftTests.swift
//  IsaacSwiftTests
//
//  Created by Tatsuya Ogawa on 2026/04/29.
//

import Testing
import CoreML
import Metal
import MetalKit
import ModelIO
import simd
@testable import IsaacSwift

struct IsaacSwiftTests {
    private struct TextureResolutionSummary {
        let submeshCount: Int
        let resolvedCounts: [BaseColorTextureSource: Int]
        let resolvedTextureNames: Set<String>
    }

    @Test func resolvesANYmalBaseColorTexturesFromUSDZ() throws {
        let packagedUSDZURL = anymalUSDZURL()
        #expect(FileManager.default.fileExists(atPath: packagedUSDZURL.path))

        let extractedAssetURL = try Renderer.preparedLoadingAssetURL(from: packagedUSDZURL)
        #expect(extractedAssetURL.pathExtension.lowercased() != "usdz")
        #expect(FileManager.default.fileExists(atPath: extractedAssetURL.path))

        let packagedSummary = try makeTextureResolutionSummary(for: packagedUSDZURL)
        let extractedSummary = try makeTextureResolutionSummary(for: extractedAssetURL)

        #expect(packagedSummary.submeshCount > 0)
        #expect(extractedSummary.submeshCount > 0)
        #expect(texturedResolutionCount(in: extractedSummary.resolvedCounts) > 0)
        #expect(texturedResolutionCount(in: extractedSummary.resolvedCounts) > texturedResolutionCount(in: packagedSummary.resolvedCounts))
        #expect(extractedSummary.resolvedTextureNames.contains("base.jpg"))
        #expect(extractedSummary.resolvedTextureNames.contains("drive.jpg"))
        #expect(extractedSummary.resolvedTextureNames.contains("hip.jpg"))
    }

    @Test func resolvesSpotBaseColorTexturesFromUSDZ() throws {
        let packagedUSDZURL = spotUSDZURL()
        #expect(FileManager.default.fileExists(atPath: packagedUSDZURL.path))

        let extractedAssetURL = try Renderer.preparedLoadingAssetURL(from: packagedUSDZURL)
        #expect(extractedAssetURL.pathExtension.lowercased() == "usd")
        #expect(FileManager.default.fileExists(atPath: extractedAssetURL.path))

        let extractedSummary = try makeTextureResolutionSummary(for: extractedAssetURL)

        #expect(extractedSummary.submeshCount > 0)
        #expect(texturedResolutionCount(in: extractedSummary.resolvedCounts) > 0)
        #expect(extractedSummary.resolvedTextureNames.contains("yellow_parts.png"))
        #expect(extractedSummary.resolvedTextureNames.contains("body_others.png"))
        #expect(extractedSummary.resolvedTextureNames.contains("yellow_leg.png"))
        #expect(extractedSummary.resolvedTextureNames.contains("black_leg.png"))
        #expect(extractedSummary.resolvedTextureNames.contains("lleg.png"))
    }

    @Test func resolvesGo2USDMaterialColorsFromUSDZ() throws {
        let assetURL = try Renderer.preparedLoadingAssetURL(from: go2USDZURL())
        let summary = try makeTextureResolutionSummary(for: assetURL,
                                                       solidColorByNodePath: IsaacSwiftRobotKind.go2.modelDefinition.solidColorByNodePath)

        #expect(summary.submeshCount > 0)
        #expect(summary.resolvedCounts[.fallback, default: 0] == 0)
        #expect(summary.resolvedCounts[.solidColor, default: 0] > 0)
    }

    @Test func resolvesGo2ArticulationFromUSDZ() throws {
        let assetURL = try Renderer.preparedLoadingAssetURL(from: go2USDZURL())
        let profile = Renderer.articulationProfile(for: .go2)
        let sceneNodes = Renderer.sceneNodes(from: assetURL, profile: profile)
        #expect(!sceneNodes.isEmpty)

        let zeroActions = Array(repeating: Float(0), count: 12)
        var activeActions = zeroActions
        activeActions[0] = 0.6

        let zeroTransforms = Renderer.worldTransforms(for: sceneNodes,
                                                      actions: zeroActions,
                                                      profile: profile)
        let activeTransforms = Renderer.worldTransforms(for: sceneNodes,
                                                        actions: activeActions,
                                                        profile: profile)
        #expect(zeroTransforms.count == sceneNodes.count)
        #expect(activeTransforms.count == sceneNodes.count)

        guard let flHipIndex = sceneNodes.firstIndex(where: { $0.path == "/go2_description/FL_hip" }),
              let flThighIndex = sceneNodes.firstIndex(where: { $0.path == "/go2_description/FL_thigh" }),
              let flCalfIndex = sceneNodes.firstIndex(where: { $0.path == "/go2_description/FL_calf" }),
              let frThighIndex = sceneNodes.firstIndex(where: { $0.path == "/go2_description/FR_thigh" }) else {
            Issue.record("Go2 joint nodes were not found in the asset hierarchy.")
            return
        }

        #expect(sceneNodes[flHipIndex].parentIndex.flatMap { sceneNodes[$0].path } == "/go2_description/base")
        #expect(sceneNodes[flThighIndex].parentIndex.flatMap { sceneNodes[$0].path } == "/go2_description/FL_hip")
        #expect(sceneNodes[flCalfIndex].parentIndex.flatMap { sceneNodes[$0].path } == "/go2_description/FL_thigh")

        #expect(!matricesApproximatelyEqual(zeroTransforms[flHipIndex], activeTransforms[flHipIndex]))
        #expect(!matricesApproximatelyEqual(zeroTransforms[flThighIndex], activeTransforms[flThighIndex]))
        #expect(!matricesApproximatelyEqual(zeroTransforms[flCalfIndex], activeTransforms[flCalfIndex]))
        #expect(matricesApproximatelyEqual(zeroTransforms[frThighIndex], activeTransforms[frThighIndex]))
    }

    @Test func loadsPolicyModelAndRunsInference() throws {
        let spotConfiguration = PolicyModelConfiguration.spot
        let anymalConfiguration = PolicyModelConfiguration.anymal
        let bundledSpotURL = PolicyModelRunner.bundledModelURL(configuration: spotConfiguration)
        let bundledAnymalURL = PolicyModelRunner.bundledModelURL(configuration: anymalConfiguration)
        let repositorySpotURL = PolicyModelRunner.repositoryModelURL(configuration: spotConfiguration)
        let repositoryAnymalURL = PolicyModelRunner.repositoryModelURL(configuration: anymalConfiguration)

        #expect(bundledSpotURL != nil || repositorySpotURL.map { FileManager.default.fileExists(atPath: $0.path) } == true)
        #expect(bundledAnymalURL != nil || repositoryAnymalURL.map { FileManager.default.fileExists(atPath: $0.path) } == true)

        let spotRunner = try PolicyModelRunner(configuration: spotConfiguration)
        let anymalRunner = try PolicyModelRunner(configuration: anymalConfiguration)
        #expect(spotRunner.observationCount == 48)
        #expect(anymalRunner.observationCount == 48)

        let spotActions = try spotRunner.predictActions(observations: spotRunner.zeroObservations())
        let anymalActions = try anymalRunner.predictActions(observations: anymalRunner.zeroObservations())
        #expect(!spotActions.isEmpty)
        #expect(!anymalActions.isEmpty)
        #expect(spotActions.reduce(true) { $0 && $1.isFinite })
        #expect(anymalActions.reduce(true) { $0 && $1.isFinite })

        let actionProvider = DemoPolicyActionProvider(runner: spotRunner)
        let demoActionsA = actionProvider.currentActions(at: 0)
        let demoActionsB = actionProvider.currentActions(at: 0.5)
        #expect(demoActionsA.count == spotActions.count)
        #expect(demoActionsB.count == spotActions.count)
        #expect(demoActionsA.reduce(true) { $0 && $1.isFinite })
        #expect(demoActionsB.reduce(true) { $0 && $1.isFinite })
    }

    @Test func bufferedPolicyActionProviderReturnsLatestActions() {
        let provider = BufferedPolicyActionProvider(actions: [0, 1, 2])
        #expect(provider.currentActions(at: 0) == [0, 1, 2])

        provider.updateActions([3, 4, 5, 6])
        #expect(provider.currentActions(at: 1.25) == [3, 4, 5, 6])

        let snapshot = provider.jointFeedbackSnapshot()
        #expect(snapshot?.currentRawActions == [3, 4, 5, 6])
        #expect(snapshot?.previousRawActions == [3, 4, 5, 6])
    }

    @Test func demoPolicyUpdateSchedulerMatchesIsaacCadence() {
        #expect(abs(IsaacPolicyRuntimeConfiguration.anymalC.physicsTimeStep - 0.005) <= 1e-8)
        #expect(abs(IsaacPolicyRuntimeConfiguration.spotFlat.physicsTimeStep - 0.002) <= 1e-8)
        #expect(abs(IsaacPolicyRuntimeConfiguration.go2.physicsTimeStep - 0.005) <= 1e-8)
        #expect(IsaacPolicyRuntimeConfiguration.anymalC.policyDecimation == 4)
        #expect(IsaacPolicyRuntimeConfiguration.spotFlat.policyDecimation == 10)
        #expect(IsaacPolicyRuntimeConfiguration.go2.policyDecimation == 4)
        #expect(abs(DemoPolicyActionProvider.isaacPolicyUpdateInterval - 0.02) <= 1e-8)

        var scheduler = PolicyUpdateScheduler(updateInterval: DemoPolicyActionProvider.isaacPolicyUpdateInterval)
        let firstStep = scheduler.shouldUpdate(at: 0.0)
        let secondStep = scheduler.shouldUpdate(at: 0.005)
        let thirdStep = scheduler.shouldUpdate(at: 0.019)
        let fourthStep = scheduler.shouldUpdate(at: 0.02)
        let fifthStep = scheduler.shouldUpdate(at: 0.035)
        let sixthStep = scheduler.shouldUpdate(at: 0.04)

        #expect(firstStep)
        #expect(!secondStep)
        #expect(!thirdStep)
        #expect(fourthStep)
        #expect(!fifthStep)
        #expect(sixthStep)
    }

    @Test func demoPolicyResetClearsFeedbackAndScheduler() throws {
        let configuration = PolicyModelConfiguration.spot
        guard PolicyModelRunner.bundledModelURL(configuration: configuration) != nil ||
              PolicyModelRunner.repositoryModelURL(configuration: configuration).map({ FileManager.default.fileExists(atPath: $0.path) }) == true
        else { return }

        let runner = try PolicyModelRunner(configuration: configuration)
        let provider = DemoPolicyActionProvider(runner: runner, configuration: .spotFlat)
        provider.updateJointState(positionDeltas: Array(repeating: 0.25, count: 12),
                                  velocities: Array(repeating: 1.5, count: 12))

        let firstActions = provider.currentActions(at: 0)
        #expect(firstActions.contains { abs($0) > 1e-6 })
        let dirtySnapshot = provider.jointFeedbackSnapshot()
        #expect(dirtySnapshot?.currentRawActions.contains { abs($0) > 1e-6 } == true)
        #expect(dirtySnapshot?.jointPositionDeltas.contains { abs($0) > 1e-6 } == true)

        provider.resetPolicyState()
        let resetSnapshot = provider.jointFeedbackSnapshot()
        #expect(resetSnapshot?.currentRawActions == Array(repeating: Float(0), count: 12))
        #expect(resetSnapshot?.previousRawActions == Array(repeating: Float(0), count: 12))
        #expect(resetSnapshot?.jointPositionDeltas == Array(repeating: Float(0), count: 12))
        #expect(resetSnapshot?.jointVelocities == Array(repeating: Float(0), count: 12))
        #expect(resetSnapshot?.lastFeedbackTime == nil)

        let actionsAfterReset = provider.currentActions(at: 0)
        #expect(actionsAfterReset.contains { abs($0) > 1e-6 },
                "resetPolicyState must reset the scheduler so the next tick recomputes actions immediately")
    }

    @Test func robotRuntimeConfigurationsMatchSimulatorDefaults() {
        let anymalConfig = IsaacPolicyRuntimeConfiguration.anymalC
        let anymalSim = IsaacSwiftAnymalSimulator(robotKind: anymalConfig.robotKind,
                                                  physicsTimeStep: anymalConfig.physicsTimeStep)
        #expect(abs(anymalSim.physicsTimeStep - anymalConfig.physicsTimeStep) <= 1e-8)
        #expect(abs(anymalSim.recommendedPhysicsTimeStep - anymalConfig.physicsTimeStep) <= 1e-8)
        #expect(abs(anymalSim.recommendedActionScale - anymalConfig.actionScale) <= 1e-6)

        let spotConfig = IsaacPolicyRuntimeConfiguration.spotFlat
        let spotSim = IsaacSwiftAnymalSimulator(robotKind: spotConfig.robotKind,
                                                physicsTimeStep: spotConfig.physicsTimeStep)
        #expect(abs(spotSim.physicsTimeStep - spotConfig.physicsTimeStep) <= 1e-8)
        #expect(abs(spotSim.recommendedPhysicsTimeStep - spotConfig.physicsTimeStep) <= 1e-8)
        #expect(abs(spotSim.recommendedActionScale - spotConfig.actionScale) <= 1e-6)

        let go2Config = IsaacPolicyRuntimeConfiguration.go2
        let go2Sim = IsaacSwiftAnymalSimulator(robotKind: go2Config.robotKind,
                                               physicsTimeStep: go2Config.physicsTimeStep)
        #expect(abs(go2Sim.physicsTimeStep - go2Config.physicsTimeStep) <= 1e-8)
        #expect(abs(go2Sim.recommendedPhysicsTimeStep - go2Config.physicsTimeStep) <= 1e-8)
        #expect(abs(go2Sim.recommendedActionScale - go2Config.actionScale) <= 1e-6)
    }

    @Test func simToPolicyJointPermutationsAreValidBijections() {
        // Every runtime config must define a 12-element permutation that is
        // a bijection over {0..<12}. This guards against typos in the
        // sim-order ↔ policy-order mapping that would otherwise let actions
        // bleed between legs (the original Spot regression).
        for config in RobotModelDefinitions.all.map(\.policyRuntimeConfiguration) {
            let perm = config.simToPolicyJointPermutation
            #expect(perm.count == 12, "perm count for \(config.robotKind) is \(perm.count)")
            #expect(Set(perm) == Set(0..<12),
                    "perm for \(config.robotKind) is not a bijection: \(perm)")
        }
    }

    @Test func anymalSimToPolicyPermutationMatchesPhysXDofOrder() {
        // Sim is leg-major:
        //   [LF_HAA, LF_HFE, LF_KFE, RF_HAA, RF_HFE, RF_KFE,
        //    LH_HAA, LH_HFE, LH_KFE, RH_HAA, RH_HFE, RH_KFE]
        // The upstream Isaac Sim class uses articulation dof_names directly,
        // but the bundled CoreML export in this repository was converted in
        // this type-grouped order:
        //   [LF_HAA, LH_HAA, RF_HAA, RH_HAA,
        //    LF_HFE, LH_HFE, RF_HFE, RH_HFE,
        //    LF_KFE, LH_KFE, RF_KFE, RH_KFE]
        // simToPolicy[sim] = policy_index of the same joint.
        let expected = [0, 4, 8, 2, 6, 10, 1, 5, 9, 3, 7, 11]
        let actual = IsaacPolicyRuntimeConfiguration.anymalC.simToPolicyJointPermutation
        #expect(actual == expected,
                "ANYmal sim→policy permutation drifted: got \(actual)")
    }

    @Test func spotSimToPolicyPermutationMatchesPhysXDofOrder() {
        // Sim is leg-major:
        //   [fl_hx, fl_hy, fl_kn, fr_hx, fr_hy, fr_kn,
        //    hl_hx, hl_hy, hl_kn, hr_hx, hr_hy, hr_kn]
        // PhysX dof_names for Spot is type-grouped (depth: hx → hy → kn),
        // leg order FL, FR, HL, HR at each depth:
        //   [fl_hx, fr_hx, hl_hx, hr_hx,
        //    fl_hy, fr_hy, hl_hy, hr_hy,
        //    fl_kn, fr_kn, hl_kn, hr_kn]
        let expected = [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11]
        let actual = IsaacPolicyRuntimeConfiguration.spotFlat.simToPolicyJointPermutation
        #expect(actual == expected,
                "Spot sim→policy permutation drifted: got \(actual)")
    }

    @Test func go2SimToPolicyPermutationMatchesUSDJointOrder() {
        // Sim is leg-major:
        //   [FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf,
        //    RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf]
        // Isaac Sim Go2 USD metadata lists robotJoints as:
        //   [FL_hip, FR_hip, RL_hip, RR_hip,
        //    FL_thigh, FL_calf, FR_thigh, FR_calf,
        //    RL_thigh, RL_calf, RR_thigh, RR_calf]
        let expected = [0, 4, 5, 1, 6, 7, 2, 8, 9, 3, 10, 11]
        let actual = IsaacPolicyRuntimeConfiguration.go2.simToPolicyJointPermutation
        #expect(actual == expected,
                "Go2 sim->policy permutation drifted: got \(actual)")
    }

    @Test func visualizedPolicyActionsUseIsaacActionScale() {
        let rawActions: [Float] = [-2, -1, -0.5, 0, 0.5, 1, 2, 3, -3, 1, -1, 0.5]
        for definition in RobotModelDefinitions.all {
            let visualized = Renderer.visualizedPolicyActions(from: rawActions,
                                                              robotKind: definition.kind)

            #expect(visualized.count == rawActions.count)

            for (index, rawAction) in rawActions.enumerated() {
                let expected = rawAction * definition.policyRuntimeConfiguration.actionScale
                #expect(abs(visualized[index] - expected) <= 1e-5)
            }
        }
    }

    @Test func groundPlaneVerticesMatchMetalVertexDescriptorLayout() {
        let halfSize: Float = 50
        let floats = Renderer.groundPlaneVertexFloats(halfSize: halfSize)
        let indices = Renderer.groundPlaneIndices()
        let vertexData = floats.withUnsafeBufferPointer { Data(buffer: $0) }

        #expect(floats.count == 4 * 8)
        #expect(vertexData.count == 4 * 32)
        #expect(indices == [0, 1, 2, 0, 2, 3])

        for vertexIndex in 0..<4 {
            let offset = vertexIndex * 8
            #expect(floats[offset + 2] == 0)
            #expect(floats[offset + 3] == 0)
            #expect(floats[offset + 4] == 0)
            #expect(floats[offset + 5] == 1)
        }

        #expect(Array(floats[0..<3]) == [-halfSize, -halfSize, 0])
        #expect(Array(floats[8..<11]) == [halfSize, -halfSize, 0])
        #expect(Array(floats[16..<19]) == [halfSize, halfSize, 0])
        #expect(Array(floats[24..<27]) == [-halfSize, halfSize, 0])
    }

    @Test func appliesPolicyActionsToANYmalJointNodes() throws {
        let assetURL = try Renderer.preparedLoadingAssetURL(from: anymalUSDZURL())
        let profile = Renderer.articulationProfile(for: .anymalC)
        let sceneNodes = Renderer.sceneNodes(from: assetURL, profile: profile)
        #expect(!sceneNodes.isEmpty)

        let zeroActions = Array(repeating: Float(0), count: 12)
        var activeActions = zeroActions
        activeActions[0] = 0.8

        let zeroTransforms = Renderer.worldTransforms(for: sceneNodes,
                                                      actions: zeroActions,
                                                      profile: profile)
        let activeTransforms = Renderer.worldTransforms(for: sceneNodes,
                                                        actions: activeActions,
                                                        profile: profile)
        #expect(zeroTransforms.count == sceneNodes.count)
        #expect(activeTransforms.count == sceneNodes.count)

        let boundJointPaths = Set([
            "/anymal/LF_HIP", "/anymal/LF_THIGH", "/anymal/LF_SHANK",
            "/anymal/RF_HIP", "/anymal/RF_THIGH", "/anymal/RF_SHANK",
            "/anymal/LH_HIP", "/anymal/LH_THIGH", "/anymal/LH_SHANK",
            "/anymal/RH_HIP", "/anymal/RH_THIGH", "/anymal/RH_SHANK",
        ])

        guard let lfHipIndex = sceneNodes.firstIndex(where: { $0.path == "/anymal/LF_HIP" }),
              let lfThighIndex = sceneNodes.firstIndex(where: { $0.path == "/anymal/LF_THIGH" }),
              let lfShankIndex = sceneNodes.firstIndex(where: { $0.path == "/anymal/LF_SHANK" }),
              let rfThighIndex = sceneNodes.firstIndex(where: { $0.path == "/anymal/RF_THIGH" }) else {
            Issue.record("ANYmal joint nodes were not found in the asset hierarchy.")
            return
        }

        #expect(sceneNodes[lfHipIndex].parentIndex.flatMap { sceneNodes[$0].path } == "/anymal/base")
        #expect(sceneNodes[lfThighIndex].parentIndex.flatMap { sceneNodes[$0].path } == "/anymal/LF_HIP")
        #expect(sceneNodes[lfShankIndex].parentIndex.flatMap { sceneNodes[$0].path } == "/anymal/LF_THIGH")

        #expect(!matricesApproximatelyEqual(zeroTransforms[lfHipIndex], activeTransforms[lfHipIndex]))
        #expect(!matricesApproximatelyEqual(zeroTransforms[lfThighIndex], activeTransforms[lfThighIndex]))
        #expect(!matricesApproximatelyEqual(zeroTransforms[lfShankIndex], activeTransforms[lfShankIndex]))
        #expect(matricesApproximatelyEqual(zeroTransforms[rfThighIndex], activeTransforms[rfThighIndex]))

        guard let untouchedNodeIndex = sceneNodes.firstIndex(where: { !boundJointPaths.contains($0.path) }) else {
            Issue.record("Could not find an untouched ANYmal node for comparison.")
            return
        }

        #expect(matricesApproximatelyEqual(zeroTransforms[untouchedNodeIndex],
                                           activeTransforms[untouchedNodeIndex]))
    }

    private func anymalUSDZURL() -> URL {
        if let bundledURL = Bundle.main.url(forResource: "anymal_c", withExtension: "usdz") {
            return bundledURL
        }

        return URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
            .appendingPathComponent("IsaacSwift/RobotAssets/anymal_c/anymal_c.usdz")
    }

    private func spotUSDZURL() -> URL {
        if let bundledURL = Bundle.main.url(forResource: "spot", withExtension: "usdz") {
            return bundledURL
        }

        return URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
            .appendingPathComponent("IsaacSwift/RobotAssets/spot/spot.usdz")
    }

    private func go2USDZURL() -> URL {
        if let bundledURL = Bundle.main.url(forResource: "go2", withExtension: "usdz") {
            return bundledURL
        }

        return URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
            .appendingPathComponent("IsaacSwift/RobotAssets/go2/go2.usdz")
    }

    private func makeDefaultTexture(device: MTLDevice) throws -> MTLTexture {
        let descriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .rgba8Unorm,
                                                                  width: 1,
                                                                  height: 1,
                                                                  mipmapped: false)
        descriptor.usage = [.shaderRead]
        descriptor.storageMode = .shared

        guard let texture = device.makeTexture(descriptor: descriptor) else {
            enum TestError: Error { case textureAllocationFailed }
            throw TestError.textureAllocationFailed
        }

        var pixel: [UInt8] = [255, 255, 255, 255]
        texture.replace(region: MTLRegionMake2D(0, 0, 1, 1),
                        mipmapLevel: 0,
                        withBytes: &pixel,
                        bytesPerRow: 4)
        return texture
    }

    private func makeTextureResolutionSummary(for assetURL: URL,
                                              solidColorByNodePath: [String: SIMD4<UInt8>] = [:]) throws -> TextureResolutionSummary {
        let asset = MDLAsset(url: assetURL)
        asset.loadTextures()
        #expect(asset.count > 0)

        guard let device = MTLCreateSystemDefaultDevice() else {
            Issue.record("Metal device is not available for texture resolution test.")
            return TextureResolutionSummary(submeshCount: 0,
                                            resolvedCounts: [:],
                                            resolvedTextureNames: [])
        }

        let defaultTexture = try makeDefaultTexture(device: device)
        let context = MaterialTextureContext(device: device,
                                             textureLoader: MTKTextureLoader(device: device),
                                             searchDirectories: [assetURL.deletingLastPathComponent()],
                                             defaultTexture: defaultTexture,
                                             textureRelativePathsByMaterialName: textureRelativePathsByMaterialName(for: assetURL),
                                             solidColorByNodePath: solidColorByNodePath)

        var submeshCount = 0
        var resolvedCounts: [BaseColorTextureSource: Int] = [:]
        var resolvedTextureNames: Set<String> = []

        func walk(_ object: MDLObject,
                  parentPath: String) {
            let nodeName = object.name.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty
                ? String(describing: type(of: object))
                : object.name
            let nodePath = parentPath.isEmpty ? "/\(nodeName)" : "\(parentPath)/\(nodeName)"

            if let mesh = object as? MDLMesh,
               let submeshes = mesh.submeshes as? [MDLSubmesh] {
                for submesh in submeshes {
                    submeshCount += 1
                    let resolution = Renderer.resolveBaseColorTexture(from: submesh.material,
                                                                      nodePath: nodePath,
                                                                      context: context)
                    resolvedCounts[resolution.source, default: 0] += 1
                    switch resolution.source {
                    case .url(let url), .string(let url):
                        resolvedTextureNames.insert(url.lastPathComponent.lowercased())
                    case .textureSampler, .solidColor, .fallback:
                        break
                    }
                }
            }

            for child in object.children.objects {
                walk(child, parentPath: nodePath)
            }

            if let instance = object.instance {
                walk(instance, parentPath: nodePath)
            }
        }

        for index in 0..<asset.count {
            walk(asset.object(at: index), parentPath: "")
        }

        return TextureResolutionSummary(submeshCount: submeshCount,
                                        resolvedCounts: resolvedCounts,
                                        resolvedTextureNames: resolvedTextureNames)
    }

    private func texturedResolutionCount(in counts: [BaseColorTextureSource: Int]) -> Int {
        counts.reduce(0) { partial, entry in
            switch entry.key {
            case .textureSampler, .url, .string:
                return partial + entry.value
            case .solidColor, .fallback:
                return partial
            }
        }
    }

    private func textureRelativePathsByMaterialName(for assetURL: URL) -> [String: [String]] {
        guard assetURL.pathExtension.lowercased() == "usd",
              let assetSource = try? String(contentsOf: assetURL, encoding: .utf8) else {
            return [:]
        }

        let pattern = #"def Material "([^"]+)"[\s\S]*?asset inputs:file = @([^@]+)@"#
        guard let regex = try? NSRegularExpression(pattern: pattern, options: []) else {
            return [:]
        }

        let nsSource = assetSource as NSString
        let fullRange = NSRange(location: 0, length: nsSource.length)
        var manifest: [String: [String]] = [:]

        regex.enumerateMatches(in: assetSource, options: [], range: fullRange) { match, _, _ in
            guard let match,
                  match.numberOfRanges == 3 else {
                return
            }

            let materialName = nsSource.substring(with: match.range(at: 1))
            let texturePath = nsSource.substring(with: match.range(at: 2))
            guard !materialName.isEmpty, !texturePath.isEmpty else {
                return
            }

            manifest[materialName, default: []].append(texturePath)
        }

        return manifest.mapValues { paths in
            var seen = Set<String>()
            return paths.filter { seen.insert($0).inserted }
        }
    }

    private func matricesApproximatelyEqual(_ lhs: matrix_float4x4,
                                            _ rhs: matrix_float4x4,
                                            tolerance: Float = 1e-5) -> Bool {
        let lhsColumns = [lhs.columns.0, lhs.columns.1, lhs.columns.2, lhs.columns.3]
        let rhsColumns = [rhs.columns.0, rhs.columns.1, rhs.columns.2, rhs.columns.3]

        for (lhsColumn, rhsColumn) in zip(lhsColumns, rhsColumns) {
            let delta = simd_abs(lhsColumn - rhsColumn)
            if delta.x > tolerance || delta.y > tolerance || delta.z > tolerance || delta.w > tolerance {
                return false
            }
        }

        return true
    }

}
