//
//  Renderer.swift
//  IsaacSwift
//
//  Created by Tatsuya Ogawa on 2026/04/29.
//

import Foundation
import Metal
import MetalKit
import ModelIO
import QuartzCore
import simd

class Renderer: NSObject, MTKViewDelegate {

    let device: MTLDevice

#if !targetEnvironment(simulator)
    let commandQueue: MTL4CommandQueue
    let commandBuffer: MTL4CommandBuffer
    let commandAllocators: [MTL4CommandAllocator]
    let commandQueueResidencySet: MTLResidencySet
    let vertexArgumentTable: MTL4ArgumentTable
    let fragmentArgumentTable: MTL4ArgumentTable
#endif

    let endFrameEvent: MTLSharedEvent

    var frameIndex = 0
    var uniformBufferIndex = 0
    var projectionMatrix = matrix_identity_float4x4
    private var orbitYaw: Float = 0
    private var orbitPitch: Float = -0.45
    private var cameraZoom: Float = 1.0
    var isTrackingEnabled: Bool = false

    let dynamicUniformBuffer: MTLBuffer
    let pipelineState: MTLRenderPipelineState
    let depthState: MTLDepthStencilState
    let defaultColorTexture: MTLTexture
    private let meshes: [MeshRenderData]
    private let sceneNodes: [SceneNode]
    let uniformsPerFrame: Int
    private var displayedPolicyActions: [Float]
    let policyPhysicsLoop: PolicyPhysicsLoop?
    let robotKind: IsaacSwiftRobotKind
    private let articulationProfile: RobotArticulationProfile

    let modelCenter: SIMD3<Float>
    let modelRadius: Float

    @MainActor
    init?(metalKitView: MTKView,
          policyActionProvider: PolicyActionProvider? = nil,
          policyRuntimeConfiguration: IsaacPolicyRuntimeConfiguration? = nil,
          robotKind: IsaacSwiftRobotKind = .anymalC) {
#if targetEnvironment(simulator)
        return nil
#else
        guard let device = metalKitView.device else {
            return nil
        }

        self.device = device
        let runtimeConfiguration = policyRuntimeConfiguration ?? IsaacPolicyRuntimeConfiguration.configuration(for: robotKind)
        self.policyPhysicsLoop = PolicyPhysicsLoop(robotKind: robotKind,
                                                   configuration: runtimeConfiguration,
                                                   provider: policyActionProvider)
        self.displayedPolicyActions = []
        self.commandQueue = device.makeMTL4CommandQueue()!
        self.commandBuffer = device.makeCommandBuffer()!
        self.commandAllocators = (0..<maxBuffersInFlight).map { _ in device.makeCommandAllocator()! }

        let vertexTableDescriptor = MTL4ArgumentTableDescriptor()
        vertexTableDescriptor.maxBufferBindCount = 8
        self.vertexArgumentTable = try! device.makeArgumentTable(descriptor: vertexTableDescriptor)

        let fragmentTableDescriptor = MTL4ArgumentTableDescriptor()
        fragmentTableDescriptor.maxBufferBindCount = 4
        fragmentTableDescriptor.maxTextureBindCount = 1
        self.fragmentArgumentTable = try! device.makeArgumentTable(descriptor: fragmentTableDescriptor)

        self.endFrameEvent = device.makeSharedEvent()!
        self.frameIndex = maxBuffersInFlight
        self.endFrameEvent.signaledValue = UInt64(frameIndex - 1)

        metalKitView.depthStencilPixelFormat = .depth32Float_stencil8
        metalKitView.colorPixelFormat = .bgra8Unorm_srgb
        metalKitView.sampleCount = 1

        let mtlVertexDescriptor = Renderer.buildMetalVertexDescriptor()

        do {
            pipelineState = try Renderer.buildRenderPipelineWithDevice(device: device,
                                                                       metalKitView: metalKitView,
                                                                       mtlVertexDescriptor: mtlVertexDescriptor)
        } catch {
            print("Unable to compile render pipeline state. Error info: \(error)")
            return nil
        }

        let depthStateDescriptor = MTLDepthStencilDescriptor()
        depthStateDescriptor.depthCompareFunction = .less
        depthStateDescriptor.isDepthWriteEnabled = true

        guard let state = device.makeDepthStencilState(descriptor: depthStateDescriptor) else {
            return nil
        }
        depthState = state

        guard let whiteTexture = Renderer.makeSolidColorTexture(device: device,
                                                                color: SIMD4<UInt8>(255, 255, 255, 255)) else {
            return nil
        }
        defaultColorTexture = whiteTexture

        let scene: AssetScene
        let resolvedKind: IsaacSwiftRobotKind
        do {
            let result = try Renderer.buildRobotScene(device: device,
                                                      mtlVertexDescriptor: mtlVertexDescriptor,
                                                      defaultTexture: whiteTexture,
                                                      robotKind: robotKind)
            scene = result.scene
            resolvedKind = result.robotKind
        } catch {
            print("Unable to load robot asset (kind=\(robotKind.rawValue)). Error info: \(error)")
            return nil
        }
        self.robotKind = resolvedKind
        self.articulationProfile = Renderer.articulationProfile(for: resolvedKind)

        meshes = scene.meshes
        sceneNodes = scene.nodes
        modelCenter = scene.boundsCenter
        modelRadius = max(scene.boundsRadius, 0.001)
        uniformsPerFrame = max(meshes.count, 1)

        let uniformBufferSize = alignedUniformsSize * maxBuffersInFlight * uniformsPerFrame
        guard let buffer = device.makeBuffer(length: uniformBufferSize, options: [.storageModeShared]) else {
            return nil
        }
        dynamicUniformBuffer = buffer
        dynamicUniformBuffer.label = "UniformBuffer"

        let residencySetDesc = MTLResidencySetDescriptor()
        residencySetDesc.initialCapacity = Renderer.residencyAllocationCount(meshes: meshes)
        let residencySet = try! device.makeResidencySet(descriptor: residencySetDesc)
        residencySet.addAllocations([dynamicUniformBuffer, defaultColorTexture])
        for mesh in meshes {
            residencySet.addAllocations(mesh.mesh.vertexBuffers.map { $0.buffer })
            residencySet.addAllocations(mesh.mesh.submeshes.map { $0.indexBuffer.buffer })
            residencySet.addAllocations(mesh.submeshes.map { $0.baseColorTexture })
        }
        residencySet.commit()
        commandQueue.addResidencySet(residencySet)
        commandQueueResidencySet = residencySet

        super.init()
#endif
    }

    class func buildMetalVertexDescriptor() -> MTLVertexDescriptor {
        let mtlVertexDescriptor = MTLVertexDescriptor()

        mtlVertexDescriptor.attributes[VertexAttribute.position.rawValue].format = .float3
        mtlVertexDescriptor.attributes[VertexAttribute.position.rawValue].offset = 0
        mtlVertexDescriptor.attributes[VertexAttribute.position.rawValue].bufferIndex = BufferIndex.meshVertices.rawValue

        mtlVertexDescriptor.attributes[VertexAttribute.normal.rawValue].format = .float3
        mtlVertexDescriptor.attributes[VertexAttribute.normal.rawValue].offset = 12
        mtlVertexDescriptor.attributes[VertexAttribute.normal.rawValue].bufferIndex = BufferIndex.meshVertices.rawValue

        mtlVertexDescriptor.attributes[VertexAttribute.texcoord.rawValue].format = .float2
        mtlVertexDescriptor.attributes[VertexAttribute.texcoord.rawValue].offset = 24
        mtlVertexDescriptor.attributes[VertexAttribute.texcoord.rawValue].bufferIndex = BufferIndex.meshVertices.rawValue

        mtlVertexDescriptor.layouts[BufferIndex.meshVertices.rawValue].stride = 32
        mtlVertexDescriptor.layouts[BufferIndex.meshVertices.rawValue].stepRate = 1
        mtlVertexDescriptor.layouts[BufferIndex.meshVertices.rawValue].stepFunction = .perVertex

        return mtlVertexDescriptor
    }


    private func updateDynamicBufferState() {
        uniformBufferIndex = (uniformBufferIndex + 1) % maxBuffersInFlight
    }

    private func uniformOffset(forMeshIndex meshIndex: Int) -> Int {
        alignedUniformsSize * ((uniformBufferIndex * uniformsPerFrame) + meshIndex)
    }

    private func writeUniforms(viewMatrix: matrix_float4x4,
                               modelMatrix: matrix_float4x4,
                               meshIndex: Int) {
        let offset = uniformOffset(forMeshIndex: meshIndex)
        let pointer = UnsafeMutableRawPointer(dynamicUniformBuffer.contents() + offset)
            .bindMemory(to: Uniforms.self, capacity: 1)
        pointer[0].projectionMatrix = projectionMatrix
        pointer[0].modelViewMatrix = simd_mul(viewMatrix, modelMatrix)
    }

    private func currentViewMatrix() -> matrix_float4x4 {
        let baseDistance = max(modelRadius * 5.0, 3.0)
        let distance = baseDistance * cameraZoom
        let orbit = matrix4x4_rotation(radians: orbitYaw, axis: SIMD3<Float>(0, 1, 0))
        let tilt = matrix4x4_rotation(radians: orbitPitch, axis: SIMD3<Float>(1, 0, 0))
        
        let center: SIMD3<Float>
        if isTrackingEnabled, let observation = policyPhysicsLoop?.simulator.currentObservation() {
            center = observation.basePositionWorld
        } else {
            center = modelCenter
        }
        
        let centerTranslation = matrix4x4_translation(-center.x, -center.y, -center.z)
        let cameraTranslation = matrix4x4_translation(0, 0, -distance)
        return simd_mul(cameraTranslation, simd_mul(tilt, simd_mul(orbit, centerTranslation)))
    }

    func applyPinchGesture(scale: Float) {
        cameraZoom = min(max(cameraZoom * (1.0 / scale), 0.2), 10.0)
    }

    func applyOrbitGesture(delta: CGPoint) {
        let yawSensitivity: Float = 0.005
        let pitchSensitivity: Float = 0.004
        orbitYaw += Float(delta.x) * yawSensitivity
        orbitPitch = min(max(orbitPitch + Float(delta.y) * pitchSensitivity, -1.2), 0.15)
    }

    private func currentPolicyActions(at time: TimeInterval) -> [Float] {
        guard let policyPhysicsLoop else {
            return displayedPolicyActions
        }
        let deltas = policyPhysicsLoop.step(at: time)
        if !deltas.isEmpty {
            displayedPolicyActions = deltas
        }
        return displayedPolicyActions
    }

    private func currentPolicyState(at time: TimeInterval) -> (actions: [Float], observation: IsaacSwiftAnymalObservation?) {
        let actions = currentPolicyActions(at: time)
        let observation = policyPhysicsLoop?.simulator.currentObservation()
        return (actions, observation)
    }

    private func currentModelMatrices(at time: TimeInterval) -> [matrix_float4x4] {
        let state = currentPolicyState(at: time)
        let modelMatrices = Self.worldTransforms(for: sceneNodes,
                                                 actions: state.actions,
                                                 profile: articulationProfile)
        guard let observation = state.observation else {
            return modelMatrices
        }

        return Self.applyPhysicsBasePose(observation,
                                         to: modelMatrices,
                                         nodes: sceneNodes,
                                         profile: articulationProfile)
    }

    private static func applyPhysicsBasePose(_ observation: IsaacSwiftAnymalObservation,
                                             to modelMatrices: [matrix_float4x4],
                                             nodes: [SceneNode],
                                             profile: RobotArticulationProfile) -> [matrix_float4x4] {
        guard let baseIndex = nodes.firstIndex(where: { $0.path == profile.baseNodePath }),
              baseIndex < modelMatrices.count else {
            return modelMatrices
        }

        let physicsBaseTransform = matrix4x4_translation(observation.basePositionWorld.x,
                                                         observation.basePositionWorld.y,
                                                         observation.basePositionWorld.z)
        let physicsBaseRotation = matrix4x4_quaternion(simd_quatf(vector: observation.baseOrientationWorldXYZW))
        let renderBaseTransform = modelMatrices[baseIndex]
        let physicsBasePose = simd_mul(physicsBaseTransform, physicsBaseRotation)
        let rootCorrection = simd_mul(physicsBasePose, simd_inverse(renderBaseTransform))

        return modelMatrices.enumerated().map { index, modelMatrix in
            guard nodes.indices.contains(index),
                  nodes[index].path != "/__ground" else {
                return modelMatrix
            }
            return simd_mul(rootCorrection, modelMatrix)
        }
    }

    func draw(in view: MTKView) {
#if !targetEnvironment(simulator)
        guard let drawable = view.currentDrawable else {
            return
        }

        guard let renderPassDescriptor = view.currentMTL4RenderPassDescriptor else {
            return
        }

        let previousValueToWaitFor = frameIndex - maxBuffersInFlight
        endFrameEvent.wait(untilSignaledValue: UInt64(previousValueToWaitFor), timeoutMS: 10)

        let commandAllocator = commandAllocators[uniformBufferIndex]
        commandAllocator.reset()
        commandBuffer.beginCommandBuffer(allocator: commandAllocator)

        updateDynamicBufferState()
        let viewMatrix = currentViewMatrix()
        let modelMatrices = currentModelMatrices(at: CACurrentMediaTime())

        guard let renderEncoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDescriptor) else {
            fatalError("Failed to create render command encoder")
        }

        renderEncoder.label = "ANYmal Render Encoder"
        renderEncoder.setCullMode(.back)
        renderEncoder.setFrontFacing(.counterClockwise)
        renderEncoder.setRenderPipelineState(pipelineState)
        renderEncoder.setDepthStencilState(depthState)
        renderEncoder.setArgumentTable(vertexArgumentTable, stages: .vertex)
        renderEncoder.setArgumentTable(fragmentArgumentTable, stages: .fragment)

        for (meshIndex, meshData) in meshes.enumerated() {
            let modelMatrix = meshData.nodeIndex < modelMatrices.count
                ? modelMatrices[meshData.nodeIndex]
                : matrix_identity_float4x4
            writeUniforms(viewMatrix: viewMatrix,
                          modelMatrix: modelMatrix,
                          meshIndex: meshIndex)

            let uniformOffset = uniformOffset(forMeshIndex: meshIndex)
            let uniformAddress = dynamicUniformBuffer.gpuAddress + UInt64(uniformOffset)
            vertexArgumentTable.setAddress(uniformAddress, index: BufferIndex.uniforms.rawValue)
            fragmentArgumentTable.setAddress(uniformAddress, index: BufferIndex.uniforms.rawValue)

            for (bufferIndex, vertexBuffer) in meshData.mesh.vertexBuffers.enumerated() {
                vertexArgumentTable.setAddress(vertexBuffer.buffer.gpuAddress + UInt64(vertexBuffer.offset),
                                              index: bufferIndex)
            }

            for submeshData in meshData.submeshes {
                fragmentArgumentTable.setTexture(submeshData.baseColorTexture.gpuResourceID,
                                                 index: TextureIndex.color.rawValue)

                renderEncoder.drawIndexedPrimitives(primitiveType: submeshData.submesh.primitiveType,
                                                    indexCount: submeshData.submesh.indexCount,
                                                    indexType: submeshData.submesh.indexType,
                                                    indexBuffer: submeshData.submesh.indexBuffer.buffer.gpuAddress + UInt64(submeshData.submesh.indexBuffer.offset),
                                                    indexBufferLength: submeshData.submesh.indexBuffer.buffer.length)
            }
        }

        renderEncoder.endEncoding()

        commandBuffer.useResidencySet((view.layer as! CAMetalLayer).residencySet)
        commandBuffer.endCommandBuffer()

        commandQueue.waitForDrawable(drawable)
        commandQueue.commit([commandBuffer])
        commandQueue.signalDrawable(drawable)
        commandQueue.signalEvent(endFrameEvent, value: UInt64(frameIndex))
        frameIndex += 1
        drawable.present()
#endif
    }

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        let aspect = Float(size.width) / Float(size.height)
        projectionMatrix = matrix_perspective_right_hand(fovyRadians: radians_from_degrees(55),
                                                         aspectRatio: aspect,
                                                         nearZ: 0.1,
                                                         farZ: max(modelRadius * 12.0, 100.0))
    }
}
