//
//  Renderer+SceneLoading.swift
//  IsaacSwift
//
//  Robot asset preparation, scene loading, and material texture resolution.
//

import Foundation
import Metal
import MetalKit
import ModelIO
import simd

extension Renderer {
    class func buildRobotScene(device: MTLDevice,
                                       mtlVertexDescriptor: MTLVertexDescriptor,
                                       defaultTexture: MTLTexture,
                                       robotKind: IsaacSwiftRobotKind) throws -> (scene: AssetScene, robotKind: IsaacSwiftRobotKind) {
        let candidates = robotKind.modelDefinition.assetCandidates

        let allocator = MTKMeshBufferAllocator(device: device)
        let mdlVertexDescriptor = MTKModelIOVertexDescriptorFromMetal(mtlVertexDescriptor)

        guard let attributes = mdlVertexDescriptor.attributes as? [MDLVertexAttribute] else {
            throw RendererError.badVertexDescriptor
        }

        attributes[VertexAttribute.position.rawValue].name = MDLVertexAttributePosition
        attributes[VertexAttribute.normal.rawValue].name = MDLVertexAttributeNormal
        attributes[VertexAttribute.texcoord.rawValue].name = MDLVertexAttributeTextureCoordinate
        attributes[VertexAttribute.texcoord.rawValue].initializationValue = vector_float4(0, 0, 0, 0)

        for candidate in candidates {
            guard let assetURL = try preparedAssetURL(for: candidate) else {
                continue
            }

            let candidateDefinition = candidate.robotKind.modelDefinition
            if let scene = try? loadScene(from: assetURL,
                                          device: device,
                                          allocator: allocator,
                                          mdlVertexDescriptor: mdlVertexDescriptor,
                                          defaultTexture: defaultTexture,
                                          modelDefinition: candidateDefinition),
               !scene.meshes.isEmpty {
                let withGround = injectGroundPlane(into: scene,
                                                   device: device,
                                                   allocator: allocator,
                                                   mdlVertexDescriptor: mdlVertexDescriptor)
                return (withGround, candidate.robotKind)
            }
        }

        throw RendererError.assetNotFound
    }

    /// Returns a copy of `scene` with a procedurally generated, checker-textured
    /// ground plane appended at world z=0. The plane spans ±50 m around the
    /// origin so its surface matches Jolt's static ground collider top face.
    /// On failure to build the plane the original scene is returned unchanged.
    private class func injectGroundPlane(into scene: AssetScene,
                                         device: MTLDevice,
                                         allocator: MTKMeshBufferAllocator,
                                         mdlVertexDescriptor: MDLVertexDescriptor) -> AssetScene {
        guard let texture = makeCheckerTexture(device: device) else {
            return scene
        }

        let groundNodeIndex = scene.nodes.count
        guard let groundMesh = try? makeGroundMesh(device: device,
                                                   allocator: allocator,
                                                   mdlVertexDescriptor: mdlVertexDescriptor,
                                                   texture: texture,
                                                   nodeIndex: groundNodeIndex,
                                                   halfSize: 50.0) else {
            return scene
        }

        var nodes = scene.nodes
        nodes.append(SceneNode(path: "/__ground",
                               name: "__ground",
                               localTransform: matrix_identity_float4x4,
                               parentIndex: nil))

        var meshes = scene.meshes
        meshes.append(groundMesh)

        return AssetScene(meshes: meshes,
                          nodes: nodes,
                          boundsCenter: scene.boundsCenter,
                          boundsRadius: scene.boundsRadius)
    }

    private class func preparedAssetURL(for candidate: RobotAssetCandidate) throws -> URL? {
        if let structuredURL = Bundle.main.url(forResource: candidate.resourceName,
                                               withExtension: candidate.resourceExtension,
                                               subdirectory: candidate.subdirectory) {
            return try preparedLoadingAssetURL(from: structuredURL,
                                               preferredRootName: candidate.resourceName)
        }

        guard let bundledURL = Bundle.main.url(forResource: candidate.resourceName,
                                               withExtension: candidate.resourceExtension) else {
            return nil
        }

        if candidate.resourceExtension.caseInsensitiveCompare("usdz") == .orderedSame {
            return try preparedLoadingAssetURL(from: bundledURL,
                                               preferredRootName: candidate.resourceName)
        }

        return try stageFlattenedAssetBundle(for: candidate)
    }

    static func preparedLoadingAssetURL(from packagedAssetURL: URL) throws -> URL {
        try preparedLoadingAssetURL(from: packagedAssetURL,
                                    preferredRootName: packagedAssetURL.deletingPathExtension().lastPathComponent)
    }

    private class func preparedLoadingAssetURL(from packagedAssetURL: URL,
                                               preferredRootName: String) throws -> URL {
        guard packagedAssetURL.pathExtension.caseInsensitiveCompare("usdz") == .orderedSame else {
            return packagedAssetURL
        }

        return try extractStoredUSDZ(at: packagedAssetURL, preferredRootName: preferredRootName)
    }

    private class func stageFlattenedAssetBundle(for candidate: RobotAssetCandidate) throws -> URL {
        let fileManager = FileManager.default
        let stagingRoot = fileManager.temporaryDirectory
            .appendingPathComponent("IsaacSwiftRobotAssets", isDirectory: true)
            .appendingPathComponent(candidate.resourceName, isDirectory: true)

        try? fileManager.removeItem(at: stagingRoot)
        try fileManager.createDirectory(at: stagingRoot, withIntermediateDirectories: true)

        let bundledFiles = bundledFiles(for: candidate)
        for file in bundledFiles {
            guard let sourceURL = Bundle.main.url(forResource: file.sourceName, withExtension: nil) else {
                throw RendererError.assetPreparationFailed("Missing bundled file: \(file.sourceName)")
            }

            let destinationURL = stagingRoot.appendingPathComponent(file.destinationRelativePath, isDirectory: false)
            let destinationDirectory = destinationURL.deletingLastPathComponent()
            try fileManager.createDirectory(at: destinationDirectory, withIntermediateDirectories: true)

            if fileManager.fileExists(atPath: destinationURL.path) {
                try fileManager.removeItem(at: destinationURL)
            }
            try fileManager.copyItem(at: sourceURL, to: destinationURL)
        }

        return stagingRoot.appendingPathComponent("\(candidate.resourceName).\(candidate.resourceExtension)",
                                                  isDirectory: false)
    }

    private class func extractStoredUSDZ(at usdzURL: URL,
                                         preferredRootName: String) throws -> URL {
        let archiveData = try Data(contentsOf: usdzURL, options: [.mappedIfSafe])
        let entries = try storedZIPEntries(in: archiveData)
        let extractionDirectoryName = "\(preferredRootName)-\(UUID().uuidString.lowercased())"

        let extractionRoot = FileManager.default.temporaryDirectory
            .appendingPathComponent("IsaacSwiftRobotAssets", isDirectory: true)
            .appendingPathComponent(extractionDirectoryName, isDirectory: true)

        try FileManager.default.createDirectory(at: extractionRoot, withIntermediateDirectories: true)

        var extractedRootCandidates: [URL] = []

        for entry in entries {
            let destinationURL = extractionRoot.appendingPathComponent(entry.path, isDirectory: entry.isDirectory)

            if entry.isDirectory {
                try FileManager.default.createDirectory(at: destinationURL, withIntermediateDirectories: true)
                continue
            }

            try FileManager.default.createDirectory(at: destinationURL.deletingLastPathComponent(),
                                                    withIntermediateDirectories: true)
            try archiveData.subdata(in: entry.payloadRange).write(to: destinationURL, options: .atomic)

            let pathExtension = destinationURL.pathExtension.lowercased()
            if pathExtension == "usd" || pathExtension == "usdc" {
                extractedRootCandidates.append(destinationURL)
            }
        }

        if let preferredURL = extractedRootCandidates.first(where: {
            $0.deletingPathExtension().lastPathComponent == preferredRootName && $0.pathExtension.lowercased() == "usd"
        }) {
            return preferredURL
        }

        if let preferredURL = extractedRootCandidates.first(where: {
            $0.deletingPathExtension().lastPathComponent == preferredRootName
        }) {
            return preferredURL
        }

        if let firstUSDURL = extractedRootCandidates.first(where: { $0.pathExtension.lowercased() == "usd" }) {
            return firstUSDURL
        }

        if let firstCandidate = extractedRootCandidates.first {
            return firstCandidate
        }

        throw RendererError.assetPreparationFailed("USDZ archive does not contain a root USD/USDC asset: \(usdzURL.lastPathComponent)")
    }

    private class func storedZIPEntries(in archiveData: Data) throws -> [StoredZIPEntry] {
        let endOfCentralDirectorySignature: UInt32 = 0x06054b50
        let centralDirectoryHeaderSignature: UInt32 = 0x02014b50
        let localFileHeaderSignature: UInt32 = 0x04034b50

        let minimumRecordLength = 22
        guard archiveData.count >= minimumRecordLength else {
            throw RendererError.assetPreparationFailed("USDZ archive is too small to be valid.")
        }

        let maxCommentLength = 65_535
        let searchStart = max(0, archiveData.count - minimumRecordLength - maxCommentLength)
        var endOfCentralDirectoryOffset: Int?

        for offset in stride(from: archiveData.count - minimumRecordLength, through: searchStart, by: -1) {
            if archiveData.uint32LE(at: offset) == endOfCentralDirectorySignature {
                endOfCentralDirectoryOffset = offset
                break
            }
        }

        guard let endOfCentralDirectoryOffset else {
            throw RendererError.assetPreparationFailed("USDZ end-of-central-directory record was not found.")
        }

        guard let entryCount = archiveData.uint16LE(at: endOfCentralDirectoryOffset + 10),
              let centralDirectorySize = archiveData.uint32LE(at: endOfCentralDirectoryOffset + 12),
              let centralDirectoryOffset = archiveData.uint32LE(at: endOfCentralDirectoryOffset + 16) else {
            throw RendererError.assetPreparationFailed("USDZ central-directory metadata is incomplete.")
        }

        let centralDirectoryRange = Int(centralDirectoryOffset)..<(Int(centralDirectoryOffset) + Int(centralDirectorySize))
        guard archiveData.indices.contains(centralDirectoryRange.lowerBound),
              centralDirectoryRange.upperBound <= archiveData.count else {
            throw RendererError.assetPreparationFailed("USDZ central-directory range is invalid.")
        }

        var entries: [StoredZIPEntry] = []
        entries.reserveCapacity(Int(entryCount))

        var cursor = centralDirectoryRange.lowerBound
        for _ in 0..<entryCount {
            guard archiveData.uint32LE(at: cursor) == centralDirectoryHeaderSignature else {
                throw RendererError.assetPreparationFailed("USDZ central-directory header signature is invalid.")
            }

            guard let compressionMethod = archiveData.uint16LE(at: cursor + 10),
                  let compressedSize = archiveData.uint32LE(at: cursor + 20),
                  let fileNameLength = archiveData.uint16LE(at: cursor + 28),
                  let extraFieldLength = archiveData.uint16LE(at: cursor + 30),
                  let fileCommentLength = archiveData.uint16LE(at: cursor + 32),
                  let localHeaderOffset = archiveData.uint32LE(at: cursor + 42) else {
                throw RendererError.assetPreparationFailed("USDZ central-directory entry is incomplete.")
            }

            let nameStart = cursor + 46
            let nameEnd = nameStart + Int(fileNameLength)
            guard nameEnd <= archiveData.count,
                  let path = String(data: archiveData.subdata(in: nameStart..<nameEnd), encoding: .utf8) else {
                throw RendererError.assetPreparationFailed("USDZ entry name could not be decoded as UTF-8.")
            }

            guard archiveData.uint32LE(at: Int(localHeaderOffset)) == localFileHeaderSignature else {
                throw RendererError.assetPreparationFailed("USDZ local-file header signature is invalid for \(path).")
            }

            guard archiveData.uint16LE(at: Int(localHeaderOffset) + 8) == compressionMethod,
                  let localFileNameLength = archiveData.uint16LE(at: Int(localHeaderOffset) + 26),
                  let localExtraFieldLength = archiveData.uint16LE(at: Int(localHeaderOffset) + 28) else {
                throw RendererError.assetPreparationFailed("USDZ local-file header is incomplete for \(path).")
            }

            guard compressionMethod == 0 else {
                throw RendererError.assetPreparationFailed("USDZ entry \(path) uses unsupported compression method \(compressionMethod).")
            }

            let payloadStart = Int(localHeaderOffset) + 30 + Int(localFileNameLength) + Int(localExtraFieldLength)
            let payloadEnd = payloadStart + Int(compressedSize)
            guard payloadStart <= payloadEnd, payloadEnd <= archiveData.count else {
                throw RendererError.assetPreparationFailed("USDZ entry payload is out of bounds for \(path).")
            }

            entries.append(StoredZIPEntry(path: path,
                                          payloadRange: payloadStart..<payloadEnd,
                                          isDirectory: path.hasSuffix("/")))

            cursor = nameEnd + Int(extraFieldLength) + Int(fileCommentLength)
        }

        return entries
    }

    private class func bundledFiles(for candidate: RobotAssetCandidate) -> [BundledAssetFile] {
        candidate.bundledFiles
    }

    private class func loadScene(from assetURL: URL,
                                 device: MTLDevice,
                                 allocator: MTKMeshBufferAllocator,
                                 mdlVertexDescriptor: MDLVertexDescriptor,
                                 defaultTexture: MTLTexture,
                                 modelDefinition: RobotModelDefinition) throws -> AssetScene {
        let asset = MDLAsset(url: assetURL, vertexDescriptor: nil, bufferAllocator: allocator)
        asset.loadTextures()
        let materialContext = MaterialTextureContext(device: device,
                                                     textureLoader: MTKTextureLoader(device: device),
                                                     searchDirectories: materialSearchDirectories(for: assetURL),
                                                     defaultTexture: defaultTexture,
                                                     textureRelativePathsByMaterialName: materialTextureRelativePathsByMaterialName(for: assetURL),
                                                     solidColorByNodePath: modelDefinition.solidColorByNodePath)

        var meshes: [MeshRenderData] = []
        var nodes: [SceneNode] = []
        var minBounds = SIMD3<Float>(repeating: Float.greatestFiniteMagnitude)
        var maxBounds = SIMD3<Float>(repeating: -Float.greatestFiniteMagnitude)

        func walk(_ object: MDLObject,
                  parentIndex: Int?,
                  parentTransform: matrix_float4x4) throws {
            let localTransform = object.transform?.matrix ?? matrix_identity_float4x4
            let worldTransform = simd_mul(parentTransform, localTransform)
            let nodeName = object.name.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty
                ? String(describing: type(of: object))
                : object.name
            let nodePath = parentIndex.flatMap { nodes.indices.contains($0) ? nodes[$0].path : nil }
                .map { "\($0)/\(nodeName)" } ?? "/\(nodeName)"
            let nodeIndex = nodes.count
            nodes.append(SceneNode(path: nodePath,
                                   name: nodeName,
                                   localTransform: localTransform,
                                   parentIndex: parentIndex))

            if let mesh = object as? MDLMesh {
                let meshBounds = transformedBounds(of: mesh.boundingBox, by: worldTransform)
                minBounds = simd_min(minBounds, meshBounds.min)
                maxBounds = simd_max(maxBounds, meshBounds.max)
                meshes.append(try makeMeshRenderData(from: mesh,
                                                     nodeIndex: nodeIndex,
                                                     nodePath: nodePath,
                                                     mdlVertexDescriptor: mdlVertexDescriptor,
                                                     materialContext: materialContext))
            }

            for child in object.children.objects {
                try walk(child,
                         parentIndex: nodeIndex,
                         parentTransform: worldTransform)
            }

            if let instance = object.instance {
                try walk(instance,
                         parentIndex: nodeIndex,
                         parentTransform: worldTransform)
            }
        }

        for index in 0..<asset.count {
            try walk(asset.object(at: index),
                     parentIndex: nil,
                     parentTransform: matrix_identity_float4x4)
        }

        guard !meshes.isEmpty else {
            throw RendererError.meshNotFound
        }

        let center = (minBounds + maxBounds) * 0.5
        let extents = maxBounds - minBounds
        let radius = max(length(extents) * 0.5, 0.001)

        return AssetScene(meshes: meshes,
                          nodes: articulationResolvedSceneNodes(from: nodes,
                                                                profile: modelDefinition.articulationProfile),
                          boundsCenter: center,
                          boundsRadius: radius)
    }

    private class func makeMeshRenderData(from mdlMesh: MDLMesh,
                                          nodeIndex: Int,
                                          nodePath: String,
                                          mdlVertexDescriptor: MDLVertexDescriptor,
                                          materialContext: MaterialTextureContext) throws -> MeshRenderData {
        if mdlMesh.vertexAttributeData(forAttributeNamed: MDLVertexAttributeNormal) == nil {
            mdlMesh.addNormals(withAttributeNamed: MDLVertexAttributeNormal, creaseThreshold: 0.0)
        }

        mdlMesh.vertexDescriptor = mdlVertexDescriptor

        let mtkMesh = try MTKMesh(mesh: mdlMesh, device: materialContext.device)

        let mdlSubmeshes = mdlMesh.submeshes as? [MDLSubmesh]
        var submeshData: [SubmeshRenderData] = []
        submeshData.reserveCapacity(mtkMesh.submeshes.count)

        for (index, submesh) in mtkMesh.submeshes.enumerated() {
            let texture = mdlSubmeshes.flatMap { list in
                guard index < list.count else { return nil }
                return resolveBaseColorTexture(from: list[index].material,
                                               nodePath: nodePath,
                                               context: materialContext).texture
            } ?? materialContext.defaultTexture

            submeshData.append(SubmeshRenderData(submesh: submesh, baseColorTexture: texture))
        }

        return MeshRenderData(mesh: mtkMesh, nodeIndex: nodeIndex, submeshes: submeshData)
    }

    static func resolveBaseColorTexture(from material: MDLMaterial?,
                                        nodePath: String? = nil,
                                        context: MaterialTextureContext) -> BaseColorTextureResolution {
        if let material {
            let materialResolution = resolveBaseColorTexture(from: material,
                                                             context: context)
            if materialResolution.source != .fallback {
                return materialResolution
            }
        }

        if let nodePath,
           let color = context.solidColorByNodePath[nodePath],
           let texture = makeSolidColorTexture(device: context.device, color: color) {
            return BaseColorTextureResolution(texture: texture, source: .solidColor)
        }

        return BaseColorTextureResolution(texture: context.defaultTexture, source: .fallback)
    }

    private class func resolveBaseColorTexture(from material: MDLMaterial,
                                               context: MaterialTextureContext) -> BaseColorTextureResolution {
        let options: [MTKTextureLoader.Option: Any] = [
            .textureUsage: NSNumber(value: MTLTextureUsage.shaderRead.rawValue),
            .textureStorageMode: NSNumber(value: MTLStorageMode.private.rawValue),
            .generateMipmaps: NSNumber(value: true),
            .SRGB: NSNumber(value: true),
        ]

        for property in candidateTextureProperties(in: material) {
            if property.type == .texture,
               let mdlTexture = property.textureSamplerValue?.texture,
               let texture = try? context.textureLoader.newTexture(texture: mdlTexture, options: options) {
                return BaseColorTextureResolution(texture: texture, source: .textureSampler)
            }

            if property.type == .URL,
               let urlValue = property.urlValue {
                if let texture = try? context.textureLoader.newTexture(URL: urlValue, options: options) {
                    return BaseColorTextureResolution(texture: texture, source: .url(urlValue))
                }

                if let resolvedURL = resolveTextureURL(urlValue, searchDirectories: context.searchDirectories),
                   let texture = try? context.textureLoader.newTexture(URL: resolvedURL, options: options) {
                    return BaseColorTextureResolution(texture: texture, source: .url(resolvedURL))
                }
            }

            if property.type == .string,
               let stringValue = property.stringValue {
                if let directURL = URL(string: stringValue),
                   let texture = try? context.textureLoader.newTexture(URL: directURL, options: options) {
                    return BaseColorTextureResolution(texture: texture, source: .string(directURL))
                }

                if let resolvedURL = resolveTextureURL(stringValue, searchDirectories: context.searchDirectories),
                   let texture = try? context.textureLoader.newTexture(URL: resolvedURL, options: options) {
                    return BaseColorTextureResolution(texture: texture, source: .string(resolvedURL))
                }
            }
        }

        if let namedTextureResolution = resolveTextureFromMaterialName(material,
                                                                       context: context,
                                                                       options: options) {
            return namedTextureResolution
        }

        for property in candidateColorProperties(in: material) {
            if let texture = makeSolidColorTexture(device: context.device, property: property) {
                return BaseColorTextureResolution(texture: texture, source: .solidColor)
            }
        }

        return BaseColorTextureResolution(texture: context.defaultTexture, source: .fallback)
    }

    private class func candidateTextureProperties(in material: MDLMaterial) -> [MDLMaterialProperty] {
        let prioritizedNames = [
            "baseColor",
            "diffuse_texture",
            "inputs:diffuse_texture",
        ]

        var properties: [MDLMaterialProperty] = prioritizedNames.compactMap { material.propertyNamed($0) }
        properties.append(contentsOf: material.properties(with: .baseColor))

        for index in 0..<material.count {
            guard let property = material[index] else {
                continue
            }

            if property.type == .texture ||
                property.type == .string ||
                property.type == .URL ||
                property.name.localizedCaseInsensitiveContains("texture") {
                properties.append(property)
            }
        }

        return deduplicatedMaterialProperties(properties)
    }

    private class func candidateColorProperties(in material: MDLMaterial) -> [MDLMaterialProperty] {
        let prioritizedNames = [
            "diffuse_color_constant",
            "inputs:diffuse_color_constant",
            "baseColor",
        ]

        var properties: [MDLMaterialProperty] = prioritizedNames.compactMap { material.propertyNamed($0) }
        properties.append(contentsOf: material.properties(with: .baseColor))

        for index in 0..<material.count {
            guard let property = material[index] else {
                continue
            }

            if property.type == .color ||
                property.type == .float3 ||
                property.type == .float4 ||
                property.name.localizedCaseInsensitiveContains("color") {
                properties.append(property)
            }
        }

        return deduplicatedMaterialProperties(properties)
    }

    private class func deduplicatedMaterialProperties(_ properties: [MDLMaterialProperty]) -> [MDLMaterialProperty] {
        var seen = Set<String>()
        return properties.filter { property in
            let key = "\(property.name)|\(property.semantic.rawValue)|\(property.type.rawValue)"
            return seen.insert(key).inserted
        }
    }

    private class func resolveTextureFromMaterialName(_ material: MDLMaterial,
                                                      context: MaterialTextureContext,
                                                      options: [MTKTextureLoader.Option: Any]) -> BaseColorTextureResolution? {
        for candidatePath in candidateTextureRelativePaths(for: material,
                                                          context: context) {
            guard let resolvedURL = resolveTextureURL(candidatePath, searchDirectories: context.searchDirectories),
                  let texture = try? context.textureLoader.newTexture(URL: resolvedURL, options: options) else {
                continue
            }

            return BaseColorTextureResolution(texture: texture, source: .string(resolvedURL))
        }

        return nil
    }

    private class func candidateTextureRelativePaths(for material: MDLMaterial,
                                                     context: MaterialTextureContext) -> [String] {
        var candidates = context.textureRelativePathsByMaterialName[material.name] ?? []
        let baseNames = candidateTextureBaseNames(for: material)
        let prefixes = ["", "materials/", "Props/materials/"]
        let extensions = ["jpg", "jpeg", "png"]

        for prefix in prefixes {
            for baseName in baseNames {
                for pathExtension in extensions {
                    candidates.append("\(prefix)\(baseName).\(pathExtension)")
                }
            }
        }

        var seen = Set<String>()
        return candidates.filter { seen.insert($0).inserted }
    }

    private class func candidateTextureBaseNames(for material: MDLMaterial) -> [String] {
        let trimmedName = material.name.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmedName.isEmpty else {
            return []
        }

        var baseNames = [trimmedName]
        if trimmedName.hasPrefix("material_") {
            baseNames.append(String(trimmedName.dropFirst("material_".count)))
        }

        return Array(Set(baseNames.map { $0.lowercased() }))
    }

    private class func materialTextureRelativePathsByMaterialName(for assetURL: URL) -> [String: [String]] {
        guard assetURL.pathExtension.caseInsensitiveCompare("usd") == .orderedSame,
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

    private class func materialSearchDirectories(for assetURL: URL) -> [URL] {
        let fileManager = FileManager.default
        let assetRoot = assetURL.deletingLastPathComponent()
        let propsDirectory = assetRoot.appendingPathComponent("Props", isDirectory: true)
        let materialsDirectory = propsDirectory.appendingPathComponent("materials", isDirectory: true)
        let rootMaterialsDirectory = assetRoot.appendingPathComponent("materials", isDirectory: true)
        var directories = [materialsDirectory, propsDirectory, assetRoot, rootMaterialsDirectory]

        if let childURLs = try? fileManager.contentsOfDirectory(at: assetRoot,
                                                                includingPropertiesForKeys: [.isDirectoryKey],
                                                                options: [.skipsHiddenFiles]) {
            for childURL in childURLs {
                let resourceValues = try? childURL.resourceValues(forKeys: [.isDirectoryKey])
                guard resourceValues?.isDirectory == true else {
                    continue
                }

                directories.append(childURL)
                directories.append(childURL.appendingPathComponent("materials", isDirectory: true))
            }
        }

        var seen = Set<String>()
        return directories.filter { url in
            let standardizedPath = url.standardizedFileURL.path
            guard fileManager.fileExists(atPath: standardizedPath) else {
                return false
            }
            return seen.insert(standardizedPath).inserted
        }
    }

    private class func resolveTextureURL(_ url: URL, searchDirectories: [URL]) -> URL? {
        if url.isFileURL, FileManager.default.fileExists(atPath: url.path) {
            return url
        }
        return resolveTextureURL(url.path, searchDirectories: searchDirectories)
    }

    private class func resolveTextureURL(_ rawValue: String, searchDirectories: [URL]) -> URL? {
        let trimmed = rawValue.trimmingCharacters(in: CharacterSet(charactersIn: "@"))
        guard !trimmed.isEmpty else {
            return nil
        }

        if trimmed.hasPrefix("file://"),
           let url = URL(string: trimmed),
           FileManager.default.fileExists(atPath: url.path) {
            return url
        }

        let candidatePath = NSString(string: trimmed).standardizingPath
        if candidatePath.hasPrefix("/"), FileManager.default.fileExists(atPath: candidatePath) {
            return URL(fileURLWithPath: candidatePath, isDirectory: false)
        }

        let normalizedRelativePath = candidatePath.hasPrefix("./")
            ? String(candidatePath.dropFirst(2))
            : candidatePath
        let basename = URL(fileURLWithPath: normalizedRelativePath).lastPathComponent

        for directory in searchDirectories {
            let relativeURL = directory.appendingPathComponent(normalizedRelativePath, isDirectory: false)
            if FileManager.default.fileExists(atPath: relativeURL.path) {
                return relativeURL
            }

            let basenameURL = directory.appendingPathComponent(basename, isDirectory: false)
            if FileManager.default.fileExists(atPath: basenameURL.path) {
                return basenameURL
            }
        }

        return nil
    }

    private class func makeSolidColorTexture(device: MTLDevice,
                                             property: MDLMaterialProperty) -> MTLTexture? {
        switch property.type {
        case .color:
            guard let cgColor = property.color else {
                return nil
            }
            return makeSolidColorTexture(device: device, color: rgbaBytes(from: cgColor))
        case .float3:
            let value = property.float3Value
            return makeSolidColorTexture(device: device,
                                         color: rgbaBytes(from: SIMD4<Float>(value.x, value.y, value.z, 1)))
        case .float4:
            return makeSolidColorTexture(device: device, color: rgbaBytes(from: property.float4Value))
        default:
            return nil
        }
    }

    class func makeSolidColorTexture(device: MTLDevice,
                                             color: SIMD4<UInt8>) -> MTLTexture? {
        let descriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .rgba8Unorm,
                                                                  width: 1,
                                                                  height: 1,
                                                                  mipmapped: false)
        descriptor.usage = [.shaderRead]
        descriptor.storageMode = .shared

        guard let texture = device.makeTexture(descriptor: descriptor) else {
            return nil
        }

        var pixel = [color.x, color.y, color.z, color.w]
        texture.replace(region: MTLRegionMake2D(0, 0, 1, 1),
                        mipmapLevel: 0,
                        withBytes: &pixel,
                        bytesPerRow: 4)
        return texture
    }

    /// Generates a black/dark-gray checkerboard texture used as the diffuse
    /// for the rendered ground plane. The texture is sampled across the full
    /// `[0, 1]` UV range of the plane so the user sees `cells × cells` tiles
    /// over the plane's physical extent.
    private class func makeCheckerTexture(device: MTLDevice,
                                          size: Int = 512,
                                          cells: Int = 32,
                                          light: SIMD4<UInt8> = SIMD4<UInt8>(220, 220, 220, 255),
                                          dark: SIMD4<UInt8> = SIMD4<UInt8>(110, 110, 110, 255)) -> MTLTexture? {
        let descriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .rgba8Unorm,
                                                                  width: size,
                                                                  height: size,
                                                                  mipmapped: false)
        descriptor.usage = [.shaderRead]
        descriptor.storageMode = .shared

        guard let texture = device.makeTexture(descriptor: descriptor) else {
            return nil
        }

        let cellSize = max(size / cells, 1)
        var pixels = [UInt8](repeating: 0, count: size * size * 4)
        for y in 0..<size {
            let cy = y / cellSize
            for x in 0..<size {
                let cx = x / cellSize
                let isLight = (cx + cy) & 1 == 0
                let color = isLight ? light : dark
                let i = (y * size + x) * 4
                pixels[i + 0] = color.x
                pixels[i + 1] = color.y
                pixels[i + 2] = color.z
                pixels[i + 3] = color.w
            }
        }
        texture.replace(region: MTLRegionMake2D(0, 0, size, size),
                        mipmapLevel: 0,
                        withBytes: &pixels,
                        bytesPerRow: size * 4)
        return texture
    }

    /// Packed as pos.xyz + normal.xyz + uv.xy, matching
    /// `buildMetalVertexDescriptor()` exactly (8 floats / 32 B per vertex).
    static func groundPlaneVertexFloats(halfSize: Float) -> [Float] {
        let h = halfSize
        return [
            -h, -h, 0, 0, 0, 1, 0, 0,
             h, -h, 0, 0, 0, 1, 1, 0,
             h,  h, 0, 0, 0, 1, 1, 1,
            -h,  h, 0, 0, 0, 1, 0, 1,
        ]
    }

    static func groundPlaneIndices() -> [UInt32] {
        [0, 1, 2, 0, 2, 3]
    }

    /// Builds a flat XY-plane MTKMesh at z=0 spanning `±halfSize` on both
    /// axes, with normals pointing along world +Z. The vertex layout matches
    /// `buildMetalVertexDescriptor()` (pos+normal+uv, 32 B stride).
    private class func makeGroundMesh(device: MTLDevice,
                                      allocator: MTKMeshBufferAllocator,
                                      mdlVertexDescriptor: MDLVertexDescriptor,
                                      texture: MTLTexture,
                                      nodeIndex: Int,
                                      halfSize: Float) throws -> MeshRenderData {
        let vertices = groundPlaneVertexFloats(halfSize: halfSize)
        let indices = groundPlaneIndices()
        let vertexCount = vertices.count / 8

        let vertexData = vertices.withUnsafeBufferPointer { Data(buffer: $0) }
        let indexData = indices.withUnsafeBufferPointer { Data(buffer: $0) }
        let vertexBuffer = allocator.newBuffer(with: vertexData, type: .vertex)
        let indexBuffer  = allocator.newBuffer(with: indexData,  type: .index)

        let submesh = MDLSubmesh(indexBuffer: indexBuffer,
                                 indexCount: indices.count,
                                 indexType: .uInt32,
                                 geometryType: .triangles,
                                 material: nil)

        let mdlMesh = MDLMesh(vertexBuffer: vertexBuffer,
                              vertexCount: vertexCount,
                              descriptor: mdlVertexDescriptor,
                              submeshes: [submesh])

        let mtkMesh = try MTKMesh(mesh: mdlMesh, device: device)
        guard let mtkSubmesh = mtkMesh.submeshes.first else {
            throw RendererError.meshNotFound
        }

        return MeshRenderData(mesh: mtkMesh,
                              nodeIndex: nodeIndex,
                              submeshes: [SubmeshRenderData(submesh: mtkSubmesh,
                                                            baseColorTexture: texture)])
    }

    private class func rgbaBytes(from color: SIMD4<Float>) -> SIMD4<UInt8> {
        SIMD4<UInt8>(clampToByte(color.x),
                     clampToByte(color.y),
                     clampToByte(color.z),
                     clampToByte(color.w))
    }

    private class func rgbaBytes(from cgColor: CGColor) -> SIMD4<UInt8> {
        let components = cgColor.components ?? [1, 1, 1, 1]

        switch components.count {
        case 2:
            return rgbaBytes(from: SIMD4<Float>(Float(components[0]),
                                                Float(components[0]),
                                                Float(components[0]),
                                                Float(components[1])))
        case 3:
            return rgbaBytes(from: SIMD4<Float>(Float(components[0]),
                                                Float(components[1]),
                                                Float(components[2]),
                                                1))
        default:
            return rgbaBytes(from: SIMD4<Float>(Float(components[0]),
                                                Float(components[1]),
                                                Float(components[2]),
                                                Float(components[3])))
        }
    }

    private class func clampToByte(_ value: Float) -> UInt8 {
        UInt8(max(0, min(255, Int((value * 255).rounded()))))
    }

    static func residencyAllocationCount(meshes: [MeshRenderData]) -> Int {
        var count = 2 // uniforms + default texture
        for mesh in meshes {
            count += mesh.mesh.vertexBuffers.count
            count += mesh.mesh.submeshes.count
            count += mesh.submeshes.count
        }
        return count
    }

}
