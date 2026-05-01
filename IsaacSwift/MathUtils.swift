//
//  MathUtils.swift
//  IsaacSwift
//
//  Shared matrix and binary parsing helpers.
//

import Foundation
import ModelIO
import simd

func transformedBounds(of bounds: MDLAxisAlignedBoundingBox,
                               by transform: matrix_float4x4) -> (min: SIMD3<Float>, max: SIMD3<Float>) {
    let minCorner = SIMD3<Float>(bounds.minBounds)
    let maxCorner = SIMD3<Float>(bounds.maxBounds)
    let corners = [
        SIMD3<Float>(minCorner.x, minCorner.y, minCorner.z),
        SIMD3<Float>(minCorner.x, minCorner.y, maxCorner.z),
        SIMD3<Float>(minCorner.x, maxCorner.y, minCorner.z),
        SIMD3<Float>(minCorner.x, maxCorner.y, maxCorner.z),
        SIMD3<Float>(maxCorner.x, minCorner.y, minCorner.z),
        SIMD3<Float>(maxCorner.x, minCorner.y, maxCorner.z),
        SIMD3<Float>(maxCorner.x, maxCorner.y, minCorner.z),
        SIMD3<Float>(maxCorner.x, maxCorner.y, maxCorner.z),
    ]

    var worldMin = SIMD3<Float>(repeating: Float.greatestFiniteMagnitude)
    var worldMax = SIMD3<Float>(repeating: -Float.greatestFiniteMagnitude)

    for corner in corners {
        let transformed = simd_mul(transform, SIMD4<Float>(corner, 1))
        let transformedPoint = SIMD3<Float>(transformed.x, transformed.y, transformed.z)
        worldMin = simd_min(worldMin, transformedPoint)
        worldMax = simd_max(worldMax, transformedPoint)
    }

    return (worldMin, worldMax)
}

func matrix4x4_rotation(radians: Float, axis: SIMD3<Float>) -> matrix_float4x4 {
    let unitAxis = normalize(axis)
    let ct = cosf(radians)
    let st = sinf(radians)
    let ci = 1 - ct
    let x = unitAxis.x
    let y = unitAxis.y
    let z = unitAxis.z
    return matrix_float4x4(columns: (
        vector_float4(ct + x * x * ci, y * x * ci + z * st, z * x * ci - y * st, 0),
        vector_float4(x * y * ci - z * st, ct + y * y * ci, z * y * ci + x * st, 0),
        vector_float4(x * z * ci + y * st, y * z * ci - x * st, ct + z * z * ci, 0),
        vector_float4(0, 0, 0, 1)
    ))
}

func matrix4x4_translation(_ translationX: Float, _ translationY: Float, _ translationZ: Float) -> matrix_float4x4 {
    matrix_float4x4(columns: (
        vector_float4(1, 0, 0, 0),
        vector_float4(0, 1, 0, 0),
        vector_float4(0, 0, 1, 0),
        vector_float4(translationX, translationY, translationZ, 1)
    ))
}

func matrix4x4_quaternion(_ quaternion: simd_quatf) -> matrix_float4x4 {
    let rotation3x3 = matrix_float3x3(quaternion)
    return matrix_float4x4(columns: (
        SIMD4<Float>(rotation3x3.columns.0, 0),
        SIMD4<Float>(rotation3x3.columns.1, 0),
        SIMD4<Float>(rotation3x3.columns.2, 0),
        SIMD4<Float>(0, 0, 0, 1)
    ))
}

func usdQuaternion(real: Float, ix: Float, iy: Float, iz: Float) -> simd_quatf {
    simd_quatf(vector: SIMD4<Float>(ix, iy, iz, real))
}

func matrix_perspective_right_hand(fovyRadians fovy: Float,
                                   aspectRatio: Float,
                                   nearZ: Float,
                                   farZ: Float) -> matrix_float4x4 {
    let ys = 1 / tanf(fovy * 0.5)
    let xs = ys / aspectRatio
    let zs = farZ / (nearZ - farZ)
    return matrix_float4x4(columns: (
        vector_float4(xs, 0, 0, 0),
        vector_float4(0, ys, 0, 0),
        vector_float4(0, 0, zs, -1),
        vector_float4(0, 0, zs * nearZ, 0)
    ))
}

func radians_from_degrees(_ degrees: Float) -> Float {
    (degrees / 180) * .pi
}

extension Data {
    func uint16LE(at offset: Int) -> UInt16? {
        guard offset >= 0, offset + 2 <= count else {
            return nil
        }

        return withUnsafeBytes { rawBuffer in
            let baseAddress = rawBuffer.baseAddress?.advanced(by: offset)
            return baseAddress?.assumingMemoryBound(to: UInt16.self).pointee.littleEndian
        }
    }

    func uint32LE(at offset: Int) -> UInt32? {
        guard offset >= 0, offset + 4 <= count else {
            return nil
        }

        return withUnsafeBytes { rawBuffer in
            let baseAddress = rawBuffer.baseAddress?.advanced(by: offset)
            return baseAddress?.assumingMemoryBound(to: UInt32.self).pointee.littleEndian
        }
    }
}
