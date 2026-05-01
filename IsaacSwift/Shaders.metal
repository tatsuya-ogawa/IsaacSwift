//
//  Shaders.metal
//  IsaacSwift
//
//  Created by Tatsuya Ogawa on 2026/04/29.
//

// File for Metal kernel and shader functions

#include <metal_stdlib>
#include <simd/simd.h>

// Including header shared between this Metal shader code and Swift/C code executing Metal API commands
#import "ShaderTypes.h"

using namespace metal;

typedef struct
{
    float3 position [[attribute(VertexAttributePosition)]];
    float3 normal   [[attribute(VertexAttributeNormal)]];
    float2 texCoord [[attribute(VertexAttributeTexcoord)]];
} Vertex;

typedef struct
{
    float4 position [[position]];
    float3 normal;
    float2 texCoord;
} ColorInOut;

vertex ColorInOut vertexShader(Vertex in [[stage_in]],
                               constant Uniforms & uniforms [[ buffer(BufferIndexUniforms) ]])
{
    ColorInOut out;

    float4 position = float4(in.position, 1.0);
    out.position = uniforms.projectionMatrix * uniforms.modelViewMatrix * position;
    out.normal = normalize((uniforms.modelViewMatrix * float4(in.normal, 0.0)).xyz);
    // USD assets use bottom-left UVs, while Metal sampling is top-left oriented.
    out.texCoord = float2(in.texCoord.x, 1.0 - in.texCoord.y);

    return out;
}

fragment float4 fragmentShader(ColorInOut in [[stage_in]],
                               constant Uniforms & uniforms [[ buffer(BufferIndexUniforms) ]],
                               texture2d<half> colorMap     [[ texture(TextureIndexColor) ]])
{
    constexpr sampler colorSampler(mip_filter::linear,
                                   mag_filter::linear,
                                   min_filter::linear);

    half4 colorSample = colorMap.sample(colorSampler, in.texCoord.xy);
    float3 lightDir = normalize(float3(0.35, 0.8, 0.45));
    float diffuse = max(dot(normalize(in.normal), lightDir), 0.0);
    float lighting = 0.25 + diffuse * 0.75;

    return float4(float3(colorSample.rgb) * lighting, colorSample.a);
}
