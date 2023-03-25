// import Scene.Scene;
// import Utils.Sampling.TinyUniformSampleGenerator;
// import Utils.Math.MathHelpers;
// import Utils.Debug.PixelDebug;
// import ISMHelpers;

// cbuffer CB
// {
//     uint gTotalNonEmisTriCount;
//     uint gLaunchWidth;
//     float gBaseTriSize;
// }

// Buffer<uint> gInstTriOffsetBuffer;
// RWBuffer<uint> gTriNumPointsBuffer;
// RWBuffer<uint> gCulledEmisTriCount;
// RWBuffer<uint> gExtraPointsCountBuffer;

// [numthreads(16, 16, 1)]
// void computeTotalPointSamples(uint3 threadID: SV_DispatchThreadID)
// {
//     printSetPixel(threadID.xy);

//     uint globalThreadID = threadID.y * gLaunchWidth + threadID.x;

//     // Discard the thread if we go out of range
//     if (globalThreadID >= gTotalNonEmisTriCount) return;

//     // Get instanceID using binary search
//     uint id = findInterval(gInstTriOffsetBuffer, globalThreadID);
//     GeometryInstanceID instanceID = { id };

//     // Get the information of current triangle
//     uint triangleIndex = globalThreadID - gInstTriOffsetBuffer[id];
//     float triArea;
//     gScene.getFaceNormalAndAreaW(instanceID, triangleIndex, triArea);

//     // Cull emissive triangles
//     uint materialID = gScene.getMaterialID(instanceID);
//     if (gScene.materials.isBasicMaterial(materialID))
//     {
//         float16_t3 emissive = gScene.materials.getBasicMaterialData(materialID).emissive;

//         print("emissive = ", emissive);

//         // Some objects have emissive textures (e.g. the train in Subway), so we cannot just ignore them
//         if (!gScene.materials.isEmissiveTextured(materialID) && any(emissive != float16_t3(0)))
//         {
//             InterlockedAdd(gCulledEmisTriCount[0], 1);
//             return;
//         }
//     }

//     // Compute how many points to generate
//     uint bufferSize;
//     gExtraPointsCountBuffer.GetDimensions(bufferSize);
//     uint numPoints = 3 + clamp(uint(triArea / gBaseTriSize), 0, bufferSize);
//     InterlockedAdd(gExtraPointsCountBuffer[numPoints - 3], 1);
//     gTriNumPointsBuffer[globalThreadID] = numPoints;

// }


// // Below is for building the point samples buffer for ISM
// typedef TinyUniformSampleGenerator SampleGenerator;

// cbuffer generateCB
// {
//     uint gTotalLightsCount;
//     uint gTotalIsmCount;
//     uint gTotalHighResIsmCount;
//     uint gFirstSpotLightID;
// }

// Buffer<uint> gPointsBufferOffsets;
// RWStructuredBuffer<IsmPointSample> gPointsBuffer;

// [numthreads(16, 16, 1)]
// void generatePointSamples(uint3 threadID: SV_DispatchThreadID)
// {
//     // printSetPixel(threadID.xy);

//     uint globalThreadID = threadID.y * gLaunchWidth + threadID.x;

//     // Discard the thread if we go out of range
//     if (globalThreadID >= gTotalNonEmisTriCount) return;

//     // Get instanceID using binary search
//     uint id = findInterval(gInstTriOffsetBuffer, globalThreadID);
//     GeometryInstanceID instanceID = { id };

//     // Get number of point samples for current triangle
//     uint numPoints = gTriNumPointsBuffer[globalThreadID];
//     print("numPoints = ", numPoints);
//     if (numPoints == 0) return;

//     // Get the three vertices of current triangle (without world transform)
//     uint triangleIndex = globalThreadID - gInstTriOffsetBuffer[id];
//     float3 triVertices[3];
//     uint3 vtxIndices = gScene.getIndices(instanceID, triangleIndex);
//     for (uint i = 0; i < 3; i++)
//         triVertices[i] = gScene.vertices[vtxIndices[i]].position;

//     // Create RNG. This is used for generate random barycentric coordinates
//     SampleGenerator sgLight = SampleGenerator(uint2(instanceID.index, triangleIndex), 1 * 13);
//     SampleGenerator sgBarycentric = SampleGenerator(uint2(1, triangleIndex), 1 * 13);

//     // Loop over each point
//     uint pointsBufferOffset = gPointsBufferOffsets[globalThreadID];
//     print("pointsBufferOffset = ", pointsBufferOffset);
//     for (uint v = 0; v < numPoints; v++)
//     {
//         IsmPointSample pointSample;
//         pointSample.instanceID = id;

//         // Sample position
//         if (v < 3)
//         {
//             pointSample.pos = triVertices[v];
//         }
//         else
//         {
//             float3 barycentrics = sample_triangle(sampleNext2D(sgBarycentric)); // Note: we need to use different sg for sampling triangle to avoid getting different values
//             pointSample.pos = barycentrics.x * triVertices[0] + barycentrics.y * triVertices[1] + barycentrics.z * triVertices[2];
//         }

//         // Sample light face
//         uint globalFaceID = uint(gTotalIsmCount * sampleNext1D(sgLight));
//         uint2 localLightIDAndFaceIdx = getLocalLightIdAndFaceIdx(globalFaceID, gFirstSpotLightID);
//         pointSample.selectedLightID = localLightIDAndFaceIdx.x;

//         // Sample high-res ISM array index
//         pointSample.selectedHighIsmArrayIdx = uint(gTotalHighResIsmCount * sampleNext1D(sgLight));

//         // Store the point sample
//         gPointsBuffer[pointsBufferOffset + v] = pointSample;
//     }
// }
