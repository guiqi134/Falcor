import Scene.Scene;
import Scene.Shading;
import Utils.Debug.PixelDebug;

[numthreads(256, 1, 1)]
void generatePointsCS(uint3 threadID: SV_DispatchThreadID)
{
    const GeometryInstanceID instanceID = { threadID.x };
    uint triangleIndex = threadID.y;

    // Get the three vertices of current triangle
    float3 triVertices[3];
    gScene.getVertexPositionsW(instanceID, triangleIndex, triVertices);

    // Get the size of this triangle
    float triSize = 0.5f * length(cross(triVertices[0], triVertices[1]));
}

[numthreads(256, 1, 1)]
void renderIsmCS()
{

}
