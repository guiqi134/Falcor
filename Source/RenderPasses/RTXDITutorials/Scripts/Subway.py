scene = "Subway/Subway.pyscene"
currentTime = 0.0
paused = True
restirPassName = "RTXDITutorial5"

def graph_ImportanceResampling():
    # What libraries (with passes) do we need to load to build our renderer?
    loadRenderPassLibrary("GBuffer.dll")
    loadRenderPassLibrary("RTXDITutorials.dll")
    loadRenderPassLibrary("ToneMapper.dll")
    loadRenderPassLibrary("AccumulatePass.dll")

    gRenderParams = {
        "useJitter" : False,
        "outputStructuredBuffer" : True,
    }
    gResamplingParams = {
        "envEmissiveScale" : 1.0,
        "triEmissiveScale" : 1.0,
        "useLowerShininess" : True,
        "ismParaDepthBias" : 0.001,
        "ismPersDepthBias" : 0.0001,
        "ismMipLevels" : 3,
        "smDepthBias" : 0.000010,
        "ismPushMode" : 1,
        "baseTriangleSize" : 0.001,
        "sceneName" : 3,
        "adaptiveLightNear" : False,
    }
    gToneMappingParams = {
        'operator': ToneMapOp.Aces,
        'autoExposure' : False,
        'exposureCompensation' : 2.0,
    }
    gAccumParams = {
        'enabled': False,
    }

    # Create a renderer (i.e., graph) containing a number of render passes
    tracer = RenderGraph("Spatiotemporal Importance Resampling")
    tracer.addPass(createPass("VBufferRT", {}), "VBuffer")
    tracer.addPass(createPass(restirPassName, gResamplingParams), "RTXDI Tutorials")
    tracer.addPass(createPass("ToneMapper", gToneMappingParams), "ToneMapping")
    tracer.addPass(createPass("AccumulatePass", gAccumParams), "Accumulation")

    # Connect the G-buffer pass to our resampling
    tracer.addEdge("VBuffer.vbuffer",         "RTXDI Tutorials.vbuffer")
    tracer.addEdge("VBuffer.mvec",            "RTXDI Tutorials.mvec")

    tracer.addEdge("RTXDI Tutorials.color", "Accumulation.input")
    tracer.addEdge("Accumulation.output", "ToneMapping.src")
    tracer.markOutput("ToneMapping.dst")

    return tracer

# Create render graph
ImportanceResampling = graph_ImportanceResampling()
try: m.addGraph(ImportanceResampling)
except NameError: None

m.loadScene(scene, buildFlags=(SceneBuilderFlags.DontMergeMaterials | SceneBuilderFlags.DontMergeMeshes | SceneBuilderFlags.FlattenStaticMeshInstances |
    SceneBuilderFlags.RTDontMergeStatic | SceneBuilderFlags.RTDontMergeDynamic))
# m.loadScene(scene, buildFlags=(SceneBuilderFlags.DontMergeMaterials | SceneBuilderFlags.DontMergeMeshes))
# m.loadScene(scene)
if (paused):
    t.pause()
t.time = currentTime

# Initial camera.
camera = [
    float3( 34.775719, 9.926392, 36.550968 ),
    float3( 33.801498, 9.920357, 36.776485 ),
    float3( 0.000000,   1.000000,  0.000000 ) ]

m.scene.camera.position = camera[0]
m.scene.camera.target = camera[1]
m.scene.camera.up = camera[2]
m.scene.camera.animated = False
m.scene.cameraSpeed = 10
m.scene.animated = True

# Add other view point
viewport = [
    float3( 34.775719, 9.926392, 36.550968 ),
    float3( 33.801498, 9.920357, 36.776485 ),
    float3( 0.000000,   1.000000,  0.000000 ) ]
m.scene.addViewpoint(viewport[0], viewport[1], viewport[2])

viewport = [
    float3(-12.666439, 2.530612, -39.123089),
    float3(-13.606641, 2.555843, -38.783409),
    float3(0, 1, 0) ]
m.scene.addViewpoint(viewport[0], viewport[1], viewport[2])
