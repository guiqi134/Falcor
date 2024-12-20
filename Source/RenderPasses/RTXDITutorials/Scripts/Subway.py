scene = "NVIDIA_Internal/Subway/Subway.pyscene"
currentTime = 0.0
paused = True
useDenoiser = False
restirPassName = "RTXDITutorial5"

def graph_ImportanceResampling():
    # What libraries (with passes) do we need to load to build our renderer?
    loadRenderPassLibrary("GBuffer.dll")
    loadRenderPassLibrary("RTXDITutorials.dll")
    loadRenderPassLibrary("ToneMapper.dll")
    loadRenderPassLibrary("AccumulatePass.dll")
    loadRenderPassLibrary("SVGFPass.dll")

    gRenderParams = {
        "useJitter" : False,
        "outputStructuredBuffer" : True,
    }
    gResamplingParams = {
        "envEmissiveScale" : 1.0,
        "triEmissiveScale" : 1.0,
        "useLowerShininess" : True,
        "ismParaDepthBias" : 0.001,
        "ismPersDepthBias" : 0.00004,
        "ismMipLevels" : 2,
        "smDepthBias" : 0.00001,
        "ismPushMode" : 1,
        "baseTriangleSize" : 0.001,
        "sceneName" : 3,
        "adaptiveLightNear" : False,
        "temporalReusingLength" : 1,
        "extraPointSamples" : 100000000,
        "triAreaClampThreshold" : float2(0.001333, 0.024016),
        "numPSMs" : 10,
    }
    gToneMappingParams = {
        'operator': ToneMapOp.Aces,
        'autoExposure' : False,
        'exposureCompensation' : 2.0,
    }
    gAccumParams = {
        'enabled': False,
    }
    gDenoiserParams = {
        'Enabled': True,
        'Iterations': 4,
        'FeedbackTap': 1,
        'VarianceEpsilon': 9.999999747378752e-05,
        'PhiColor': 1.0,
        'PhiNormal': 50.0,
        'Alpha': 0.02000000074505806,
        'MomentsAlpha': 0.50000000298023224
    }

    # Create a renderer (i.e., graph) containing a number of render passes
    tracer = RenderGraph("Spatiotemporal Importance Resampling")
    tracer.addPass(createPass(restirPassName, gResamplingParams), "RTXDI Tutorials")
    tracer.addPass(createPass("ToneMapper", gToneMappingParams), "ToneMapping")
    tracer.addPass(createPass("AccumulatePass", gAccumParams), "Accumulation")

    # Connect the G-buffer pass to our resampling
    if useDenoiser:
        tracer.addPass(createPass("GBufferRaster", {}), "GBufferRaster")
        tracer.addPass(createPass("SVGFPass", gDenoiserParams), "SVGFPass")
        tracer.addEdge("GBufferRaster.vbuffer", "RTXDI Tutorials.vbuffer")
        tracer.addEdge("GBufferRaster.mvec", "RTXDI Tutorials.mvec")
        tracer.addEdge("GBufferRaster.posW", "RTXDI Tutorials.posW")
        tracer.addEdge("GBufferRaster.normW", "RTXDI Tutorials.normW")
        tracer.addEdge("GBufferRaster.albedo", "RTXDI Tutorials.albedo")
        tracer.addEdge("GBufferRaster.linearZ", "RTXDI Tutorials.linearZ")
        tracer.addEdge("GBufferRaster.emissive", "RTXDI Tutorials.emissive")
        tracer.addEdge("GBufferRaster.pnFwidth", "RTXDI Tutorials.pnFwidth")
        tracer.addEdge("RTXDI Tutorials.color", "SVGFPass.Color")
        tracer.addEdge("RTXDI Tutorials.mvec", "SVGFPass.MotionVec")
        tracer.addEdge("RTXDI Tutorials.posW", "SVGFPass.WorldPosition")
        tracer.addEdge("RTXDI Tutorials.normW", "SVGFPass.WorldNormal")
        tracer.addEdge("RTXDI Tutorials.albedo", "SVGFPass.Albedo")
        tracer.addEdge("RTXDI Tutorials.linearZ", "SVGFPass.LinearZ")
        tracer.addEdge("RTXDI Tutorials.emissive", "SVGFPass.Emission")
        tracer.addEdge("RTXDI Tutorials.pnFwidth", "SVGFPass.PositionNormalFwidth")
        tracer.addEdge("SVGFPass.Filtered image", "Accumulation.input")
    else:
        tracer.addPass(createPass("VBufferRT", {}), "VBuffer")
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
    SceneBuilderFlags.RTDontMergeStatic | SceneBuilderFlags.RTDontMergeInstanced))
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
    float3(-92.211182, -1.413139, -37.098988),
    float3(-91.277931, -1.436587, -37.457447),
    float3(0, 1, 0) ]
m.scene.addViewpoint(viewport[0], viewport[1], viewport[2])

viewport = [
    float3(34.544006, 9.681636, 36.267693),
    float3(33.546528, 9.749392, 36.246582),
    float3(0, 1, 0) ]
m.scene.addViewpoint(viewport[0], viewport[1], viewport[2])


for light in m.scene.lights:
    light.intensity *= 5.0
