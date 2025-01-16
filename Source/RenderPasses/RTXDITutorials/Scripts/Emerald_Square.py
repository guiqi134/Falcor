import os

scene = "EmeraldSquare/EmeraldSquare_Dusk.pyscene"
currentTime = 0.0
paused = True
useDenoiser = False
restirPassName = "RTXDITutorial5"

def check_and_create_folder(folder):
    isdir = os.path.isdir(folder)
    if not isdir:
        os.mkdir(folder)

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
        "smDepthBias" : 0.000002,
        "ismPushMode" : 1,
        "baseTriangleSize" : 0.1,
        "sceneName" : 2,
        "adaptiveLightNear" : True,
        "temporalReusingLength" : 1,
        "extraPointSamples" : 100000000,
        "triAreaClampThreshold" : float2(0.002239, 0.011563),
        "numPSMs" : 10,
        "useBloom" : True,
    }
    gToneMappingParams = {
        'operator': ToneMapOp.Aces,
        'autoExposure' : False,
        'exposureCompensation' : 2.4,
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
        # 'Alpha': 0.15000000074505806,
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
renderGraph = graph_ImportanceResampling()
try: m.addGraph(renderGraph)
except NameError: None

m.loadScene(scene, buildFlags=(SceneBuilderFlags.DontMergeMaterials | SceneBuilderFlags.DontMergeMeshes))
# m.loadScene(scene, buildFlags=SceneBuilderFlags.DontMergeMeshes)
if (paused):
    t.pause()
t.time = currentTime

# Initial camera.
camera = [
    float3( 9.500712, 33.203568, 87.346031 ),
    float3( 9.008278, 32.750584, 86.602852 ),
    float3( 0.000456,   1.000000,  0.000688 ) ]

m.scene.camera.animated = False
m.scene.camera.position = camera[0]
m.scene.camera.target = camera[1]
m.scene.camera.up = camera[2]
m.scene.cameraSpeed = 20
m.scene.animated = True

# Add other view point
viewport = [
    float3( 9.500712, 33.203568, 87.346031 ),
    float3( 9.008278, 32.750584, 86.602852 ),
    float3( 0.000456,   1.000000,  0.000688 ) ]
m.scene.addViewpoint(viewport[0], viewport[1], viewport[2])

viewport2 = [
    float3(4.695141, 1.320251, 3.350114),
    float3(5.669813, 1.354105, 3.571178),
    float3(0, 1, 0) ]
m.scene.addViewpoint(viewport2[0], viewport2[1], viewport2[2])

viewport3 = [
    float3(-35.645020, 1.631449, 74.611137),
    float3(-36.392250, 1.593515, 73.947655),
    float3(0, 1, 0) ]
m.scene.addViewpoint(viewport3[0], viewport3[1], viewport3[2])

# For flying teapots
viewport4 = [
    float3(-2.018832, 2.449105, -42.738617),
    float3(-1.921012, 2.434831, -41.743515),
    float3(0, 1, 0) ]
m.scene.addViewpoint(viewport4[0], viewport4[1], viewport4[2])

# for light in m.scene.lights:
#     light.intensity *= 5.0

# Capturing images under same view
capturing_animation = False
if capturing_animation:
    currSceneName = "FlyingTeapots"

    # adjust selected camera if needed
    for camera in m.scene.cameras:
        if camera.name == "Camera.003":
            m.scene.camera = camera
    m.scene.camera.animated = True
    m.scene.animated = True

    # Set frame capture output path
    outputPath = "D:/OneDrive - University of Utah/Desktop/Projects/ReSTIR_ShadowMap/Images"
    m.frameCapture.outputDir = outputPath
    check_and_create_folder(m.frameCapture.outputDir)

    # Shared parameters for ReSTIR pass
    resampling_common_params = {
        "envEmissiveScale" : 1.0, "triEmissiveScale" : 1.0, "useLowerShininess" : True, "ismParaDepthBias" : 0.001, "ismPersDepthBias" : 0.00004,
        "ismMipLevels" : 2, "smDepthBias" : 0.000002, "ismPushMode" : 1, "baseTriangleSize" : 0.1, "sceneName" : 2, "adaptiveLightNear" : True,
        "temporalReusingLength" : 1, "extraPointSamples" : 100000000, "triAreaClampThreshold" : float2(0.002239, 0.011563), "useBloom" : True,
        "sortingRules" : 2
    }

    # Parameters to change in comparison.
    methods_compareBaselinesOurs = {
        "Ours_24SMs_25,5ms" : {
            "resamplingParams" : {**resampling_common_params, "numPSMs" : 24, "visibility" : Visibility.ShadowMap_ISM, "rankingImportance" : 0, "flickerReduction" : 2} # 25.5ms
        },
        "Ours_10SMs_17,4ms" : {
            "resamplingParams" : {**resampling_common_params, "numPSMs" : 10, "visibility" : Visibility.ShadowMap_ISM, "rankingImportance" : 0, "flickerReduction" : 2} # 17.4ms
        },
        "DistanceBaseline_38SMs_25,6ms" : {
            "resamplingParams" : {**resampling_common_params, "numPSMs" : 38, "visibility" : Visibility.ShadowMap_FullyLit, "rankingImportance" : 1, "flickerReduction" : 0} # 25.6ms
        },
        "DistanceBaseline_24SMs_17,7ms" : {
            "resamplingParams" : {**resampling_common_params, "numPSMs" : 24, "visibility" : Visibility.ShadowMap_FullyLit, "rankingImportance" : 1, "flickerReduction" : 0} # 17.7ms
        },
        "ISMs_11,6ms" : {
            "resamplingParams" : {**resampling_common_params, "numPSMs" : 24, "visibility" : Visibility.AllISM, "rankingImportance" : 0, "flickerReduction" : 2} # 11.6ms
        },
        "Reference_222SMs_129ms" : {
            "resamplingParams" : {**resampling_common_params, "numPSMs" : 222, "visibility" : Visibility.ShadowMap_FullyLit, "rankingImportance" : 0, "flickerReduction" : 2}
        },
    }

    # Animation capture setting
    warmUpFrames = 100
    animationFrames = 900 # 30 frames/s
    animationCaptureFrame = 200
    animationCaptureFrame -= 1

    m.clock.framerate = 30

    for method_name, value in methods_compareBaselinesOurs.items():
        # print(method_name)
        # print(value["resamplingParams"])

        # Update resampling pass params
        renderGraph.updatePass("RTXDI Tutorials", value["resamplingParams"])

        # Create base file name
        baseFileName = currSceneName + "_Dynamic_Camera003" + "_" + method_name

        print(baseFileName)

        # warm up
        m.scene.animated = True
        m.clock.stop()
        for i in range(warmUpFrames):
            m.renderFrame()

        # Start capturing frames
        m.clock.frame = 600
        m.clock.play()
        for i in range(animationFrames):
            m.renderFrame()
            if i == animationCaptureFrame:
                # capture image
                m.frameCapture.baseFilename = baseFileName
                m.frameCapture.capture()
        m.clock.stop()

    print("End script capturing")

    exit()
