
# Define important scene parameters
scene = "RTXDITutorialsBistro/BistroExterior.pyscene"
# scene = 'Arcade/Arcade.pyscene'

currentTime = 0.0
paused = False
animateCamera = True

restirPassName = "RTXDITutorial5" # RTXDICombinedTutorial

# Initial camera.  This camera is inconsistent between various Bistro versions,
#    but seems to give decent (if not identical) views for most recent versions.
camera = [
    float3( -16.477287, 3.650037, 1.189330 ),
    float3( -15.491768, 3.507777, 1.097058 ),
    float3( 0.000000,   1.000000,  0.000000 ) ]

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
        "triEmissiveScale" : 15.91,
        "useLowerShininess" : True,
        "ismDepthBias" : 0.01,
        "ismMipLevels" : 3,
        "ismPushMode" : 1,
        "smDepthBias" : 0.000050,
        "baseTriangleSize" : 0.1,
        "adaptiveLightNear" : True,
    }
    gToneMappingParams = {
        'operator': ToneMapOp.Aces,
        'autoExposure' : False,
        'exposureCompensation' : 4.4,
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

ImportanceResampling = graph_ImportanceResampling()
try: m.addGraph(ImportanceResampling)
except NameError: None

m.loadScene(scene)
m.scene.setEnvMap("RTXDITutorialsBistro/san_giuseppe_bridge_4k.hdr")
if (paused):
    t.pause()
t.time = currentTime
m.scene.camera.animated = animateCamera
m.scene.camera.position = camera[0]
m.scene.camera.target = camera[1]
m.scene.camera.up = camera[2]
m.scene.cameraSpeed = 5
m.scene.camera.focalLength = 35.0
m.scene.animated = False

# Add some view points
m.scene.addViewpoint()

viewport1 = [
    float3( -30.359646, 7.155272, -11.375874 ),
    float3( -29.451384, 7.025773, -10.978018 ),
    float3( 0.000000,   1.000000,  0.000000 ) ]
m.scene.addViewpoint(viewport1[0], viewport1[1], viewport1[2])

# viewport2 = [
#     float3( -16.477287, 3.650037, 1.189330 ),
#     float3( -15.491768, 3.507777, 1.097058 ),
#     float3( 0.000000,   1.000000,  0.000000 ) ]
