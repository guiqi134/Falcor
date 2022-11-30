
# Define important scene parameters
scene = "ZeroDay/ZeroDay.pyscene"
currentTime = 0.0
paused = False
restirPassName = "RTXDITutorial5"

# Initial camera.
camera = [
    float3( 12.667848, -0.026128, -0.044340 ),
    float3( 11.669118, -0.074956, -0.031895 ),
    float3( -0.000925,   1.000000,  0.000012 ) ]

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
        "ismDepthBias" : 0.001,
        "ismMipLevels" : 3,
        "smDepthBias" : 0.000001,
        "ismPushMode" : 1,
        "baseTriangleSize" : 0.1,
        "sceneName" : 1,
    }
    gToneMappingParams = {
        'operator': ToneMapOp.Aces,
        'autoExposure' : False,
        'exposureCompensation' : 1.3,
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

m.loadScene(scene)
if (paused):
    t.pause()
t.time = currentTime

m.scene.camera.animated = False
m.scene.camera.position = camera[0]
m.scene.camera.target = camera[1]
m.scene.camera.up = camera[2]
m.scene.cameraSpeed = 1
m.scene.animated = False

# Add some view points
# viewport1 = [
#     float3( -30.359646, 7.155272, -11.375874 ),
#     float3( -29.451384, 7.025773, -10.978018 ),
#     float3( 0.000000,   1.000000,  0.000000 ) ]
# m.scene.addViewpoint(viewport1[0], viewport1[1], viewport1[2])
