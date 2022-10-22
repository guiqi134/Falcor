scene = "SunTemple_v4/SunTemple/SunTemple.pyscene"
currentTime = 0.0
paused = False
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
    }
    gToneMappingParams = {
        'operator': ToneMapOp.Aces,
        'autoExposure' : False,
        'exposureCompensation' : 1.0,
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

# Initial camera.
camera = [
    float3(1.525114, 9.631269, -8.024083),
    float3(1.416287, 9.453702, -7.046010),
    float3(0, 1, 0) ]

m.scene.camera.animated = False
m.scene.camera.position = camera[0]
m.scene.camera.target = camera[1]
m.scene.camera.up = camera[2]
m.scene.cameraSpeed = 2
m.scene.animated = False

