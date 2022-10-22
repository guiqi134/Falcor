# scene = "ReSTIRShadowMap/FailCase.pyscene"
scene = "Bistro_v5_2/BistroInterior.pyscene"
# scene = "EmeraldSquare/EmeraldSquare_Dusk.pyscene" # make this to thousands of lights
# scene = "SunTemple_v4/SunTemple/SunTemple.pyscene"
# scene = "ReSTIRShadowMap/testScenes.pyscene"
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
        'exposureCompensation' : 1.2,
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
    float3(3.947781, 4.311863, 1.382004),
    float3(4.808643, 4.050728, 0.945284),
    float3(0, 1, 0) ]

m.scene.camera.animated = False
m.scene.camera.position = camera[0]
m.scene.camera.target = camera[1]
m.scene.camera.up = camera[2]
m.scene.cameraSpeed = 2
m.scene.animated = False

