
# Define important scene parameters
scene = 'Arcade/Arcade.pyscene'
currentTime = 0.0
paused = True
animateCamera = False

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
        "triEmissiveScale" : 0.32,
        "useLowerShininess" : True,
    }
    gToneMappingParams = {
        'operator': ToneMapOp.Aces,
        'autoExposure' : False,
        'exposureCompensation' : 2.5, # default one is 0.0f
    }
    gAccumParams = {
        'enabled': False,
    }

    # Create a renderer (i.e., graph) containing a number of render passes 
    tracer = RenderGraph("Spatiotemporal Importance Resampling")
    tracer.addPass(createPass("VBufferRT", {}), "VBuffer")
    tracer.addPass(createPass("RTXDITutorial1", gResamplingParams), "RTXDI Tutorial 1")
    tracer.addPass(createPass("ToneMapper", gToneMappingParams), "ToneMapping")
    tracer.addPass(createPass("AccumulatePass", gAccumParams), "Accumulation")

    # Connect the G-buffer pass to our resampling
    tracer.addEdge("VBuffer.vbuffer",         "RTXDI Tutorial 1.vbuffer")
    tracer.addEdge("VBuffer.mvec",            "RTXDI Tutorial 1.mvec")

    tracer.addEdge("RTXDI Tutorial 1.color", "Accumulation.input")
    tracer.addEdge("Accumulation.output", "ToneMapping.src")
    tracer.markOutput("ToneMapping.dst")

    #tracer.markOutput("Accumulation.output")
    #tracer.markOutput("Resampling.color")

    return tracer

ImportanceResampling = graph_ImportanceResampling()
try: m.addGraph(ImportanceResampling)
except NameError: None

m.loadScene(scene)
if (paused):
    t.pause()
t.time = currentTime
m.scene.camera.animated = animateCamera
