scene = "TestScenes/CornellBox.pyscene"
# scene = 'Arcade/Arcade.pyscene'
# scene = 'RTXDITutorialsBistro/BistroExterior.pyscene'

def graph_ISM():
    # What libraries (with passes) do we need to load to build our renderer?
    loadRenderPassLibrary("GBuffer.dll")
    loadRenderPassLibrary("ISM.dll")
    loadRenderPassLibrary("AccumulatePass.dll")
    loadRenderPassLibrary("ToneMapper.dll")

    gRenderParams = {
        "useJitter" : False,
        "outputStructuredBuffer" : True,
    }
    gISMParams = {

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
    tracer = RenderGraph("Imperfect Shadow Map")
    tracer.addPass(createPass("VBufferRT", {}), "VBuffer")
    tracer.addPass(createPass("ISM", {}), "ISM")
    tracer.addPass(createPass("AccumulatePass", gAccumParams), "Accumulation")
    tracer.addPass(createPass("ToneMapper", gToneMappingParams), "ToneMapping")

    # Connect the V-buffer pass to our pass
    tracer.addEdge("VBuffer.vbuffer",         "ISM.vbuffer")
    tracer.addEdge("VBuffer.mvec",            "ISM.mvec")

    tracer.addEdge("ISM.color", "Accumulation.input")
    tracer.addEdge("Accumulation.output", "ToneMapping.src")
    tracer.markOutput("ToneMapping.dst")

    return tracer

# Create render graph
ISM = graph_ISM()
try: m.addGraph(ISM)
except NameError: None

m.loadScene(scene)
m.scene.animated = False
m.scene.camera.animated = False


# For Bistro Exterior
# camera = [
#     float3( -16.477287, 3.650037, 1.189330 ),
#     float3( -15.491768, 3.507777, 1.097058 ),
#     float3( 0.000000,   1.000000,  0.000000 ) ]

# m.scene.camera.position = camera[0]
# m.scene.camera.target = camera[1]
# m.scene.camera.up = camera[2]
# m.scene.cameraSpeed = 5