scene = "EmeraldSquare/EmeraldSquare_Dusk.pyscene" # make this to thousands of lights
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
        "ismDepthBias" : 0.001,
        "ismMipLevels" : 3,
        "smDepthBias" : 0.000001,
        "ismPushMode" : 1,
        "baseTriangleSize" : 10,
        "sceneName" : 2,
        "adaptiveLightNear" : True,
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
m.scene.animated = False

# Add other view point
viewport1 = [
    float3(-55.607540, 7.932875, 29.874557),
    float3(-54.797466, 7.826821, 29.297897),
    float3(0, 1, 0) ]
m.scene.addViewpoint(viewport1[0], viewport1[1], viewport1[2])
