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
        "ismParaDepthBias" : 0.01,
        "ismPersDepthBias" : 0.0001,
        "ismMipLevels" : 2,
        "ismPushMode" : 1,
        "smDepthBias" : 0.000003,
        "baseTriangleSize" : 0.006,
        "adaptiveLightNear" : True,
        "temporalReusingLength" : 1,
        "extraPointSamples" : 100000000,
        "triAreaClampThreshold" : float2(0.002558, 0.079261), # triangle removed scene
        "numPSMs" : 24,
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

# Global Scene data
gScenes = {
    "Bistro" : {"scene_path" : "RTXDITutorialsBistro/BistroExterior.pyscene", "output_path" : "ShadowMapsTest/Bistro/", "toneMappingCompensation" : 0.0},
    "ZeroDay" : {"scene_path" : "ZeroDay/ZeroDay.pyscene", "output_path" : "ShadowMapsTest/ZeroDay/", "toneMappingCompensation" : 1.5},
    "EmeraldSquare" : {"scene_path" : "NVIDIA_Internal/emeraldSquare/EmeraldSquare.pyscene", "output_path" : "ShadowMapsTest/EmeraldSquare/", "toneMappingCompensation" : 3.0},
}

gResamplingParamsZeroDay = {
    "envEmissiveScale" : 1.0,
    "triEmissiveScale" : 15.91,
    "useLowerShininess" : True,
    "ismParaDepthBias" : 0.003, # Reset this for only ISM
    "ismPersDepthBias" : 0.0003,
    "ismMipLevels" : 3,
    "smDepthBias" : 0.000001,
    "ismPushMode" : 1,
    "baseTriangleSize" : 0.001,
    "sceneName" : 1,
    "temporalReusingLength" : 1,
}

# Get loaded scene data
currSceneName = "Bistro" # remember change this to target scene
currScene = gScenes[currSceneName]
toneMappingCompensation = currScene["toneMappingCompensation"]
currScenePath = "D:/Falcor_Scenes/" + currScene["scene_path"]
outputPath = "D:/OneDrive - University of Utah/Desktop/Projects/ReSTIR_AreaReservoir/" + currScene["output_path"]

