from falcor import *

def render_graph_PathTracerReSTIR():
    g = RenderGraph("PathTracerReSTIR")
    loadRenderPassLibrary("GBuffer.dll")
    # loadRenderPassLibrary("SpatialReusePass.dll")
    loadRenderPassLibrary("PathTracerReSTIR.dll")
    loadRenderPassLibrary("AccumulatePass.dll")
    loadRenderPassLibrary("ToneMapper.dll")
    AccumulatePass = createPass("AccumulatePass", {'enabled': True, 'precisionMode': AccumulatePrecision.Single})
    g.addPass(AccumulatePass, "AccumulatePass")
    ToneMapper = createPass("ToneMapper", {'autoExposure': False, 'exposureCompensation': 0.0})
    g.addPass(ToneMapper, "ToneMapper")
    PathTracerReSTIR = createPass("PathTracerReSTIR", {'mMaxBounces': 0, 'mComputeDirect': True})
    g.addPass(PathTracerReSTIR, "PathTracerReSTIR")
    GBufferRT = createPass("GBufferRT", {'samplePattern': SamplePattern.Stratified, 'sampleCount': 16})
    g.addPass(GBufferRT, "GBufferRT")
    g.addEdge("AccumulatePass.output", "ToneMapper.src")
    g.addEdge("GBufferRT.posW", "PathTracerReSTIR.posW")
    g.addEdge("GBufferRT.normW", "PathTracerReSTIR.normalW")
    g.addEdge("GBufferRT.tangentW", "PathTracerReSTIR.tangentW")
    g.addEdge("GBufferRT.faceNormalW", "PathTracerReSTIR.faceNormalW")
    g.addEdge("GBufferRT.viewW", "PathTracerReSTIR.viewW")
    g.addEdge("GBufferRT.diffuseOpacity", "PathTracerReSTIR.mtlDiffOpacity")
    g.addEdge("GBufferRT.specRough", "PathTracerReSTIR.mtlSpecRough")
    g.addEdge("GBufferRT.emissive", "PathTracerReSTIR.mtlEmissive")
    g.addEdge("GBufferRT.matlExtra", "PathTracerReSTIR.mtlParams")
    g.addEdge("PathTracerReSTIR.color", "AccumulatePass.input")
    g.markOutput("ToneMapper.dst")
    return g

PathTracerReSTIR = render_graph_PathTracerReSTIR()
try: m.addGraph(PathTracerReSTIR)
except NameError: None
