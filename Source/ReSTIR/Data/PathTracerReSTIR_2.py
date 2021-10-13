from falcor import *

def render_graph_PathTracerReSTIR():
    g = RenderGraph('PathTracerReSTIR_2')
    loadRenderPassLibrary('GBuffer.dll')
    loadRenderPassLibrary('SpatialReusePass.dll')
    loadRenderPassLibrary('PathTracerReSTIR.dll')
    loadRenderPassLibrary('AccumulatePass.dll')
    loadRenderPassLibrary('ToneMapper.dll')

    GBufferRT = createPass('GBufferRT', {'samplePattern': SamplePattern.Stratified, 'sampleCount': 16, 'useAlphaTest': True, 'adjustShadingNormals': True, 'forceCullMode': False, 'cull': CullMode.CullBack, 'texLOD': TexLODMode.Mip0, 'useTraceRayInline': False})
    g.addPass(GBufferRT, 'GBufferRT')
    SpatialReusePass = createPass('SpatialReusePass')
    g.addPass(SpatialReusePass, 'SpatialReusePass')
    PathTracerReSTIR = createPass('PathTracerReSTIR', {'maxBounces': 0, 'computeDirect': True, 'useReSTIR': False})
    g.addPass(PathTracerReSTIR, 'PathTracerReSTIR')
    AccumulatePass = createPass('AccumulatePass', {'enabled': True, 'autoReset': True, 'precisionMode': AccumulatePrecision.Single, 'subFrameCount': 0, 'maxAccumulatedFrames': 0})
    g.addPass(AccumulatePass, 'AccumulatePass')
    ToneMapper = createPass('ToneMapper', {'useSceneMetadata': True, 'exposureCompensation': 0.0, 'autoExposure': False, 'filmSpeed': 100.0, 'whiteBalance': False, 'whitePoint': 6500.0, 'operator': ToneMapOp.Aces, 'clamp': True, 'whiteMaxLuminance': 1.0, 'whiteScale': 11.199999809265137, 'fNumber': 1.0, 'shutter': 1.0, 'exposureMode': ExposureMode.AperturePriority})
    g.addPass(ToneMapper, 'ToneMapper')

    g.addEdge('GBufferRT.posW', 'SpatialReusePass.posW')
    g.addEdge('GBufferRT.normW', 'SpatialReusePass.normalW')
    g.addEdge('GBufferRT.tangentW', 'SpatialReusePass.tangentW')
    g.addEdge('GBufferRT.faceNormalW', 'SpatialReusePass.faceNormalW')
    g.addEdge('GBufferRT.viewW', 'SpatialReusePass.viewW')
    g.addEdge('GBufferRT.diffuseOpacity', 'SpatialReusePass.mtlDiffOpacity')
    g.addEdge('GBufferRT.specRough', 'SpatialReusePass.mtlSpecRough')
    g.addEdge('GBufferRT.emissive', 'SpatialReusePass.mtlEmissive')
    g.addEdge('GBufferRT.matlExtra', 'SpatialReusePass.mtlParams')
    g.addEdge('GBufferRT.mvec', 'SpatialReusePass.mvec')

    g.addEdge('SpatialReusePass.output', 'PathTracerReSTIR.input')

    g.addEdge('PathTracerReSTIR.color', 'AccumulatePass.input')

    g.addEdge('AccumulatePass.output', 'ToneMapper.src')

    g.markOutput('ToneMapper.dst')
    return g

PathTracerReSTIR = render_graph_PathTracerReSTIR()
try: m.addGraph(PathTracerReSTIR)
except NameError: None
