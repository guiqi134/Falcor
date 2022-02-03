from falcor import *

def render_graph_Offline():
    g = RenderGraph('Offline')
    loadRenderPassLibrary('AccumulatePass.dll')
    loadRenderPassLibrary('GBuffer.dll')
    loadRenderPassLibrary('AreaLightReSTIR.dll')
    loadRenderPassLibrary('SimplePostFX.dll')
    loadRenderPassLibrary('ToneMapper.dll')
    AccumulatePass = createPass('AccumulatePass', {'enabled': False, 'autoReset': True, 'precisionMode': AccumulatePrecision.Single, 'subFrameCount': 0, 'maxAccumulatedFrames': 0})
    g.addPass(AccumulatePass, 'AccumulatePass')
    AreaLightReSTIR = createPass('AreaLightReSTIR', {'enableTemporalResampling': False, 'enableSpatialResampling': False, 'storeFinalVisibility': False})
    g.addPass(AreaLightReSTIR, 'AreaLightReSTIR')
    VBufferRT = createPass('VBufferRT', {'samplePattern': SamplePattern.Center, 'sampleCount': 16, 'useAlphaTest': True, 'adjustShadingNormals': True, 'forceCullMode': False, 'cull': CullMode.CullBack, 'useTraceRayInline': False})
    g.addPass(VBufferRT, 'VBufferRT')
    ToneMapper = createPass('ToneMapper', {'useSceneMetadata': True, 'exposureCompensation': 0.0, 'autoExposure': False, 'filmSpeed': 100.0, 'whiteBalance': False, 'whitePoint': 6500.0, 'operator': ToneMapOp.Aces, 'clamp': True, 'whiteMaxLuminance': 1.0, 'whiteScale': 11.199999809265137, 'fNumber': 1.0, 'shutter': 1.0, 'exposureMode': ExposureMode.AperturePriority})
    g.addPass(ToneMapper, 'ToneMapper')
    SimplePostFX = createPass('SimplePostFX', {'enabled': False, 'wipe': 0.0, 'bloomAmount': 0.0, 'starAmount': 0.0, 'starAngle': 0.10000000149011612, 'vignetteAmount': 0.0, 'chromaticAberrationAmount': 0.0, 'barrelDistortAmount': 0.0, 'saturationCurve': float3(1.000000,1.000000,1.000000), 'colorOffset': float3(0.500000,0.500000,0.500000), 'colorScale': float3(0.500000,0.500000,0.500000), 'colorPower': float3(0.500000,0.500000,0.500000), 'colorOffsetScalar': 0.0, 'colorScaleScalar': 0.0, 'colorPowerScalar': 0.0})
    g.addPass(SimplePostFX, 'SimplePostFX')
    g.addEdge('VBufferRT.vbuffer', 'AreaLightReSTIR.vbuffer')
    g.addEdge('VBufferRT.mvec', 'AreaLightReSTIR.motionVecs')
    g.addEdge('AreaLightReSTIR.color', 'AccumulatePass.input')
    g.addEdge('AccumulatePass.output', 'SimplePostFX.src')
    g.addEdge('SimplePostFX.dst', 'ToneMapper.src')
    g.markOutput('ToneMapper.dst')
    return g

Offline = render_graph_Offline()
try: m.addGraph(Offline)
except NameError: None
