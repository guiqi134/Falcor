from falcor import *

def render_graph_ShadowMap():
    g = RenderGraph('ShadowMap')
    loadRenderPassLibrary('ShadowMap.dll')
    ShadowMap = createPass('ShadowMap')
    g.addPass(ShadowMap, 'ShadowMap')
    g.markOutput('ShadowMap.color')
    return g

ShadowMap = render_graph_ShadowMap()
try: m.addGraph(ShadowMap)
except NameError : None
