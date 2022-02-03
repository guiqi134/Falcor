from falcor import *

def render_graph_DepthPass():
    g = RenderGraph('DepthPass')
    loadRenderPassLibrary('DepthPass.dll')
    DepthPass = createPass('DepthPass')
    g.addPass(DepthPass, 'DepthPass')
    g.markOutput('DepthPass.depth')
    return g

DepthPass = render_graph_DepthPass()
try: m.addGraph(DepthPass)
except NameError : None
