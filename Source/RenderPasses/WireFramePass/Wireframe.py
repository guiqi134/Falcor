# scene = "TestScenes/CornellBox.pyscene"
scene = 'Arcade/Arcade.pyscene'

def graph_wireframe():
    # What libraries (with passes) do we need to load to build our renderer?
    loadRenderPassLibrary("WireFramePass.dll")

    # Create a renderer (i.e., graph) containing a number of render passes 
    tracer = RenderGraph("Wireframe")
    tracer.addPass(createPass("Wireframe"), "wireframe")

    # Connect the V-buffer pass to our pass
    tracer.markOutput("wireframe.output")

    return tracer

# Create render graph
wireframe = graph_wireframe()
try: m.addGraph(wireframe)
except NameError: None

m.loadScene(scene)
