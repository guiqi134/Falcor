#include "CopyToOutputPass.h"

namespace
{
    const ChannelList kGBufferChannels =
    {
        { "posW",           "gPosW",            "world space position",         true /* optional */, ResourceFormat::RGBA32Float },
        { "normW",          "gNormW",           "world space normal",           true /* optional */, ResourceFormat::RGBA32Float },
        { "diffuseOpacity", "gDiffuseOpacity",  "diffuse color and opacity",    true /* optional */, ResourceFormat::RGBA32Float },
        { "specRough",      "gSpecRough",       "specular color and roughness", true /* optional */, ResourceFormat::RGBA32Float },
        { "matlExtra",      "gMatlExtra",       "additional material data",     true /* optional */, ResourceFormat::RGBA32Float },
    };
    const std::string kVBufferName = "vbuffer";
    const std::string kDepthName = "depth";
    const std::string kOutputChannel = "output";
}


CopyToOutputPass::SharedPtr CopyToOutputPass::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    return SharedPtr(new CopyToOutputPass(dict));
}

CopyToOutputPass::CopyToOutputPass(const Dictionary& dict)
{
    // Create a dropdown list to display in the GUI.  Start with no displayable output buffers. 
    mDisplayableBuffers.push_back({ -1, "< None >" });
}

RenderPassReflection CopyToOutputPass::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;
    addRenderPassInputs(reflector, kGBufferChannels, Resource::BindFlags::ShaderResource);
    reflector.addOutput(kOutputChannel, "Output data").bindFlags(ResourceBindFlags::RenderTarget | ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource).format(ResourceFormat::RGBA32Float);
    return reflector;
}

void CopyToOutputPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // Get a pointer to a Falcor texture resource for our output 
    //Texture::SharedPtr outTex = Texture::create2D(1600, 900, ResourceFormat::RGBA32Float, 1, 1, nullptr, ResourceBindFlags::AllColorViews);
    Texture::SharedPtr outTex = renderData[kOutputChannel]->asTexture();
    if (!outTex) return;

    // Grab our input buffer, as selected by the user from the GUI
    Texture::SharedPtr inTex = renderData[kGBufferChannels[0].name]->asTexture();

    // If we have selected an invalid texture, clear our output to black and return.
    //if (!inTex || mSelectedBuffer == uint32_t(-1))
    //{
    //    pRenderContext->clearRtv(outTex->getRTV().get(), float4(0.0f, 0.0f, 0.0f, 1.0f));
    //    return;
    //}

    // Copy the selected input buffer to our output buffer.
    pRenderContext->blit(inTex->getSRV(), outTex->getRTV());
}

void CopyToOutputPass::renderUI(Gui::Widgets& widget)
{
    widget.dropdown("Displayed", mDisplayableBuffers, mSelectedBuffer);
}
