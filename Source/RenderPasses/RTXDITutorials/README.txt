This directory includes a set of tutorials that gives a (hopefully simple) incremental approach 
to integrating RTXDI.  This is *different* than the built-in Falcor integration (see the direcory
`Source/Falcor/Rendering/RTXDI`, which is a well-encapsulted integration for our path tracer,
with design decisions specific to that use case.)

Note:  Please see the Falcor README to ensure you have RTXDI installed appropriately for use in
Falcor.  These tutorials assume a correct installation.

Why look at these tutorials?
 * Rendering thousands or millions of lights with engines designed for dozens of lights is hard.
 * Every engine works somewhat differently, and will introduce different integration headaches.
 * This means the RTXDI API is *complex* to allow integration in systems designed numerous ways.
 * The RTXDI API is probably not one anyone *wants*, but it provides the pieces everyone needs.

Additionally, many SDKs are designed assuming you know how to use the provided black box.  RTXDI
is no different.  But the underlying ReSTIR algorithm is a fairly new rendering method, and it
can look like black magic (even to those that invented the algorithms involved).  Many people who
want to try RTXDI, thus, don't really know how the algorithm should work and what they can expect.
This is complicated by the garbage-in / garbage-out principle.  If you provide RTXDI with bad 
inputs, you will get unsatisfactory results.  But what are good inputs?

RTXDI is fundamentally a simple SDK.
 * It wraps default, known-good settings for ReSTIR sampling behind an API that broadly ensures 
   you don't misconfigure and get uncontrollable bias or poor performance. (These are the provided 
   shader headers in `rtxdi-sdk/include/rtxdi` which form the library core.)
 * Because you didn't write these shaders, they use certain GPU resources that you need to provide.

The complexity comes from:
 * You can resample in different ways (with no reuse, temporal reuse, spatial reuse, spatiotemporal 
   reuse, and even more complex techniques.)  Some configurations improve performance significantly.
 * Different configurations require different resources you must provde to RTXDI.
 * Your engine may have many different light types.  How do you hook them up efficiently?
 * How does RTXDI handle dynamic lights -- those moving or turning on or off between frames?
 * People want RTXDI to work across DirectX, Vulkan, and even GLSL.  This adds messy language bits, 
   silly macros, and (to some) ugly and confusing code and APIs.

How do you get a handle on this complexity?  How do you start an integration, or just play around?

My recommendation is always to start simple and build up complexity slowly.  While (perhaps) more
time consuming than an all-at-once integration, it allows you to evaluate RTXDI and demonstrate the 
benefits it can provide, along the way before you've completed integration.

Tutorial 1:

We start simple.  A key question for any library:  how do I know I hooked up the required resources 
correctly?  Because RTXDI is a provided with source code, we can connect up the basic GPU resources 
(given a loaded Falcor scene) and do standard Monte Carlo sampling of direct lighting from scene lights.
This is, in essence, a reference mode.  It's not efficient, but if you accumulate multiple frames 
in a static scene, this is what you should expect to get from a RTXDI integration.  If this is *not*
what you expect, the resources from your engine are misconfigured.  Fix those before moving on.

Tutorial 2:







