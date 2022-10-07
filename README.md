# Software based 3D Renderer
This repository houses a CPU based renderer that can draw 3D graphics. The program 
is written in pure C++. **No** graphics APIs(OpenGL, Vulkan, DirectX) are 
involved. Naturally, the rendering process is slow compared to traditional methods.

The definition of "drawing" in this context is a bit loose. Mainly, this software 
*computes* the color values of the pixels. The user could possibly write these 
values to a file in an image format, display them on a window, or do whatever they 
want. By default, the pixel values are displayed on an [SDL](https://www.libsdl.org/) window.

The goal of this project is to *really* learn how 3D graphics are rendered. 
Graphics APIs abstract away so much, which is useful from a programmer's
perspective, but they also prevent the user from understanding how the 
graphics pipeline really works and how those 3D shapes are drawn and colored.

There are many ways to draw 3D shapes on a screen. This particular software attempts to imitate a modern GPU graphics pipeline. In short, 

* The 3D model file is imported
* Triangles are transformed using matrix multiplications(Vertex shader)
* Triangles are rasterized, i.e. fragments are generated
* Fragments are processed: lighting and textures are applied to each fragment(Fragment shader)
* Calculated fragment values are displayed on a window

More to come...


*This project is inspired by [tinyrenderer](https://github.com/ssloy/tinyrenderer/wiki).*