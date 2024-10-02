---Texture Filtering---

In ray tracing, correct texture filtering can be achieved simply by sending more rays.
- Expensive
- Ray differentials solve this far more efficently:
    - Allow us to choose an approprialtely sized texture from a set of pre-filtered textures

- Texture filtering in OpenGL
    - OpenGL defines minification and magnification filters

    - Magnification filters are: nearest & linear
        - nearest = Take the texel value nearest to the texture coordinate
        - Linear = Lineary interpolate between the four (aquadrilaterol) nearest texels
    
    - Minification filters are: nearest, linear, mipmap
        - nearest = same as above
            - results in sharp aliased renderings, looks pixilated
        - linear = same as above
            - smoorth output, gets a little blurry
            - Only looks at 4 pixels!
        - mipmap = Pre-filtered stack of intergers, each half the size of the lower level, down to |x|
