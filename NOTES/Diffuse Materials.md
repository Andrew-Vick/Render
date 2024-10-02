Types: Matte, Flat, Gambertian

When light interacts with a diffues material, it is reflected in a random direction. 

           p0   Surface
       n   |  / 
         \ | /
<) -------->/ P     (imagine P is at the tangental point between a sphere and the surface)
           /
          /

To choose P0, choose a uniformaly distributed, random point in the unit cube.
Check if the points is inside the sphere, if x^2 + y^2 + z^2 <= 1,
point is in sphere.

Given a point in the sphere, to get a point on the surface, normalize it.
Inside of sphere 4/3 * pi * r^3 ≈ 5. 


REFLECTED RAY:
0 = P
D = P + n + unit(P0 - n)
Reflected ray attenuates light by its attenuation (color)
    Color * reflected ray
      ↳(of the material)