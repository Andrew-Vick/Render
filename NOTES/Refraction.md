TIR = Total Internal Reflection
Schlict = When you see yourself in mirror

##
Chance of Reflection
##

- Takes hairy/expensive physics to solve...
- Most ray tracers approximte this using 
    Schlits approimation
- For a accurate physics render engine you need to take the perfomance hit and solve the math.

Pseudo:
double schlick(double cos_angle_incidence, double ratio_indices_refraction)
{
    double t;
    t = (1-:r)/(1+r:r);
    t = t * t;
    return t + (1-t) * pow(1-cai, 5);
}
Given Schlick:
IF TIR or random \in [0, 1) < schlick
    reflect
else
    refact



##
Simple Derivation of algorithm for approx
##

               \    |
                \   |
                 \  |
                  \ |
        ------------|------------
                    |\
                    | \
                    |  \
                    |   \/
                        

par = parallel
per = perpendicular
vec = vector

r' = r_par + r_per

r_per = n / n_1  * (r + cos(Θ) n)
r_par = √{1 - |r_perp|^2} * n

x_vec * y_vec = |x_vec||y_vec| * cos(Θ)


##
Texturing
##

Probably want to store my textures as floats, colors in [0, 1) because you'll 
only need to scale them once.

To texture an object, we need a mapping, <u, v> from the surface of the object to 
the texture.

   Texture
    _______________________  v
    |                     |  ⬇︎                  A "point" in a texture is called a texel
    |                     |
    |                     |
    |                     |
    |                     |
    |                     |
    -----------------------
   u ➡︎

Getting a texture coordinates on the surface of a sphere.

                    Y
                    |
                    |
                    |   *
                    |   |
                    |_____________X
                   / \  |
                  /   \ |
                 / ... \|
            Z   /  ˄˄phi(𝝋)°
Our ray has already hit the sphere so we need to 
input there equations to solve for theta and phi

Convert from <x, y, z> to <theta Θ, phi 𝝋>, and then to <u, v>

Y = cos Θ
Z = sin(Θ) * sin(phi 𝝋)
X = sin(Θ) * cos(phi 𝝋)

Θ = acos(-Y)
𝝋 = atan2(-Z, x) + Θ

u = 𝝋 / 2𝝅
v = 𝜭 / 𝝅

###
To Texture a triangle:
###

1. Assign texture coordinates to each of the vertices
2. Use barycentric coordinates to linearly interpolate texture coordinates for interla points
    t_a = 
    {
        t - Denotes point will be for texture
        a - denotes that this is where point 'a' from the triangle is mapped to the texture
    }

    uv = t_a + β(t_b - t_a) + 𝛄(t_c - t_a)
    Some vector equations can be used to interpolate normals.
    assign normals to trinagle vertices in a triangle mesh, such that
    the "false normals" are the actual normals at the surface we wish to approximate.
    When rendering, use the barycenteric coordinates to interporlate between the false normals 
    and do reflections using the calculated normal.