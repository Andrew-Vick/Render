A plane is fully defined by a normal and a point on the plane.

* A point, p0, on the plane satisfies the equation:

    D - Direction
    n - normal

    (p - p0) * n = 0
    substitute a ray for p0:
    ((o + tD) - p0) * n = 0
    (o - p0 + tD) * n = 0
    (o - p0) * n + (tD * n) = 0
    tD * n = (p0 - o) * n
    t = ((p0 - o) * n) / (D * n) CAREFUL OF DIVIDE BY ZERO
                                 Geometrically if D*n == 0 then, the ray is parallel to plane

if(D*n == 0) THIS IS NOT A SAFE COMPARISSON w/ FLOATING POINT
            In general never use '==' with float

Instead of comparing flahs point with ==, 
Example way to do it in C and C++:
    - #define epsilon 1e-5 <- FLOATS
                      1e-15 <- Double

    - #define cmpfloat(x, y) (abs(x-y) <= epsilon), then theyre equal
    
    - INstead of if (D * n), use if(cmpfloat(D * n, 0))...
    Depending on compiler and flags, if(f1 == f2) // both floats may give a warning

To complete Ray-triangle intersection:
    - First, find 't' for Ray-plane intersection. With t, get p.
    - Calculate barycentric coordinates of p. If they're all positive, we have hit the triangle
    - If any negative, or if D*n is zero, we missed

