#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#define X 1024
#define Y 768

#define TY 1300
#define TX 827

#define frand() (rand() / (RAND_MAX + 1.0))

#define SCALING_FACTOR = 0.5 

typedef double vec3[3];

typedef struct
{
  vec3 a, b, c;
  vec3 ta, tb, tc;
} triangle;

int8_t img[Y][X][3]; // Zero by placement

vec3 texture[TY][TX];

void vec3_cross(vec3 c, vec3 v, vec3 w)
{
  c[0] = v[1] * w[2] - w[1] * v[2];
  c[1] = v[2] * w[0] - w[2] * v[0];
  c[2] = v[0] * w[1] - w[0] * v[1];
}

double vec3_dot(vec3 v, vec3 w)
{
  return v[0] * w[0] + v[1] * w[1] + v[2] * w[2];
}

void vec3_sub(vec3 s, vec3 v, vec3 w)
{
  s[0] = v[0] - w[0];
  s[1] = v[1] - w[1];
  s[2] = v[2] - w[2];
}

void vec3_add(vec3 s, vec3 v, vec3 w)
{
  s[0] = v[0] + w[0];
  s[1] = v[1] + w[1];
  s[2] = v[2] + w[2];
}

void vec3_mul_scalar(vec3 v, vec3 w, double s)
{
  v[0] = s * w[0];
  v[1] = s * w[1];
  v[2] = s * w[2];
}

void get_texture()
{
  int x, y;

  for (y = 0; y < TY; y++)
  {
    for (x = 0; x < TX; x++)
    {
      if (y < TY / 2)
      {
        texture[y][x][0] = 1.0;
      }
      else
      {
        if (x < TX / 2)
        {
          texture[y][x][1] = 1.0;
        }
        else
        {
          texture[y][x][2] = 1.0;
        }
      }
    }
  }
}

void load_texture()
{
  FILE *f;
  char s[80];
  int x, y;
  int r, g, b;

  f = fopen("earth2.ppm", "r");
  fscanf(f, "%s\n%d %d %d\n", s, &x, &x, &x);
  for (y = 0; y < TY; y++)
  {
    for (x = 0; x < TX; x++)
    {
      fscanf(f, "%d %d %d", &r, &g, &b);
      texture[y][x][0] = r / 256.0;
      texture[y][x][1] = g / 256.0;
      texture[y][x][2] = b / 256.0;
    }
  }
  fclose(f);
}

int main(int argc, char *argv[])
{
  FILE *f;
  triangle t;
  vec3 p, q;
  vec3 n, na, nb, nc;
  vec3 d1, d2; // differences
  vec3 bary;
  vec3 uv;
  int x, y;
  vec3 color;
  int i, aa;

  aa = 1;

  if (argc == 2)
  {
    aa = atoi(argv[1]);
  }

  t.a[0] = 1.0;
  t.a[1] = 0.0;
  t.b[0] = 0.0;
  t.b[1] = 1.0;
  t.c[0] = 0.0;
  t.c[1] = 0.0;
  t.a[2] = t.b[2] = t.c[2] = 0;

  t.ta[0] = 0.0;
  t.ta[1] = 1.0;
  t.tb[0] = 0.0;
  t.tb[1] = 0.0;
  t.tc[0] = 1.0;
  t.tc[1] = 0.5;
  t.ta[2] = t.tb[2] = t.ta[2] = 0;

  // gen_texture();
  load_texture();

  //  n = (b - a) x (c - a)
  vec3_sub(d1, t.b, t.a);
  vec3_sub(d2, t.c, t.a);
  vec3_cross(n, d1, d2);

  p[2] = q[2] = 0;

  /* This "camera" gives us an orthographic projection */
  for (y = 0; y < Y; y++)
  {
    p[1] = (y / (double)Y);
    for (x = 0; x < X; x++)
    {
      color[0] = color[1] = color[2] = 0;
      p[0] = (x / (double)X);

      for (i = 0; i < aa; i++)
      {
        q[1] = p[1] + frand() * (1.0 / Y);
        q[0] = p[0] + frand() * (1.0 / X);

        // na = (c - b) x (p - b)
        vec3_sub(d1, t.c, t.b);
        vec3_sub(d2, q, t.b);
        vec3_cross(na, d1, d2);
        // nb = (a - c) x (p - c)
        vec3_sub(d1, t.a, t.c);
        vec3_sub(d2, q, t.c);
        vec3_cross(nb, d1, d2);
        // nc = (b - a) x (p - a)
        vec3_sub(d1, t.b, t.a);
        vec3_sub(d2, q, t.a);
        vec3_cross(nc, d1, d2);

        bary[0] = vec3_dot(n, na) / vec3_dot(n, n);
        bary[1] = vec3_dot(n, nb) / vec3_dot(n, n);
        bary[2] = vec3_dot(n, nc) / vec3_dot(n, n);

        // If we remove the test, triangles will tessellate the plane
        if (bary[0] >= 0 && bary[1] >= 0 && bary[2] >= 0)
        {
          // uv = ta +beta(tb-ta) + gamma(tc+ta)
          vec3_sub(d1, t.tb, t.ta);
          vec3_mul_scalar(d1, d1, bary[1]);

          vec3_sub(d2, t.tc, t.ta);
          vec3_mul_scalar(d2, d2, bary[2]);

          vec3_add(uv, d1, d2);
          vec3_add(uv, uv, t.ta);

          /* color according to barycentric coordinates -- nice gradient
          color[0] += bary[0];
          color[1] += bary[1];
          color[2] += bary[2];
          */

          color[0] += texture[(int)(uv[1] * TY)][(int)(uv[0] * TX)][0];
          color[1] += texture[(int)(uv[1] * TY)][(int)(uv[0] * TX)][1];
          color[2] += texture[(int)(uv[1] * TY)][(int)(uv[0] * TX)][2];
        }
      }
      img[y][x][0] = color[0] * 255.999 / aa;
      img[y][x][1] = color[1] * 255.999 / aa;
      img[y][x][2] = color[2] * 255.999 / aa;
    }
  }

  f = fopen("barycentric.ppm", "wb");
  fprintf(f, "P6\n%d\n%d\n255\n", X, Y);
  fwrite(img, 3, X * Y, f);
  fclose(f);
}
