#include <stdio.h>
#include <stdint.h>

#define X 1024
#define Y 768

#define R 0
#define G 1
#define B 2

int main(int argc, char *argv[])
{
  FILE *img;
  uint8_t raster[Y][X][3];
  int x, y;

  img = fopen("image.ppm", "w");

  for (y=0;y<Y;y++){
    for(x=0;x<X;x++){
      raster[y][x][R] = 255 - (y * 255 / Y);
      raster[y][x][G] = (x * 255) / X;
      raster[y][x][B] = (y * 255 / Y);
     }
  }
  printf("P6\n%d\n%d\n255\n",X,Y); 

  fwrite(raster, 3, X * Y, img);

  fclose(img);

  return 0;
}
