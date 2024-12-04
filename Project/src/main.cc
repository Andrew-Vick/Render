#include "rtweekend.h"
#include <array>
#include <optional>

#include "bvh.h"
#include "camera.h"
#include "constant_medium.h"
#include "hittable.h"
#include "hittable_list.h"
#include "material.h"
#include "post_process.h"
#include "quad.h"
#include "sphere.h"
#include "texture.h"
#include "thread_pool.h"
#include "mesh_import.h"

/**
 * @brief Render Notation
 *
 * vec3: 3D vector
 * point3: 3D point
 * color: RGB color
 * (x, y, z); (r, g, b): 3D coordinates and RGB color
 *
 * COORDINATE SYSTEM:
 * X-axis: +X is to the right, -x is to the right
 * Y-axis: Vertical axis (bottom to top)
 * Z-axis: Depth axis (front to back) -- +Z is towards the camera
 *
 * QUAD:
 * Q(point3(x, y, z){Bottom-Left Corner},
 * u(vec3(x, y, z) vector to the right
 * v(vec3(x, y, z) vector to the top
 *
 * QUAD(corner, u, v, mat)
 * mat(material)
 *
 * BOX:
 * box(corner1, coner2, shared_ptr<material> mat)
 * corner1(point3(x,y,z)): One corner of the box
 * corner2(point3(x,y,z)): The opposite corner of the box
 * (if corner 1 is bottom-left-front, corner 2 is top-right-back)
 *
 * SPHERE:
 * sphere(center, radius, shared_ptr<material> mat)
 * center(point3(x, y, z)): Center of the sphere
 * radius(double): Radius of the sphere
 */

void bouncing_spheres()
{
  hittable_list world;

  auto checker = make_shared<checker_texture>(0.32, color(.2, .3, .1), color(.9, .9, .9));
  world.add(make_shared<sphere>(point3(0, -1000, 0), 1000, make_shared<lambertian>(checker)));

  for (int a = -11; a < 11; a++)
  {
    for (int b = -11; b < 11; b++)
    {
      auto choose_mat = random_double();
      point3 center(a + 0.9 * random_double(), 0.2, b + 0.9 * random_double());

      if ((center - point3(4, 0.2, 0)).length() > 0.9)
      {
        shared_ptr<material> sphere_material;

        if (choose_mat < 0.8)
        {
          // diffuse
          auto albedo = color::random() * color::random();
          sphere_material = make_shared<lambertian>(albedo);
          auto center2 = center + vec3(0, random_double(0, .5), 0);
          world.add(make_shared<sphere>(center, center2, 0.2, sphere_material));
        }
        else if (choose_mat < 0.95)
        {
          // metal
          auto albedo = color::random(0.5, 1);
          auto fuzz = random_double(0, 0.5);
          sphere_material = make_shared<metal>(albedo, fuzz);
          world.add(make_shared<sphere>(center, 0.2, sphere_material));
        }
        else
        {
          // glass
          sphere_material = make_shared<dielectric>(1.5);
          world.add(make_shared<sphere>(center, 0.2, sphere_material));
        }
      }
    }
  }

  auto material1 = make_shared<dielectric>(1.5);
  world.add(make_shared<sphere>(point3(0, 1, 0), 1.0, material1));

  auto material2 = make_shared<lambertian>(color(0.4, 0.2, 0.1));
  world.add(make_shared<sphere>(point3(-4, 1, 0), 1.0, material2));

  auto material3 = make_shared<metal>(color(0.7, 0.6, 0.5), 0.0);
  world.add(make_shared<sphere>(point3(4, 1, 0), 1.0, material3));

  auto empty_material = shared_ptr<material>();
  hittable_list lights;
  lights.add(
      make_shared<quad>(point3(343, 554, 332), vec3(-130, 0, 0), vec3(0, 0, -105), empty_material));
  lights.add(make_shared<sphere>(point3(190, 90, 190), 90, empty_material));

  world = hittable_list(make_shared<bvh_node>(world));

  camera cam;

  cam.aspect_ratio = 16.0 / 9.0;
  cam.image_width = 400;
  cam.samples_per_pixel = 50;
  cam.max_depth = 50;
  cam.set_background(make_shared<solid_color>(color(0.70, 0.80, 1.00)), false);

  cam.vfov = 20;
  cam.lookfrom = point3(13, 2, 3);
  cam.lookat = point3(0, 0, 0);
  cam.vup = vec3(0, 1, 0);

  cam.defocus_angle = 0.6;
  cam.focus_dist = 10.0;

  ThreadPool pool(std::thread::hardware_concurrency());

  cam.render(world, lights, pool);
}

void checkered_spheres()
{
  hittable_list world;

  auto checker = make_shared<checker_texture>(0.32, color(.2, .3, .1), color(.9, .9, .9));

  world.add(make_shared<sphere>(point3(0, -10, 0), 10, make_shared<lambertian>(checker)));
  world.add(make_shared<sphere>(point3(0, 10, 0), 10, make_shared<lambertian>(checker)));

  camera cam;

  cam.aspect_ratio = 16.0 / 9.0;
  cam.image_width = 400;
  cam.samples_per_pixel = 100;
  cam.max_depth = 50;
  cam.set_background(make_shared<solid_color>(color(0.70, 0.80, 1.00)), false);

  cam.vfov = 20;
  cam.lookfrom = point3(13, 2, 3);
  cam.lookat = point3(0, 0, 0);
  cam.vup = vec3(0, 1, 0);

  cam.defocus_angle = 0;

  // cam.render(world);
}

void earth()
{
  auto earth_texture = make_shared<image_texture>("earthmap.jpg");
  auto earth_surface = make_shared<lambertian>(earth_texture);
  auto globe = make_shared<sphere>(point3(0, 0, 0), 2, earth_surface);

  camera cam;

  cam.aspect_ratio = 16.0 / 9.0;
  cam.image_width = 400;
  cam.samples_per_pixel = 100;
  cam.max_depth = 50;
  cam.set_background(make_shared<solid_color>(color(0.70, 0.80, 1.00)), false);

  cam.vfov = 20;
  cam.lookfrom = point3(0, 0, 12);
  cam.lookat = point3(0, 0, 0);
  cam.vup = vec3(0, 1, 0);

  cam.defocus_angle = 0;

  // cam.render(hittable_list(globe));
}

void perlin_spheres()
{
  hittable_list world;

  auto pertext = make_shared<noise_texture>(4);
  world.add(make_shared<sphere>(point3(0, -1000, 0), 1000, make_shared<lambertian>(pertext)));
  world.add(make_shared<sphere>(point3(0, 2, 0), 2, make_shared<lambertian>(pertext)));

  camera cam;

  cam.aspect_ratio = 16.0 / 9.0;
  cam.image_width = 400;
  cam.samples_per_pixel = 100;
  cam.max_depth = 50;
  // cam.background = color(0.70, 0.80, 1.00);

  cam.vfov = 20;
  cam.lookfrom = point3(13, 2, 3);
  cam.lookat = point3(0, 0, 0);
  cam.vup = vec3(0, 1, 0);

  cam.defocus_angle = 0;

  // cam.render(world);
}

void quads()
{
  hittable_list world;

  auto left_red = make_shared<lambertian>(color(1.0, 0.2, 0.2));
  auto back_green = make_shared<lambertian>(color(0.2, 1.0, 0.2));
  auto right_blue = make_shared<lambertian>(color(0.2, 0.2, 1.0));
  auto upper_orange = make_shared<lambertian>(color(1.0, 0.5, 0.0));
  auto lower_teal = make_shared<lambertian>(color(0.2, 0.8, 0.8));

  // Quads
  world.add(make_shared<quad>(point3(-3, -2, 5), vec3(0, 0, -4), vec3(0, 4, 0), left_red));
  world.add(make_shared<quad>(point3(-2, -2, 0), vec3(4, 0, 0), vec3(0, 4, 0), back_green));
  world.add(make_shared<quad>(point3(3, -2, 1), vec3(0, 0, 4), vec3(0, 4, 0), right_blue));
  world.add(make_shared<quad>(point3(-2, 3, 1), vec3(4, 0, 0), vec3(0, 0, 4), upper_orange));
  world.add(make_shared<quad>(point3(-2, -3, 5), vec3(4, 0, 0), vec3(0, 0, -4), lower_teal));

  camera cam;

  cam.aspect_ratio = 1.0;
  cam.image_width = 400;
  cam.samples_per_pixel = 100;
  cam.max_depth = 50;
  // cam.background = color(0.70, 0.80, 1.00);

  cam.vfov = 80;
  cam.lookfrom = point3(0, 0, 9);
  cam.lookat = point3(0, 0, 0);
  cam.vup = vec3(0, 1, 0);

  cam.defocus_angle = 0;

  // cam.render(world);
}

void simple_light()
{
  hittable_list world;

  auto pertext = make_shared<noise_texture>(4);
  world.add(make_shared<sphere>(point3(0, -1000, 0), 1000, make_shared<lambertian>(pertext)));
  world.add(make_shared<sphere>(point3(0, 2, 0), 2, make_shared<lambertian>(pertext)));

  auto difflight = make_shared<diffuse_light>(color(4, 4, 4));
  world.add(make_shared<sphere>(point3(0, 7, 0), 2, difflight));
  world.add(make_shared<quad>(point3(3, 1, -2), vec3(2, 0, 0), vec3(0, 2, 0), difflight));

  auto empty_material = shared_ptr<material>();
  hittable_list lights;
  lights.add(make_shared<sphere>(point3(0, 1, 0), 90, empty_material));

  camera cam;

  cam.aspect_ratio = 16.0 / 9.0;
  cam.image_width = 400;
  cam.samples_per_pixel = 100;
  cam.max_depth = 50;
  cam.set_background(make_shared<solid_color>(color(0.0, 0.0, 0.0)), false);

  cam.vfov = 20;
  cam.lookfrom = point3(26, 3, 6);
  cam.lookat = point3(0, 2, 0);
  cam.vup = vec3(0, 1, 0);

  cam.defocus_angle = 0;

  // cam.redner(world, lights);
}

void cornell_box()
{
  hittable_list world;

  auto red = make_shared<lambertian>(color(.65, .05, .05));
  auto white = make_shared<lambertian>(color(0.89, 0.67, 0.59));
  auto green = make_shared<lambertian>(color(.12, .45, .15));
  auto blue = make_shared<lambertian>(color(.15, .15, .65));
  auto purple = make_shared<lambertian>(color(.65, .05, .65));
  auto light = make_shared<diffuse_light>(color(45, 45, 45));
  auto empty_material = shared_ptr<material>();

  hittable_list world_objects;

  // Cornell box sides
  world_objects.add(make_shared<quad>(point3(555, 0, 0), vec3(0, 0, 555), vec3(0, 555, 0), green)); // left

  /**
   * the right wall's corner is at x = 0, y = 0, its z is away from the camera at 555, floor vector is pointing to x = 0, y=0, z-555(coming at you)
   * the vector for height is starting at x = 0, z =0 and it pointing up to 555
   *
   * bottom corner is away from camera at bottom right and is drawn using vector coming to you and going up from the origin
   */
  world_objects.add(make_shared<quad>(point3(0, 0, 555), vec3(0, 0, -555), vec3(0, 555, 0), red)); // right

  world_objects.add(make_shared<quad>(point3(0, 555, 0), vec3(555, 0, 0), vec3(0, 0, 555), blue));    // ceiling
  world_objects.add(make_shared<quad>(point3(0, 0, 555), vec3(555, 0, 0), vec3(0, 0, -555), purple)); // floor

  // this will create a back wall
  // world_objects.add(make_shared<quad>(point3(555, 0, 555), vec3(-555, 0, 0), vec3(0, 555, 0), white)); //back

  auto wall_material = make_shared<lambertian>(color(1.0, 1.0, 1.0)); // White wall material
  auto glass_material = make_shared<dielectric>(1.5);                 // Glass material for the window

  // Dimensions
  double wall_width = 555;
  double wall_height = 555;
  double window_width = 200;
  double window_height = 200;
  double window_x_offset = (wall_width - window_width) / 2;   // Center the window horizontally
  double window_y_offset = (wall_height - window_height) / 2; // Center the window vertically

  // Sun sphere outside room
  double intensity = 500.0; // Adjust intensity as needed
  color sunlight_emit = intensity * color(0.98, 0.58, 0.0039);
  auto sunlight_material = make_shared<diffuse_light>(sunlight_emit); // Bright sunlight
  world_objects.add(make_shared<sphere>(point3(555 - window_x_offset - window_width, 450, 700), 100, sunlight_material));

  // Ground
  world_objects.add(make_shared<sphere>(point3(0, -1000, 0), 1000, make_shared<lambertian>(color(0, 1, 0))));

  // dust in room
  world_objects.add(make_shared<constant_medium>(box(point3(555,555,555), point3(0,0,0), empty_material), 0.001, color(0.89, 0.8, 0.78)));

  // Back Wall (4 surrounding quads + glass center)
  // Bottom part of wall
  world_objects.add(make_shared<quad>(
      point3(555, 0, 555),         // Bottom-left corner of the bottom quad
      vec3(-555, 0, 0),            // Full width to the left
      vec3(0, window_y_offset, 0), // Up to the bottom of the window
      wall_material));

  // Top part of wall
  world_objects.add(make_shared<quad>(
      point3(555, window_y_offset + window_height, 555),         // Bottom-left corner of the top quad
      vec3(-wall_width, 0, 0),                                   // Full width to the left
      vec3(0, wall_height - window_y_offset - window_height, 0), // Up to the ceiling
      wall_material));

  // // Left part of wall
  world_objects.add(make_shared<quad>(
      point3(555, window_y_offset, 555), // Bottom-left corner of the left quad
      vec3(-window_x_offset, 0, 0),      // Width to the left of the window
      vec3(0, window_height, 0),         // Height of the window
      wall_material));

  // Right part of wall
  world_objects.add(make_shared<quad>(
      point3(555 - window_x_offset - window_width, window_y_offset, 555), // Bottom-left corner of the right quad
      vec3(-180, 0, 0),                                                   // Width to the right of the window
      vec3(0, window_height, 0),                                          // Height of the window
      wall_material));

  // Center glass window
  world_objects.add(make_shared<quad>(
      point3(555 - window_x_offset, window_y_offset, 555), // Bottom-left corner of the glass quad
      vec3(-window_width, 0, 0),                           // Window width
      vec3(0, window_height, 0),                           // Window height
      glass_material));

  // Light
  world_objects.add(make_shared<quad>(point3(213, 554, 227), vec3(130, 0, 0), vec3(0, 0, 105), light));

  // Box
  auto earth_texture = make_shared<image_texture>("earthmap.jpg");
  auto earth = make_shared<lambertian>(earth_texture);
  shared_ptr<hittable> box1 = box(point3(0, 0, 0), point3(165, 330, 165), earth);
  box1 = make_shared<rotate_y>(box1, 15);
  box1 = make_shared<translate>(box1, vec3(265, 0, 295));
  world_objects.add(box1);

  shared_ptr<hittable> box2 = box(point3(0, 0, 0), point3(165, 165, 165), white);
  box2 = make_shared<rotate_y>(box2, -18);
  box2 = make_shared<translate>(box2, vec3(130, 0, 65));
  world_objects.add(box2);

  // Load and convert mesh to triangles
  // std::vector<Mesh> meshes;
  // hittable_list mesh_list;
  // if (MeshImporter::LoadMesh("/Users/andrewvick/Coms336/Project/src/Textures/meshes/vaze3.OBJ", meshes))
  // {
  //   MeshImporter importer;
  //   for (const auto &mesh : meshes)
  //   {
  //     std::vector<shared_ptr<hittable>> triangles;
  //     //importer.convertMeshToTriangles(mesh, triangles, blue, 1);

  //     // Add triangles to the mesh_list
  //     for (const auto &tri : triangles)
  //     {
  //       if (tri)
  //       {
  //         mesh_list.add(tri);
  //       }
  //       else
  //       {
  //         std::cerr << "null triangle skipped!" << std::endl;
  //       }
  //     }
  //   }

  //   // Create a BVH for all the triangles in the mesh_list
  //   if (!mesh_list.objects.empty())
  //   {
  //     auto moved_mesh = make_shared<translate>(make_shared<bvh_node>(mesh_list), vec3(190, 90, 190));
  //     world_objects.add(moved_mesh);
  //   }
  // }
  // else
  // {
  //   std::cerr << "Failed to load the mesh!" << std::endl;
  //   return;
  // }

  world_objects.add(make_shared<quad>(point3(343, 554, 332), vec3(-130, 0, 0), vec3(0, 0, -105), white));

  world.add(make_shared<bvh_node>(world_objects));

  // Light Sources
  hittable_list lights;
  lights.add(
      make_shared<quad>(point3(213, 554, 227), vec3(130, 0, 0), vec3(0, 0, 105), empty_material));
  lights.add(make_shared<sphere>(point3(555 - window_x_offset - window_width, 450, 700), 100, empty_material));

  camera cam;

  cam.aspect_ratio = 1.0;
  cam.image_width = 600;
  cam.samples_per_pixel = 300;
  cam.max_depth = 100;
  cam.set_background(make_shared<solid_color>(color(0.529, 0.807, 0.922)), false);

  cam.vfov = 40;
  cam.lookfrom = point3(278, 278, -750);
  cam.lookat = point3(278, 278, 0);
  cam.vup = vec3(0, 1, 0);

  cam.defocus_angle = 0;

  ThreadPool pool(std::thread::hardware_concurrency());

  // Render the scene
  cam.render(world, lights, pool);
}

void cornell_smoke()
{
  hittable_list world;

  auto red = make_shared<lambertian>(color(.65, .05, .05));
  auto white = make_shared<lambertian>(color(.73, .73, .73));
  auto green = make_shared<lambertian>(color(.12, .45, .15));
  auto light = make_shared<diffuse_light>(color(7, 7, 7));

  world.add(make_shared<quad>(point3(555, 0, 0), vec3(0, 555, 0), vec3(0, 0, 555), green));
  world.add(make_shared<quad>(point3(0, 0, 0), vec3(0, 555, 0), vec3(0, 0, 555), red));
  world.add(make_shared<quad>(point3(113, 554, 127), vec3(330, 0, 0), vec3(0, 0, 305), light));
  world.add(make_shared<quad>(point3(0, 555, 0), vec3(555, 0, 0), vec3(0, 0, 555), white));
  world.add(make_shared<quad>(point3(0, 0, 0), vec3(555, 0, 0), vec3(0, 0, 555), white));
  world.add(make_shared<quad>(point3(0, 0, 555), vec3(555, 0, 0), vec3(0, 555, 0), white));

  shared_ptr<hittable> box1 = box(point3(0, 0, 0), point3(165, 330, 165), white);
  box1 = make_shared<rotate_y>(box1, 15);
  box1 = make_shared<translate>(box1, vec3(265, 0, 295));

  shared_ptr<hittable> box2 = box(point3(0, 0, 0), point3(165, 165, 165), white);
  box2 = make_shared<rotate_y>(box2, -18);
  box2 = make_shared<translate>(box2, vec3(130, 0, 65));

  world.add(make_shared<constant_medium>(box1, 0.01, color(0, 0, 0)));
  world.add(make_shared<constant_medium>(box2, 0.01, color(1, 1, 1)));

  camera cam;

  cam.aspect_ratio = 1.0;
  cam.image_width = 600;
  cam.samples_per_pixel = 200;
  cam.max_depth = 50;
  // cam.background = color(0, 0, 0);
  cam.set_background(make_shared<solid_color>(color(0.0, 0.0, 0.0)), false);

  cam.vfov = 40;
  cam.lookfrom = point3(278, 278, -800);
  cam.lookat = point3(278, 278, 0);
  cam.vup = vec3(0, 1, 0);

  cam.defocus_angle = 0;

  ////cam.render(world);
}

void final_scene(int image_width, int samples_per_pixel, int max_depth)
{
  hittable_list boxes1;
  auto ground = make_shared<lambertian>(color(0.48, 0.83, 0.53));

  int boxes_per_side = 20;
  for (int i = 0; i < boxes_per_side; i++)
  {
    for (int j = 0; j < boxes_per_side; j++)
    {
      auto w = 100.0;
      auto x0 = -1000.0 + i * w;
      auto z0 = -1000.0 + j * w;
      auto y0 = 0.0;
      auto x1 = x0 + w;
      auto y1 = random_double(1, 101);
      auto z1 = z0 + w;

      boxes1.add(box(point3(x0, y0, z0), point3(x1, y1, z1), ground));
    }
  }

  hittable_list world;

  world.add(make_shared<bvh_node>(boxes1));

  auto light = make_shared<diffuse_light>(color(7, 7, 7));
  world.add(make_shared<quad>(point3(123, 554, 147), vec3(300, 0, 0), vec3(0, 0, 265), light));

  auto center1 = point3(400, 400, 200);
  auto center2 = center1 + vec3(30, 0, 0);
  auto sphere_material = make_shared<lambertian>(color(0.7, 0.3, 0.1));
  world.add(make_shared<sphere>(center1, center2, 50, sphere_material));

  world.add(make_shared<sphere>(point3(260, 150, 45), 50, make_shared<dielectric>(1.5)));
  world.add(make_shared<sphere>(
      point3(0, 150, 145), 50, make_shared<metal>(color(0.8, 0.8, 0.9), 1.0)));

  auto boundary = make_shared<sphere>(point3(360, 150, 145), 70, make_shared<dielectric>(1.5));
  world.add(boundary);
  world.add(make_shared<constant_medium>(boundary, 0.2, color(0.2, 0.4, 0.9)));
  boundary = make_shared<sphere>(point3(0, 0, 0), 5000, make_shared<dielectric>(1.5));
  world.add(make_shared<constant_medium>(boundary, .0001, color(1, 1, 1)));

  auto emat = make_shared<lambertian>(make_shared<image_texture>("earthmap.jpg"));
  world.add(make_shared<sphere>(point3(400, 200, 400), 100, emat));
  auto pertext = make_shared<noise_texture>(0.2);
  world.add(make_shared<sphere>(point3(220, 280, 300), 80, make_shared<lambertian>(pertext)));

  hittable_list boxes2;
  auto white = make_shared<lambertian>(color(.73, .73, .73));
  int ns = 1000;
  for (int j = 0; j < ns; j++)
  {
    boxes2.add(make_shared<sphere>(point3::random(0, 165), 10, white));
  }

  world.add(make_shared<translate>(
      make_shared<rotate_y>(
          make_shared<bvh_node>(boxes2), 15),
      vec3(-100, 270, 395)));

  auto empty_material = shared_ptr<material>();
  hittable_list lights;
  lights.add(
      make_shared<quad>(point3(343, 554, 332), vec3(-130, 0, 0), vec3(0, 0, -105), empty_material));
  lights.add(make_shared<sphere>(point3(190, 90, 190), 90, empty_material));

  camera cam;

  cam.aspect_ratio = 1.0;
  cam.image_width = image_width;
  cam.samples_per_pixel = samples_per_pixel;
  cam.max_depth = max_depth;
  // cam.background = color(0, 0, 0);
  cam.set_background(make_shared<solid_color>(color(0.0, 0.0, 0.0)), false);

  cam.vfov = 40;
  cam.lookfrom = point3(478, 278, -600);
  cam.lookat = point3(278, 278, 0);
  cam.vup = vec3(0, 1, 0);

  cam.defocus_angle = 0;

  ThreadPool pool(std::thread::hardware_concurrency());

  cam.render(world, lights, pool);
}

void cube_map_test()
{
  hittable_list world;
  hittable_list mesh_list;

  // Define cubemap faces
  std::array<std::string, 6> cubemap_faces = {
      "/Users/andrewvick/Coms336/Project/src/Textures/CubeMaps/4k_cobble_hdr/px.hdr", // +X
      "/Users/andrewvick/Coms336/Project/src/Textures/CubeMaps/4k_cobble_hdr/nx.hdr", // -X
      "/Users/andrewvick/Coms336/Project/src/Textures/CubeMaps/4k_cobble_hdr/py.hdr", // +Y
      "/Users/andrewvick/Coms336/Project/src/Textures/CubeMaps/4k_cobble_hdr/ny.hdr", // -Y
      "/Users/andrewvick/Coms336/Project/src/Textures/CubeMaps/4k_cobble_hdr/pz.hdr", // +Z
      "/Users/andrewvick/Coms336/Project/src/Textures/CubeMaps/4k_cobble_hdr/nz.hdr"  // -Z
  };

  // Create the cubemap texture from the 6 faces
  auto cubemap_texture = std::make_shared<cube_map_texture>(cubemap_faces);

  // Add objects to the scene
  auto red = make_shared<lambertian>(color(1, 0, 0));
  auto light = make_shared<diffuse_light>(color(15, 15, 15));

  // Import a triangle mesh using MeshImporter (e.g., some .obj file)

  // Light
  world.add(make_shared<sphere>(point3(0, 300, 0), 50, light));

  world.add(make_shared<sphere>(point3(0, 0, 0), 10, red));

  auto empty_material = shared_ptr<material>();
  hittable_list lights;
  // lights.add(
  //     make_shared<quad>(point3(343, 554, 332), vec3(-130, 0, 0), vec3(0, 0, -105), empty_material));
  lights.add(make_shared<sphere>(point3(0, 300, 0), 50, empty_material));

  world = hittable_list(make_shared<bvh_node>(world));

  // Set up the camera
  camera cam;

  cam.aspect_ratio = 1.0;
  cam.image_width = 1440;
  cam.samples_per_pixel = 100;
  cam.max_depth = 20;

  // Set the background to the cubemap texture
  cam.set_background(cubemap_texture, true);
  // cam.set_background(make_shared<solid_color>(color(0.6, 0.6, 0.6)), false);

  cam.vfov = 90;
  cam.lookfrom = point3(0, 0, -20);
  cam.lookat = point3(0, 0, 0);
  cam.vup = vec3(0, 1, 0);

  cam.defocus_angle = 0;

  ThreadPool pool(std::thread::hardware_concurrency());

  // Render the scene
  cam.render(world, lights, pool);
}

void stars()
{
  hittable_list world;
  hittable_list lights;

  // Materials
  auto star_emission = color(1, 1, 1) * 300.0;
  auto earth_texture = make_shared<image_texture>("earthmap.jpg");

  auto earth_emission = make_shared<diffuse_light>(earth_texture, 100);

  auto dummy_material = shared_ptr<material>();

  auto grassTex = make_shared<grass_texture>(5.0);
  auto grassNormalMap = make_shared<normal_map_texture>(20.0, 0.5);
  auto grassMat = make_shared<lambertian>(grassTex, grassNormalMap);

  int starCount = 0;
  const int totalStars = 1000;
  const double starDistance = 5000.0;
  double starRadius = 0.0;
  auto star_mat = make_shared<diffuse_light>(star_emission);
  double moon_radius = 800.0;

  vec3 moon_position = vec3(-5000, 3000, -4000);
  vec3 starPosition = vec3(0, 0, 0);

  while (starCount < totalStars)
  {
    // Generate a random direction in the upper hemisphere
    vec3 dir = unit_vector(vec3::random(-1, 1));
    if (dir.y() <= -0.1 ) // Exclude directions below the horizon
      continue;

    starPosition = dir * starDistance;

    // Exclude stars that intersect with the moon
    starRadius = random_double(5.0, 25.0);

    vec3 result = moon_position - starPosition;
    if (result.length() < 1000 + starRadius)
      continue;

    vec3 starPosition = dir * starDistance;
    
    starCount++;

    double star_brightness = random_double(0.3, 1.0) * 500.0;
    star_emission = color(star_brightness, star_brightness, star_brightness);
    star_mat = make_shared<diffuse_light>(star_emission);

    // Add stars to the world
    world.add(make_shared<sphere>(starPosition, starRadius, star_mat));
    // Optionally, add to lights list if you're using it for importance sampling
    lights.add(make_shared<sphere>(starPosition, starRadius, dummy_material));
  }

  // Ground
  world.add(make_shared<sphere>(
      point3(0, -1000, 0), 1000, grassMat)); // Adjusted to a darker green

  // Moon (placed out of frame but adds ambient lighting)
  world.add(make_shared<sphere>(moon_position, moon_radius, earth_emission));
  lights.add(make_shared<sphere>(moon_position, moon_radius, dummy_material));

  world.add(make_shared<bvh_node>(world));

  camera cam;

  cam.aspect_ratio = 1;
  cam.image_width = 500;
  cam.samples_per_pixel = 10;
  cam.max_depth = 10;
  cam.set_background(make_shared<solid_color>(color(0, 0, 0)), false);

  // Camera positioned to look at the horizon
  cam.lookfrom = point3(0, 2, -10);
  cam.lookat = point3(-5000, 3000, -4000);
  cam.vfov = 60; // Narrower field of view to focus on the horizon

  cam.vup = vec3(0, 1, 0);
  cam.defocus_angle = 0;

  ThreadPool pool(std::thread::hardware_concurrency());

  // Render the scene
  cam.render(world, lights, pool);
}

void prism()
{
  hittable_list world_objects;
  hittable_list world;
  hittable_list lights;

  auto glass = make_shared<dielectric>(1.5); // Example for colored glass
  auto white = make_shared<lambertian>(color(0.73, 0.73, 0.73));
  auto blue = make_shared<lambertian>(color(0.1, 0.2, 0.5));
  auto empty_material = shared_ptr<material>();

  std::vector<Mesh> meshes;
  hittable_list mesh_list;
  if (MeshImporter::LoadMesh("/Users/andrewvick/Coms336/Project/src/Textures/meshes/20255_Triangular_Prism_V1.OBJ", meshes))
  {
    MeshImporter importer;
    for (const auto &mesh : meshes)
    {
      std::vector<shared_ptr<hittable>> triangles;
      importer.mesh_to_tri_mat(mesh, triangles, glass);

      // Add triangles to the mesh_list
      for (const auto &tri : triangles)
      {
        if (tri)
        {
          mesh_list.add(tri);
        }
        else
        {
          std::cerr << "null triangle skipped!" << std::endl;
        }
      }
    }

    // Create a BVH for all the triangles in the mesh_list
    if (!mesh_list.objects.empty())
    {
      auto moved_mesh = make_shared<translate>(make_shared<bvh_node>(mesh_list), vec3(0, 10, 0));
      world_objects.add(moved_mesh);
    }
  }
  else
  {
    std::cerr << "Failed to load the mesh!" << std::endl;
    return;
  }

  //world_objects.add(make_shared<quad>(point3(-50, 0, 50), vec3(100, 0, 0), vec3(0, 50, 0), white)); // back

  world_objects.add(make_shared<sphere>(
      point3(0, -1000, 0), 1000, white)); // Adjusted to a darker green

  world_objects.add(make_shared<sphere>(
      point3(0, 50, 0),                     // Center of the sphere
      10,                                           // Radius of the light
      make_shared<diffuse_light>(color(30, 30, 30)) // Bright light
      ));

    lights.add(make_shared<sphere>(
      point3(0, 50, 0),                     // Center of the sphere
      10,                                           // Radius of the light
      empty_material // Bright light
      ));

  world.add(make_shared<bvh_node>(world_objects));

  camera cam;

  cam.aspect_ratio = 1.0;
  cam.image_width = 600;
  cam.samples_per_pixel = 50;
  cam.max_depth = 10;
  cam.set_background(make_shared<solid_color>(color(0, 0, 0)), false);

  cam.lookfrom = point3(0, 10, -30); // Position the camera in front of the prism
  cam.lookat = point3(0, 0, 0);  // Focus on the prism center
  cam.vfov = 45;                         // Narrow field of view for focused observation

  cam.vup = vec3(0, 1, 0);

  cam.defocus_angle = 0;

  ThreadPool pool(std::thread::hardware_concurrency());

  // Render the scene
  cam.render(world, lights, pool);
}


int main(int arg, char *argv[])
{
  std::string command = "7";
  if (arg >= 2)
  {
    command = argv[1];
  }

  switch (std::stoi(command))
  {
  case 1:
    bouncing_spheres();
    break;
  case 2:
    checkered_spheres();
    break;
  case 3:
    earth();
    break;
  case 4:
    perlin_spheres();
    break;
  case 5:
    quads();
    break;
  case 6:
    simple_light();
    break;
  case 7:
    cornell_box();
    break;
  case 8:
    cornell_smoke();
    break;
  case 9:
    final_scene(800, 10000, 40);
    break;
  case 10:
    final_scene(400, 250, 4);
    break;
  case 11:
    cube_map_test();
    break;
  case 12:
    stars();
    break;
  case 13:
    prism();
    break;
  }

  return 0;
}
