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

inline bool check_line_sphere_intersection(
    const point3 &camera_pos,
    const point3 &sphere_center,
    double sphere_radius,
    const point3 &object_pos)
{
  vec3 direction = object_pos - camera_pos;           // Line direction from camera to object
  vec3 camera_to_sphere = sphere_center - camera_pos; // Vector from camera to sphere center

  // Compute the cross product and its length
  vec3 cross_prod = cross(direction, camera_to_sphere);
  double distance = cross_prod.length() / direction.length(); // Shortest distance from sphere center to line

  // Check if the distance is less than the sphere's radius
  return distance < sphere_radius;
}

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

void add_mesh_to_scene(const std::string &file_path, hittable_list &world_objects, const double &rotation, const vec3 &translation)
{
  std::vector<Mesh> meshes;
  hittable_list mesh_list;
  if (MeshImporter::LoadMesh(file_path, meshes))
  {
    MeshImporter importer;
    for (const auto &mesh : meshes)
    {
      std::vector<shared_ptr<hittable>> triangles;
      importer.convertMeshToTriangles(mesh, triangles);

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
      auto moved_mesh = make_shared<translate>(make_shared<rotate_y>(make_shared<bvh_node>(mesh_list), rotation), translation);
      world_objects.add(moved_mesh);
    }
  }
  else
  {
    std::cerr << "Failed to load the mesh: " << file_path << std::endl;
  }
  meshes.clear();
  mesh_list.clear();
}

void final_class_render()
{

  /**
   * @brief Cornell Box
   * 
   * back right bottom corner is 0,0,0 (intersection of right, back and floor)
   * left is +x, right is -x
   */

  hittable_list world;
  hittable_list world_objects;
  hittable_list lights;

  // Colors
  color sun_color = color(1, 1, 1);              // Sunlight
  color red = color(.65, .05, .05);               // Red
  color white = color(0.89, 0.67, 0.59);          // White
  color green = color(.12, .45, .15);             // Green
  color blue = color(.15, .15, .65);              // Blue
  color purple = color(.65, .05, .65);            // Purple
  color black = color(0, 0, 0);                   // Black
  color shooting_star = color(0.88, 0.35, 0.133); // Yellowish-White

  // Textures
  auto moon_texture = make_shared<image_texture>("/Users/andrewvick/Coms336/Project/src/Textures/moonmap.jpg"); // Moon texture
  auto wall_material = make_shared<normal_map_texture>(50.0, 0.3);                                              // Wall Texture
  auto red_tex = make_shared<solid_color>(red);                                                                 // Red texture
  auto green_tex = make_shared<solid_color>(green);                                                             // Green texture
  auto blue_tex = make_shared<solid_color>(blue);                                                               // Blue texture
  auto purple_tex = make_shared<solid_color>(purple);                                                           // Purple texture
  auto white_tex = make_shared<solid_color>(white);                                                             // White texture
  auto black_tex = make_shared<solid_color>(black);                                                             // Black texture
  auto checker_tex = make_shared<checker_texture>(20, white_tex, black_tex);                                  // Checker texture
  auto grass_tex = make_shared<grass_texture>(5.0);                                                           // Grass texture                               

  // Materials
  auto glass_portal_mat = make_shared<dielectric>(1.2);                // Glass material for portal
  auto glass_material = make_shared<dielectric>(1.5);                  // Glass material
  auto light = make_shared<diffuse_light>(color(1.0, 0.97, 0.84), 20);    // Light material
  auto aluminum = make_shared<metal>(color(0.8, 0.85, 0.88), 0.0);     // Aluminum material
  auto checker_material = make_shared<lambertian>(checker_tex);        // Checker material
  auto blue_material = make_shared<lambertian>(blue_tex);                   // Blue material
  auto white_material = make_shared<lambertian>(white_tex);                 // White material
  auto empty_material = shared_ptr<material>();                        // Empty material
  auto sunlight_material = make_shared<diffuse_light>(sun_color, 200);  // Sunlight material
  auto moon_emission = make_shared<diffuse_light>(moon_texture, 20.0); // Moon emission material
  auto grass_material = make_shared<lambertian>(grass_tex);            // Grass material

  world_objects.add(make_shared<sphere>(point3(277.5, 1000, 800), 50, sunlight_material));
  lights.add(make_shared<sphere>(point3(277.5, 1000, 800), 50, empty_material));

  world_objects.add(make_shared<quad>(point3(-5000, 0, 555), vec3(10000, 0, 0), vec3(0, 0, 7000), grass_material));

  // Add specific stars beyond the moon, visible through the window
  std::vector<vec3> star_positions;
  double min_distance = 150.0; // Minimum distance between stars

  int num_stars = 150;
  int attempts = 0;                         
  vec3 camera_pos = point3(278, 278, -750); 

  // Moon light
  vec3 moon_pos = point3(-100, 700, 4000); 
  world_objects.add(make_shared<sphere>(moon_pos, 100, moon_emission));
  lights.add(make_shared<sphere>(moon_pos, 100, empty_material));

  while (star_positions.size() < num_stars && attempts < num_stars * 10)
  {
    // Generate a random point on the window
    double window_x = random_double(77.5, 555);
    double window_y = random_double(250, 555);

    // Direction from camera to the window point
    vec3 direction = vec3(window_x, window_y, 555) - camera_pos;
    direction = unit_vector(direction);

    // Random distance beyond the window
    double star_distance = random_double(9000.0, 10000.0);

    // Compute star position
    point3 new_position = camera_pos + star_distance * direction;
    bool too_close = false;

    // Check for minimum distance to existing stars
    for (const auto &pos : star_positions)
    {
      if ((new_position - pos).length() < min_distance)
      {
        too_close = true;
        break;
      }
    }

    // Check if the star is obstructed by the moon
    if (!too_close && check_line_sphere_intersection(camera_pos, moon_pos, 100.0, new_position))
    {
      too_close = true;
    }

    if (!too_close)
    {
      star_positions.push_back(new_position);

      double star_radius = random_double(1.0, 3.0);
      double star_brightness = random_double(0.3, 1.0) * 25.0;
      color star_emission = color(star_brightness, star_brightness, star_brightness);
      auto star_mat = make_shared<diffuse_light>(star_emission);
      world_objects.add(make_shared<sphere>(new_position, star_radius, star_mat));
      //lights.add(make_shared<sphere>(new_position, star_radius, empty_material));
    }

    attempts++;
  }

  // Shooting star
  auto moving_sphere = make_shared<sphere>(point3(300, 520, 2000), point3(600, 510, 6000), 5, make_shared<diffuse_light>(shooting_star, 30.0));
  world_objects.add(moving_sphere);
  lights.add(moving_sphere);

  // Cubemap sphere and glass sphere
  // Makes a portal looking object
  std::array<std::string, 6> cubemap_faces = {
      "/Users/andrewvick/Coms336/Project/src/Textures/CubeMaps/gum_trees_4k/px.hdr", // +X
      "/Users/andrewvick/Coms336/Project/src/Textures/CubeMaps/gum_trees_4k/nx.hdr", // -X
      "/Users/andrewvick/Coms336/Project/src/Textures/CubeMaps/gum_trees_4k/py.hdr", // +Y
      "/Users/andrewvick/Coms336/Project/src/Textures/CubeMaps/gum_trees_4k/ny.hdr", // -Y
      "/Users/andrewvick/Coms336/Project/src/Textures/CubeMaps/gum_trees_4k/pz.hdr", // +Z
      "/Users/andrewvick/Coms336/Project/src/Textures/CubeMaps/gum_trees_4k/nz.hdr"  // -Z
  };
  auto cubemap_texture = make_shared<cube_map_texture>(cubemap_faces);
  // center the sphere texture, can change values to move which part of the cubemap is visible
  // +y will show more ground, -y will show more sky
  cubemap_texture->set_sphere_center(point3(255, 50, 400)); 
  auto cubemap_material = make_shared<lambertian>(cubemap_texture);
  auto inner_sphere = make_shared<sphere>(point3(255, 90, 400), 70, cubemap_material);
  auto outer_sphere = make_shared<sphere>(point3(255, 90, 400), 85, glass_portal_mat);

  world_objects.add(inner_sphere);
  world_objects.add(outer_sphere);

  // Load and convert mesh to triangles
  add_mesh_to_scene("/Users/andrewvick/Coms336/Project/src/Textures/meshes/Flint2.obj", world_objects, 220, vec3(520, 0, 300));
  add_mesh_to_scene("/Users/andrewvick/Coms336/Project/src/Textures/meshes/table2.obj", world_objects, 90, vec3(70, 30, 277.5));
  add_mesh_to_scene("/Users/andrewvick/Coms336/Project/src/Textures/meshes/chess2.obj", world_objects, 90, vec3(70, 75, 277.5));

  // Cornell box sides
  world_objects.add(make_shared<quad>(point3(0, 0, 555), vec3(0, 0, -555), vec3(0, 555, 0), make_shared<lambertian>(red_tex, wall_material))); // right
  world_objects.add(make_shared<quad>(point3(0, 555, 0), vec3(555, 0, 0), vec3(0, 0, 555), make_shared<lambertian>(blue_tex, wall_material))); // ceiling
  world_objects.add(make_shared<quad>(point3(0, 0, 555), vec3(555, 0, 0), vec3(0, 0, -555), checker_material));                                // floor

  // Dimensions of wall and window
  double wall_width = 555;
  double wall_height = 555;
  double window_width = 400;
  double window_height = 400;
  double window_x_offset = (wall_width - window_width) / 2;   
  double window_y_offset = (wall_height - window_height) / 2; 

  // Dimensions of mirror
  double mirror_width = 200;
  double mirror_height = 200;
  double mirror_x_offset = (wall_width - mirror_width) / 2;   
  double mirror_y_offset = (wall_height - mirror_height) / 2; 

  // Left Wall (4 surrounding quads + mirror center)
  world_objects.add(make_shared<quad>(point3(555, 0, 0), vec3(0, 0, mirror_x_offset), vec3(0, mirror_y_offset, 0), make_shared<lambertian>(purple_tex, wall_material)));                                                                          // Bottom portion of left wall
  world_objects.add(make_shared<quad>(point3(555, mirror_y_offset + mirror_height, 0), vec3(0, 0, wall_width - mirror_x_offset), vec3(0, wall_height - mirror_y_offset - mirror_height, 0), make_shared<lambertian>(purple_tex, wall_material))); // Top portion of left wall
  world_objects.add(make_shared<quad>(point3(555, mirror_y_offset, 0), vec3(0, 0, mirror_x_offset), vec3(0, mirror_height, 0), make_shared<lambertian>(purple_tex, wall_material)));                                                              // Left portion of left wall
  world_objects.add(make_shared<quad>(point3(555, 0, mirror_width + mirror_x_offset), vec3(0, 0, wall_width - (mirror_width + mirror_x_offset)), vec3(0, 555, 0), make_shared<lambertian>(purple_tex, wall_material)));                           // Right portion of left wall
  world_objects.add(make_shared<quad>(point3(555, 0, mirror_x_offset), vec3(0, 0, mirror_width), vec3(0, mirror_height + mirror_y_offset, 0), aluminum));                                                                                         // Mirror

  // Back Wall (4 surrounding quads + glass center)
  world_objects.add(make_shared<quad>(point3(555, 0, 555), vec3(-555, 0, 0), vec3(0, window_y_offset, 0), make_shared<lambertian>(white_tex, wall_material)));                                                                    // Bottom portion of back wall
  world_objects.add(make_shared<quad>(point3(555, window_y_offset + window_height, 555), vec3(-wall_width, 0, 0), vec3(0, wall_height - window_y_offset - window_height, 0), make_shared<lambertian>(white_tex, wall_material))); // Top portion of back wall
  world_objects.add(make_shared<quad>(point3(555, window_y_offset, 555), vec3(-window_x_offset, 0, 0), vec3(0, window_height, 0), make_shared<lambertian>(white_tex, wall_material)));                                            // Left portion of back wall
  world_objects.add(make_shared<quad>(point3(555 - window_x_offset - window_width, window_y_offset, 555), vec3(-180, 0, 0), vec3(0, window_height, 0), make_shared<lambertian>(white_tex, wall_material)));                       // Right portion of back wall
  world_objects.add(make_shared<quad>(point3(555 - window_x_offset, window_y_offset, 555), vec3(-window_width, 0, 0), vec3(0, window_height, 0), glass_material));                                                                // Window


  // Room Light
  world_objects.add(make_shared<quad>(point3(213, 554, 227), vec3(130, 0, 0), vec3(0, 0, 105), light));
  lights.add(make_shared<quad>(point3(213, 554, 227), vec3(130, 0, 0), vec3(0, 0, 105), empty_material));

  // Smoke around light
  world_objects.add(make_shared<constant_medium>(make_shared<sphere>(point3(277.5, 700, 277.5), 200, white_material), 0.0005, color(0.4, 0.4, 0.4)));

  // world_objects.add(make_shared<sphere>(point3(277.5, 200, 277.5), 20, light));
  // lights.add(make_shared<sphere>(point3(277.5, 200, 277.5), 20, empty_material));

world.add(make_shared<bvh_node>(world_objects));

camera cam;

cam.aspect_ratio = 1.0;
cam.image_width = 1440;
cam.vfov = 40;
cam.samples_per_pixel = 10000;
cam.max_depth = 50;
cam.set_background(make_shared<solid_color>(color(0, 0, 0)), false);

cam.lookfrom = point3(278, 278, -750);
cam.lookat = point3(278, 278, 0);
// cam.lookfrom = point3(150, 145, 100);
// cam.lookat = point3(70, 75, 277.5);
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

void book_final_scene(int image_width, int samples_per_pixel, int max_depth)
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
    final_class_render();
    break;
  case 8:
    cornell_smoke();
    break;
  case 9:
    book_final_scene(800, 10000, 40);
    break;
  case 10:
    book_final_scene(400, 250, 4);
    break;
  }

  return 0;
}
