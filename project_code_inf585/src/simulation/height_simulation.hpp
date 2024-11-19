#pragma once

#include "cgp/cgp.hpp"
#include "boundary.hpp"
//#include "simulation.hpp"

using namespace cgp;

struct Line {
    std::vector<vec2> points;
};

const vec2 invalid_point(-1, -1); //define an invalid point value

//une fonction pour calculer la divergence d'un champ de vecteur
void compute_divergence(const cgp::grid_2D<cgp::vec2>& velocity, cgp::grid_2D<float>& divergence);

//SHALLOW WATER SIMULATION
void update_height(grid_2D<float>& height, grid_2D<float>& height_previous, grid_2D<float>& divergence, float dt, boundary_condition boundary);
void update_velocity(grid_2D<vec2>& velocity, grid_2D<float>& height, float dt, float g);
void set_reflective_boundary_height(cgp::grid_2D<float>& height);
void set_reflective_boundary_velocity(cgp::grid_2D<cgp::vec2>& velocity);

//visualization:
//1st: the height as color
vec3 height_to_color(float height);

//generating waves
//std::vector<vec3> detection(std::vector<vec3> points, grid_2D<vec3> height_vector);
vec2 compute_gradient(const grid_2D<float>& height, int x, int y);
void detect_wave_fronts(const grid_2D<float>& height, const grid_2D<vec2>& velocity, grid_2D<int>& wave_front, float tH);

bool is_inside(int x, int y, int2 dim);
void enlarge_point_set(const grid_2D<int>& Ps, grid_2D<int>& Pb, float pd, const grid_2D<vec2>& velocity);

void flood_fill(const grid_2D<int>& Pb, grid_2D<int>& visited, int x, int y, std::vector<vec2>& segment);
void segment_enlarged_set(const grid_2D<int>& Pb, std::vector<std::vector<vec2>>& segments);

//constructing a line of points
void construct_line_from_segment(const std::vector<vec2>& segment, std::vector<Line>& lines, const grid_2D<float>& height);

//updating with time the wave front
void update_wave_fronts(std::vector<Line>& waveLines, const grid_2D<float>& height_field, float dt, float g, float tH, float dx);
vec2 project_point(const vec2& startPoint, const grid_2D<float>& height_field, float c, float dt);


//TYPE 2
vec2 project_to_maximum_height(const vec2& startPoint, const grid_2D<float>& height_field, float searchRadius);
vec2 project_point_adjusted(const vec2& startPoint, const grid_2D<float>& heightField, float c, float dt, float tH);

inline bool is_diff(vec2 u, vec2 v) {
    return (u.x != v.x || u.y != v.y);
};

void adaptive_resample(std::vector<vec2>& line, float dx);


//FOR VISUALIZATION
numarray<vec3> convert_line_to_numarray3D(const Line& line, const grid_2D<float>& height_field);
