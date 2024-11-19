#pragma once

#include "cgp/cgp.hpp"
#include "simulation/height_simulation.hpp"


void initialize_density_visual(cgp::mesh_drawable& density_visual, size_t N);
void initialize_grid(cgp::curve_drawable& grid_visual, size_t N);
//void initialize_grid(cgp::curve_drawable& floor_visual, cgp::curve_drawable& left_wall_visual,cgp:: curve_drawable& right_wall_visual, size_t N, float wall_height);

void update_velocity_visual(cgp::curve_drawable& velocity_visual, cgp::numarray<cgp::vec3>& velocity_grid_data, cgp::grid_2D<cgp::vec2> const& velocity, float scale);
void mouse_velocity_to_grid(cgp::grid_2D<cgp::vec2>& velocity, cgp::vec2 const& mouse_velocity, cgp::mat4 const& P_inv, cgp::vec2 const& p_mouse);

//added
void initialize_height_visual(cgp::mesh_drawable& height_visual, size_t N, const cgp::grid_2D<float>& height_field);
void update_height_visual(cgp::mesh_drawable& height_visual, size_t N, cgp::numarray<cgp::vec3> height_vector, const cgp::grid_2D<float>& height_field);

//TODO fix complexity: too computation heavy
void visualize_wave_fronts(std::vector<Line>& waveLines, const grid_2D<float>& height_field, const environment_generic_structure& environment);
void visualize_wave_fronts_mod(std::vector<Line>& waveLines, const grid_2D<float>& height_field, environment_generic_structure const& environment, std::vector<curve_drawable>& drawables);
void initialize_wave_visual(std::vector<curve_drawable>& waveFrontDrawables, size_t N, numarray<vec3> positions);
