#include "scene.hpp"


#include "helper/helper.hpp"
#include "simulation/simulation.hpp"
#include "simulation/height_simulation.hpp"

using namespace cgp;

const float M_PI = 3.14159265358979323846f;


//fonction pour initialiser une fonction sinusoidale
static void initialize_height_with_wave(grid_2D<float>& height, float amplitude, float wavelength, float phase, float time) {
	size_t const N = height.dimension.x;
	for (size_t y = 0; y < N; ++y) {
		for (size_t x = 0; x < N; ++x) {
			// dynamically adjust the wave phase with time
			float wave = 0.2; //+amplitude * std::sin(2 * M_PI * x / wavelength + phase + time);
			height(x, y) += wave;
		}
	}
	set_boundary(height);
}

static void initialize_height_with_wave(grid_2D<float>& height, float amplitude, float wavelength, float phase) {
	size_t const N = height.dimension.x;
	for (size_t y = 0; y < N; ++y) {
		for (size_t x = 0; x < N; ++x) {
			// Simple wave equation, adjust as necessary for your simulation
			float wave = amplitude * std::sin(2 * M_PI * x / wavelength + phase);
			height(x, y) = wave;
		}
	}
	set_boundary(height);
}

void scene_structure::initialize()
{
	camera_projection = camera_projection_orthographic{ -1.1f, 1.1f, -1.1f, 1.1f, -10, 10, window.aspect_ratio()};
	camera_control.initialize(inputs, window); // Give access to the inputs and window global state to the camera controler
	//camera_control.look_at({ 0.0f, 0.0f, 3.0f }, {0,0,0}, {0,0,1});
	camera_control.look_at({ 0.8f, -1.2f, 0.4f }, { 0,0,0 }, { 0,0,1 });
	global_frame.initialize_data_on_gpu(mesh_primitive_frame());

	// Initialize the shapes of the scene
	// ***************************************** //
	initialize_fields(gui.density_type);
	int const N = velocity.dimension.x;
	/*initialize_density_visual(density_visual, N);
	density_visual.texture.initialize_texture_2d_on_gpu(density);*/

	//grid
	initialize_grid(grid_visual, N);
	grid_visual.color = { 0.8,0,0 };

	/*initialize_grid(floor_visual, left_wall_visual, right_wall_visual, N, 1.0f);
	floor_visual.color = { 0,0,0.5 };
	right_wall_visual.color = { 0,0,0.5 };
	left_wall_visual.color = { 0,0,0.5 };*/

	velocity_grid_data.resize(2 * N * N);
	velocity_visual.initialize_data_on_gpu(velocity_grid_data);
	velocity_visual.color = vec3(0, 0, 0);

	grid_visual.display_type = curve_drawable_display_type::Segments;
	/*floor_visual.display_type = curve_drawable_display_type::Segments;
	right_wall_visual.display_type = curve_drawable_display_type::Segments;
	left_wall_visual.display_type = curve_drawable_display_type::Segments;*/

	velocity_visual.display_type = curve_drawable_display_type::Segments;

	//added
	initialize_height_visual(height_visual, N, height_field);
	
	//adding color 
	height_visual.texture.initialize_texture_2d_on_gpu(height_color);
	//TODO add texture
	
	//initialize the wave front visual
	size_t N_waves = 2;
	numarray<vec3> wave_positions_fake;
	wave_positions_fake.resize(10);
	wave_positions_fake.fill({ 0,0,0 });
	initialize_wave_visual(waveFrontDrawables, N_waves, wave_positions_fake);
}

void scene_structure::simulate(float dt)
{
	velocity_previous = velocity;
	density_previous = density;
	//added
	height_previous = height_field;


	//added
	// SHALLOW WATER SIMULATION
	// Step 1 : Compute the divergence of the velocity field
	compute_divergence(velocity, divergence);
	// Step 2: Update height field based on continuity equation
	height_field.fill(0.00f);
	update_height(height_field, height_previous, divergence, dt, copy); //height_previous =height_field;
	// Step 3: Update velocity field based on momentum equation
	update_velocity(velocity, height_field, dt, 9.8f); // avec g=9.8 m/s^2
	velocity_previous = velocity;


	//TRACKING WAVES
	// Detection of Ps based on criteria
	float pH = 0.3f;
	//float dx = 2.0f / height_field.dimension[0];
	// updated the tH depending on what we want
	int grid_size = height_field.dimension[0];
	float dx = 1.0f / grid_size;	//one grid unit selected
	float tH = pH * g * dt * dx;

	//check: ok
	detect_wave_fronts(height_field, velocity, wave_front, tH);
	//std::cout << wave_front << std::endl;

	//check: bof
	//prend trop de points
	enlarge_point_set(wave_front, Pb, 2*dx, velocity); // delta_x is grid spacing
	//problème détecte toute la vague pas que la crete
	//std::cout << Pb << std::endl;

	//for debugging purposes
	std::vector< std::vector<vec2>> one_segment;
	if (segments.size() != 0) one_segment.push_back(segments[0]);

	//check: 
	//computation , testing with wave_front only
	//Pb = wave_front;
	segments.clear();
	segment_enlarged_set(Pb, segments);
	//testing
	/*if (segments.size() != 0){
		for (int i = 0; i < 10; i++)
			std::cout << segments[0][i] << std::endl;
		std::cout << 'test' << std::endl;
	}*/
		
	//debugging: less computation
	//segment_enlarged_set(Pb, one_segment);


	// TODO : THIS CODE IS NEEDED BUT TOO COMPUTATION HEAVY ON MY MACHINE
	if (!segments.empty() && !touch) {
		waveLines.clear();
		for (const auto& segment : segments) {
			construct_line_from_segment(segment, waveLines, height_field);
		}
		update_wave_fronts(waveLines, height_field, dt, g, tH, dx);
	}
	
	//add a condition to be safe
	/*for (const auto& segment : segments) {
		construct_line_from_segment(segment, waveLines, height_field);
	}*/
	//update_wave_fronts(waveLines, height_field, dt, g, tH, dx);

	////set_reflective_boundary_height(height_field);
	////set_reflective_boundary_velocity(velocity);
}

//void scene_structure::initialize_density(density_type_structure density_type, size_t N)
//{	
//	density.resize(N, N);
//	density.fill({ 1,1,1 });
//	if (density_type == density_texture) {
//		convert(image_load_png(project::path+"assets/texture.png"), density);
//		//TODO
//	}
//
//	if (density_type == view_velocity_curl) {
//		density.resize(N, N); density.fill({ 1,1,1 });
//	}
//
//	density_previous = density;
//}

void scene_structure::initialize_fields(density_type_structure density_type)
{
	size_t const N = 60;
	velocity.resize(N, N); velocity.fill({ 0,0 }); velocity_previous = velocity;
	//initialize_density(density_type, N);
	divergence.clear(); divergence.resize(N, N);
	gradient_field.clear(); gradient_field.resize(N, N);

	//added 
	// Initialize height field
	height_field.resize(N, N); 
	//height_field.fill(0.02f); // Start with a flat surface for testing
	//initial time
	float time = glfwGetTime();
	initialize_height_with_wave(height_field, 0.1f, 40.f, 0.0f, time);
	height_previous = height_field;

	// initialize the height vector and color based on the height field
	height_vector.resize(N* N);
	height_color.resize(N, N);

	float height_min = std::numeric_limits<float>::max();
	float height_max = -std::numeric_limits<float>::max();

	// Find the minimum and maximum height to scale the colors
	for (size_t y = 0; y < N; ++y) {
		for (size_t x = 0; x < N; ++x) {
			height_min = std::min(height_min, height_field(x, y));
			height_max = std::max(height_max, height_field(x, y));
		}
	}

	// Fill the height vector and color
	for (size_t y = 0; y < N; ++y) {
		for (size_t x = 0; x < N; ++x) {
			// Set the position
			height_vector[x, y] = { x, y, height_field(x,y) };

			// Determine the color based on height
			float normalized_height = (height_field(x, y) - height_min) / (height_max - height_min); //on calcul pr chaque frame max et min
			vec3 color = height_to_color(normalized_height); 
			height_color(x, y) = color;
		}
	}

	wave_front.resize(N, N);
	Pb.resize(N, N);
	Pb.fill(0);

}
int display_count = 0;

void scene_structure::display_frame()
{
	// Set the light to the current position of the camera
	environment.light = camera_control.camera_model.position();
	
	timer.update();
	float const dt = 0.2f * timer.scale;
	simulate(dt);
	//density_visual.texture.update(density);
	//update_velocity_visual(velocity_visual, velocity_grid_data, velocity, gui.velocity_scaling);
	//draw(density_visual, environment);

	//added
	// Update the visualization with the new height field
	size_t const N = height_field.dimension.x;
	update_height_visual(height_visual, N, height_vector, height_field); // TODO
	// render the updated height simulation
	draw(height_visual, environment);
	if (gui.display_wireframe)
		draw_wireframe(height_visual, environment);

	//testing depth issues
	//glDisable(GL_DEPTH_TEST);
	//std::cout << waveLines[0].points[0] << std::endl;
	
	/*if(display_count/10 == 0)
		visualize_wave_fronts(waveLines, height_field, environment);
	display_count++;*/

	//computation heavy
	/*if (display_count/10 == 0) {
		visualize_wave_fronts_mod(waveLines, height_field, environment, waveFrontDrawables);
	}
		display_count++;*/
	//visualize_wave_fronts_mod(waveLines, height_field, environment, waveFrontDrawables);
	//glEnable(GL_DEPTH_TEST);


	if (gui.display_grid) {
		draw(grid_visual, environment);
		/*draw(floor_visual, environment);
		draw(right_wall_visual, environment);
		draw(left_wall_visual, environment);*/
	}

	if (gui.display_velocity)
		draw(velocity_visual, environment);

}

void scene_structure::display_gui()
{
	ImGui::SliderFloat("Timer scale", &timer.scale, 0.01f, 4.0f, "%0.2f");
	ImGui::Checkbox("Grid", &gui.display_grid); ImGui::SameLine();
	ImGui::Checkbox("WireFrame", &gui.display_wireframe); 
	ImGui::Checkbox("Velocity", &gui.display_velocity);
	ImGui::SliderFloat("Velocity scale", &gui.velocity_scaling, 0.1f, 1.0f, "0.2f");

	if (gui.display_velocity == false)
		velocity.fill(vec2(0,0));
}

void scene_structure::mouse_move_event()
{
	vec2 const& p = inputs.mouse.position.current;
	if (inputs.mouse.click.left) {
		velocity_track.add(vec3(p, 0.0f), timer.t);
		mouse_velocity_to_grid(velocity, velocity_track.velocity.xy(), camera_projection.matrix_inverse(), p);
		touch = true; 
	}
	else {
		velocity_track.set_record(vec3(p, 0.0f), timer.t);
		touch = false;
	}
}
void scene_structure::mouse_click_event()
{
	camera_control.action_mouse_click(environment.camera_view);
}
void scene_structure::idle_frame()
{
	camera_control.idle_frame(environment.camera_view);
}

