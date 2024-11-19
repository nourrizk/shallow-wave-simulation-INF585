#include "helper.hpp"


using namespace cgp;



void initialize_density_visual(mesh_drawable& density_visual, size_t N)
{
	float const dx = 2.0f/(N-1.0f);
	density_visual.initialize_data_on_gpu(mesh_primitive_quadrangle({-1-dx/2,-1-dx/2,0},{1+dx/2,-1-dx/2,0},{1+dx/2,1+dx/2,0}, {-1-dx/2, 1+dx/2, 0}) );
	
	density_visual.material.phong = {1,0,0,1};
	density_visual.material.color = {1,1,1};
	density_visual.material.texture_settings.inverse_v = false;
}

void initialize_height_visual(mesh_drawable& height_visual, size_t N, const grid_2D<float>& height_field) {

    std::vector<vec3> positions;
    std::vector<vec3> normals;
    std::vector<uint3> indices;
    std::vector<vec3> colors;

    float const dx = 2.0f / (N - 1.0f);
    float height_min = std::numeric_limits<float>::max();
    float height_max = -std::numeric_limits<float>::max();

    // Find the minimum and maximum height to scale the colors
    for (size_t y = 0; y < N; ++y) {
        for (size_t x = 0; x < N; ++x) {
            height_min = std::min(height_min, height_field(x, y));
            height_max = std::max(height_max, height_field(x, y));
        }
    }

    // Generate positions based on the height field
    for (size_t y = 0; y < N; ++y) {
        for (size_t x = 0; x < N; ++x) {
            float local_height = height_field(x, y);
            //std::cout << local_height << std::endl;
            positions.push_back({ -1.0f + dx * x, -1.0f + dx * y, local_height });

            // Generate color based on the height
            float normalized_height = (local_height - height_min) / (height_max - height_min);
            vec3 color = height_to_color(normalized_height); // height to color
            colors.push_back(color);
        }
    }

    // Generate normals (initially just upwards)
    normals.assign(N * N, { 0,0,1 });

    // Generate indices for two triangles per grid cell
    for (size_t y = 0; y < N - 1; ++y) {
        for (size_t x = 0; x < N - 1; ++x) {
            size_t idx = y * N + x;
            indices.push_back({ idx, idx + 1, idx + N });
            indices.push_back({ idx + 1, idx + N + 1, idx + N });
        }
    }

    // Update normals based on the positions to have smooth shading
    for (size_t y = 1; y < N - 1; ++y) {
        for (size_t x = 1; x < N - 1; ++x) {
            vec3& n = normals[y * N + x];
            const vec3& p = positions[y * N + x];

            vec3 p_left = positions[y * N + x - 1];
            vec3 p_right = positions[y * N + x + 1];
            vec3 p_down = positions[(y - 1) * N + x];
            vec3 p_up = positions[(y + 1) * N + x];

            vec3 d_horiz = p_right - p_left;
            vec3 d_vert = p_up - p_down;
            n = normalize(cross(d_horiz, d_vert));
        }
    }

    mesh terrain_mesh;
    terrain_mesh.position = positions;
    terrain_mesh.normal = normals;
    terrain_mesh.connectivity = indices;
    terrain_mesh.color = colors;

    terrain_mesh.fill_empty_field(); // Compute missing normals

    height_visual.initialize_data_on_gpu(terrain_mesh);
    //height_visual.texture.load_and_initialize_texture_2d_on_gpu(convert(image_load_jpg("waves.jpg")), height_vector); // optional texture

    height_visual.material.phong = { 1,0,0,1 };
    height_visual.material.color = { 0,0, 0.8f };
    height_visual.material.texture_settings.inverse_v = false;
}


void update_height_visual(mesh_drawable& height_visual, size_t N, numarray<vec3> height_vector, const grid_2D<float>& height_field) {
    //retrieve the current vertex positions from the mesh_drawable    
    //numarray<vec3>& vertices = height_vector;
    //numarray<vec3>& vertices = height_visual.vao.data.vertex_buffer;  
    //// itterate on the height field and update the z coord of each vertex
    //size_t const N = height_field.dimension.x;
    //for (size_t y = 0; y < N; ++y) {
    //    for (size_t x = 0; x < N; ++x) {
    //        // need the index in the vertex array
    //        size_t index = y * N + x;
    //        // update the z value based on the height field
    //        vertices[index].z = height_field(x, y);
    //    }
    //}
    //send the updated vertex positions to the GPU
    //height_visual.vbo_position.update(vertices);

    std::vector<vec3> positions;
    std::vector<vec3> normals;
    //std::vector<uint3> indices;   //no need en fait, pas de changement de connectivité
    std::vector<vec3> colors;

    float const dx = 2.0f / (N - 1.0f);
    float height_min = std::numeric_limits<float>::max();
    float height_max = -std::numeric_limits<float>::max();

    // Find the minimum and maximum height to scale the colors
    for (size_t y = 0; y < N; ++y) {
        for (size_t x = 0; x < N; ++x) {
            height_min = std::min(height_min, height_field(x, y));
            height_max = std::max(height_max, height_field(x, y));
        }
    }

    // Generate positions based on the height field
    for (size_t y = 0; y < N; ++y) {
        for (size_t x = 0; x < N; ++x) {
            float local_height = height_field(x, y);
            positions.push_back({ -1.0f + dx * x, -1.0f + dx * y, local_height });

            // Generate color based on the height
            float normalized_height = (local_height - height_min) / (height_max - height_min);
            vec3 color = height_to_color(normalized_height); // height to color
            colors.push_back(color);
        }
    }

    normals.assign(N * N, { 0,0,1 });

    for (size_t y = 1; y < N - 1; ++y) {
        for (size_t x = 1; x < N - 1; ++x) {
            vec3& n = normals[y * N + x];
            const vec3& p = positions[y * N + x];

            vec3 p_left = positions[y * N + x - 1];
            vec3 p_right = positions[y * N + x + 1];
            vec3 p_down = positions[(y - 1) * N + x];
            vec3 p_up = positions[(y + 1) * N + x];

            vec3 d_horiz = p_right - p_left;
            vec3 d_vert = p_up - p_down;
            n = normalize(cross(d_horiz, d_vert));
        }
    }


    height_visual.vbo_position.update(positions);
    height_visual.vbo_color.update(colors);
    height_visual.vbo_normal.update(normals);

}



//Grid consiste en 1 plan uniquement
void initialize_grid(curve_drawable& grid_visual, size_t N)
{

    numarray<vec3> edges;
	float const e = 1e-4f;
    float const dx = 2.0f/(N-1.0f);
    for(int kx=0; kx<=N; ++kx) {
        float const x  = kx*dx-1;
        edges.push_back( vec3(x-dx/2, -dx/2-1, e) );
        edges.push_back( vec3(x-dx/2, 1+dx/2, e) );
    }
    for(int ky=0; ky<=N; ++ky) {
        float const y  = ky*dx-1;
        edges.push_back( vec3(-dx/2-1, y-dx/2, e) );
        edges.push_back( vec3(1+dx/2, y-dx/2, e) );
    }

    grid_visual.initialize_data_on_gpu(edges);
}


//void initialize_grid(curve_drawable& floor_grid_visual,
//    curve_drawable& left_wall_visual,
//    curve_drawable& right_wall_visual,
//    size_t N, float wall_height)
//{
//    numarray<vec3> floor_edges;
//    numarray<vec3> left_wall_edges;
//    numarray<vec3> right_wall_edges;
//    float const e = 1e-4f; // slight elevation pr pas que le grid soit dans le plan
//    float const dx = 2.0f / (N - 1.0f);
//
//    // Generate floor grid edges
//    for (int kx = 0; kx < N; ++kx) {
//        for (int ky = 0; ky < N; ++ky) {
//            float const x = kx * dx - 1;
//            float const y = ky * dx - 1;
//            if (kx < N - 1) { // Line segment in x direction
//                floor_edges.push_back(vec3(x, y, e));
//                floor_edges.push_back(vec3(x + dx, y, e));
//            }
//            if (ky < N - 1) { // Line segment in y direction
//                floor_edges.push_back(vec3(x, y, e));
//                floor_edges.push_back(vec3(x, y + dx, e));
//            }
//        }
//    }
//
//    // Generate left wall grid edges
//    for (int ky = 0; ky < N; ++ky) {
//        float const y = ky * dx - 1;
//        // Vertical line segments
//        left_wall_edges.push_back(vec3(-1, y, e));
//        left_wall_edges.push_back(vec3(-1, y, e + wall_height));
//
//        // Horizontal line segments
//        if (ky < N - 1) {
//            left_wall_edges.push_back(vec3(-1, y, e + ky * wall_height / (N - 1.0f)));
//            left_wall_edges.push_back(vec3(-1, y + dx, e + (ky + 1) * wall_height / (N - 1.0f)));
//        }
//    }
//
//    // Generate right wall grid edges
//    for (int ky = 0; ky < N; ++ky) {
//        float const y = ky * dx - 1;
//        // Vertical line segments
//        right_wall_edges.push_back(vec3(1, y, e));
//        right_wall_edges.push_back(vec3(1, y, e + wall_height));
//
//        // Horizontal line segments
//        if (ky < N - 1) {
//            right_wall_edges.push_back(vec3(1, y, e + ky * wall_height / (N - 1.0f)));
//            right_wall_edges.push_back(vec3(1, y + dx, e + (ky + 1) * wall_height / (N - 1.0f)));
//        }
//    }
//
//    // Initialize drawables for the grids
//    floor_grid_visual.initialize_data_on_gpu(floor_edges);
//    left_wall_visual.initialize_data_on_gpu(left_wall_edges);
//    right_wall_visual.initialize_data_on_gpu(right_wall_edges);
//}



void update_velocity_visual(curve_drawable& velocity_visual, numarray<vec3>& velocity_grid_data, grid_2D<vec2> const& velocity, float scale)
{
	int const N = int(velocity.dimension.x);
	float const dL = 2.0f/(N-1.0f);
	float const lambda = 0.01f * scale;

	for(int kx=0; kx<N; ++kx){
		for(int ky=0; ky<N; ++ky){
			vec3 const p0 = {-1+kx*dL, -1+ky*dL, 1e-4f};
			size_t const offset = velocity.index_to_offset(kx,ky);
			velocity_grid_data[2*offset+0] = p0;
			velocity_grid_data[2*offset+1] = p0 + lambda*vec3(velocity(kx,ky),0.0f);
		}
	}

	velocity_visual.vbo_position.update(velocity_grid_data);
}

void mouse_velocity_to_grid(grid_2D<vec2>& velocity, vec2 const& mouse_velocity, mat4 const& P_inv, vec2 const& p_mouse)
{
    size_t const N = velocity.dimension.x;
	float const L = 2.0f/(N-1.0f);

	vec2 const picked = (P_inv*vec4(p_mouse,0,1)).xy();
	vec2 const p0 = vec2(-1-L/2,-1-L/2); // grid extremity
	vec2 const p1 = vec2( 1+L/2, 1+L/2);
	int const x = std::floor( N*(picked.x-p0.x)/(p1.x-p0.x) );
	int const y = std::floor( N*(picked.y-p0.y)/(p1.y-p0.y) );
	for (int dx = -5; dx < 5; ++dx)
	{
		for (int dy = -5; dy < 5; ++dy)
		{
			int const xc = x+dx;
			int const yc = y+dy;

			if(xc>1 && yc>1 && xc<int(N-2) && yc<int(N-2)) {
				// Set mouse speed to the corresponding entry of the velocity
				float const dist = norm(picked-vec2{-1-L/2+xc*L, -1-L/2+yc*L});
				float const weight = exp(-(dist*dist)/(0.05f*0.05f));

				//velocity(xc,yc) += 5.0f * weight * mouse_velocity;
                velocity(xc,yc) += 2.0f * weight * mouse_velocity;
			}
		}
	}
}



void visualize_wave_fronts(std::vector<Line>& waveLines, const grid_2D<float>& height_field, const environment_generic_structure& environment) {
    // iterate over each waveLine to convert and visualize
    for (const Line& line : waveLines) {
        numarray<vec3> positions = convert_line_to_numarray3D(line, height_field);

        curve_drawable drawable;
        drawable.initialize_data_on_gpu(positions); 
        drawable.color = vec3(0.0f, 1.0f, 0.0f);    //debugging
        drawable.display_type = curve_drawable_display_type::Curve;

        // render the drawable
        draw(drawable, environment);
    }
}

void visualize_wave_fronts_mod(std::vector<Line>& waveLines, const grid_2D<float>& height_field, environment_generic_structure const& environment, std::vector<curve_drawable>& waveFronts) {
    for (size_t i = 0; i < waveLines.size(); ++i) {
        const Line& line = waveLines[i];
        numarray<vec3> positions = convert_line_to_numarray3D(line, height_field);

        // check if we have an existing drawable for this line; if not, initialize one
        if (i >= waveFronts.size() && i < 5) {      //avoir simultanément 5 vagues max
            curve_drawable drawable;
            drawable.initialize_data_on_gpu(positions);
            drawable.color = vec3(0.0f, 1.0f, 0.0f); //debugging
            drawable.display_type = curve_drawable_display_type::Curve;
            waveFronts.push_back(drawable);
        }
        else {
            // update existing drawable with new positions
            waveFronts[i].clear();
            waveFronts[i].vbo_position.update(positions);
            waveFronts[i].color = vec3(0.0f, 1.0f, 0.0f); //debugging
        }

        // render the drawable
        draw(waveFronts[i], environment);
    }

    // retirer les drawables en plus?
}

void initialize_wave_visual(std::vector<curve_drawable>& waveFrontDrawables, size_t N, numarray<vec3> positions) {
    //N est le nombre de vagues
    for (size_t i = 0; i < N; ++i) {
		curve_drawable drawable;
		drawable.initialize_data_on_gpu(positions);
		drawable.color = { 0,0,0 };
		drawable.display_type = curve_drawable_display_type::Curve;
		waveFrontDrawables.push_back(drawable);
	}
}