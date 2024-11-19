#include "height_simulation.hpp"

using namespace cgp;

void compute_divergence(const cgp::grid_2D<cgp::vec2>& velocity, cgp::grid_2D<float>& divergence)
{
    int2 dim = divergence.dimension;
    for (int y = 1; y < dim[1] - 1; ++y) {
        for (int x = 1; x < dim[0] - 1; ++x) {
            //std::cout << "dim " << x << " , " << dim[0] << std::endl;
            float div = (velocity(x + 1, y).x - velocity(x - 1, y).x) / 2.0 +
                (velocity(x, y + 1).y - velocity(x, y - 1).y) / 2.0;
            divergence(x, y) = div;
        }
    }
    set_boundary(divergence);
}


void update_height(grid_2D<float>& height, grid_2D<float>& height_previous, grid_2D<float>& divergence, float dt, boundary_condition boundary)
{
    size_t const N = height.dimension.x;
    for (size_t y = 0; y < N; ++y) {
        for (size_t x = 0; x < N; ++x) {
            height(x, y) = height_previous(x, y) - dt * divergence(x, y);
        }
    }
    // Set boundary conditions for height
    set_boundary(height);
}


void update_velocity(grid_2D<vec2>& velocity, grid_2D<float>& height, float dt, float g)
{
    size_t const N = velocity.dimension.x;
    for (size_t y = 0; y < N; ++y) {
        for (size_t x = 0; x < N; ++x) {
            if (x > 0 && x < N - 1 && y > 0 && y < N - 1) { // Avoid boundary cells for simplicity
                float gradX = (height(x + 1, y) - height(x - 1, y)) * 0.5f;
                float gradY = (height(x, y + 1) - height(x, y - 1)) * 0.5f;
                velocity(x, y).x -= dt * g * gradX;
                velocity(x, y).y -= dt * g * gradY;
            }
        }
    }
    // Set boundary conditions for velocity
    set_boundary(velocity);
}

vec3 height_to_color(float normalized_height) {
    // Ensure that normalized_height is clamped between 0.0 and 1.0
    normalized_height = std::max(0.0f, std::min(1.0f, normalized_height));
    //light blue for the hightest points
    vec3 light_blue = vec3{ 0.82f, 0.93f, 0.95f };
    //dark en bas
    vec3 dark_blue = vec3{ 0.0f, 0.0f, 0.5f };

    // Interpolate between light blue and dark blue based on height
    return light_blue * normalized_height + dark_blue * (1.0f - normalized_height);

}

//boundaries

void set_reflective_boundary_height(grid_2D<float>& height) {
    size_t const N = height.dimension.x;
    // Top and bottom boundaries
    for (size_t x = 0; x < N; ++x) {
        height(x, 0) = height(x, 1); // Reflect bottom
        height(x, N - 1) = height(x, N - 2); // Reflect top
    }
    // Left and right boundaries
    for (size_t y = 0; y < N; ++y) {
        height(0, y) = height(1, y); // Reflect left
        height(N - 1, y) = height(N - 2, y); // Reflect right
    }
}

void set_reflective_boundary_velocity(grid_2D<cgp::vec2>& velocity) {
    size_t const N = velocity.dimension.x;
    // Reflect velocities at the boundaries
    for (size_t x = 0; x < N; ++x) {
        velocity(x, 0).y = -velocity(x, 1).y; // Reflect bottom
        velocity(x, N - 1).y = -velocity(x, N - 2).y; // Reflect top
    }
    for (size_t y = 0; y < N; ++y) {
        velocity(0, y).x = -velocity(1, y).x; // Reflect left
        velocity(N - 1, y).x = -velocity(N - 2, y).x; // Reflect right
    }
}


//CREATING WAVES
//detecting the points
//TODO: s'assurer que a la bonne valeur height_vector???

//function to compute the gradient of the height field in x and y
//vec2 compute_gradient(const grid_2D<float>& height, int x, int y) {
//    vec2 gradient_h = {
//        (height(x + 1, y) - height(x - 1, y)) / 2.0f,
//        (height(x, y + 1) - height(x, y - 1)) / 2.0f
//    };
//    return gradient_h;
//}

vec2 compute_gradient(const grid_2D<float>& height, int x, int y) {
    float dx = 0.0f;
    float dy = 0.0f;

    // Handle boundary cases for x
    if (x > 0 && x < height.dimension.x - 1) {
        dx = (height(x + 1, y) - height(x - 1, y)) / 2.0f;
    }
    else if (x == 0) {
        dx = (height(x + 1, y) - height(x, y)) / 2.0f;
    }
    else if (x == height.dimension.x - 1) {
        dx = (height(x, y) - height(x - 1, y)) / 2.0f;
    }

    // Handle boundary cases for y
    if (y > 0 && y < height.dimension.y - 1) {
        dy = (height(x, y + 1) - height(x, y - 1)) / 2.0f;
    }
    else if (y == 0) {
        dy = (height(x, y + 1) - height(x, y)) / 2.0f;
    }
    else if (y == height.dimension.y - 1) {
        dy = (height(x, y) - height(x, y - 1)) / 2.0f;
    }

    return { dx, dy };
}


void detect_wave_fronts(const grid_2D<float>& height, const grid_2D<vec2>& velocity, grid_2D<int>& wave_front, float tH)
{
    int2 dim = height.dimension;
    for (int y = 1; y < dim[1] - 1; ++y) {
        for (int x = 1; x < dim[0] - 1; ++x) {
            vec2 gradient_h = compute_gradient(height, x, y);
            float grad_magnitude = norm(gradient_h);

            //check: ok
            if (grad_magnitude > tH) { // use the threshold to avoid division by zero
                float dot_product = dot(normalize(gradient_h), velocity(x, y));
                /*if (wave_front(x - 1, y) == 1 || wave_front(x + 1, y) == 1 )
                    wave_front(x, y) = 0;
                else*/
                    wave_front(x, y) = (grad_magnitude > tH && dot_product < 0) ? 1 : 0;
            }
            else {
                wave_front(x, y) = 0;
            }
        }
    }
}

bool is_inside(int x, int y, int2 dim) {
    return x >= 0 && x < dim[0] && y >= 0 && y < dim[1];
}


void enlarge_point_set(const grid_2D<int>& Ps, grid_2D<int>& Pb, float pd, const grid_2D<vec2>& velocity) {

    int2 dim_ps = Ps.dimension;
    for (int y = 0; y < dim_ps[1]; ++y) {
        for (int x = 0; x < dim_ps[0]; ++x) {
            if (!Ps(x, y)) continue;

            // Check surrounding points within a distance pd
            for (int dy = -pd; dy <= pd; ++dy) {
                for (int dx = -pd; dx <= pd; ++dx) {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (is_inside(nx, ny, dim_ps) && std::sqrt(dx * dx + dy * dy) <= pd) {
                        Pb(nx, ny) = 1; //pour true
                    }
                }
            }
        }
    }
}


void flood_fill(const grid_2D<int>& Pb, grid_2D<int>& visited, int x, int y, std::vector<vec2>& segment) {
    if (!is_inside(x, y, Pb.dimension) || visited(x, y) || !Pb(x, y)) return;

    visited(x, y) = true;
    segment.push_back(vec2(x, y));

    flood_fill(Pb, visited, x + 1, y, segment);
    flood_fill(Pb, visited, x - 1, y, segment);
    flood_fill(Pb, visited, x, y + 1, segment);
    flood_fill(Pb, visited, x, y - 1, segment);
}


void segment_enlarged_set(const grid_2D<int>& Pb, std::vector<std::vector<vec2>>& segments) {
    int2 dim_pb = Pb.dimension;
    grid_2D<int> visited(dim_pb[0], dim_pb[1]);

    for (int y = 0; y < dim_pb[1]; ++y) {
        for (int x = 0; x < dim_pb[0]; ++x) {
            if (!visited(x, y) && Pb(x, y)) {
                std::vector<vec2> segment;
                flood_fill(Pb, visited, x, y, segment);
                segments.push_back(segment);
            }
        }
    }
}


// The main function to construct lines from a segment
void construct_line_from_segment(const std::vector<vec2>& segment, std::vector<Line>& lines, const grid_2D<float>& height) {
    if (segment.empty()) return;

    Line line;
    vec2 current_point = segment.front(); // Starting with the first point of the segment
    line.points.push_back(current_point); // Add the starting point to the line

    for (size_t i = 1; i < segment.size(); ++i) {
        vec2 gradient = compute_gradient(height, current_point.x, current_point.y);

        // Normalize the gradient to get a unit vector for the tangent direction
        float norms = norm(gradient);
        vec2 tangent = (norms > 0) ? vec2(gradient.x / norms, gradient.y / norms) : vec2(0, 0);

        // Find the next point in the segment that is closest to the current direction
        auto next_point_it = std::min_element(segment.begin() + i, segment.end(), [&](const vec2& a, const vec2& b) {
            vec2 dir_to_a = a - current_point;
            vec2 dir_to_b = b - current_point;
            // Compare the lengths of the vectors from the current point to a and b, aligned with the tangent
            return norm(dir_to_a) < norm(dir_to_b);
            });

        if (next_point_it != segment.end()) {
            // move to the next point and add it to the line
            current_point = *next_point_it;
            line.points.push_back(current_point);
        }
    }

    // Add to the list of lines
    lines.push_back(line);
}

//modifs honeh mish wadhin TODO
void update_wave_fronts(std::vector<Line>& waveLines, const grid_2D<float>& height_field, float dt, float g, float tH, float dx) {
    for (Line& line : waveLines) {
        std::vector<vec2> newPoints;
        for (auto& point : line.points) {
            float H = height_field(point.x, point.y); 
            float c = std::sqrt(g * H);

            //TYPE 3:
            vec2 new_point = project_point_adjusted(point, height_field, c, dt, tH);
            //TODO: on hold / simplified
            //if (is_diff(new_point, invalid_point)) {
                newPoints.push_back(new_point);
            //}


            //TYPE 2: projection simplified
           // point = project_point(point, height_field, c, dt);

            //TYPE 1: sans projection
            //vec2 gradient = compute_gradient(height_field, point.x, point.y);
            //vec2 direction = -normalize(gradient); // Assuming you have a normalize function
            //// Basic movement update without considering bisection projection
            //point.x += direction.x * c * dt;
            //point.y += direction.y * c * dt;

            // tried diff types de projection along the grad
            // adaptive resampling
        }
        line.points = newPoints;
    }

    // remove or merge points based on new positions and conditions of the paper
    for (auto& line : waveLines) {
        adaptive_resample(line.points, dx);
    }
}


vec2 project_point(const vec2& startPoint, const grid_2D<float>& height_field, float c, float dt) {
    vec2 gradient = compute_gradient(height_field, startPoint.x, startPoint.y); //TODO: prevent division by zero
    if (norm(gradient) < 0.01) {
		// If the gradient is too small, consider the point invalid
		//return invalid_point;
        return startPoint;
	}
    vec2 direction = -normalize(gradient); // Direction of steepest descent

    // Initial projection of the point
    vec2 endPoint = startPoint + direction * c * dt;

    // Bisection method parameters
    vec2 left = startPoint;
    vec2 right = endPoint;
    vec2 mid = (left + right) * 0.5f;

    for (int i = 0; i < 2; ++i) { // Limit the iterations for performance : 2-4 as mentioned in the paper, ici 2 pr computation 
        mid = (left + right) * 0.5f;
        vec2 midGradient = compute_gradient(height_field, mid.x, mid.y);
        vec2 midDirection = -normalize(midGradient);

        // Check if the mid-point's direction aligns with the original direction closely enough
        if (dot(midDirection, direction) > 0.99) { // checking alignment, can modify the threshold
            // If aligned, this is a good projection
            break;
        }
        else {
            // Adjust the bisection bounds
            if (dot(midDirection, direction) < 0) {
                right = mid;
            }
            else {
                left = mid;
            }
        }
    }
    return mid; // The projected point along the steepest gradient
}


//TYPE 2:

vec2 project_to_maximum_height(const vec2& startPoint, const grid_2D<float>& height_field, float searchRadius) {
    vec2 maxPoint = startPoint;
    float maxHeight = height_field(startPoint.x, startPoint.y);

    // Search within a small neighborhood around the startPoint for the maximum height
    for (float dy = -searchRadius; dy <= searchRadius; dy += 1.0) {
        for (float dx = -searchRadius; dx <= searchRadius; dx += 1.0) {
            vec2 currentPoint = startPoint + vec2(dx, dy);
            if (!is_inside(currentPoint.x, currentPoint.y, height_field.dimension)) continue; // Stay within bounds

            float currentHeight = height_field(currentPoint.x, currentPoint.y);
            if (currentHeight > maxHeight) {
                maxHeight = currentHeight;
                maxPoint = currentPoint;
            }
        }
    }

    return maxPoint;
}

// adjusting the project_point function to include finding the maximum height before projecting along the steepest gradient
vec2 project_point_adjusted(const vec2& startPoint, const grid_2D<float>& heightField, float c, float dt, float tH) {
    // Project upwards to find maximum height within a small radius
    vec2 maxPoint = project_to_maximum_height(startPoint, heightField, 2.0); 

    // après perform projection from maxPoint along the steepest gradient, as mentioned in the paper
    vec2 projectedPoint = project_point(maxPoint, heightField, c, dt); // Reuse the previously defined projection function

    // Validate the new point
    vec2 up = -compute_gradient(heightField, startPoint.x, startPoint.y);
    if (norm(up) < 0.01) {
        // If the gradient is too small, consider the point invalid
        return startPoint;
        //return invalid_point;
    }
    if (norm(projectedPoint - startPoint) > 2 * c || norm(up) < tH / 2) {
        //TODO : do we return the invalid point or the start point for simplicity?
        //startPoint = pas de mouvement pr invalid cases
        //return start point for simplicity
        return startPoint;
        //return invalid_point;
    }

    return projectedPoint;
}


void adaptive_resample(std::vector<vec2>& line, float dx) {
    std::vector<vec2> newLine;
    newLine.reserve(line.size());

    for (size_t i = 0; i < line.size(); ++i) {
        if (i > 0) {
            const vec2& prev = line[i - 1];
            vec2& current = line[i];

            // Calculate distance between the current and previous point
            float dist = norm(current - prev);

            // Add a new point if the distance is larger than 2delta x
            if (dist > 2 * dx) {
                vec2 midpoint = (prev + current) / 2.0f;
                newLine.push_back(midpoint);
            }

            // Merge current and previous points if the distance is less than delta x/2
            else if (dist < dx / 2.0f) {
                // Merge by averaging their positions; replace the previous point in newLine
                newLine.back() = (prev + current) / 2.0f;
                continue; // Skip adding the current point as it's merged
            }
        }

        // Prevent folding by checking the direction of segments
        if (i > 1) {
            const vec2& prev = newLine[newLine.size() - 2]; // Second last point in newLine
            const vec2& last = newLine.back(); // Last point in newLine
            vec2 dirPrevLast = normalize(last - prev);
            vec2 dirLastCurrent = normalize(line[i] - last);

            // Check if segments are folding by examining the dot product
            if (dot(dirPrevLast, dirLastCurrent) <= 0) {
                // Replace the last point by averaging it with the current to prevent folding
                newLine.back() = (last + line[i]) / 2.0f;
                continue;
            }
        }

        // sinon add the current point to newLine
        if (i < line.size()) {
            newLine.push_back(line[i]);
        }
    }

    // update the original line with the resampled points
    line = std::move(newLine);
}


//FOR VISUALIZATION
numarray<vec3> convert_line_to_numarray3D(const Line& line, const grid_2D<float>& height_field) {
    numarray<vec3> positions(line.points.size());
    for (size_t i = 0; i < line.points.size(); ++i) {
        float z = height_field(line.points[i].y, line.points[i].x); //0.5f; 
        positions[i] = vec3(line.points[i].y, line.points[i].x, z);
    }
    return positions;
}