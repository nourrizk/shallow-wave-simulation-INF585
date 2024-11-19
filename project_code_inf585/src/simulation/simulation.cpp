#include "simulation.hpp"
#include "height_simulation.hpp"

using namespace cgp;



void divergence_free(grid_2D<vec2>& new_velocity, grid_2D<vec2> const& velocity, grid_2D<float>& divergence, grid_2D<float>& gradient_field)
{
    // v = projection of v0 on divergence free vector field
    //
    // v : Final vector field to be filled
    // v0: Initial vector field (non divergence free)
    // divergence: temporary buffer used to compute the divergence of v0
    // gradient_field: temporary buffer used to compute v = v0 - nabla(gradient_field)


    // TO do:
    // 1. Compute divergence of v0
    compute_divergence(velocity, divergence);
    
    // 2. Compute gradient_field such that nabla(gradient_field)^2 = div(v0)
    int2 dim_grad = gradient_field.dimension;
    int iterations = 10;
    for (int it = 0; it < iterations; ++it) {
        for (int y = 1; y < dim_grad[1] - 1; ++y) {
            for (int x = 1; x < dim_grad[0] - 1; ++x) {
                // Using the discrete Laplacian approximation
                gradient_field(x, y) = (gradient_field(x + 1, y) + gradient_field(x - 1, y) +
                    gradient_field(x, y + 1) + gradient_field(x, y - 1) -
                    divergence(x, y)) * 0.25;
            }
        }
        // Apply boundary conditions after each iteration
        set_boundary(gradient_field);
    }
    set_boundary(gradient_field);

    // 3. Compute v = v0 - nabla(gradient_field)
    int2 dim_v = new_velocity.dimension;
    for (int y = 1; y < dim_v[1] - 1; ++y) {
        for (int x = 1; x < dim_v[0] - 1; ++x) {
            vec2 grad;
            grad.x = (gradient_field(x + 1, y) - gradient_field(x - 1, y)) / 2.0f;
            grad.y = (gradient_field(x, y + 1) - gradient_field(x, y - 1)) / 2.0f;
            new_velocity(x, y) = vec2{ velocity(x, y).x - grad.x, velocity(x, y).y - grad.y };
        }
    }
    set_boundary(new_velocity);
}