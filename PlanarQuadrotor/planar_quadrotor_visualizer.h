#ifndef PLANAR_QUADROTOR_VISUALIZER_H
#define PLANAR_QUADROTOR_VISUALIZER_H

#include <SDL.h>
#include <memory>
#include <Eigen/Dense>
#include "planar_quadrotor.h"

class PlanarQuadrotorVisualizer {
public:
    PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr);

    void render(std::shared_ptr<SDL_Renderer> &gRenderer);
    void drawPropeller(std::shared_ptr<SDL_Renderer> &gRenderer, int offsetX);

private:
    PlanarQuadrotor *quadrotor_ptr;
};

#endif //PLANAR_QUADROTOR_VISUALIZER_H