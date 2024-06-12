#include "planar_quadrotor_visualizer.h"

const int QUADROTOR_WIDTH = 180;
const int QUADROTOR_HEIGHT = 20;
const int PROPELLER_WIDTH = 40;
const int PROPELLER_HEIGHT = 4;

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {

    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x = state[0];
    float q_y = state[1];
    float q_theta = state[2];

    SDL_SetRenderDrawColor(gRenderer.get(), 0x80, 0x00, 0x00, 0xFF);
    SDL_Rect quadrotorRect;
    quadrotorRect.x = q_x - QUADROTOR_WIDTH/2;
    quadrotorRect.y = q_y - QUADROTOR_HEIGHT/2;
    quadrotorRect.w = QUADROTOR_WIDTH;
    quadrotorRect.h = QUADROTOR_HEIGHT;

    SDL_Surface* surface = SDL_CreateRGBSurface(0, quadrotorRect.w, quadrotorRect.h, 32, 0, 0, 0, 0);
    SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 0x80, 0x00, 0x00));

    SDL_Texture* texture = SDL_CreateTextureFromSurface(gRenderer.get(), surface);
    SDL_FreeSurface(surface);

    SDL_RenderCopyEx(gRenderer.get(), texture, NULL, &quadrotorRect, q_theta, NULL, SDL_FLIP_NONE);
    SDL_DestroyTexture(texture);

    drawPropeller(gRenderer, -QUADROTOR_WIDTH / 2 + PROPELLER_WIDTH / 2);
    drawPropeller(gRenderer, QUADROTOR_WIDTH / 2 - PROPELLER_WIDTH / 2);
}

void PlanarQuadrotorVisualizer::drawPropeller(std::shared_ptr<SDL_Renderer> &gRenderer, int offsetX) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x = state[0];
    float q_y = state[1];

    int centerX = q_x + offsetX;
    int centerY = q_y - QUADROTOR_HEIGHT / 2 - PROPELLER_WIDTH/2;

    SDL_Rect rect;
    rect.x = centerX - PROPELLER_WIDTH / 2;
    rect.y = centerY - PROPELLER_HEIGHT / 2;
    rect.w = PROPELLER_WIDTH;
    rect.h = PROPELLER_HEIGHT;

    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0x00, 0xFF);
    SDL_Surface* surface = SDL_CreateRGBSurface(0, rect.w, rect.h, 32, 0, 0, 0, 0);
    SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 0x00, 0x00, 0x00));

    SDL_Texture* texture = SDL_CreateTextureFromSurface(gRenderer.get(), surface);
    SDL_FreeSurface(surface);

    static float rotationAngle = 0;
    rotationAngle += 5;

    SDL_RenderCopyEx(gRenderer.get(), texture, NULL, &rect, rotationAngle, NULL, SDL_FLIP_NONE);
    SDL_DestroyTexture(texture);

    // Draw a line from the center of the propeller to the center of the quadrotor
    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0x00, 0xFF);
    for (int i = -2; i <= 2; ++i) {
        SDL_RenderDrawLine(gRenderer.get(), centerX + i, centerY, centerX + i, q_y-QUADROTOR_HEIGHT/2);
    }
}