#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualization
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicating http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate propellers (extra points)
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer)
{
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;

    /* x, y, theta coordinates */
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];

    SDL_Surface* combinedSurface = SDL_CreateRGBSurface(0, 140, 80, 32, 0, 0, 0, 0);
    SDL_FillRect(combinedSurface, nullptr, SDL_MapRGB(combinedSurface->format, 0xFF, 0xFF, 0xFF));

    SDL_Rect grayRect = { 10, 60, 120, 20 };
    SDL_FillRect(combinedSurface, &grayRect, SDL_MapRGB(combinedSurface->format, 0xC0, 0xC0, 0xC0));

    SDL_Rect redRect1 = { 15, 0, 5, 60 };
    SDL_Rect redRect2 = { 120, 0, 5, 60 };
    SDL_FillRect(combinedSurface, &redRect1, SDL_MapRGB(combinedSurface->format, 0xFF, 0x00, 0x00));
    SDL_FillRect(combinedSurface, &redRect2, SDL_MapRGB(combinedSurface->format, 0xFF, 0x00, 0x00));

    SDL_Rect blueRect1 = { 0, 10, 35, 5 };
    SDL_Rect blueRect2 = { 105, 10, 35, 5 };
    SDL_FillRect(combinedSurface, &blueRect1, SDL_MapRGB(combinedSurface->format, 0x00, 0x00, 0xFF));
    SDL_FillRect(combinedSurface, &blueRect2, SDL_MapRGB(combinedSurface->format, 0x00, 0x00, 0xFF));

    SDL_Texture* combinedTexture = SDL_CreateTextureFromSurface(gRenderer.get(), combinedSurface);
    SDL_FreeSurface(combinedSurface);

    SDL_Rect quadrotorRect = { static_cast<int>(q_x - 60), static_cast<int>(q_y - 70), 120, 85 };

    double angle_degrees = q_theta * (180 / M_PI);

    SDL_Point center = { quadrotorRect.w / 2, quadrotorRect.h / 2 };
    SDL_RenderCopyEx(gRenderer.get(), combinedTexture, nullptr, &quadrotorRect, angle_degrees, &center, SDL_FLIP_NONE);
    SDL_DestroyTexture(combinedTexture);

}