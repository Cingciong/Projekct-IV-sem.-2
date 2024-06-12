/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"
#include "matplot/matplot.h"



namespace mp = matplot;

Eigen::MatrixXf LQR(PlanarQuadrotor &quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 5e-3, 5e-3, 1e3, 2*5e-2, 5e-2, 2 / 2 / M_PI;
    R.row(0) << 3e1, 5;
    R.row(1) << 5, 3e1;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;
    
    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor &quadrotor, const Eigen::MatrixXf &K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

int main(int argc, char* args[])
{
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    initial_state[0] = SCREEN_WIDTH / 2;
    initial_state[1] = SCREEN_HEIGHT / 2;

    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);

    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.5;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        float delay;
        int x, y;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);

        while (!quit)
        {
            //events
            while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN)
                {
                    if (e.button.button == SDL_BUTTON_LEFT)
                    {
                        SDL_GetMouseState(&x, &y);
                        std::cout << "Left mouse button clicked Mouse position: (" << x << ", " << y << ")"<< std::endl;
                        goal_state << x, y, 0, 0, 0, 0;
                        quadrotor.SetGoal(goal_state);
                    }
                }
                else if (e.type == SDL_KEYDOWN)
                {
                    if (e.key.keysym.sym == SDLK_p)
                    {
                        char a;
                        std::cout << "Plotting x, y over time" << std::endl;
                        mp::plot(x_history, y_history);
                        std::cin>>a;
                        std::cout << "Plotting x, y, theta over time" << std::endl;
                        mp::plot(theta_history);
                    }
                }
                state = quadrotor.GetState();
                x_history.push_back(SCREEN_WIDTH-state[0]);
                y_history.push_back(SCREEN_HEIGHT- state[1]);
                theta_history.push_back(state[2]);
            }

            SDL_Delay((int) dt * 1000);

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);
            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);
        }
    }
    SDL_Quit();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}


