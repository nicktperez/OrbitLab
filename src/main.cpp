#include "app/Application.hpp"

#include <SDL3/SDL_main.h>

#include <exception>
#include <iostream>

int main(int, char**) {
    try {
        orbitlab::app::Application application;
        return application.run();
    } catch (const std::exception& exception) {
        std::cerr << "OrbitLab could not start: " << exception.what() << '\n';
        return 1;
    }
}
