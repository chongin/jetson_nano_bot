#include <iostream>
#include <SDL2/SDL.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <csignal>


std::map<char, std::string> KEY_MAPPING = {
    {'W', "up"},
    {'S', "down"},
    {'A', "left"},
    {'D', "right"},
    {'Q', "speed+"}, //w
    {'E', "speed-"} //s
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "manual_control_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("drive_car", 10);

    SDL_Init(SDL_INIT_VIDEO); // Initialize SDL

    SDL_Window* window = SDL_CreateWindow("Drive Car", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 640, 480, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);

    bool keyDown[SDL_NUM_SCANCODES] = {false}; // Array to track key state
    bool running = true;
    std_msgs::String message;
    const char* key = nullptr;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_QUIT:
                    running = false;
                    break;
                case SDL_KEYDOWN:
                    keyDown[event.key.keysym.scancode] = true;

                    key = SDL_GetScancodeName(event.key.keysym.scancode);
                    if (KEY_MAPPING.count(*key) > 0)
                    {
                        message.data = KEY_MAPPING[*key];
                        pub.publish(message);
                    }

                    std::cout << "Key Pressed: " << key << std::endl;
                    break;
                case SDL_KEYUP:
                    keyDown[event.key.keysym.scancode] = false;

                    key = SDL_GetScancodeName(event.key.keysym.scancode);
                    if (KEY_MAPPING.count(*key) > 0)
                    {
                        message.data = "stop";
                        pub.publish(message);
                    }

                    std::cout << "Key Released: " << SDL_GetScancodeName(event.key.keysym.scancode) << std::endl;
                    break;
                default:
                    // Handle other event types
                    break;
            }
        }
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit(); // Cleanup SDL

    return 0;
}
