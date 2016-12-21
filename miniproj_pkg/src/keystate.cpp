#include <stdlib.h>
#include "ros/ros.h"
#include <SDL.h>
#include <iostream>
#include "geometry_msgs/Vector3.h"

const int DIRECTION_SIZE = 2;

enum ControlScheme {
    STANDARD = 1,
    INVERTED = -1
};

class KeystateController {

public:
    ~KeystateController();
    KeystateController(ControlScheme scheme);

    //Checking keyboard state:
    bool checkState();

    //For publishing
    void publish();

    //For saving control information
    int direction[2];

private:
    //Fro holding keyboard state
    const Uint8 *state;

    //For publishing information with ROS
    ros::NodeHandle n;
    ros::Publisher p;

    //Storing current control scheme
    ControlScheme scheme;

    //Event handler
    //SDL_Event e;

    SDL_Window * window;
    SDL_Renderer * renderer;

    SDL_Surface * image;
    SDL_Texture * texture;
};

//Constructor
KeystateController::KeystateController(ControlScheme scheme) {
    //Intializing publisher:
    p = n.advertise<geometry_msgs::Vector3>("direction", 1);
    this->scheme = scheme;

    //Initialize SDL
    SDL_Init(SDL_INIT_VIDEO);

    //Retrieving new keyboard state
    //SDL_PumpEvents();
    state = SDL_GetKeyboardState(NULL);

    //Setup SDL window, texture and surface
    window = SDL_CreateWindow("SDL2 Keyboard/Mouse events", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 200, 200, 0);
    renderer = SDL_CreateRenderer(window, -1, 0);
    image = SDL_LoadBMP("/home/christoffer/catkin_ws/src/miniproj_pkg/src/joystick.bmp");
    texture = SDL_CreateTextureFromSurface(renderer, image);
    SDL_FreeSurface(image);
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

    //render
    //Creating rectangle x0 y0 x1   y1
    SDL_Rect dstrect = { 0, 0, 200, 200 };
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, NULL, &dstrect);
    SDL_RenderPresent(renderer);
}

//Destructor
KeystateController::~KeystateController() {

    //Release resources to prevent leeks
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    //Closing sdl;
    SDL_Quit();
}

//function that saves the keystate, and returns true if any key is pressed
bool KeystateController::checkState() {
    //std::cout << "Checking state" << std::endl;

    bool keyPressed = false;

    //Requesting update for keyboard state
    //SDL_PollEvent(&e);
    SDL_PumpEvents();

    //resetting values of direction
    for (int i = 0; i < DIRECTION_SIZE; i++) {
        direction[i] = 0;
    }

    //checking for arrow keys
    if (state[SDL_SCANCODE_LEFT]) {
        direction[1] -= 1*scheme;
        keyPressed = true;
    }

    if (state[SDL_SCANCODE_RIGHT]) {
        direction[1] += 1*scheme;
        keyPressed = true;
    }

    if (state[SDL_SCANCODE_UP]) {
        direction[0] -= 1*scheme;
        keyPressed = true;
    }

    if (state[SDL_SCANCODE_DOWN]) {
        direction[0] += 1*scheme;
        keyPressed = true;
    }

    return keyPressed;
}

void KeystateController::publish() {
    geometry_msgs::Vector3 msg;
    msg.x = direction[0];
    msg.y = direction[1];
    msg.z = 0;

    p.publish(msg);
}

ControlScheme getControlScheme() {
    char yn;
    bool validIn = false;
    ControlScheme s;

    while (!validIn) {

        std::cout << "Do you want inverted controls? y/n" << std::endl;
        std::cin >> yn;

        //Check if input is valid
        if (yn == 'y') {
            validIn = true;
            s = INVERTED;
            return s;
        } else if (yn == 'n') {
            validIn = true;
            s = STANDARD;
            return s;
        } else {
            std::cout << "Invalid input" << std::endl;
        }
    }
}


int main(int argc, char **argv) {

    //Initializing
    ros::init(argc, argv, "keystate_node");

    //Create instance of class
    KeystateController k(getControlScheme());

    //While ROS is running, loop forever
    //Loop 10 times per second
    std::cout << "Commencing Loop" << std::endl;
    ros::Rate loopRate(50);
    while (ros::ok()) {

        //Check for keystate
        if (k.checkState()) {
            //std::cout << "Keypress detected" << std::endl;
        }

        k.publish();

        //Handle publishing etc..
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}

/*
─────────▄──────────────▄────
────────▌▒█───────────▄▀▒▌───
────────▌▒▒▀▄───────▄▀▒▒▒▐───
───────▐▄▀▒▒▀▀▀▀▄▄▄▀▒▒▒▒▒▐───
─────▄▄▀▒▒▒▒▒▒▒▒▒▒▒█▒▒▄█▒▐───
───▄▀▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▀██▀▒▌───
──▐▒▒▒▄▄▄▒▒▒▒▒▒▒▒▒▒▒▒▒▀▄▒▒▌──
──▌▒▒▐▄█▀▒▒▒▒▄▀█▄▒▒▒▒▒▒▒█▒▐──
─▐▒▒▒▒▒▒▒▒▒▒▒▌██▀▒▒▒▒▒▒▒▒▀▄▌─
─▌▒▀▄██▄▒▒▒▒▒▒▒▒▒▒▒░░░░▒▒▒▒▌─
─▌▀▐▄█▄█▌▄▒▀▒▒▒▒▒▒░░░░░░▒▒▒▐─
▐▒▀▐▀▐▀▒▒▄▄▒▄▒▒▒▒▒░░░░░░▒▒▒▒▌
▐▒▒▒▀▀▄▄▒▒▒▄▒▒▒▒▒▒░░░░░░▒▒▒▐─
─▌▒▒▒▒▒▒▀▀▀▒▒▒▒▒▒▒▒░░░░▒▒▒▒▌─
─▐▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▐──
──▀▄▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▄▒▒▒▒▌──
────▀▄▒▒▒▒▒▒▒▒▒▒▄▄▄▀▒▒▒▒▄▀───
───▐▀▒▀▄▄▄▄▄▄▀▀▀▒▒▒▒▒▄▄▀─────
──▐▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▀▀────────
Wow Such Programming
Much amaze
*/
