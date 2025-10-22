
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdbool.h>

#define WINDOW_WIDTH 1200
#define WINDOW_HEIGHT 800
#define MAX_BODIES 100
#define G 0.5 // Adjusted gravitational constant for visible simulation
#define SOFTENING 5.0 // Softening factor to prevent extreme forces at close distances
#define TIME_STEP 0.1
#define BODY_RADIUS 5

typedef struct {
    double x, y;
    double vx, vy;
    double ax, ay;
    double mass;
    bool active;
    Uint8 r, g, b; // Color
} Body;

Body bodies[MAX_BODIES];
int body_count = 0;

// Initialize bodies with random positions and velocities
void init_bodies() {
    srand(time(NULL));
    body_count = 5;
    
    for (int i = 0; i < body_count; i++) {
        bodies[i].x = rand() % WINDOW_WIDTH;
        bodies[i].y = rand() % WINDOW_HEIGHT;
        bodies[i].vx = (rand() % 100 - 50) / 50.0;
        bodies[i].vy = (rand() % 100 - 50) / 50.0;
        bodies[i].ax = 0;
        bodies[i].ay = 0;
        bodies[i].mass = 100 + rand() % 900;
        bodies[i].active = true;
        bodies[i].r = rand() % 256;
        bodies[i].g = rand() % 256;
        bodies[i].b = rand() % 256;
    }
}

// Calculate gravitational forces between all bodies
void calculate_forces() {
    // Reset accelerations
    for (int i = 0; i < body_count; i++) {
        if (!bodies[i].active) continue;
        bodies[i].ax = 0;
        bodies[i].ay = 0;
    }
    
    // Calculate pairwise forces
    for (int i = 0; i < body_count; i++) {
        if (!bodies[i].active) continue;
        
        for (int j = i + 1; j < body_count; j++) {
            if (!bodies[j].active) continue;
            
            double dx = bodies[j].x - bodies[i].x;
            double dy = bodies[j].y - bodies[i].y;
            double dist_sq = dx * dx + dy * dy + SOFTENING * SOFTENING;
            double dist = sqrt(dist_sq);
            
            // Gravitational force magnitude
            double force = G * bodies[i].mass * bodies[j].mass / dist_sq;
            
            // Force components
            double fx = force * dx / dist;
            double fy = force * dy / dist;
            
            // Apply force to both bodies (Newton's third law)
            bodies[i].ax += fx / bodies[i].mass;
            bodies[i].ay += fy / bodies[i].mass;
            bodies[j].ax -= fx / bodies[j].mass;
            bodies[j].ay -= fy / bodies[j].mass;
        }
    }
}

// Update positions and velocities
void update_bodies() {
    for (int i = 0; i < body_count; i++) {
        if (!bodies[i].active) continue;
        
        // Update velocity
        bodies[i].vx += bodies[i].ax * TIME_STEP;
        bodies[i].vy += bodies[i].ay * TIME_STEP;
        
        // Update position
        bodies[i].x += bodies[i].vx * TIME_STEP;
        bodies[i].y += bodies[i].vy * TIME_STEP;
        
        // Wrap around screen edges
        if (bodies[i].x < 0) bodies[i].x = WINDOW_WIDTH;
        if (bodies[i].x > WINDOW_WIDTH) bodies[i].x = 0;
        if (bodies[i].y < 0) bodies[i].y = WINDOW_HEIGHT;
        if (bodies[i].y > WINDOW_HEIGHT) bodies[i].y = 0;
    }
}

// Draw a filled circle
void draw_circle(SDL_Renderer *renderer, int cx, int cy, int radius) {
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            if (dx * dx + dy * dy <= radius * radius) {
                SDL_RenderDrawPoint(renderer, cx + dx, cy + dy);
            }
        }
    }
}

// Render all bodies
void render_bodies(SDL_Renderer *renderer) {
    for (int i = 0; i < body_count; i++) {
        if (!bodies[i].active) continue;
        
        // Set color based on body
        SDL_SetRenderDrawColor(renderer, bodies[i].r, bodies[i].g, bodies[i].b, 255);
        
        // Draw body as a circle with size based on mass
        int radius = BODY_RADIUS + (int)(bodies[i].mass / 200);
        draw_circle(renderer, (int)bodies[i].x, (int)bodies[i].y, radius);
    }
}

// Add a new body at mouse position
void add_body(int x, int y, int mass) {
    if (body_count < MAX_BODIES) {
        bodies[body_count].x = x;
        bodies[body_count].y = y;
        bodies[body_count].vx = 0;
        bodies[body_count].vy = 0;
        bodies[body_count].ax = 0;
        bodies[body_count].ay = 0;
        bodies[body_count].mass = mass;
        bodies[body_count].active = true;
        bodies[body_count].r = rand() % 256;
        bodies[body_count].g = rand() % 256;
        bodies[body_count].b = rand() % 256;
        body_count++;
    }
}

int main(int argc, char *argv[]) {
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("SDL initialization failed: %s\n", SDL_GetError());
        return 1;
    }
    
    // Create window
    SDL_Window *window = SDL_CreateWindow(
        "Gravity Simulator - Click to add bodies, Space to reset",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH,
        WINDOW_HEIGHT,
        SDL_WINDOW_SHOWN
    );
    
    if (!window) {
        printf("Window creation failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    
    // Create renderer
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        printf("Renderer creation failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    
    // Initialize bodies
    init_bodies();
    
    // Main loop
    bool running = true;
    SDL_Event event;
    
    printf("Gravity Simulator Controls:\n");
    printf("- Left Click: Add small body\n");
    printf("- Right Click: Add large body\n");
    printf("- Space: Reset simulation\n");
    printf("- ESC: Exit\n\n");
    
    while (running) {
        // Handle events
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
            else if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    running = false;
                }
                else if (event.key.keysym.sym == SDLK_SPACE) {
                    init_bodies();
                    printf("Simulation reset!\n");
                }
            }
            else if (event.type == SDL_MOUSEBUTTONDOWN) {
                int x, y;
                SDL_GetMouseState(&x, &y);
                
                if (event.button.button == SDL_BUTTON_LEFT) {
                    add_body(x, y, 500);
                    printf("Added small body at (%d, %d)\n", x, y);
                }
                else if (event.button.button == SDL_BUTTON_RIGHT) {
                    add_body(x, y, 2000);
                    printf("Added large body at (%d, %d)\n", x, y);
                }
            }
        }
        
        // Physics update
        calculate_forces();
        update_bodies();
        
        // Render
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // Black background
        SDL_RenderClear(renderer);
        render_bodies(renderer);
        SDL_RenderPresent(renderer);
        
        // Delay to control frame rate
        SDL_Delay(16); // ~60 FPS
    }
    
    // Cleanup
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    
    return 0;
}