
#include <SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdbool.h>

#define INITIAL_WINDOW_WIDTH 1200
#define INITIAL_WINDOW_HEIGHT 800
#define MAX_BODIES 100
#define G 0.5 // Adjusted gravitational constant for visible simulation
#define SOFTENING 5.0 // Softening factor to prevent extreme forces at close distances
#define TIME_STEP 0.1
#define BODY_RADIUS 5

// Global window dimensions (can be changed when resized)
int window_width = INITIAL_WINDOW_WIDTH;
int window_height = INITIAL_WINDOW_HEIGHT;

typedef struct {
    double x, y;
    double vx, vy;
    double ax, ay;
    double mass;
    double radius; // Collision radius
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
        bodies[i].x = rand() % window_width;
        bodies[i].y = rand() % window_height;
        bodies[i].vx = (rand() % 100 - 50) / 50.0;
        bodies[i].vy = (rand() % 100 - 50) / 50.0;
        bodies[i].ax = 0;
        bodies[i].ay = 0;
        bodies[i].mass = 100 + rand() % 900;
        bodies[i].radius = BODY_RADIUS + (bodies[i].mass / 200);
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
        if (bodies[i].x < 0) bodies[i].x = window_width;
        if (bodies[i].x > window_width) bodies[i].x = 0;
        if (bodies[i].y < 0) bodies[i].y = window_height;
        if (bodies[i].y > window_height) bodies[i].y = 0;
    }
}

// Check and handle collisions (merge bodies)
void handle_collisions() {
    for (int i = 0; i < body_count; i++) {
        if (!bodies[i].active) continue;
        
        for (int j = i + 1; j < body_count; j++) {
            if (!bodies[j].active) continue;
            
            double dx = bodies[j].x - bodies[i].x;
            double dy = bodies[j].y - bodies[i].y;
            double dist = sqrt(dx * dx + dy * dy);
            
            // Check if bodies are colliding
            if (dist < bodies[i].radius + bodies[j].radius) {
                // Merge the smaller body into the larger one
                // Conservation of momentum
                Body *larger, *smaller;
                int larger_idx, smaller_idx;
                
                if (bodies[i].mass >= bodies[j].mass) {
                    larger = &bodies[i];
                    smaller = &bodies[j];
                    larger_idx = i;
                    smaller_idx = j;
                } else {
                    larger = &bodies[j];
                    smaller = &bodies[i];
                    larger_idx = j;
                    smaller_idx = i;
                }
                
                // Calculate new velocity using conservation of momentum
                double total_mass = larger->mass + smaller->mass;
                larger->vx = (larger->mass * larger->vx + smaller->mass * smaller->vx) / total_mass;
                larger->vy = (larger->mass * larger->vy + smaller->mass * smaller->vy) / total_mass;
                
                // Add masses together
                larger->mass = total_mass;
                larger->radius = BODY_RADIUS + (larger->mass / 200);
                
                // Update color based on mass ratio (blend colors)
                double ratio = smaller->mass / total_mass;
                larger->r = (Uint8)(larger->r * (1 - ratio) + smaller->r * ratio);
                larger->g = (Uint8)(larger->g * (1 - ratio) + smaller->g * ratio);
                larger->b = (Uint8)(larger->b * (1 - ratio) + smaller->b * ratio);
                
                // Deactivate the absorbed body
                smaller->active = false;
                
                printf("Collision! Body %d absorbed body %d (new mass: %.1f)\n", 
                       larger_idx, smaller_idx, larger->mass);
            }
        }
    }
}

// Draw a filled circle with smooth edges
void draw_circle(SDL_Renderer *renderer, int cx, int cy, int radius) {
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            if (dx * dx + dy * dy <= radius * radius) {
                SDL_RenderDrawPoint(renderer, cx + dx, cy + dy);
            }
        }
    }
}

// Draw a circle with glow effect
void draw_glowing_circle(SDL_Renderer *renderer, int cx, int cy, int radius, Uint8 r, Uint8 g, Uint8 b) {
    // Draw outer glow (multiple layers with decreasing opacity)
    for (int glow = 3; glow > 0; glow--) {
        int glow_radius = radius + glow * 3;
        Uint8 alpha = 30 / glow; // Decreasing opacity for outer layers
        SDL_SetRenderDrawColor(renderer, r, g, b, alpha);
        
        // Draw glow ring
        for (int angle = 0; angle < 360; angle += 5) {
            double rad = angle * M_PI / 180.0;
            int x = cx + (int)(glow_radius * cos(rad));
            int y = cy + (int)(glow_radius * sin(rad));
            
            // Draw small circle for glow point
            for (int dy = -2; dy <= 2; dy++) {
                for (int dx = -2; dx <= 2; dx++) {
                    if (dx * dx + dy * dy <= 4) {
                        SDL_RenderDrawPoint(renderer, x + dx, y + dy);
                    }
                }
            }
        }
    }
    
    // Draw main body with gradient
    for (int layer = radius; layer > 0; layer--) {
        // Brighten towards center
        float brightness = 1.0 + (radius - layer) * 0.3 / radius;
        Uint8 bright_r = (Uint8)fmin(255, r * brightness);
        Uint8 bright_g = (Uint8)fmin(255, g * brightness);
        Uint8 bright_b = (Uint8)fmin(255, b * brightness);
        
        SDL_SetRenderDrawColor(renderer, bright_r, bright_g, bright_b, 255);
        draw_circle(renderer, cx, cy, layer);
    }
}

// Render all bodies with improved graphics
void render_bodies(SDL_Renderer *renderer) {
    // Enable alpha blending for glow effects
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    
    for (int i = 0; i < body_count; i++) {
        if (!bodies[i].active) continue;
        
        // Draw body with glow effect
        draw_glowing_circle(renderer, (int)bodies[i].x, (int)bodies[i].y, 
                           (int)bodies[i].radius, bodies[i].r, bodies[i].g, bodies[i].b);
        
        // Draw velocity indicator (small trail line)
        if (bodies[i].vx != 0 || bodies[i].vy != 0) {
            SDL_SetRenderDrawColor(renderer, bodies[i].r, bodies[i].g, bodies[i].b, 128);
            int trail_x = (int)(bodies[i].x - bodies[i].vx * 5);
            int trail_y = (int)(bodies[i].y - bodies[i].vy * 5);
            SDL_RenderDrawLine(renderer, (int)bodies[i].x, (int)bodies[i].y, trail_x, trail_y);
        }
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
        bodies[body_count].radius = BODY_RADIUS + (mass / 200);
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
    
    // Create window (resizable)
    SDL_Window *window = SDL_CreateWindow(
        "Gravity Simulator - Click to add bodies, Space to reset, Resize window",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        window_width,
        window_height,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
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
    
    printf("Gravity Simulator with Collision Detection:\n");
    printf("- Left Click: Add small body\n");
    printf("- Right Click: Add large body\n");
    printf("- Space: Reset simulation\n");
    printf("- ESC: Exit\n");
    printf("- Bodies merge on collision (conservation of momentum)\n\n");
    
    while (running) {
        // Handle events
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
            else if (event.type == SDL_WINDOWEVENT) {
                if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                    window_width = event.window.data1;
                    window_height = event.window.data2;
                    printf("Window resized to %dx%d\n", window_width, window_height);
                }
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
        handle_collisions();
        
        // Render with gradient background
        // Deep space gradient (dark blue to black)
        for (int y = 0; y < window_height; y++) {
            float ratio = (float)y / window_height;
            Uint8 r = (Uint8)(5 * (1 - ratio));
            Uint8 g = (Uint8)(10 * (1 - ratio));
            Uint8 b = (Uint8)(25 * (1 - ratio));
            SDL_SetRenderDrawColor(renderer, r, g, b, 255);
            SDL_RenderDrawLine(renderer, 0, y, window_width, y);
        }
        
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