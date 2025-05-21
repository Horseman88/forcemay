import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import sys

# Vertices for a cube
vertices = (
    (1, -1, -1),
    (1, 1, -1),
    (-1, 1, -1),
    (-1, -1, -1),
    (1, -1, 1),
    (1, 1, 1),
    (-1, -1, 1),
    (-1, 1, 1)
)

# Edges of the cube
edges = (
    (0, 1),
    (0, 3),
    (0, 4),
    (2, 1),
    (2, 3),
    (2, 7), # Corrected from (2, 6) to (2,7) to match vertex 7
    (6, 3),
    (6, 4),
    (6, 7), # Corrected from (6,5) to (6,7) to match vertex 7
    (5, 1),
    (5, 4),
    (5, 7)  # Corrected from (5,6) to (5,7) to match vertex 7
)

# Surfaces of the cube
surfaces = (
    (0,1,2,3),
    (3,2,7,6), # Corrected: (3,2,7,6) instead of (3,2,6,7) - ensure winding order or face culling
    (6,7,5,4), # Corrected: (6,7,5,4) instead of (6,7,5,4)
    (4,5,1,0),
    (1,5,7,2), # Corrected: (1,5,7,2) instead of (1,5,6,2)
    (4,0,3,6)  # Corrected: (4,0,3,6) instead of (4,0,3,6)
)

# Colors for surfaces
colors = (
    (1,0,0), # Red
    (0,1,0), # Green
    (0,0,1), # Blue
    (1,1,0), # Yellow
    (1,0,1), # Magenta
    (0,1,1), # Cyan
)

def draw_cube():
    glBegin(GL_QUADS)
    for i, surface in enumerate(surfaces):
        glColor3fv(colors[i % len(colors)]) # Cycle through colors
        for vertex_index in surface:
            glVertex3fv(vertices[vertex_index])
    glEnd()

    # Draw edges for clarity (optional)
    glColor3fv((0,0,0)) # Black color for edges
    glBegin(GL_LINES)
    for edge in edges:
        for vertex_index in edge:
            glVertex3fv(vertices[vertex_index])
    glEnd()

def main():
    try:
        pygame.init()
        display_width = 800
        display_height = 600
        display = (display_width, display_height)
        pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
        pygame.display.set_caption("Pygame 3D Drone Environment")

        # Setup 3D perspective
        gluPerspective(45, (display_width / display_height), 0.1, 50.0)
        glTranslatef(0.0, 0.0, -10) # Move camera back to see the cube
        glRotatef(0, 0, 0, 0) # Initial rotation

    except pygame.error as e:
        print(f"Pygame initialization error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"An error occurred during setup: {e}")
        sys.exit(1)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    glRotatef(1, 0, 1, 0)
                if event.key == pygame.K_RIGHT:
                    glRotatef(-1, 0, 1, 0)
                if event.key == pygame.K_UP:
                    glRotatef(1, 1, 0, 0)
                if event.key == pygame.K_DOWN:
                    glRotatef(-1, 1, 0, 0)
            # Mouse controls for rotation (optional)
            if event.type == pygame.MOUSEMOTION:
                if pygame.mouse.get_pressed()[0]: # Left mouse button
                    dx, dy = event.rel
                    glRotatef(dy * 0.5, 1, 0, 0) # Rotate around X axis
                    glRotatef(dx * 0.5, 0, 1, 0) # Rotate around Y axis


        # glRotatef(1, 3, 1, 1) # Auto-rotate cube for dynamic view
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        
        # Enable depth testing for correct 3D rendering
        glEnable(GL_DEPTH_TEST)
        
        draw_cube()
        
        pygame.display.flip()
        pygame.time.wait(10)

if __name__ == '__main__':
    main()
