import pygame
import sys
import math

WIDTH, HEIGHT = 800, 600
FPS = 60

ORBIT_RADIUS = 120.0
TANGENTIAL_SPEED = 300.0
ANGULAR_SPEED = TANGENTIAL_SPEED / ORBIT_RADIUS
DIVE_SPEED = 320.0

DOT_RADIUS = 5
SAFETY_MARGIN = 12
TARGET_SPEED = 220.0

EPS = 1.0

BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)
GREY = (200, 200, 200)

DEBUG = True

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Orbit/Dive — bounce + corner glide (no comments)")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 18)

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def normalize(x, y):
    n = math.hypot(x, y)
    if n < 1e-9:
        return 1.0, 0.0
    return x / n, y / n

def reflect_point(x, y, left=None, right=None, top=None, bottom=None):
    rx, ry = x, y
    if left is not None and x < left:
        rx = 2 * left - x
    if right is not None and x > right:
        rx = 2 * right - x
    if top is not None and y < top:
        ry = 2 * top - y
    if bottom is not None and y > bottom:
        ry = 2 * bottom - y
    return rx, ry

def rotate90(x, y, direction=1):
    return (-y, x) if direction >= 0 else (y, -x)

def predict_phase_step(udx, udy, dt, orbit_dir):
    dtheta = ANGULAR_SPEED * orbit_dir * dt
    c, s = math.cos(dtheta), math.sin(dtheta)
    ndx = udx * c - udy * s
    ndy = udx * s + udy * c
    return normalize(ndx, ndy)

def inside_box(x, y):
    return (SAFETY_MARGIN <= x <= WIDTH - SAFETY_MARGIN and
            SAFETY_MARGIN <= y <= HEIGHT - SAFETY_MARGIN)

red_x, red_y = WIDTH * 0.5, HEIGHT * 0.5
unit_dx, unit_dy = 1.0, 0.0
orbit_direction = 1
radial_distance = ORBIT_RADIUS

state = "ORBIT"
glide_axis = None
glide_sign = 0

dive_count = 0
min_center_dist = float("inf")

running = True
while running:
    dt = clock.tick(FPS) / 1000.0

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
            elif event.key == pygame.K_SPACE:
                if state == "ORBIT":
                    state = "INWARD"
                    dive_count += 1
            elif event.key == pygame.K_c:
                orbit_direction *= -1
            elif event.key == pygame.K_d:
                DEBUG = not DEBUG

    keys = pygame.key.get_pressed()
    tvx = (keys[pygame.K_RIGHT] - keys[pygame.K_LEFT]) * TARGET_SPEED
    tvy = (keys[pygame.K_DOWN] - keys[pygame.K_UP]) * TARGET_SPEED
    red_x = clamp(red_x + tvx * dt, SAFETY_MARGIN, WIDTH - SAFETY_MARGIN)
    red_y = clamp(red_y + tvy * dt, SAFETY_MARGIN, HEIGHT - SAFETY_MARGIN)

    if state == "INWARD":
        radial_distance = max(0.0, radial_distance - DIVE_SPEED * dt)
        if radial_distance <= EPS:
            state = "OUTWARD"
            radial_distance = EPS
    elif state == "OUTWARD":
        radial_distance = min(ORBIT_RADIUS, radial_distance + DIVE_SPEED * dt)
        if radial_distance >= ORBIT_RADIUS - EPS:
            state = "ORBIT"
            radial_distance = ORBIT_RADIUS
    else:
        radial_distance = ORBIT_RADIUS if state != "WALL_GLIDE" else radial_distance

    cur_gx = red_x + unit_dx * radial_distance
    cur_gy = red_y + unit_dy * radial_distance

    if state != "WALL_GLIDE":
        ndx, ndy = predict_phase_step(unit_dx, unit_dy, dt, orbit_direction)
        gx_nom = red_x + ndx * radial_distance
        gy_nom = red_y + ndy * radial_distance

        left, right = SAFETY_MARGIN, WIDTH - SAFETY_MARGIN
        top, bottom = SAFETY_MARGIN, HEIGHT - SAFETY_MARGIN

        hit_x = (gx_nom < left) or (gx_nom > right)
        hit_y = (gy_nom < top) or (gy_nom > bottom)

        if not (hit_x or hit_y):
            unit_dx, unit_dy = ndx, ndy
            green_x, green_y = gx_nom, gy_nom
        elif hit_x ^ hit_y:
            gx_ref, gy_ref = reflect_point(gx_nom, gy_nom, left=left, right=right, top=top, bottom=bottom)
            unit_dx, unit_dy = normalize(gx_ref - red_x, gy_ref - red_y)
            orbit_direction *= -1
            green_x, green_y = gx_ref, gy_ref
        else:
            state = "WALL_GLIDE"
            green_x, green_y = cur_gx, cur_gy
            tx, ty = rotate90(unit_dx, unit_dy, direction=orbit_direction)
            if abs(tx) >= abs(ty):
                glide_axis = 'x'
                glide_sign = 1 if tx >= 0 else -1
                green_y = top if gy_nom < top else bottom
            else:
                glide_axis = 'y'
                glide_sign = 1 if ty >= 0 else -1
                green_x = left if gx_nom < left else right
            unit_dx, unit_dy = normalize(green_x - red_x, green_y - red_y)
    else:
        if glide_axis == 'x':
            green_x = clamp(green_x + glide_sign * TANGENTIAL_SPEED * dt, SAFETY_MARGIN, WIDTH - SAFETY_MARGIN)
            green_y = SAFETY_MARGIN if abs(green_y - SAFETY_MARGIN) < abs(green_y - (HEIGHT - SAFETY_MARGIN)) else HEIGHT - SAFETY_MARGIN
        else:
            green_y = clamp(green_y + glide_sign * TANGENTIAL_SPEED * dt, SAFETY_MARGIN, HEIGHT - SAFETY_MARGIN)
            green_x = SAFETY_MARGIN if abs(green_x - SAFETY_MARGIN) < abs(green_x - (WIDTH - SAFETY_MARGIN)) else WIDTH - SAFETY_MARGIN

        unit_dx, unit_dy = normalize(green_x - red_x, green_y - red_y)
        radial_distance = math.hypot(green_x - red_x, green_y - red_y)

        test_udx, test_udy = predict_phase_step(unit_dx, unit_dy, dt, orbit_direction)
        gx_test = red_x + test_udx * ORBIT_RADIUS
        gy_test = red_y + test_udy * ORBIT_RADIUS

        if inside_box(gx_test, gy_test):
            radial_distance = ORBIT_RADIUS
            unit_dx, unit_dy = normalize(test_udx, test_udy)
            green_x = red_x + unit_dx * radial_distance
            green_y = red_y + unit_dy * radial_distance
            state = "ORBIT"

        if (green_x in (SAFETY_MARGIN, WIDTH - SAFETY_MARGIN) and
            green_y in (SAFETY_MARGIN, HEIGHT - SAFETY_MARGIN)):
            glide_sign *= -1

    min_center_dist = min(min_center_dist, math.hypot((red_x + unit_dx * radial_distance) - red_x,
                                                      (red_y + unit_dy * radial_distance) - red_y))

    screen.fill(BLACK)
    pygame.draw.rect(screen, (40, 40, 40),
                     (SAFETY_MARGIN, SAFETY_MARGIN,
                      WIDTH - 2 * SAFETY_MARGIN, HEIGHT - 2 * SAFETY_MARGIN), 1)
    pygame.draw.circle(screen, WHITE, (int(red_x), int(red_y)), int(ORBIT_RADIUS), 1)
    pygame.draw.circle(screen, RED, (int(red_x), int(red_y)), DOT_RADIUS)
    pygame.draw.circle(screen, GREEN, (int(green_x), int(green_y)), DOT_RADIUS)

    if DEBUG:
        angle = (math.atan2(unit_dy, unit_dx) + 2 * math.pi) % (2 * math.pi)
        lines = [
            f"state={state}  r={radial_distance:6.1f}  angle={angle:.2f} rad",
            f"orbit_dir={'CW' if orbit_direction==1 else 'CCW'}  dives={dive_count}",
            f"glide_axis={glide_axis}  glide_sign={glide_sign}",
            "SPACE: dive,  C: flip orbit dir,  D: debug,  ESC: quit"
        ]
        for i, s in enumerate(lines):
            screen.blit(font.render(s, True, GREY), (10, 10 + 18 * i))

    pygame.display.flip()

pygame.quit()
sys.exit()
