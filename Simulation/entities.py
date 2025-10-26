import math
import pygame
from .config import Config
from .utils import clamp, normalize, reflect_point, rotate90

class Target:
    """The red target the agent rotates around. (Moves with arrow keys)"""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.x = cfg.WIDTH*0.5
        self.y = cfg.HEIGHT*0.5

    @property
    def pos(self) -> tuple[float, float]:
        return self.x, self.y

    def update(self, dt: float, keys):
        tvx = (keys[pygame.K_RIGHT] - keys[pygame.K_LEFT]) * self.cfg.TARGET_SPEED
        tvy = (keys[pygame.K_DOWN] - keys[pygame.K_UP]) * self.cfg.TARGET_SPEED

        self.x = clamp(self.x + tvx * dt,
                       self.cfg.SAFETY_MARGIN,
                       self.cfg.WIDTH - self.cfg.SAFETY_MARGIN)
        self.y = clamp(self.y + tvy * dt,
                       self.cfg.SAFETY_MARGIN,
                       self.cfg.HEIGHT - self.cfg.SAFETY_MARGIN)

    def draw(self, surf: pygame.Surface):
        pygame.draw.circle(surf, self.cfg.RED, (int(self.x), int(self.y)), self.cfg.DOT_RADIUS)
        # orbit guide circle
        pygame.draw.circle(surf, self.cfg.WHITE, (int(self.x), int(self.y)), int(self.cfg.ORBIT_RADIUS), 1)

class OrbitingAgent:
    """The green agent that orbits/dives/glides relative to the target."""

    def __init__(self, cfg: Config, target: Target):
        self.cfg = cfg
        self.target = target

        # Orientation on the orbit (unit direction from target to agent)
        self.udx, self.udy = 1.0, 0.0

        # Orbit parameters
        self.orbit_direction = 1  # 1 => CW display, -1 => CCW
        self.radial_distance = cfg.ORBIT_RADIUS

        # State machine
        self.state = "ORBIT"  # ORBIT, INWARD, OUTWARD, WALL_GLIDE
        self.glide_axis: str | None = None  # 'x' or 'y'
        self.glide_sign: int = 0

        # Agent position
        self.gx = self.target.x + self.udx * self.radial_distance
        self.gy = self.target.y + self.udy * self.radial_distance

        # Stats
        self.dive_count = 0
        self.min_center_dist = float("inf")

    def trigger_dive(self):
        if self.state == "ORBIT":
            self.state = "INWARD"
            self.dive_count += 1

    def flip_orbit_dir(self):
        self.orbit_direction *= -1

    def _predict_phase_step(self, udx: float, udy: float, dt: float) -> tuple[float, float]:
        dtheta = self.cfg.ANGULAR_SPEED * self.orbit_direction * dt
        c, s = math.cos(dtheta), math.sin(dtheta)
        ndx = udx * c - udy * s
        ndy = udx * s + udy * c
        return normalize(ndx, ndy)

    def _inside_box(self, x: float, y: float) -> bool:
        return (
            self.cfg.SAFETY_MARGIN <= x <= self.cfg.WIDTH - self.cfg.SAFETY_MARGIN and
            self.cfg.SAFETY_MARGIN <= y <= self.cfg.HEIGHT - self.cfg.SAFETY_MARGIN
        )

    def update(self, dt: float):
        tx, ty = self.target.pos

        # Radial motion (diving in/out)
        if self.state == "INWARD":
            self.radial_distance = max(0.0, self.radial_distance - self.cfg.DIVE_SPEED * dt)
            if self.radial_distance <= self.cfg.EPS:
                self.state = "OUTWARD"
                self.radial_distance = self.cfg.EPS
        elif self.state == "OUTWARD":
            self.radial_distance = min(self.cfg.ORBIT_RADIUS, self.radial_distance + self.cfg.DIVE_SPEED * dt)
            if self.radial_distance >= self.cfg.ORBIT_RADIUS - self.cfg.EPS:
                self.state = "ORBIT"
                self.radial_distance = self.cfg.ORBIT_RADIUS
        else:
            self.radial_distance = self.cfg.ORBIT_RADIUS if self.state != "WALL_GLIDE" else self.radial_distance

        # Current nominal
        cur_gx = tx + self.udx * self.radial_distance
        cur_gy = ty + self.udy * self.radial_distance

        left, right = self.cfg.SAFETY_MARGIN, self.cfg.WIDTH - self.cfg.SAFETY_MARGIN
        top, bottom = self.cfg.SAFETY_MARGIN, self.cfg.HEIGHT - self.cfg.SAFETY_MARGIN

        if self.state != "WALL_GLIDE":
            ndx, ndy = self._predict_phase_step(self.udx, self.udy, dt)
            gx_nom = tx + ndx * self.radial_distance
            gy_nom = ty + ndy * self.radial_distance

            hit_x = (gx_nom < left) or (gx_nom > right)
            hit_y = (gy_nom < top) or (gy_nom > bottom)

            if not (hit_x or hit_y):
                self.udx, self.udy = ndx, ndy
                self.gx, self.gy = gx_nom, gy_nom
            elif hit_x ^ hit_y:
                gx_ref, gy_ref = reflect_point(gx_nom, gy_nom, left=left, right=right, top=top, bottom=bottom)
                self.udx, self.udy = normalize(gx_ref - tx, gy_ref - ty)
                self.flip_orbit_dir()
                self.gx, self.gy = gx_ref, gy_ref
            else:
                # Corner -> wall glide
                self.state = "WALL_GLIDE"
                self.gx, self.gy = cur_gx, cur_gy
                tx90, ty90 = rotate90(self.udx, self.udy, direction=self.orbit_direction)
                if abs(tx90) >= abs(ty90):
                    self.glide_axis = 'x'
                    self.glide_sign = 1 if tx90 >= 0 else -1
                    self.gy = top if gy_nom < top else bottom
                else:
                    self.glide_axis = 'y'
                    self.glide_sign = 1 if ty90 >= 0 else -1
                    self.gx = left if gx_nom < left else right
                self.udx, self.udy = normalize(self.gx - tx, self.gy - ty)
        else:
            # Glide along the wall
            if self.glide_axis == 'x':
                self.gx = clamp(self.gx + self.glide_sign * self.cfg.TANGENTIAL_SPEED * dt, left, right)
                self.gy = top if abs(self.gy - top) < abs(self.gy - bottom) else bottom
            else:
                self.gy = clamp(self.gy + self.glide_sign * self.cfg.TANGENTIAL_SPEED * dt, top, bottom)
                self.gx = left if abs(self.gx - left) < abs(self.gx - right) else right

            self.udx, self.udy = normalize(self.gx - tx, self.gy - ty)
            self.radial_distance = math.hypot(self.gx - tx, self.gy - ty)

            test_udx, test_udy = self._predict_phase_step(self.udx, self.udy, dt)
            gx_test = tx + test_udx * self.cfg.ORBIT_RADIUS
            gy_test = ty + test_udy * self.cfg.ORBIT_RADIUS

            if self._inside_box(gx_test, gy_test):
                self.radial_distance = self.cfg.ORBIT_RADIUS
                self.udx, self.udy = normalize(test_udx, test_udy)
                self.gx = tx + self.udx * self.radial_distance
                self.gy = ty + self.udy * self.radial_distance
                self.state = "ORBIT"

            if (self.gx in (left, right)) and (self.gy in (top, bottom)):
                self.glide_sign *= -1

        self.min_center_dist = min(
            self.min_center_dist,
            math.hypot((tx + self.udx * self.radial_distance) - tx,
                       (ty + self.udy * self.radial_distance) - ty)
        )

    def draw(self, surf: pygame.Surface):
        pygame.draw.circle(surf, self.cfg.GREEN, (int(self.gx), int(self.gy)), self.cfg.DOT_RADIUS)