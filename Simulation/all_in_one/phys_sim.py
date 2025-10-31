import sys, math, pygame
from dataclasses import dataclass

# ========== Config & utils ==========

@dataclass
class Config:
    WIDTH: int = 900
    HEIGHT: int = 650
    FPS: int = 60

    ORBIT_RADIUS: float = 120.0
    TANGENTIAL_SPEED: float = 350.0
    DIVE_SPEED: float = 500.0

    DOT_RADIUS: int = 6
    SAFETY_MARGIN: int = 12

    DRAG: float = 4.0
    RESTITUTION: float = 0.25
    FRICTION: float = 0.6
    BETA: float = 0.20
    SLOP: float = 0.01
    FMAX: float = 1200.0

    STUN_TIME: float = 0.3
    STUN_FORCE_SCALE: float = 0.5

    EPS: float = 1.0

    BLACK: tuple = (8, 8, 10)
    RED: tuple = (250, 70, 70)
    GREEN: tuple = (70, 250, 140)
    WHITE: tuple = (235, 235, 235)
    GREY: tuple = (150, 150, 150)
    BOX: tuple = (60, 60, 70)

def clamp(v, lo, hi): return max(lo, min(hi, v))
def dot(ax, ay, bx, by): return ax*bx + ay*by
def norm(ax, ay):
    d = math.hypot(ax, ay)
    return ((ax/d, ay/d), d) if d > 1e-9 else ((1.0, 0.0), 0.0)

# ========== Physics primitives ==========

@dataclass
class Body:
    x: float; y: float
    vx: float = 0.0; vy: float = 0.0
    m: float = 1.0
    r: float = 6.0

def clamp_mag(Fx, Fy, Fmax):
    mag = math.hypot(Fx, Fy)
    if mag == 0 or mag <= Fmax: return Fx, Fy
    k = Fmax / mag
    return Fx*k, Fy*k

def resolve_dynamic_collision(a: Body, b: Body, e=0.25, mu=0.6, beta=0.2, slop=0.01):
    (nx, ny), dist = norm(a.x - b.x, a.y - b.y)
    pen = (a.r + b.r) - dist
    if pen <= 0: return False

    rvx, rvy = a.vx - b.vx, a.vy - b.vy
    vrel_n = dot(rvx, rvy, nx, ny)
    invM = (1.0/a.m + 1.0/b.m)
    collided = False

    if vrel_n < 0:
        j = -(1.0 + e) * vrel_n / invM
        a.vx += (j/a.m)*nx; a.vy += (j/a.m)*ny
        b.vx -= (j/b.m)*nx; b.vy -= (j/b.m)*ny

        tx, ty = -ny, nx
        vrel_t = dot(rvx, rvy, tx, ty)
        jt = -vrel_t / invM
        jt = max(-mu*abs(j), min(mu*abs(j), jt))
        a.vx += (jt/a.m)*tx; a.vy += (jt/a.m)*ty
        b.vx -= (jt/b.m)*tx; b.vy -= (jt/b.m)*ty
        collided = True

    corr = beta * max(pen - slop, 0.0) / invM
    a.x += (corr/a.m)*nx; a.y += (corr/a.m)*ny
    b.x -= (corr/b.m)*nx; b.y -= (corr/b.m)*ny
    return collided

def resolve_wall_collision(a: Body, left, right, top, bottom, e=0.2, mu=0.6, beta=0.2, slop=0.01):
    collided = False
    if a.x - a.r < left:
        pen = left - (a.x - a.r); nx, ny = 1.0, 0.0
        vrel_n = a.vx*nx + a.vy*ny
        if vrel_n < 0:
            j = -(1+e) * vrel_n * a.m
            a.vx += (j/a.m)*nx
            jt = -a.vy * a.m
            jt = max(-mu*abs(j), min(mu*abs(j), jt)); a.vy += jt/a.m
            collided = True
        a.x += beta * max(pen - slop, 0.0)
    if a.x + a.r > right:
        pen = (a.x + a.r) - right; nx, ny = -1.0, 0.0
        vrel_n = a.vx*nx + a.vy*ny
        if vrel_n < 0:
            j = -(1+e) * vrel_n * a.m
            a.vx += (j/a.m)*nx
            jt = -a.vy * a.m
            jt = max(-mu*abs(j), min(mu*abs(j), jt)); a.vy += jt/a.m
            collided = True
        a.x -= beta * max(pen - slop, 0.0)
    if a.y - a.r < top:
        pen = top - (a.y - a.r); nx, ny = 0.0, 1.0
        vrel_n = a.vx*nx + a.vy*ny
        if vrel_n < 0:
            j = -(1+e) * vrel_n * a.m
            a.vx += (j/a.m)*nx; a.vy += (j/a.m)*ny
            jt = -a.vx * a.m
            jt = max(-mu*abs(j), min(mu*abs(j), jt)); a.vx += jt/a.m
            collided = True
        a.y += beta * max(pen - slop, 0.0)
    if a.y + a.r > bottom:
        pen = (a.y + a.r) - bottom; nx, ny = 0.0, -1.0
        vrel_n = a.vx*nx + a.vy*ny
        if vrel_n < 0:
            j = -(1+e) * vrel_n * a.m
            a.vx += (j/a.m)*nx; a.vy += (j/a.m)*ny
            jt = -a.vx * a.m
            jt = max(-mu*abs(j), min(mu*abs(j), jt)); a.vx += jt/a.m
            collided = True
        a.y -= beta * max(pen - slop, 0.0)
    return collided

# ========== Controller ==========

def orbit_dive_force(robot: Body, target: Body, state: str, orbit_dir: int,
                     R: float, v_t_des: float, k_pr=10.0, k_dr=6.0,
                     k_pt=6.0, feedforward=True, Fmax=1200.0):
    rx, ry = robot.x - target.x, robot.y - target.y
    (erx, ery), dist = norm(rx, ry)
    etx, ety = (-ery, erx) if orbit_dir >= 0 else (ery, -erx)

    v_r = dot(robot.vx, robot.vy, erx, ery)
    v_t = dot(robot.vx, robot.vy, etx, ety)

    if state == "INWARD":
        R_des, v_t_set = 0.0, 0.0
    else:
        R_des, v_t_set = R, v_t_des

    Fr = (-k_pr * (dist - R_des) - k_dr * v_r)
    Ft = (-k_pt * (v_t - v_t_set))

    Fff = 0.0
    if feedforward and dist > 1e-3 and state != "INWARD":
        Fff = -(v_t_set**2 / dist)

    Fx = (Fr + Fff) * erx + Ft * etx
    Fy = (Fr + Fff) * ery + Ft * ety
    return clamp_mag(Fx, Fy, Fmax)

# ========== Entities ==========

class Target:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.body = Body(cfg.WIDTH*0.5, cfg.HEIGHT*0.5, m=9999.0, r=cfg.DOT_RADIUS+2)

    def update(self, dt: float):
        keys = pygame.key.get_pressed()
        tvx = (keys[pygame.K_RIGHT] - keys[pygame.K_LEFT]) * 220.0
        tvy = (keys[pygame.K_DOWN]  - keys[pygame.K_UP])   * 220.0
        self.body.x = clamp(self.body.x + tvx*dt, self.cfg.SAFETY_MARGIN, self.cfg.WIDTH - self.cfg.SAFETY_MARGIN)
        self.body.y = clamp(self.body.y + tvy*dt, self.cfg.SAFETY_MARGIN, self.cfg.HEIGHT - self.cfg.SAFETY_MARGIN)

    def draw(self, surf):
        pygame.draw.circle(surf, self.cfg.RED, (int(self.body.x), int(self.body.y)), self.cfg.DOT_RADIUS)

class Robot:
    def __init__(self, cfg: Config, target: Target):
        self.cfg = cfg
        self.target = target
        self.body = Body(cfg.WIDTH*0.5 + cfg.ORBIT_RADIUS, cfg.HEIGHT*0.5, m=1.0, r=cfg.DOT_RADIUS)
        ang_vel = cfg.TANGENTIAL_SPEED / cfg.ORBIT_RADIUS
        self.body.vx = 0.0
        self.body.vy = ang_vel * cfg.ORBIT_RADIUS

        self.state = "ORBIT"
        self.orbit_dir = 1
        self.stun = 0.0
        self.dives = 0

    def command_dive(self):
        if self.state == "ORBIT":
            self.state = "INWARD"
            self.dives += 1

    def flip_orbit(self):
        self.orbit_dir *= -1

    def update(self, dt: float):
        Fx, Fy = orbit_dive_force(
            self.body, self.target.body, self.state, self.orbit_dir,
            self.cfg.ORBIT_RADIUS, self.cfg.TANGENTIAL_SPEED,
            k_pr=10.0, k_dr=6.0, k_pt=6.0, feedforward=True, Fmax=self.cfg.FMAX
        )
        if self.stun > 0:
            Fx *= self.cfg.STUN_FORCE_SCALE
            Fy *= self.cfg.STUN_FORCE_SCALE
            self.stun = max(0.0, self.stun - dt)

        Fx += -self.cfg.DRAG * self.body.vx
        Fy += -self.cfg.DRAG * self.body.vy

        self.body.vx += (Fx / self.body.m) * dt
        self.body.vy += (Fy / self.body.m) * dt
        self.body.x  += self.body.vx * dt
        self.body.y  += self.body.vy * dt

        left, right = self.cfg.SAFETY_MARGIN, self.cfg.WIDTH - self.cfg.SAFETY_MARGIN
        top, bottom = self.cfg.SAFETY_MARGIN, self.cfg.HEIGHT - self.cfg.SAFETY_MARGIN

        hit_wall = resolve_wall_collision(
            self.body, left, right, top, bottom,
            e=self.cfg.RESTITUTION, mu=self.cfg.FRICTION, beta=self.cfg.BETA, slop=self.cfg.SLOP
        )
        hit_target = resolve_dynamic_collision(
            self.body, self.target.body,
            e=self.cfg.RESTITUTION, mu=self.cfg.FRICTION, beta=self.cfg.BETA, slop=self.cfg.SLOP
        )
        if hit_wall or hit_target:
            self.stun = self.cfg.STUN_TIME

        er_vec, r_now = norm(self.body.x - self.target.body.x, self.body.y - self.target.body.y)
        if self.state == "INWARD" and r_now <= 4.0:
            self.state = "OUTWARD"
        elif self.state == "OUTWARD" and r_now >= self.cfg.ORBIT_RADIUS - self.cfg.EPS:
            self.state = "ORBIT"

        erx, ery = er_vec
        etx, ety = -ery, erx  # CCW tangent
        vt_cw  = self.body.vx*(-etx) + self.body.vy*(-ety)
        vt_ccw = self.body.vx*( etx) + self.body.vy*( ety)
        self.orbit_dir = 1 if vt_ccw >= vt_cw else -1

    def draw(self, surf):
        pygame.draw.circle(surf, self.cfg.GREEN, (int(self.body.x), int(self.body.y)), self.cfg.DOT_RADIUS)

# ========== App / Game loop ==========

class App:
    def __init__(self, cfg=Config()):
        pygame.init()
        self.cfg = cfg
        self.screen = pygame.display.set_mode((cfg.WIDTH, cfg.HEIGHT))
        pygame.display.set_caption("Vortex — Physics Orbit/Dive")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 18)
        self.debug = True

        self.target = Target(cfg)
        self.robot = Robot(cfg, self.target)

    def handle_events(self):
        for e in pygame.event.get():
            if e.type == pygame.QUIT: return False
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE: return False
                if e.key == pygame.K_SPACE: self.robot.command_dive()
                if e.key == pygame.K_c: self.robot.flip_orbit()
                if e.key == pygame.K_d: self.debug = not self.debug
        return True

    def draw(self):
        self.screen.fill(self.cfg.BLACK)

        pygame.draw.rect(
            self.screen, self.cfg.BOX,
            (self.cfg.SAFETY_MARGIN, self.cfg.SAFETY_MARGIN,
             self.cfg.WIDTH - 2*self.cfg.SAFETY_MARGIN, self.cfg.HEIGHT - 2*self.cfg.SAFETY_MARGIN), 1
        )

        pygame.draw.circle(self.screen, self.cfg.WHITE,
                           (int(self.target.body.x), int(self.target.body.y)),
                           int(self.cfg.ORBIT_RADIUS), 1)

        self.target.draw(self.screen)
        self.robot.draw(self.screen)

        if self.debug:
            er_vec, r_now = norm(self.robot.body.x - self.target.body.x,
                                 self.robot.body.y - self.target.body.y)
            ex, ey = er_vec
            angle = math.atan2(ey, ex) % (2*math.pi)
            lines = [
                f"state={self.robot.state}  stun={self.robot.stun:.2f}s",
                f"r={r_now:6.1f}  angle={angle:.2f} rad  dir={'CCW' if self.robot.orbit_dir==1 else 'CW'}",
                "SPACE: dive   C: flip dir   D: HUD   ESC: quit"
            ]
            for i, s in enumerate(lines):
                self.screen.blit(self.font.render(s, True, self.cfg.GREY), (10, 10 + 18*i))

        pygame.display.flip()

    def run(self):
        running = True
        while running:
            dt_ms = self.clock.tick(self.cfg.FPS)
            dt = max(1e-3, dt_ms/1000.0)
            running = self.handle_events()
            self.target.update(dt)
            self.robot.update(dt)
            self.draw()
        pygame.quit(); sys.exit()

if __name__ == "__main__":
    App().run()
