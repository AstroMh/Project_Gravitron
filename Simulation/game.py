import sys
import pygame
from .config import Config
from .entities import Target, OrbitingAgent
from .hud import HUD

class Game:
    def __init__(self, cfg: Config | None = None):
        self.cfg = cfg or Config()
        pygame.init()
        self.screen = pygame.display.set_mode((self.cfg.WIDTH, self.cfg.HEIGHT))
        pygame.display.set_caption("Circular Movement Simulation Version 01")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 18)

        self.target = Target(self.cfg)
        self.agent = OrbitingAgent(self.cfg, self.target)
        self.hud = HUD(self.cfg, self.font)

        self.debug = self.cfg.DEBUG
        self.running = True

    def _draw_bounds(self):
        pygame.draw.rect(
            self.screen,
            (40, 40, 40),
            (self.cfg.SAFETY_MARGIN,
             self.cfg.SAFETY_MARGIN,
             self.cfg.WIDTH - 2 * self.cfg.SAFETY_MARGIN,
             self.cfg.HEIGHT - 2 * self.cfg.SAFETY_MARGIN),
            1
        )

    def handle_event(self, event: pygame.event.Event):
        if event.type == pygame.QUIT:
            self.running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                self.running = False
            elif event.key == pygame.K_SPACE:
                self.agent.trigger_dive()
            elif event.key == pygame.K_c:
                self.agent.flip_orbit_dir()
            elif event.key == pygame.K_d:
                self.debug = not self.debug

    def run(self):
        while self.running:
            dt = self.clock.tick(self.cfg.FPS) / 1000.0

            for event in pygame.event.get():
                self.handle_event(event)

            keys = pygame.key.get_pressed()
            self.target.update(dt, keys)
            self.agent.update(dt)

            self.screen.fill(self.cfg.BLACK)
            self._draw_bounds()
            self.target.draw(self.screen)
            self.agent.draw(self.screen)
            if self.debug:
                self.hud.draw(self.screen, self.agent)

            pygame.display.flip()

        pygame.quit()
        sys.exit()
