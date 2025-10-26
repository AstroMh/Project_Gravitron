import math
import pygame
from .config import Config
from .entities import OrbitingAgent

class HUD:
    def __init__(self, cfg: Config, font: pygame.font.Font):
        self.cfg = cfg
        self.font = font

    def draw(self, surf: pygame.Surface, agent: OrbitingAgent):
        angle = (math.atan2(agent.udy, agent.udx) + 2 * math.pi) % (2 * math.pi)
        lines = [
            f"state={agent.state}  r={agent.radial_distance:6.1f}  angle={angle:.2f} rad",
            f"orbit_dir={'CW' if agent.orbit_direction==1 else 'CCW'}  dives={agent.dive_count}",
            f"glide_axis={agent.glide_axis}  glide_sign={agent.glide_sign}",
            "SPACE: dive,  C: flip orbit dir,  D: debug,  ESC: quit"
        ]
        for i, s in enumerate(lines):
            surf.blit(self.font.render(s, True, self.cfg.GREY), (10, 10 + 18 * i))
