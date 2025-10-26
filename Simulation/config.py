from dataclasses import dataclass

@dataclass
class Config:
    WIDTH: int = 800
    HEIGHT: int = 600
    FPS: int = 60

    ORBIT_RADIUS: float = 120.0
    TANGENTIAL_SPEED: float = 300.0
    DIVE_SPEED: float = 320.0

    DOT_RADIUS: int = 5
    SAFETY_MARGIN: int = 12
    TARGET_SPEED: float = 220.0
    EPS: float = 1.0

    BLACK: tuple[int, int, int] = (0, 0, 0)
    RED: tuple[int, int, int] = (255, 0, 0)
    GREEN: tuple[int, int, int] = (0, 255, 0)
    WHITE: tuple[int, int, int] = (255, 255, 255)
    GREY: tuple[int, int, int] = (200, 200, 200)

    DEBUG: bool = True

    @property
    def ANGULAR_SPEED(self) -> float:
        # v_t = r * omega -> omega = v_t / r
        return self.TANGENTIAL_SPEED / self.ORBIT_RADIUS