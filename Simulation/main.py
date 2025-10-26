from .game import Game
from .config import Config

def main():
    Game(Config()).run()

if __name__ == "__main__":
    main()
