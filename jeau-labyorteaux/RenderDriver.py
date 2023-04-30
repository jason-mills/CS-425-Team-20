import sys
from Renderer import Renderer
import open3d.visualization.gui as gui


def main():
    editor = Renderer(sys.argv[1])


if __name__ == '__main__':
    main()