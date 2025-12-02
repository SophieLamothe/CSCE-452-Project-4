import yaml
import numpy as np

def load_world(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    resolution = data["resolution"]
    map_text = data["map"]

    rows = [
        list(row.strip())
        for row in map_text.splitlines()
        if row.strip() != ""
    ]

    rows.reverse()  # bottom row first

    height = len(rows)
    width = len(rows[0])

    grid = np.zeros((height, width), dtype=np.int8)

    for y in range(height):
        for x in range(width):
            grid[y, x] = 100 if rows[y][x] == "#" else 0

    return resolution, width, height, grid
