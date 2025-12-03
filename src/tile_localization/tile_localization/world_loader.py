import yaml
import numpy as np

def load_world(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    resolution = data["resolution"]
    map_text = data["map"]

    rows = [
        list(row.rstrip())
        for row in map_text.splitlines()
        if row.strip() != ""
    ]

    rows.reverse()

    height = len(rows)
    width = max(len(row) for row in rows) if rows else 0

    grid = np.zeros((height, width), dtype=np.int8)

    for y in range(height):
        for x in range(min(len(rows[y]), width)):
            # Tokenizing light/dark values
            grid[y, x] = 100 if rows[y][x] == "#" else 0

    return resolution, width, height, grid