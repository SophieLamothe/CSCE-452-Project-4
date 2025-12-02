# CSCE452_Project4

## Run using launch files:
- cd CSCE452_Project4
- colcon build
- source install/setup.bash

## Rviz setup
✅ 2. Set the RViz Fixed Frame

In the left panel:

Global Options → Fixed Frame → map

If map does not exist, create it manually:

In your /floor publisher, msg.header.frame_id should be "map"

If you haven't set it yet, I'll show you how — but for now set Fixed Frame to map.

✅ 3. Add a "Map" Display

Left panel → Add (bottom-left) → By display type → Map

Then set:

Setting	Value
Topic	/floor
Color Scheme	map (or raw)
Enabled	✔

You should now see the tile map appear as black and white blocks.

If the map is invisible, check:

Map → Draw Behind = Off

Map → Alpha = 1.0

Floor map resolution and dimensions appear correct

✅ 4. Add the Estimated Pose

Pose2D is not directly supported in RViz, but we can visualize it as an Arrow by converting it inside the visualizer.

You have two options:

Option A — Easiest: Use "Pose" Display

RViz is fine with receiving Pose messages without orientation.

Add → Pose

Set:

Setting	Value
Topic	/estimated_pose
Shape	Arrow
Color	e.g., yellow
Enabled	✔

If RViz complains about missing header stamps or frame_id, we will update your code to publish them.

Option B — Use a TF Frame ("robot")

Only needed if you want to be fancy.

Later you may generate a TF:

map → robot


Then visualize the robot with:

Add → TF

Or Add → Axes (frame = robot)

But that requires more code.

✅ 5. Add a Grid (optional but helpful)

Add → Grid

Settings:

Setting	Value
Plane	XY
Color	Light gray
Cell Size	1.0
Enabled	✔

This helps visually check position alignment.
