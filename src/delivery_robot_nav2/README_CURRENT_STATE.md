# delivery_robot_nav2 — current tested state

- `costmaps_v1.yaml` and `costmaps_only.launch.py`: standalone Global/Local Costmap bring-up. Both lifecycle nodes reached `active [3]`; local grid was 60x60 at 0.05 m. This is a development test, not the final full Nav2 architecture.
- `planner_v1.yaml` and `planner_only.launch.py`: tested Global Planner. Global costmap must use StaticLayer + InflationLayer only. NavFn A* returned a successful path (`error_code=0`) to goal (-0.613, 0.339).
- Do not run `costmaps_only.launch.py` together with `planner_only.launch.py`; Planner Server hosts its own global costmap.
- Controller, behavior server and BT Navigator are not yet tested.
