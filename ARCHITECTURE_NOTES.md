# Architecture Notes: Why this Controller Works Better

This controller succeeds because it is not a single reactive visual controller and it is not a one-shot map planner. It continuously combines several imperfect sensors into a practical navigation loop.

## 1. Evidence-based occupancy grid

The LiDAR map is not a single scan image. Each ray adds free-space evidence along the beam and occupied evidence at obstacle endpoints. This makes walls become crisp over time and reduces the effect of occasional noisy measurements.

## 2. Separate semantic layers

Green floor and low-clearance obstacles are not stored as ordinary LiDAR obstacles. They are separate hard-forbidden layers, so later LiDAR free-space rays cannot erase them. This is a major reason the final map can show both walls and green carpets cleanly.

## 3. Frontier exploration instead of assuming targets are visible

The robot searches unknown space using frontiers: known free cells touching unknown cells. This matters for harder mazes where blue/yellow are not visible from the start.

## 4. Mission state machine

The high-level mission is explicit:

```text
SEARCH_BLUE -> NAVIGATE_BLUE -> SEARCH_YELLOW -> NAVIGATE_YELLOW -> FINISHED
```

Yellow can be cached while searching for blue, so after blue is reached the robot can immediately use an earlier yellow observation instead of starting from zero.

## 5. Target approach planning

The robot does not drive directly at the pillar center. It samples safe standoff points around the target and uses A* to reach an approach point, then verifies arrival visually using color coverage.

## 6. Continuous invalidation and recovery

The active path is invalidated when newly mapped green or low-clearance obstacles intersect it. Collision recovery backs up, marks the contact area temporarily, and replans. That is why it can solve mazes where a simple A* path becomes invalid while moving.

## 7. Low-clearance / floating-wall handling

The depth camera is used to detect horizontal crossbars or floating walls that 2-D LiDAR may miss. It classifies the nearest central depth surface by its bottom image edge and only promotes low-clearance obstacles after confirmations.

## 8. Practical fallbacks

When no reachable frontier is found, the controller first clears temporary recovery marks, then does a 360-degree scan, and finally performs safe exploratory movement instead of immediately terminating.
