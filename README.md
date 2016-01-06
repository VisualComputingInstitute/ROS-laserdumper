# ROS-laserdumper
Dump ROS LaserScan data into csv files.

# Usage

Run like so:

```
rosrun lb_laserdumper lb_laserdumper _file:=scan.csv _scan:=/scan
```

The two values shown above are actually the default values of these parameters.
A few notes:

- Scans are written out one scan per line.
- Laser-scans are **appended** to `scan.csv` if the file already exists.
- `_file` may contain unix-y things like `~/foo.csv`.
