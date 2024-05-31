# Toy Example of Hybrid Astar Algorithm
![path](path.gif)
Minimal hybird A* path planning algorithm implementation in C++ using Bazel build system and openCV for display. The project demonstrated:
- Hybrid A* path planning algorithm
- (fake) Reeds-Shepp path for shortcut the path
- Dijkstra's algorithm for guiding the Hybrid A* algorithm
- OpenCV for display
- Saving GIFs using openCV
- Bazel build system
- Hedron's Compile Commands Extractor for Bazel

## Installation

- [Bazel](https://bazel.build/install)         
- openCV ```sudo apt install -y libopencv-dev```
- To refresh the compile_commands.json, run ```bazel run @hedron_compile_commands//:refresh_all```, for more info, check out [Hedron's Compile Commands Extractor for Bazel](https://github.com/hedronvision/bazel-compile-commands-extractor)
## Usage

```bash
bazel run grid
```

