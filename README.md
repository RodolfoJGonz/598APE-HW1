# 598APE-HW1 — Ray Tracer

Implementation of the CS598APE HW1 ray tracer, plus a series of
profiling-driven optimizations. The renderer produces still images
(PPM/PNG) and animated movies (MP4, encoded losslessly).

- `main.cpp`, `src/` — the renderer; `inputs/` — scenes (pianoroom static,
  globe animated, elephant = 111,748-triangle mesh with orbiting camera);
  `data/` — mesh files; `images/` — textures; `docker/` + `dockerrun.sh` —
  build environment; `notes.txt` — development log (timings, future work).

## 1. Tools

No other dependencies (no libraries, no submodules). The code uses GNU
statement expressions, so g++ is required (clang will not compile it).
Everything above is installed in the Docker image
(`docker/Dockerfile`, Ubuntu 24.04):

```bash
docker build -t <user>/598ape docker/
./dockerrun.sh        # mounts this repo at /host and opens a shell
cd /host && make
```

Build flags: `-O3 -g -Werror -fopenmp -fno-omit-frame-pointer -flto`
(the `-fno-omit-frame-pointer` is required for `perf --call-graph fp`).

## 2. Build and run

```bash
make clean && make    # compiles src/*.cpp, links ./main.exe
```

```
./main.exe [-H <h>] [-W <w>] [-F <frames>] [--movie] [--png] [--ppm]
           [-o <out>] [-i <scene.ray>] [-a <animation>] [--help]
```

- **Use `--ppm` for benchmarks** (PNG goes through ImageMagick per
  frame and is lossy).
- Multi-frame runs write `<out>.tmp.%07d.ppm` per frame, then encode with
  ffmpeg **losslessly** (`ffv1` → `libx264 -preset veryslow -qp 0`). The
  encode is slow by design — it exists so the movie contains exactly the
  rendered pixels.
- **Timing:** the program prints `Total time to create images=X seconds`
  — that is render-only time and the number to compare between builds;
  movie wall-clock additionally includes the lossless encode.

### Benchmarks

```bash
# B1 — static scene, single frame
./main.exe -i inputs/pianoroom.ray --ppm -o output/pianoroom.ppm -H 500 -W 500

# B2 — elephant mesh, 24 frames @ 100×100
./main.exe -i inputs/elephant.ray --ppm -a inputs/elephant.animate \
           --movie -F 24 -W 100 -H 100 -o output/sphere.mp4

# B3 — same scene @ 300×300
./main.exe -i inputs/elephant.ray --ppm -a inputs/elephant.animate \
           --movie -F 24 -W 300 -H 300 -o output/sphere.mp4
```

Reference times (16-core machine, "Total time" line): B1 ≈ 0.04 s;
B2 ≈ 3.06 s; B3 ≈ 20.9 s. Times scale with core count and machine speed;
**ratios between builds are the reproducible quantity.**

### Changing the mesh (elephant scene)

The elephant is a `mesh` shape in `inputs/elephant.ray`, specified by:

```
mesh
<vertex file> <num verts> <face file> <num faces> <off_x> <off_y> <off_z>
<texture block>
<normal-map block>
```

The vertex file has one `x y z` per line; the face file has one `i j k`
per line (three 0-based vertex indices). The offsets translate every
vertex (centering the model under the camera and light).

Two meshes ship in `data/`: the demo sphere (`x.txt`/`f.txt`, 3,168
triangles — fast) and the full elephant (`elepx.txt`/`elepf.txt`, 111,748
triangles). To switch between them, comment one `mesh` line out and
uncomment the other in `inputs/elephant.ray`:

```
# data/x.txt 1586 data/f.txt 3168 -1.58 -.43 2.7
data/elepx.txt 62779 data/elepf.txt 111748 -1.58 -.43 2.7
```

(This is exactly the scene change commit `44f7013` made; the current
`inputs/elephant.ray` uses the full mesh.) For your own model, export
vertices + triangle indices to those two formats and keep it near the
origin, ~1–2 units across — the camera orbits at radius 2. The bounding-
sphere gate (section 3) is rebuilt automatically from whatever mesh loads.

### Running the baseline

The baseline is commit **`19bbc81`** — the last commit before optimization
work began (original upstream code is `d98f5dc`). The baseline
`src/Makefile` compiled `src/` at **-O0** and the render loop was
**serial** (no OpenMP) — check it out as-is, and do not port the
optimized Makefile flags back in.

```bash
git checkout 19bbc81 && make clean && make
./main.exe -i inputs/pianoroom.ray --ppm -o output/pianoroom.ppm -H 500 -W 500
# reference: Total time to create images=1.070294 seconds
```

## 3. The optimizations, and how to evaluate each

Timings are render-only ("Total time to create images") on the reference
16-core machine.

| Commit | Change | How to evaluate | Effect |
|---|---|---|---|
| `dfa831d` | -O0→-O3 for `src/`; `-flto`, `-fopenmp`, `-fno-omit-frame-pointer`; collect+sort→min-search in `calcColor`; `#pragma omp parallel for schedule(dynamic,16)` over pixels in `refresh` | B1: `19bbc81` vs `dfa831d` | 1.070 → 0.040 s (~27×) |
| `eb38003` + `44cf0a2` | `solveScalers` algebra rearrangement (fewer flops in the Cramer's rule used by every plane/triangle test and shadow test); `44cf0a2` fixes a slip introduced by `eb38003` | effect is profile-visible (`solveScalers` self-time) | small |
| `44f7013` | **Bounding-sphere gate** for meshes; same commit swaps `elephant.ray` to the full mesh | recipe below | 43 s → 3.06 s (render), 14× on B2 |
| `16f1046` | **Möller–Trumbore** in `Triangle::getIntersection` | B3: 35 s → 20.9 s; profile `solveScalers` 11.3% → 2.1% | ~1.7× on B3 |
| `c58cfd1` (HEAD) | `calcColor`: hoist `1 - opacity` / `1 - reflection` out of the per-channel blends | small timing delta | small |

Notes per commit:

- **`dfa831d`** bundles four independent changes; the intermediate 500×500
  timings (sort→min 0.745 s, +OpenMP 0.096 s, +dynamic schedule 0.072 s,
  +O3 0.040 s) are in `notes.txt`. The OpenMP loop is safe because each
  pixel is an independent pure function of the scene writing only to its
  own 3-byte slot; the schedule is dynamic because pixel costs vary ~50×
  (background pixel vs mesh-hit pixel).
- **`44f7013`** — when a mesh loads, `main.cpp` computes a bounding sphere
  (bbox center, radius = max vertex distance) and stores it as
  `Autonoma::gate`; `calcColor` skips the entire triangle scan for rays
  that miss it (~97% of rays in the elephant scene, since the elephant
  occupies ~3% of the frame). Valid while the mesh encloses all hittable
  geometry (true for `elephant.ray`). To evaluate the code change without
  the scene swap:

  ```bash
  git checkout 44f7013
  git checkout 44f7013^ -- main.cpp src/light.cpp src/light.h src/shape.cpp
  make clean && make
  ./main.exe -i inputs/elephant.ray --ppm -a inputs/elephant.animate \
             --movie -F 24 -W 100 -H 100 -o output/base_g.mp4   # ≈ 43 s wall
  git checkout 44f7013 -- main.cpp src/light.cpp src/light.h src/shape.cpp
  make clean && make
  ./main.exe -i inputs/elephant.ray --ppm -a inputs/elephant.animate \
             --movie -F 24 -W 100 -H 100 -o output/opt_g.mp4    # ≈ 3 s render
  ```
- **`16f1046`** — replaces the plane test + Cramer's rule + three half-plane
  sign tests with the standard Möller–Trumbore test: one division instead of
  three, early rejection as soon as a barycentric coordinate is out of
  range, no `solveScalers`. Double-sided (mesh winding unknown) and keeps
  the original `t > 0` visibility contract, hence bit-identical output.
