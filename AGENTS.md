# PhysicsEngine: guide for AI agents

## Current stage

The repository contains a compilable C++20 MVP of a deterministic N-body space
physics engine. The current implementation is suitable for correctness tests and
initial performance measurements. It is not yet a production orbital integrator.

Validated in the current stage:

- fixed-sector LWC coordinates with `int64_t` sector indices and `double` local meters;
- direct softened Newtonian gravity, `O(N^2)`;
- Velocity Verlet integration;
- fixed timestep accumulator;
- pre-sized SoA state and no capacity-changing operation in `step()` or the force pass;
- persistent native worker threads with fixed body ranges;
- bit-identical results between one thread and the tested parallel configuration;
- external acceleration (thrust) per body;
- bulk position/sector and velocity read APIs;
- a 2000-body benchmark executable.

## Ownership map

- `include/physics/PhysicsEngine.hpp`: public data contract, SoA API, thread-pool state,
  thrust controls, bulk output layout, and fixed timestep declaration.
- `src/PhysicsEngine.cpp`: LWC displacement, direct gravity, worker synchronization,
  normalization, Velocity Verlet, thrust composition, and bulk copies.
- `src/main.cpp`: four-body manual demonstration with 1000 one-hour fixed steps.
- `tests/PhysicsEngineTests.cpp`: focused behavior tests and serial/parallel equality test.
- `tests/benchmark.cpp`: deterministic 2000-body throughput measurement.
- `CMakeLists.txt`: C++20 targets and CTest registration.

## Data and numerical methods

Physical state uses structure-of-arrays vectors. For body `i`, local position is
`(positionX[i], positionY[i], positionZ[i])` and world position is the sector plus
that local offset. Relative displacement is computed as:

`(sectorB - sectorA) * SectorSize + localB - localA`

Gravity uses softened acceleration:

`a_i += G * m_j * r_ij / (|r_ij|^2 + epsilon^2)^(3/2)`

The outer body range is partitioned into fixed contiguous ranges. Each worker writes
only its own acceleration slots, while the inner source-body loop keeps a fixed
serial order. This is why the serial and parallel results are bit-identical for the
same compiler and floating-point settings.

## Zero-allocation contract

All SoA arrays and worker threads are created in the `PhysicsEngine` constructor.
`computeAccelerations()` and `step()` must not create containers, resize vectors,
spawn threads, or change vector capacity. Bulk APIs require caller-owned output
buffers sized to `3 * bodyCount()` values. Callers must not invoke mutating methods
concurrently with simulation or bulk reads.

## Development commands

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
& .\build\Release\space_physics_benchmark.exe
& .\build\Release\space_physics_demo.exe
```

## Next roadmap

1. Add a public immutable snapshot or double-buffered exchange protocol for render/network consumers.
2. Add optional spatial queries with caller-provided output buffers and explicit overflow reporting.
3. Add allocation instrumentation and a benchmark matrix for 1, 2, 4, and hardware threads.
4. Add compiler-specific SIMD diagnostics and optional AVX2 kernels behind a build flag.
5. Add stronger orbital regression fixtures, energy/momentum drift metrics, and long-run accuracy checks.
6. Consider Barnes-Hut or a hybrid near/far solver when `O(N^2)` no longer meets the target body count.

## Agent rules

- Preserve C++20 and the no-external-dependency policy.
- Do not replace SoA with body objects or add per-step allocations.
- Keep deterministic serial and parallel paths comparable in tests.
- Update this document when ownership, numerical methods, public layout, or roadmap changes.
- Run build, CTest, demo, and relevant benchmark checks before declaring a physics change complete.