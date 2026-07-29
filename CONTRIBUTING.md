# Contributing to OrbitLab

Thank you for helping improve OrbitLab.

## Development workflow

1. Create a focused branch from the current default branch.
2. Configure a Release or Debug build with CMake.
3. Keep physics changes independent of `src/app`.
4. Add or update tests for observable behavior.
5. Run the full local checks before opening a pull request:

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Run `clang-format` using the repository configuration on changed C++ files when available.
Avoid broad mechanical reformatting in behavior-focused changes.

For focused hardening work, use the matching preset:

```sh
cmake --preset asan-ubsan
cmake --build --preset asan-ubsan --parallel
ctest --preset asan-ubsan
```

GPU changes must retain a functional CPU/headless path. Add a deterministic CPU-reference
comparison for new compute kernels and document which shader formats were actually built
and exercised.

## Design and architecture expectations

- Prefer value types and RAII; do not introduce owning raw pointers.
- Keep platform and UI dependencies out of `include/orbitlab` and `src/core`.
- Validate external data before mutating the running simulation.
- Preserve deterministic tests by using explicit random seeds.
- Document non-obvious ownership, numeric, or performance decisions.
- Do not report benchmark numbers without the build type, compiler, hardware, and method.

## Pull requests

Describe the user-visible behavior, architectural impact, and verification performed. Keep
pull requests small enough to review. Generated build output, fetched dependencies, local
simulation files, and credentials must not be committed.

By contributing, you agree that your work is licensed under the project's MIT License.
