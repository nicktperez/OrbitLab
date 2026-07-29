# OrbitLab Adaptive Fidelity Method

## Status

The OrbitLab Adaptive Fidelity Method (OAFM) is an original experimental
numerical-control heuristic implemented for this project. It is not a new law of
gravity, and the current evidence is not a mathematical proof of universal
accuracy or performance.

The method answers one engineering question:

> Can a deterministic combination of local acceleration, changing acceleration,
> and closing-encounter timescales improve an eccentric-orbit result over a coarse
> fixed step while using fewer steps than a fine fixed baseline?

That is a falsifiable hypothesis. OrbitLab runs the same automated comparison from
the desktop UI, CLI, and test suite.

## Equation

For body \(i\), let \(\ell_i\) be its nearest-neighbor distance, \(\mathbf a_i\)
its gravitational acceleration, and \(\mathbf j_i\) the finite-difference estimate
of jerk:

\[
\mathbf j_i \approx
\frac{\mathbf a_i(t)-\mathbf a_i(t-h_{\mathrm{previous}})}
     {h_{\mathrm{previous}}}.
\]

OrbitLab constructs three local timescales:

\[
\tau_{a,i} =
\sqrt{\frac{\max(\ell_i,\epsilon)}
           {\lVert\mathbf a_i\rVert}},
\]

\[
\tau_{j,i} =
\frac{\lVert\mathbf a_i\rVert}
     {\lVert\mathbf j_i\rVert},
\]

and, for pairs that are approaching,

\[
\tau_{c,i} =
\min_{k\ne i}
\frac{\lVert\mathbf r_{ik}\rVert}
     {\max(0,-\hat{\mathbf r}_{ik}\cdot\mathbf v_{ik})}.
\]

They are combined as a weighted inverse-quadratic timescale:

\[
\tau_i =
\left(
\frac{1}{\tau_{a,i}^{2}}
+\frac{w_j}{\tau_{j,i}^{2}}
+\frac{w_c}{\tau_{c,i}^{2}}
\right)^{-1/2}.
\]

The raw global step is:

\[
h_{\mathrm{raw}} =
\operatorname{clamp}
\left(
s\min_i\tau_i,\,
h_{\min},\,
h_{\max}
\right),
\]

where \(s\) is a safety factor. OrbitLab rounds this down to a power-of-two
subdivision of \(h_{\max}\):

\[
h =
\operatorname{clamp}
\left(
h_{\max}
2^{-\left\lceil
\log_2(h_{\max}/h_{\mathrm{raw}})
\right\rceil},
\frac{h_{\mathrm{previous}}}{4},
2h_{\mathrm{previous}}
\right).
\]

The last clamp prevents sudden fourfold shrinkage or greater-than-twofold growth.
Dyadic steps reduce unnecessary timestep jitter and make the controller's choices
easier to reproduce and inspect.

## Dimensional check

Each input to the combined equation has units of time:

- \(\tau_a:\sqrt{L/(L/T^2)}=T\)
- \(\tau_j:(L/T^2)/(L/T^3)=T\)
- \(\tau_c:L/(L/T)=T\)

The weights and safety factor are dimensionless. Therefore the selected \(h\) has
units of time regardless of the simulation's chosen physical unit system.

## Implementation boundary

`AdaptiveFidelityController` samples exact direct accelerations to audit the current
state, estimates jerk from consecutive accepted samples, and returns a decision
without mutating the simulation. `Application` or the CLI commits that decision only
after the corresponding simulation step succeeds.

The exact audit is intentionally independent of the active approximate solver. Its
cost is \(O(n^2)\), so interactive use is capped at 4,096 bodies. This makes the
current method an accuracy experiment, not a large-system optimization.

The current validated domain uses Runge–Kutta 4. Adaptive stepping is rejected with
the other integrators rather than silently applying an unvalidated combination.

## Falsification record

The first implementation used velocity Verlet. Its adaptive eccentric-orbit run
improved final position but produced worse energy drift than the coarse fixed
baseline. Variable steps disrupt the method's fixed-step symplectic behavior, so the
test correctly failed.

The pass condition was not weakened. OAFM v1 instead states RK4 as a precondition,
and the original comparison is rerun unchanged:

- adaptive final-position error must be lower than coarse fixed-step error;
- absolute adaptive energy drift must be lower than coarse fixed-step drift;
- adaptive step count must be lower than the fine fixed-step count.

## Reproduce the experiment

```sh
./build/orbitlab_cli validate-method
./build/orbitlab_cli validate-method --output orbitlab-method-report.json
ctest --test-dir build --output-on-failure
```

The report includes the equation settings and raw results for coarse fixed, fine
fixed, and adaptive runs. Runtime is reported but is not part of the pass condition,
because timing depends on hardware and system load.

## What has and has not been established

Current evidence establishes that the implementation:

- is dimensionally consistent;
- reacts to deterministic approaching encounters;
- emits bounded deterministic dyadic steps;
- improves the stated eccentric-orbit scenario under RK4;
- survives JSON persistence and headless execution.

It does not establish:

- optimal weights or safety factor;
- superiority across arbitrary N-body systems;
- preservation of symplectic structure;
- a formal global-error bound;
- better wall-clock performance for every body count.

Future validation should include randomized scenario families, adversarial close
encounters, statistical confidence intervals, comparison with established embedded
Runge–Kutta controllers, and a derivation of error bounds under explicit assumptions.
