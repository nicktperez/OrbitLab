# OrbitLab Design System

## Intent

OrbitLab is used by a recruiter at a desk, often beside an IDE or README, evaluating the
clarity of both the software and its implementation. The UI resembles a compact optical
instrument in a dim lab: a neutral dark field, crisp markings, and restrained green status
indicators. The strategy is restrained; color communicates state and celestial identity.

## Color

The source palette is expressed in OKLCH; runtime values are converted to sRGB for SDL and
Dear ImGui.

- Background: `oklch(0.075 0 0)`
- Canvas: `oklch(0.055 0 0)`
- Surface: `oklch(0.13 0.008 160)`
- Raised surface: `oklch(0.18 0.010 160)`
- Primary: `oklch(0.68 0.13 160)`
- Accent: `oklch(0.78 0.12 78)`
- Ink: `oklch(0.94 0.006 160)`
- Muted: `oklch(0.70 0.012 160)`
- Error: `oklch(0.67 0.17 25)`
- Selection: `oklch(0.82 0.10 160)`

Primary color is reserved for running/active state, selection, and primary actions. Amber
is reserved for warnings and the Sun. Errors always include text.

## Typography

Use Dear ImGui's bundled sans-serif font at a readable 16 px base size. Titles use weight
and spacing rather than a display face. Numeric telemetry uses tabular alignment where
available. Labels use sentence case; avoid small tracked uppercase text.

## Layout

A fixed 368 px instrument panel sits at the left. The simulation canvas owns the remaining
space. The panel follows the operator's workflow: transport and status, scenario, selected
body, numerical tools, then persistence. Advanced plots and performance controls use
collapsed progressive-disclosure sections. Controls use a consistent 28–32 px height and
8 px spacing.

## Components

- Transport controls: compact, adjacent buttons for pause/run and single-step.
- Status strip: FPS, body count, and step duration presented as aligned rows.
- Inspector: direct labeled inputs with validation feedback next to the affected state.
- Preset chooser: a familiar combo box followed by an explicit load action.
- Error banner: tinted full-width surface with plain-language error text.
- Canvas overlay: minimal scale/readout text; no floating decorative cards.
- 3D navigation: standard orbit/pan/zoom gestures, with explicit azimuth/elevation controls
  under progressive disclosure for discoverability and precise reset views.
- Depth: body scale and trail projection communicate distance; color remains reserved for
  identity rather than being repurposed as an ambiguous depth heatmap.
- Experimental method: a collapsed instrument section explains the purpose first, keeps
  equation weights under secondary disclosure, and presents pass/fail in text as well as color.

## Motion

Only simulation motion, camera orbit, trajectory accumulation, and direct hover/active
feedback are used.
UI transitions are immediate; the simulator must feel responsive and deterministic.
