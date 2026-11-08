# Changes

## Unreleased

- Rust edition 2024 (minimum Rust 1.85) for every crate but osm2streets-py, whose pyo3 0.17 macros trip the 2024 unsafe lints

- Roads tagged `lanes:bus` (or another per-mode lane count) without `lanes`, and lanes tagged `width=0`, no longer come out zero metres wide, which made rendering panic
- Renderers skip a road or lane whose outline isn't a valid polygon instead of panicking (needs geom's `try_make_polygons`)
- Footway and path outlines skip a side that a sharp bend makes impossible to shift, instead of panicking
- Sidewalk corners are left out at an intersection where a road can't be shifted to its edges, instead of panicking (`RoadEdge::try_calculate`)

## 0.1.4

- Add `debugClockwiseOrderingForIntersectionGeojson`
- Add speed limit to roads
- Bugfixes

## 0.1.3

- New API calls to dump the entire StreetNetwork as JSON
- Various importing improvements (5 months of changes; just go see git changelog)

## 0.1.2

- Adding a second package, `osm2streets-js-node` with `--target nodejs`

## 0.1.1

- Releasing the NPM package again, but with `--target web` this time

## 0.1.0

- Initial NPM package
