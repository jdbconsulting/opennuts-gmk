# opennuts-gmk -- Geometric Modeling Kernel

A minimal, dependency-free, C++23 geometric modeling kernel with NURBS
B-rep support. Ships with an LSP server for the **OpenNuts** MCAD
language and a JSON-RPC geometry server suitable for driving a 3D
display extension.

Priorities, in order:

1. **Data safety.** No exceptions on the hot path; every fallible
   operation returns `Status`/`Result<T>`. Topology storage uses
   versioned handles so stale references are detected, not
   dereferenced.
2. **Predictable memory.** No external dependencies. Pools and arenas
   back the data structures that would otherwise allocate per call.
3. **First-class precision.** All world coordinates are stored as
   `int64_t` femtometres (1 unit = 10⁻¹⁵ m), giving ±9 223 km of range
   with sub-picometre precision. Angles are `int32_t` thousandths-of-
   a-degree clamped to ±180°.
4. **Small surface area.** ~5 000 lines of code across the kernel and
   servers, intended to be readable end-to-end by a small team.

## Layout

```
opennuts-gmk/
├── CMakeLists.txt
├── include/gmk/             Public headers (kernel API)
│   ├── result.hpp            Status enum + Result<T> error-as-value type
│   ├── units.hpp             INT64 lengths, INT32 angles, conversions
│   ├── arena.hpp             Bump allocator
│   ├── pool.hpp              Versioned dense object pool
│   ├── math/                 Vec3i / Vec3d / AABB / Transform
│   ├── geom/                 Curves and Surfaces
│   │   ├── curve.hpp, nurbs_curve.hpp, analytic_curves.hpp
│   │   └── surface.hpp, nurbs_surface.hpp, analytic_surfaces.hpp
│   ├── brep/                 B-rep topology and primitive constructors
│   ├── ops/                  Body-level operations (booleans -- stubbed)
│   ├── tess/                 Tessellation to triangle meshes
│   ├── lang/                 OpenNuts MCAD language (lexer, parser, AST)
│   ├── server/               JSON, JSON-RPC, LSP, geometry server
│   └── version.hpp
├── src/                     Implementations of the above.
├── wasm/entry.cpp           C-ABI entry points for the emscripten build.
├── tests/                   Hand-rolled test harness, ~30 unit tests.
└── examples/widget.opennuts An OpenNuts source file demonstrating the language.
```

## Build

### Native

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
build/bin/opennuts check        # build a few primitives and validate them
build/bin/gmk_tests             # run the unit-test suite
```

### WASM (emscripten)

Requires the `emsdk` SDK on your `PATH`:

```
source ~/emsdk/emsdk_env.sh
emcmake cmake -S . -B build-wasm
cmake --build build-wasm -j
```

This produces `build-wasm/bin/gmk_wasm.js` + `gmk_wasm.wasm`. The
JavaScript surface is a tiny C ABI: `gmk_version`, `gmk_session_new`,
`gmk_session_free`, `gmk_session_dispatch`. The dispatch function takes
a JSON-RPC request string and returns the response.

## CLI

The native binary is invoked as `opennuts <command>`:

| command                | description                                       |
| ---------------------- | ------------------------------------------------- |
| `version`              | print the kernel version                          |
| `check`                | build a handful of primitives, validate, mesh     |
| `lsp`                  | run the OpenNuts LSP server on stdio              |
| `geom-server`          | run the geometry JSON-RPC server (LSP framing)    |
| `geom-server --lines`  | as above, newline-delimited framing               |
| `parse <file>`         | parse an `.opennuts` source and print diagnostics |

## Architecture notes

### Fixed-point world coordinates

All world-space coordinates live in `length_t = int64_t` femtometres.
The femtometre scale is comfortable below any physical CAD tolerance
(atomic bond lengths are ~10⁻¹⁰ m = 10⁵ fm) while allowing ±9 223 km of
model range. Numerical algorithms (NURBS evaluation, surface
intersection, transforms) convert to `double` metres for the maths and
back to `length_t` at the boundary; converting a value within ±9 km
incurs at most ~1 fm of drift, which is below the kernel's stated
working tolerance of 1 pm.

`sat_add` / `sat_sub` clamp to `[LENGTH_MIN, LENGTH_MAX]` (which is
INT64_MAX/2, giving one bit of headroom so that `a − b` between two
legal values can never wrap).

### Topology storage

B-rep entities (vertex, edge, coedge, loop, face, shell) live inside
`Pool<T, Tag>` containers. Each pool stores dense `std::vector<T>` data
alongside a parallel `std::vector<uint16_t>` of generations. Handles
are 24-bit-index + 16-bit-generation, with the convention that *odd*
generations are live and *even* generations are free. Allocating an
entity flips the generation up to the next odd value; freeing flips to
the next even value. A stale handle is therefore always detected with
a single comparison.

Geometry carriers (`Curve`, `Surface`) are heap-allocated through
`std::unique_ptr` pools because they are polymorphic, but every
concrete type implements `clone()` so `Body::clone()` performs a
complete deep copy without reflection or visitors.

### NURBS

The NURBS curve and surface implementations follow Piegl & Tiller,
*The NURBS Book* — algorithms A2.1 (FindSpan), A2.2 (BasisFuns), A2.3
(basis derivatives), A4.1 (rational point), A4.2 (rational
derivative), A5.1 (knot insertion), A3.5/A4.4 (tensor product
surface). Working buffers are sized for degree up to 8 on the stack so
evaluation never allocates.

Analytic curves (line, circle, ellipse) and surfaces (plane, sphere,
cylinder, cone, torus) are first-class `Curve`/`Surface` subclasses;
they can be promoted to exact NURBS representations on demand
(`CircleCurve::to_nurbs` uses the standard rational-quadratic 9-
control-point form).

### Boolean operations

`gmk::ops::union_bodies` / `intersect_bodies` / `subtract_bodies` are
declared with their final signatures but currently return
`Status::NotImplemented`. Implementing them requires the SSI pipeline
(surface-surface intersection -> face splitting -> region
classification -> stitching) which is intentionally not part of this
initial scope. Everything downstream of booleans (validators,
serialisers, the geometry-server protocol) already accepts the
resulting bodies, so wiring the implementation in is a localised
change.

### Servers

Both servers share `gmk::jsonrpc::read_message` / `write_message` and
the small `gmk::json` parser. The LSP server speaks Content-Length-
framed JSON-RPC and supports `initialize`, `didOpen`, `didChange`,
`didClose`, `hover` and `shutdown`. The geometry server exposes the
primitive constructors, bounding-box queries, validation, and
tessellation. Both servers can be driven over stdio or any user-
supplied stream pair, which makes them trivially testable.

## OpenNuts language

A compact MCAD source language. An example:

```opennuts
unit mm;
tolerance 0.001;

body widget {
    box(width: 50, depth: 30, height: 10);
    sphere(radius: 5) at (25, 15, 10);
    cylinder(radius: 2, height: 12) at (10, 10, 0) axis (0, 0, 1);
}
```

The grammar is intentionally small: top-level directives (`unit`,
`tolerance`) and `body NAME { ... }` blocks containing primitive
constructors with optional `at (...)` and `axis (...)` placement
clauses, plus boolean ops (`union`, `intersect`, `subtract`). The
parser is recovering, so the LSP can produce diagnostics on broken
files without giving up the entire AST.

## License

Internal project; copyright the OpenNuts authors.
