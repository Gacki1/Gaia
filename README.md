*English · [Deutsch](README.de.md)*

# Gaia — a planet in C++17 and Vulkan, by hand

A planet at Star Citizen scale: **1000 km radius, one world unit = one metre**,
flyable end to end from orbit down to **sub-metre ground detail** (LOD depth 18,
~0.48 m cells), with no loading screen. No UE5, no Godot, no engine — C++17,
Vulkan 1.1, my own window, my own maths, my own image loader, my own font atlas.

I am building this because I want to know how it *actually* works. An engine
would have solved every problem in this README for me — and taken everything I
learned from them along with it.

![From orbit](docs/orbit.png)

| | |
|---|---|
| ![Mountain range](docs/range.png) | ![Ground](docs/ground.png) |
| ![Cockpit](docs/cockpit.png) | |

---

## What is already there

**Terrain.** A cube-sphere with a per-face quadtree LOD, frustum culling and
geomorphing. The height field has two bands: a continental band at a fixed 8
octaves (LOD-invariant, so the coastline can never move) plus a detail band whose
octave count grows with the LOD level — through a prefix-safe fBm with a **fixed**
normaliser, so that adding an octave does not rescale the ones already there.

Measured: land height p50 537 m, p90 1485 m, max 6478 m. Local relief over a 2 km
radius p50 261 m, p90 1030 m, max 3619 m (for scale: the Alps ~800 m, the
Himalaya ~1500 m). Slope p50 6.8°, p90 27.0°.

**Plate tectonics instead of noise for the landmarks.** Ten Fibonacci-distributed
plate seeds, jittered from the planet seed; `plateAt` is a linear scan over ten
dot products and returns the nearest and second-nearest plate. Of ~24 boundaries,
**4** carry a mountain range, chosen by their **on-land length** — not by a hash,
because by hash one range lay entirely and another 93% in the sea.

Each range is a **belt** of 1–3 parallel ridges with longitudinal valleys between
them, 62–118 km wide and 1101–1287 km long. Every property comes from a hash of
the boundary identity: crest height, width and sharpness, asymmetry, terracing,
ridge count, ridge spacing, and sign (a quarter of the ranges are rift valleys
rather than mountains). The two most alike ranges still differ by 0.57 in their
strongest dimension — and only on axes that actually get rendered.

The boundary is **domain-warped**: not the distance to the boundary, but the
direction, *before* the partition is looked up. Crest tortuosity 1.400 over
1136 km of walked crest, 88 km of departure from a great circle. The belts cover
5.1% of the land.

**Materials.** Six triplanar-projected ground textures in one sampler array,
blended through eight one-byte vertex weights. Selection follows the **climate
zones** (elevation, slope, latitude, moisture) — the same four quantities the
palette uses, so that a material boundary and a colour boundary are the same line
on the ground. A climate slot may hold several textures as **variants**, chosen by
a canopy field; layers are shared, so a variant that reuses a texture already in
the set costs nothing at all.

**Atmosphere.** An HDR offscreen target, a composite pass with tonemapping,
Rayleigh scattering with aerial perspective, and a sun disc. Reversed-Z with an
infinite far plane — at 10⁹ m the depth is still > 0.

**Ship.** A flight model with a flight assist, a cockpit with switches, landing
gear, landing, and a quantum drive to six outposts. The landing is a test: it has
to touch down on all four legs, measured 4/4 at 1.9° of tilt.

**Editor.** A tool-style UI in creative mode for every runtime parameter of the
planet, with a live rebuild. Planets are text files; one table feeds the editor,
the writer and the reader so the three cannot drift apart. Sixteen validation
checks reject an invalid parameter set instead of drawing it.

**Tests.** A headless test binary with **463 assertions** that runs without
Vulkan.

Frame rates without the validation layer: 104–252 fps, all above 60.

---

## How I did it

This is the part that taught me the most. Four rules, every one of them learned
the hard way:

**1. Decisions belong in Vulkan-free headers.** Everything that determines the
terrain lives in headers with not a single Vulkan dependency. That is why a
headless test binary can pin them down, and why I can *measure* a claim about the
planet instead of looking at it.

**2. Every threshold sits on a measured quantile, never on what looks sensible on
a 0..1 scale.** This is the rule I broke most often and paid for most dearly. An
fBm with a gain of 0.38 does not fill [0,1] — it piles up around the middle. A
band that looks reasonable on paper hands one variant 99% of the ground and makes
the other unreachable. So the test prints the distribution first, and then I set
the constant.

**3. An A/B is not a finding until the build is proven.** See below.

**4. A test that can pass on 8 samples is not measuring what it names.** Three of
my own tests were built that way. The crest-walk test marched 64 km of a 1287 km
chain and satisfied its tortuosity band doing so; the walked length is now part of
the assertion. The uniqueness test scored `peakRelief` as a dimension of
difference — a field the terrain **never reads**. And the test that was supposed
to guard the LOD sphere checked it against the same wrong surface the code did.

---

## Technical difficulties

The interesting ones. Every number here is measured, not estimated.

### The tight LOD sphere bounded a surface no vertex lies on

The sphere the LOD uses to measure its distance to a patch was built from
`terrainRadius` — and the detail band and the rock spires are added on top of
that, afterwards. So every vertex sat **outside** its own sphere by roughly the
local detail height. Because the split rule measures distance as
`|cam − centre| − surfaceRadius`, and the geomorph band is derived from the same
relationship, a patch was given up *before* its vertices had finished morphing:
at level 17, least-morphed vertex 0.0000 with 0.27 m of pop left.

The test that was meant to catch exactly this compared against `terrainRadius`
too — so it passed while the invariant was violated. Both corrected: worst slack
is now 0.0 m at 13×13 sampling, and the morph is 1.0000 at every level.

Plain slack would have been the wrong fix here: 3 km added to a 12 m sphere makes
the distance ≈ 0 and subdivides everything to maximum depth — that is the
461,517-leaf collapse the file header documents. Sampling the real surface moves
the **centre** with it, so the sphere stays tight.

### "Dune waves": the corner in `ridge(n) = (1 − |n|)²`

Fine dark filaments in closed loops all over flat ground, everywhere. The
derivative jumps from +2 to −2 at n = 0 — a genuine corner, and it is *deliberate*,
it is what makes knife-edged ridges. But the noise crosses zero everywhere,
including where the band carries almost no amplitude, and the zero level sets of a
noise field are **closed loops**.

Two cuts settled it: with every scanned texture switched off they remain (so:
geometry, not material), and with the ridge fold switched off they vanish. In
between I ruled out terracing, plateau terracing, the rock spires and the
triplanar projection — all innocent.

Fixed with `sqrt(n² + ε²) − ε`, normalised so `ridge(±1)` still reaches the valley
floor exactly. The first attempt failed a different test: giving the softening to
**both** consumers moved the relief-versus-elevation correlation from 0.155 to
0.204 against a 0.20 bar, because the fold's distribution is what places the
relief quantiles. Separated — sharp for the relief field, soft for the height band
— it sits at 0.106.

### Why you must not warp the input of a high-frequency field spatially

I wanted ribs running down a mountain flank, so I squashed the detail band across
the belt, faded in with the range's own height. I got ribs — and concentric
contour rings over every mountain on the planet.

The arithmetic says why: the detail band is sampled at a base frequency of ~345,
so a displacement of 0.33 in the input is **114 noise periods**. The squash factor
varied with `|orogeny|`, whose level sets run parallel to the ridge — so the noise
slid by dozens of periods along exactly those lines, and the sliding drew them.
Any spatially varying warp of a high-frequency field does this; only a constant
one is safe, and a constant one has a seam at the belt edge.

### A domain warp cannot tear — but it can fold

The first warp displaced the **distance** to the plate boundary by a scalar. At
0.5 crest widths the chain was still a ruler; at 1.3 it broke into a dotted line
of separate lumps, because a distance offset moves each point of the crest
independently and pulls it apart.

Applied to the **input**, the field moves coherently and the chain can no longer
tear. But it can fold. Tortuosity against warp frequency at amplitude 0.20:
2.5 → 1.06 (a ruler), 6.0 → 1.60 (good), 10.0 → **17.1**, 16.0 → the chain
fragments after 144 km. The last two are the same failure: a domain warp stops
being **injective** once the displacement gradient reaches 1, and then one
boundary has several preimages. So the test checks a **band**; one-sided, it would
have passed the 17.1 with flying colours.

### The zenith flip: `cos(π/2) = −4.4·10⁻⁸`

The camera basis was built from `right = cross(worldUp, forward)`. With `forward`
pointing almost straight up the cross product is almost zero — and in float
`cos(π/2)` is not 0 but −4.4·10⁻⁸. The normalised vector then points in an
arbitrary direction and **swings 180° for 10⁻⁴ radians of pitch**. Measured:
180.0° (Euler) against 0.0198° (quaternion). Fixed via yaw/pitch → quaternion →
basis.

My first test for it was wrong: it asserted NaN. There is no NaN, there is
instability — the test had to measure the instability, not check for a symptom
that never occurs.

### `IWICBitmapScaler` swaps red and blue

The format converter sat *before* the scaler in the WIC chain and was silently
overruled. Proven by bypassing the scaler: an exact match without it, reversed
values with it. This had also corrupted every normal map — R was actually B, i.e.
the z component as x. Fixed by ordering it frame → scaler → converter.

### The tile that would not close

The patch origin was folded modulo 2 m while the shader also sampled at 23 m. 23
is not a multiple of 2, so the macro layer jumped at **every** patch boundary.
Fixed with 24 m and a fold modulo the macro tile; a `static_assert` checks the
divisibility, and I verified that it fires at 23.

### Two measurement traps that cost me hours

`cmd //c "build.bat Release"` from a Bash shell fails and prints only *"'build.bat'
is not recognized"*. Piped into `grep -c "error C"` it returns 0 — which reads
exactly like a clean build. Three A/B experiments in a row came back
byte-identical, each looked like a real finding about the renderer, and all three
were the same unchanged binary. It surfaced only when I forced the fragment shader
to pure red and *still* nothing changed.

And `Set-Content -Encoding utf8` writes a **BOM** in Windows PowerShell 5.1. C++
tolerates it, GLSL does not: `#version` breaks, the build reports the error and
success for the exe anyway, and the screenshot silently uses the stale `.spv`.

Since then: build only through PowerShell, verify `Build OK`, check the `.spv`
timestamp on shader edits — and when an A/B comes back byte-identical, do **not**
interpret it; prove the build first.

### Others, briefly

- **Slider identity was a local float.** Dragging one slider dragged every row
  below it, which walked the parameter set invalid, whereupon the planet refused to
  rebuild. Three symptoms, one cause.
- **UI colours washed out.** The swapchain is `_SRGB`, so display-authored colours
  are gamma-encoded. My own comment claimed decoding was unnecessary — it was
  wrong.
- **The mouse wheel was consumed twice.** `consumeWheel()` clears as it reads, so
  the editor always got zero.
- **A dangling pointer after `init()`** replaced the world: it halved the triangle
  count and cost the landing test a leg, without a single error message.
- **`memcmp` on a parameter struct.** Padding is not guaranteed to be initialised;
  replaced with a field comparison guarded by a `static_assert` on the size.
- **Physics and rendering on different planets.** The ground query built its own
  default world — the ship landed on invisible terrain.
- **Terraced plateaus broke the geomorph.** A riser of 0.10 at a mix of 0.85
  creates ~1.5 km walls that the parent's half-resolution grid cannot resolve.
  Bisected (rifts alone green, plateaus alone red), resolved at 0.22/0.60.
- **The culling slack for mountains was 1 km short** (4000 m against ranges up to
  5060 m). A patch whose 5×5 sample grid missed the spine could be culled while
  visible.

---

## What is still to come

**Vegetation through a PCG system.** The scatter generator is already one at
heart: a deterministic point grid on the cube face, seam-consistent, density from
the terrain, aligned to the true surface normal, built in a background job. What
is missing, in this order:

1. **Real instancing.** Today it is one draw call per object — measured 4.83 ms of
   command recording for 1580 draws, against an 11.1 ms budget. The time is linear
   in the instance count, so 30,000 trees would be ~100 ms of recording alone.
   That is not an optimisation, it is a precondition.
2. **A glTF loader** for tree models.
3. **Auto-LOD at load time**: edge collapse to three levels plus a billboard
   impostor, so I do not have to ship LODs.
4. **The PCG layer**: species, density from the canopy field (already there),
   exclusion rules between layers, clearings.
5. **Range**, from 450 m out to kilometres.

*Explicitly not Nanite.* The renderer has zero compute pipelines, zero indirect
draws and no mesh shaders — that would be a rebuild. And for alpha-tested foliage
it is the wrong tool anyway.

**Terrain.**
- **Per-octave softening of the ridge fold.** One constant is a compromise: sharp
  enough for crests means sharp enough for scratches. Sharp at landform scale and
  smooth at metre scale needs a different ε per octave.
- **Talus aprons at the foot of a cliff.** Slope p99 is 70.7° and the maximum
  88.6° — the rock spires meet flat ground vertically. A real cliff has a foot.
- **Segmented ranges** (en échelon), overlapping sub-ranges with passes between
  them.
- **Drainage.** The real reason natural terrain looks natural. Done properly it
  needs flow accumulation across the whole planet.

**Rendering.**
- **Reproject the triplanar mapping.** On **42% of the sphere** the second
  projection carries ≥15% of the weight, up to 50/50 — two offset copies of the
  same texture. A higher blend exponent shrinks that area to 5.9%, but the worst
  case stays at 0.500: on the 45° line two components are *equal*, and no power
  makes them unequal. The fix runs into two real obstacles — a surface-following
  projection breaks the modulo arithmetic that only exists because of float
  precision at 10⁶ m, and a continuous tangent field on a sphere necessarily has a
  singularity by the hairy ball theorem.
- **A leaf budget with a priority queue** instead of today's altitude-dependent
  split factor. That factor is a proxy: it allocates the budget by how much planet
  is in view, and it works, but the clean rule is "split the patch with the largest
  screen-space error until the budget runs out".
- **Clouds, moons, a gas giant in the sky.**

**Assets.** Two of the six ground materials are still generated placeholders, and
I am missing a bright grass/moss scan — the clearing variant is switched off
because the only texture available for it reads like beach sand. The mechanism is
in place; only the texture is missing.

---

## Building

```
build.bat            :: Release -> build\Release\planet.exe
build.bat Debug      :: with Vulkan validation
build.bat Release run
```

Needs Visual Studio with MSVC and the Vulkan SDK. The paths to `vcvars64.bat` and
Ninja are at the top of `build.bat`.

Tests:

```
build\Release\planet_tests.exe
```

The acceptance runs I go through after every terrain change:

```
build\Release\planet.exe --flight-test 6000 --validate
build\Release\planet.exe --travel-test 60000 --validate
build\Release\planet.exe --freecam --alt 3 --frames 300 --validate
build\Release\planet.exe --freecam --alt 60 --skim 400 --frames 600 --validate
build\Release\planet.exe --cockpit --look-down 24 --frames 200 --validate
build\Release\planet.exe --frames 600 --creative-at 200 --validate
```

Useful flags for looking around: `--spawn-range` parks the camera on a mountain
range, `--spawn-canopy` at a forest edge, `--spawn-spire` in a spire province —
without them no screenshot will ever find any of these, because together they
cover only a few per cent of the land.

## What is not in the repository

`src/textures/` (308 MB of scanned ground materials) and `src/example/` (art
direction reference images) are third-party assets and are excluded. The loader
searches recursively under `src/textures` and only accepts a folder that also
holds a base colour map — so the layout is free to change, and **a missing
material is not an error**: the renderer substitutes a generated placeholder so
the world always draws. The startup line prints which folder resolved to which
layer.

`shots/` (rendered screenshots) is excluded as well.
