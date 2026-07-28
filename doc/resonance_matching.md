# Matching resonance lists between sweeps

You have two resonance lists from the same array and you need to know which
entry in one is which physical device in the other. The lists are different
lengths, every frequency has moved, some devices were found in one sweep and
not the other, and you would rather not go through four hundred of them by
hand.

```python
from souk_readout_tools import resonance_matching as rm

idx = rm.match_index('mirror/resonances/HPAM8.resonances',
                     'room/resonances/HPAM8.resonances')
```

`idx[i]` is the entry of the second list that is the same device as entry `i`
of the first, or `-1` if there isn't one. That is the whole tool, for most
purposes.

---

## 1. Quick start

The one-liner above is fine when you only want the mapping. When you want to
see whether to believe it:

```python
m = rm.match_resonances('mirror/resonances/HPAM8.resonances',
                        'room/resonances/HPAM8.resonances')
m.summary()
m.plot()
```

```
matching HPAM8.resonances (389) -> HPAM8.resonances (384)
  shift: df/f = -4.640e-05 (-54.89 kHz at 1183 MHz)   (scatter 4.82e-06)
  tolerance: 65.9 kHz (linewidth floor; 5*sigma = 28.5 kHz, ...)
  assignment (free): 381 pairs
  monotonic cross-check: agrees on 381/381 pairs
  matched 381 of 389 (A) / 384 (B)  [97.9% of A in the common range]
  residuals: median -0.05 kHz, robust sigma 5.47 kHz, max 39.4 kHz
    ok               378
    outlier            3
    unmatched_a        8
    unmatched_b        3
```

And to get the physics out:

```python
qi_mirror, qi_room = m.compare('Qi')     # equal-length, matched devices only
m.plot_compare('Qi')
```

On the campaign those numbers came from, `Qi` falls by a factor 2.7 between a
blanked aperture and a 300 K room, and `Qc` holds to 0.2%.

Nothing above needed to be told what had happened to the array.

---

## 2. How it works

Four stages. Each can be overridden, and none of them has to be.

**Stage 1 — the global shift.** Resonators rarely stay put, and they usually
move together: a constant fractional shift `df/f` is what ageing, a loading
change or a thermal cycle produce. Fitting that first means the matching only
has to cope with the per-device scatter about it, which is far smaller than
the shift itself. The model is chosen automatically from `none`, `constant`,
`proportional`, `affine` and `linear_frac`, preferring the simplest that fits,
so a process with genuinely no global shift is not fitted a spurious one.

The starting value comes from a brute scan: try candidate fractional shifts,
keep whichever lines up the most resonances. There is no match yet to fit to,
and the shift can be much larger than the spacing between resonances, so
scanning is the honest way to begin.

**Stage 2 — the cost of each possible pairing.** Every pair within `tolerance`
of each other, after the shift, is scored on how far apart it is.

**Stage 3 — assignment.** The scores are solved globally, so no device is
claimed by two neighbours at once, and anything with no counterpart inside the
tolerance is left unmatched rather than forced onto something. The
order-preserving solution is always computed as well, and disagreements
between the two are reported: that is exactly where the frequencies alone
cannot decide.

**Stage 4 — fingerprints and rescue.** Frequency is not the only thing known
about a resonator. `Qc`, `phi`, linewidth and the rest are measured on the
confident pairs to find out which of them survived, and those that did are
used to settle pairings frequency cannot — and, for a device that moved too
far to be found any other way, to identify it with frequency ignored
altogether. See §8 and §10.

---

## 3. Inputs

Both arguments take any of these, in either combination:

| Input | Recognised by | What comes with it |
|---|---|---|
| list / 1-D array of frequencies | numeric, 1-D | frequencies only |
| dict of columns | a frequency-like key | every other equal-length column |
| structured array | `dtype.names` | as above |
| `.resonances` file | extension | `fwhm`, `Ql`, `Qc`, `Qi`, `dip_depth` |
| KIDLAB `.txt` toneslist | extension | names, offset attenuation |
| `.csv` (fit summary or `#ID,Frequency(Hz)`) | extension | every numeric column |
| `.npy` / `.npz` | extension | whatever is inside, recursively |
| power-sweep run directory or its `measurement.json` | directory | the run's **best-power fits**: `fr`, `Ql`, `Qi`, `Qc`, `phi`, `anl` |
| parsed sweep dict (`sweep_f`/`sweep_i`/`sweep_q`) | keys present | resonances **found automatically**, plus the sweep for plotting |
| `ResonanceSearchResult` / list of `ResonanceResult` | attributes | all finder outputs |

Frequency columns are recognised as any of `frequency`, `frequency_hz`,
`Frequency(Hz)`, `freq`, `freqs`, `f`, `fr`, `fr_hz`, `center_hz`. Other
columns are mapped onto canonical names (`Qinternal` and `qi` both become
`Qi`, `Linewidth(Hz)` becomes `fwhm`) so a `.resonances` file and a fit
summary compare without renaming anything. Unrecognised columns keep their
own names and are carried through untouched.

**Units** are guessed from the magnitude — the readout band is far enough from
1 GHz-as-a-bare-number that `1.2`, `1200` and `1.2e9` are unambiguous — and
the guess is printed. Pass `units='mhz'` to be certain.

**Parameter source matters more than you would think.** The empirical
estimates in a `.resonances` file are contaminated by whatever else changed:
across the mirror/room load change their `Qc` moves 13% while the *fitted*
`Qc` moves 0.8%. For matching alone this rarely matters. For fingerprints and
rescue it decides whether they work at all (§8). If you have run a power
sweep, pass the run directory.

---

## 4. Reading the output

### The index

```python
m.index       # length of A: which entry of B, or -1
m.index_b     # length of B: which entry of A, or -1
```

Both are indices into the lists **as you passed them in**, never into any
internal sorted order. This is true everywhere in the API.

### The pair table

```python
m.pairs        # one row per pairing, including the unmatched
m.table()      # the same, plus every parameter from both sides, _a / _b
m.matched      # matched rows only
m.unmatched_a, m.unmatched_b
m.flagged()    # everything worth a second look
```

Columns: `a_index`, `b_index`, `f_a`, `f_b`, `df_hz`, `df_frac`,
`residual_hz`, `residual_sigma`, `ambiguity`, `flag`.

`ambiguity` is the one to sort by when checking by hand: it is the best cost
divided by the runner-up's, so 0 is unambiguous and 1 is a coin flip.

### Flags

| Flag | Means | What to do |
|---|---|---|
| `ok` | matched, unremarkable | nothing |
| `crossed` | matched, but its ordering swapped with a neighbour | check if the array is dense; real, but worth seeing |
| `ambiguous` | the runner-up was nearly as good | look at `m.candidates()`, or `plot_pair` |
| `outlier` | matched, but far from the global shift | often real physics (§8); confirm with `plot_pair` |
| `rescued` | paired with frequency ignored (§10) | always confirm — these are the surprising ones |
| `manual` | you set it yourself | nothing |
| `unmatched_a` / `unmatched_b` | no counterpart, inside the band the other sweep covered | genuinely lost or gained |
| `out_of_range` | no counterpart, and the other sweep never looked there | **not** a missing device |

The `out_of_range` distinction matters. Matching a 528–1996 MHz sweep against
a 528–2052 MHz one reports 101 `out_of_range`, not 101 dead detectors.

### Clusters

A blend that resolved into two, or two that merged into one, is neither an
error nor a simple pairing:

```python
m.groups()          # [{'a_indices', 'b_indices', 'kind', 'frequency'}, ...]
```

`kind` is `n_a:n_b` — `1:1`, `2:1`, `1:3`, whatever falls out. The one-to-one
map in `m.index` still holds inside each cluster; the cluster records the rest.
On the mirror/room pair this finds six `2:1` clusters, where pairs that were
resolved at 4 K blended into one at 300 K.

### The trace

`m.summary()` reprints every decision the matcher made, in order. Read it when
something looks wrong — it names the model it chose, the tolerance and which
term set it, how many pairs the two solvers agreed on, which fingerprints
survived, and what the rescue pass did or declined to do.

---

## 5. Adjusting it by hand

The matcher will not always agree with you, and you are sometimes right.

```python
m = m.set(a=17, b=22)              # by index
m = m.set(a=1.23456e9, b=1.23012e9)  # or by frequency, in Hz
m = m.unmatch(a=17)                # break a pairing
m = m.lock(3, 4)                   # fix these where they are
m = m.rematch()                    # re-solve everything else around the locks
```

Integers select by position, floats by nearest frequency, so "the one that was
number 12" and "the resonator at 1.2345 GHz" both work.

Every edit returns a **new** match; the one you started from is unchanged.
Anything you set is locked, which means `rematch()` holds it and solves the
rest of the array optimally around it — the pins are imposed on the cost
matrix, not forced on afterwards. `unmatch` is remembered too: an entry you
deliberately unmatched will not quietly reappear, not even via the rescue pass.

### Editing as a file

```python
m.save('match.csv')
# ... edit the b_index column in a spreadsheet or an editor ...
m = rm.load_match('match.csv', mirror, room)
```

This is the one that gets used at three in the morning. Set `b_index` to `-1`
to drop a pairing.

---

## 6. Plotting

Five plots, each returning a matplotlib `Figure`. All work whether or not the
lists came from sweeps; when they did, the underlying S21 is drawn too.

### `m.plot()` — the overlay

Both sweeps, one above the other, with a line joining every matched pair.
Crossings, unmatched devices (marked ✗) and any rescue claiming a device moved
across the band are all visible as shapes rather than numbers. Pass
`f_range=(f_min, f_max)` or just zoom.

*Good:* near-vertical links, evenly spaced, few ✗.
*Bad:* a fan of sloping links (the shift model is wrong), or clumps of ✗ in
one region (a detection-threshold difference, not dead devices).

### `m.plot_shift()` — the physics check

`df/f` against frequency, with the fitted model drawn on.

*Flat band of points:* the array moved together — a loading change or ageing.
*Sloping or curved:* the global model is the wrong shape; try
`shift_model='linear_frac'`.
*Tight core with a sparse tail:* a subset of devices was disturbed on its own.
That is what trapped flux looks like, and it is not a matching problem.
*Broad, symmetric, no core:* the shift was not fitted properly, or the two
sweeps really are unrelated.

### `m.plot_quality()` — is the tolerance sane

Left, the residual histogram with the tolerance drawn on; right, the ambiguity
of each match.

*Good:* the histogram sits well inside the tolerance lines, with room to
spare, and almost every ambiguity is near zero.
*Bad:* the histogram fills the tolerance (too tight to trust — the true
partners are being cut off), or a broad band of high ambiguity (the array is
too dense for the tolerance in use).

### `m.plot_compare(a_values, b_values=None)` — the science

See §7.

### `m.plot_pair(a_index)` — one device, close up

Both raw traces around a single device, plotted against detuning from each
fitted centre so they overlay however far it moved. This is the check to make
before believing anything flagged `rescued`, `outlier` or `ambiguous`.

---

## 7. Comparing quantities

```python
qi_a, qi_b = m.compare('Qi')                 # by name, present in both lists
nep_a, nep_b = m.compare(nep_mirror, nep_room)  # or any two arrays
resp_a, resp_b = m.compare('Qi', my_room_array)  # or one of each
```

The two returned arrays are always the same length as each other and line up
device by device.

Arrays you pass must be **one entry per resonance in that list's input order**
— the order you handed the list to `match_resonances`, not sorted, not the
matched subset. Lengths are checked and the error names both expected lengths.

Layout is chosen with `on=`:

| `on` | Length | Missing entries |
|---|---|---|
| `'matched'` (default) | devices found in both | none |
| `'a'` | list A | NaN where A had no counterpart |
| `'b'` | list B | NaN where B had no counterpart |
| `'union'` | everything in either | NaN on the side that lacks it |

Dicts work too, and give dicts back:

```python
va, vb = m.compare({'Qi': qi_a, 'noise': n_a}, {'Qi': qi_b, 'noise': n_b})
```

### Carrying information forward

```python
xy_room = m.transfer(xy_mirror)         # per-device, onto B's ordering
names_room = m.transfer(names, fill=None)
```

This is how a beam map survives a cooldown: match the new wideband sweep
against the old list, `transfer` the positions, and the mapping from frequency
to focal-plane position carries over without re-running beam maps. Devices
that are new in B come back as NaN (or your `fill`).

### One warning

Matching on a quantity and then comparing it biases the comparison towards
"nothing changed". By default the fingerprint stage may well select `Qi` — it
is a good discriminator even when it has halved, because it halved for
everyone (§8). If `Qi` is your measurement, either exclude it:

```python
m = rm.match_resonances(a, b, fingerprint=['Qc', 'phi'])
```

or check what was used in `m.fingerprint`. For a well-separated array the
fingerprint changes almost nothing — on the mirror/room pair it moved 2 pairs
out of 379 — so this is a real effect but usually a small one.

---

## 8. The physics

Perturbations are not a list of scenarios to look up; they compose from a few
primitives.

| | Primitive | How it shows up |
|---|---|---|
| **P1** | global fractional shift | `df/f` roughly constant across the array |
| **P2** | per-device frequency scatter | residual about the global model; enough of it reorders neighbours |
| **P3** | coupling / environment change | `Qc`, `phi` move — stray capacitance, ground plane, box modes |
| **P4** | internal-loss change | `Qi` and dip depth move — loading, TLS, temperature, oxide |
| **P5** | degeneracy change | blends resolve or merge; N:M clusters, not just pairs |
| **P6** | observation change | different span, step, detection threshold, readout power |
| **P7** | trapped magnetic flux | a susceptible *subset* moves, either way, on any cool through Tc |

What you did, and which primitives it fires:

| Operation | Vented? | P1 | P2 | P3 | P4 | P5 | P6 | P7 |
|---|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
| Loading change (window, cold load, filter) | no | small | — | — | **yes** | rare | maybe | — |
| Thermal cycle, no vent | no | small | small | — | small | rare | maybe | **yes** |
| Thermal cycle **with vent** → ageing, oxidation | yes | **yes** | **yes** | some | **yes** | some | maybe | **yes** |
| Remount | yes | **yes** | **yes** | **yes** | **yes** | some | maybe | **yes** |
| Move in the focal plane (needs a vent) | yes | **yes** | **yes** | **yes** | **yes** | some | maybe | **yes** |
| Lithographic trim (vent + remount) | yes | *per-device, no global model* | **yes** | **yes** | **yes** | **yes**, by design | usually | **yes** |
| Readout power change | no | apparent | — | — | apparent | — | **yes** | — |

Two things fall out of this table that are easy to get wrong:

**Ageing is not independent.** Films oxidise because you vented. An unvented
thermal cycle does not age the array — but it still re-rolls P7, so it is not
scatter-free either.

**Trim breaks the global model but keeps the order.** Some devices move and
others do not, so `shift_model` must be allowed to collapse to `none`; but
nothing overtakes anything, which makes `order='monotonic'` genuinely
informative rather than a fallback. Trim also creates P5 deliberately — that
is what it is for.

### What P7 implies for the algorithm

Trapped flux makes the residual distribution **heavy-tailed**: most devices sit
tight, a susceptible minority moves far. Three defaults follow from that
directly, and they are not matters of taste:

- the tolerance is set from a **median absolute deviation**, not an RMS, which
  the tail would inflate until the window swallowed the neighbours;
- the `outlier` flag is informative rather than a nuisance — the tail *is* the
  susceptible subset, and listing it is a result;
- the rescue pass exists at all, because the far tail is the device that ends
  up below the bottom of the band.

`plot_shift` and `plot_quality` show this directly: a tight core with a sparse
tail is P7; broad symmetric scatter with no core is a bad global fit.

### Fingerprints

Which properties survive depends on what you did, so the tool measures it
rather than assuming. Each feature gets a global offset and a residual
scatter, exactly as the frequencies do — so **a property that changed the same
way for every device still identifies devices perfectly well.** What
disqualifies a feature is scatter, not change.

`fingerprint='auto'` prints its verdict:

```
  fingerprint (auto, from 370 confident pairs):
    Qc      scatter 0.008 vs spread 0.628  ratio 0.01  offset +0.002  -> USED
    phi     scatter 0.010 vs spread 0.498  ratio 0.02  offset +0.000  -> USED
    Qi      scatter 0.037 vs spread 0.456  ratio 0.08  offset -0.990  -> USED
    Ql      scatter 0.109 vs spread 0.520  ratio 0.21  offset -0.280  -> USED
    anl     ...                            ratio  inf                 -> dropped
```

`Qi` fell by a factor 2.7 (`offset -0.990` in log) and is still one of the best
fingerprints available, because it fell for everyone. `anl` is dropped because
the tuning pinned it to a target, so it distinguishes nothing. That table is a
physics result in its own right: it tells you which resonator properties
survived the perturbation.

Under a remount, expect `Qc` to be rejected — P3 is exactly the case where
geometry stopped being reliable. You do not have to know that in advance; the
selector will find out.

Register your own with

```python
rm.FEATURES['responsivity'] = {'transform': 'log'}   # or 'linear', or 'angle'
```

---

## 9. When it goes wrong

| Symptom | Likely cause | Reach for |
|---|---|---|
| Very few matches; residuals all offset the same way | global shift mis-fitted | `shift_model='proportional'`, or pass `shift_model=-4.6e-5` directly |
| `plot_shift` sloping or curved | wrong shift *shape* | `shift_model='linear_frac'` or `'affine'` |
| Lots of `ambiguous` in a dense patch | tolerance too wide for the spacing | tighten `tolerance=`; or `order='monotonic'` if you know the order held |
| Histogram fills the tolerance in `plot_quality` | tolerance too tight | widen `tolerance=`, or check the shift model first |
| "Half my detectors vanished" | almost always P6 | check the `out_of_range` count and `coverage_a` / `coverage_b` before believing it |
| A rescue you do not believe | fingerprint coincidence | `m.plot_pair(i)`, then `m.unmatch(a=i)`; or `rescue=False` |
| Fingerprint used the thing you are measuring | circularity | `fingerprint=['Qc']`, or `fingerprint=None` |
| A handful simply wrong | — | `m.set(a=..., b=...)`, `m.rematch()` |

### How well does it actually work

Perturbing a real 389-resonance list by a known amount, with known truth:

| Perturbation | Recall | Precision |
|---|---:|---:|
| scatter 0.002 × spacing (what the real load change looks like) | 0.992 | 1.000 |
| scatter 0.05 × spacing | 0.984 | 0.990 |
| scatter 0.10 × spacing | 0.950 | 0.954 |
| scatter 0.30 × spacing | 0.716 | 0.821 |
| `df/f` anywhere from 0 to −5×10⁻³ | ≥0.98 | ≥0.99 |
| 30% of devices dropped **and** 30% spurious added | 0.961 | 0.885 |

The size of the shift is irrelevant — it is fitted out. What matters is the
scatter relative to the spacing between resonances, and real data sits about
fifty times inside the safe region. Above ~0.3 × spacing the problem is
genuinely ambiguous and no algorithm can rescue it.

---

## 10. Every control

### `match_resonances(a, b, ...)`

| Parameter | Default | What it does |
|---|---|---|
| `shift_model` | `'auto'` | `'none'`, `'constant'`, `'proportional'`, `'affine'`, `'linear_frac'`, `'auto'`, or a number taken as a fixed `df/f`. `'auto'` fits all five and keeps the simplest not meaningfully beaten (needs a 20% better scatter or 1% more matches to step up). Set it when you know the physics: `'none'` after a trim, `'proportional'` after a cooldown. |
| `tolerance` | `'auto'` | How far apart (Hz) two entries may be, after the shift, and still be one device. `'auto'` takes 5× the measured scatter, floored at one linewidth (you cannot localise better than that, and it stops the tolerance collapsing when two lists are nearly identical) and capped at 0.4× the resonance spacing (beyond which "it moved" and "that is its neighbour" are indistinguishable). The summary says which term won. |
| `order` | `'free'` | `'free'` solves globally and lets devices reorder. `'monotonic'` forbids reordering — use it when you know the order held, e.g. after a trim, and in dense arrays where it settles near-ties. `'nearest'` keeps only mutual nearest neighbours: the simplest thing that can work, useful as a sanity check. The monotonic solution is always computed as a cross-check regardless. |
| `fingerprint` | `'auto'` | `'auto'` measures which properties survived and uses those; a list forces a choice; a `{name: weight}` dict weights them; `None` matches on frequency alone. Fingerprints only break ties — they can never pull in a pair frequency excluded, nor push one out. |
| `rescue` | `'auto'` | Pair up leftovers with frequency ignored. `'auto'` does so only when the measured fingerprint accuracy justifies it; `'count_only'` pairs a lone leftover on each side by elimination and nothing else; `False` disables it. |
| `coverage_a`, `coverage_b` | `None` | `(f_min, f_max)` each sweep actually covered. Taken from the sweep when the list came from one, otherwise the span of the list itself. Set them when a list is a subset of a wider sweep, so devices outside the overlap are reported `out_of_range` rather than lost. |
| `units` | `'auto'` | `'hz'`, `'mhz'`, `'ghz'`. Only consulted for bare numbers. |
| `iterations` | `3` | Match/refit cycles while estimating the shift. More rarely helps. |
| `outlier_sigma` | `5.0` | Matched pairs further than this from the global model are flagged `outlier`. Still matched. |
| `ambiguity_threshold` | `0.5` | Pairs whose runner-up cost more than this fraction of the best are flagged `ambiguous`. |
| `fingerprint_max_ratio` | `0.5` | A feature is used if its pair scatter is below this fraction of its array-wide spread. Lower is stricter. |
| `fingerprint_weight` | `0.1` | How much say fingerprints get against frequency in the main assignment, as a weighted average of two costs that both run 0–1. |
| `locked` | `None` | `{a_index: b_index}` held fixed; `-1` holds an entry deliberately unmatched. Normally set for you by `m.set()`. |
| `verbose` | `True` | Print the decisions as they are made. |

### Rescue gates

A rescue is accepted only when the fingerprints have been *shown* to work.
A leave-one-out test on the confident pairs measures how often they name the
right device out of the whole array; `rescue='auto'` engages above 80%.
An individual rescue then needs a fingerprint distance ≤ 3.0 and a runner-up
at least 2× worse. Those numbers come from the measured distributions — on
fitted parameters, true pairs sit at a median distance of 0.74 (90th
percentile 3.05) and random pairs at 34.8.

Anything not accepted is recorded in `m.suggested` rather than applied, and
everything accepted is flagged `rescued` rather than absorbed into `ok`.

### Module-level

| Name | Purpose |
|---|---|
| `rm.FEATURES` | The fingerprint registry. Add to it to use your own quantities. |
| `rm.as_resonance_list(obj)` | The loader, if you want the parsed list without matching. |
| `rm.calibrate_fingerprint(...)` | The feature-stability measurement on its own. |
| `rm.fingerprint_power(...)` | The leave-one-out discriminating-power test. |
| `rm.estimate_transform(a, b)` | Just the shift fit. |
| `rm.choose_tolerance(...)` | Just the tolerance rule. |

---

## 11. Worked examples

### 11.1 Two bare frequency lists

```python
idx = rm.match_index(freqs_before, freqs_after)
moved = idx >= 0
print(f'{moved.sum()} of {len(idx)} devices carried over')
```

No parameters, no plots, no files. Units are detected, so MHz is fine.

### 11.2 A loading change, with the science at the end

```python
m = rm.match_resonances('mirror/resonances/HPAM8.resonances',
                        'room/resonances/HPAM8.resonances')
m.summary()
m.plot()
m.plot_shift()
qi_cold, qi_warm = m.compare('Qi')
print(f'Qi ratio {np.median(qi_warm / qi_cold):.3f}')
m.plot_compare('Qi')
```

### 11.3 The same, from the power-sweep runs

```python
m = rm.match_resonances('mirror/tuning/power_sweep', 'room/tuning/power_sweep')
```

Identical call; it finds `analysis/best_power.json` and uses each device's
fitted parameters at its tuned power. The residual scatter drops from 5.5 kHz
to 3.9 kHz and the fingerprints become about forty times sharper. Prefer this
whenever the runs exist.

### 11.4 A vented cool-down — reading the ladder

```python
m = rm.match_resonances(old_run, new_run)
```

Expect a larger `df/f`, more scatter, and a fingerprint verdict that has
changed: `Qc` may be dropped if the array was remounted (P3). Read the
`fingerprint (auto, ...)` block before trusting any rescue.

### 11.5 Suspected trapped flux

`plot_shift` shows a tight core and a handful of devices well off it, in both
directions. Check first that it is not a bad global fit:

```python
m.summary()                      # is the residual median ~0?
m.plot_quality()                 # tight core, or broad?
outliers = m.flagged('outlier')
for row in outliers[:5]:
    m.plot_pair(row['a_index'])  # same device, or a mismatch?
```

If the core is tight, the model is right and the tail is physical. That is a
cryostat finding — shielding — not a matching problem.

### 11.6 After a trim

```python
m = rm.match_resonances(before, after, shift_model='none', order='monotonic')
for g in m.groups():
    if g['kind'] != '1:1':
        print(g['kind'], g['a_indices'], g['b_indices'])
```

`shift_model='none'` because trim moves some devices and not others, so there
is no global shift to fit; `order='monotonic'` because nothing overtakes
anything. The `1:2` and `1:3` clusters are the separations you were trying to
achieve.

### 11.7 A device that moved a long way

```python
m.summary()
```

```
  rescue: 14 leftover in A, 9 in B; fingerprints name the right device 93% of the time
    1 rescued, 13 suggested but not applied
      A  1842.1043 MHz -> B   990.3120 MHz   df/f -0.462  fingerprint 0.81, runner-up 8.1x worse
```

Then confirm it, because a rescue is also consistent with a spurious detection
or a blend that resolved:

```python
m.plot_pair(341)
m = m.unmatch(a=341)     # if you do not believe it
```

If the fingerprints are weak — survey estimates rather than fits — the tool
says so and declines to rescue anything, but will still show you a shortlist:

```python
m.candidates(341, n=5)
```

### 11.8 Carrying a beam map across a cooldown

```python
m = rm.match_resonances(old_list, new_wideband_sweep)
xy_new = m.transfer(xy_old, fill=None)
missing = [i for i, v in enumerate(xy_new) if v is None]
print(f'{len(missing)} new devices have no beam-map position')
```

---

## 12. API summary

```
match_index(a, b, **kw)                -> ndarray, A->B, -1 where unmatched
match_resonances(a, b, **kw)           -> ResonanceMatch
as_resonance_list(obj, units='auto')   -> ResonanceList
load_match(path, a, b)                 -> ResonanceMatch (honours edited b_index)

ResonanceMatch
  .index .index_b .pairs .matched .unmatched_a .unmatched_b .flagged()
  .table() .compare(a, b=None, on=) .aligned(on=) .transfer(values, fill=)
  .groups(merge_window=) .candidates(a_index, n=, by=)
  .set(a=, b=) .unmatch(a=|b=) .lock(*a) .rematch(**kw) .locked
  .summary(show=) .save(path)
  .plot() .plot_shift() .plot_quality() .plot_compare(...) .plot_pair(i)
  .transform .tolerance .fingerprint .fingerprint_power .rescued .suggested

ResonanceList
  .frequency .params .labels .order .source .coverage .sweep
  .spacing .linewidth
```

Every function's docstring says the same as this page, so `help(rm.match_resonances)`
is enough when you are offline.
