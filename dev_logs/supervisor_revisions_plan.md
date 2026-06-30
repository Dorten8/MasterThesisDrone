# Supervisor Revision Plan — Chronological Task List

**Source:** `thesis/JS_thesis_2026-06-29_commented.pdf` (42 annotations)
**Date:** 2026-06-29
**Workflow:** Each task is a discrete, verifiable unit.

---

## METHODOLOGY (Section 3)

### Task M6 — Replace/add diagram (user has one in Figma/Draw.io)
**👤 HUMAN** — user must provide diagram

| Source | Comment 13 |
|--------|-----------|
| File   | `thesis/Sections/Methodology.tex` |

**Issue:** "maybe add a picture, user has one ready in Figma or Drawio" — possibly architecture diagram, flight director state machine, or another diagram.

**User must provide:** Which diagram is this referring to? Provide the file, Claude will insert it.

---

### Task M8 — "horizontal ffs!!! also user will add proper diagram later!"
**👤 HUMAN** — user must provide corrected diagram

| Source | Comment 20 |
|--------|-----------|
| File   | `thesis/Sections/Methodology.tex` |

**Issue:** A figure (likely the flight director state machine or the architecture diagram) is oriented vertically when it should be horizontal. The supervisor also says the user will add a proper diagram later.

**User must provide:** Which figure? And the replacement diagram if ready now. If not ready, Claude will mark as TODO.

---

### Task M9 — "Move this whole thing into MEthodology!"
**🤖 ROBOT** — but 👤 **HUMAN preview required**

| Source | Comment 21 |
|--------|-----------|
| File   | `thesis/Sections/Experiments.tex` → move to `thesis/Sections/Methodology.tex` |

**Issue:** Some content in Experiments should be in Methodology. "but it is good, do not change shit around!!" — move it, but don't rewrite it.

**Likely target:** The Motion Capture Setup section (`\subsection{Motion Capture Setup}` in Experiments.tex, lines 8-27). This was already partially addressed in the EKF consolidation, but the MoCap setup content itself (coordinate frames, OptiTrack description) is arguably methodology. Comment 24 also says "to be merged and moved to methodology, require preview for user before executing."

**Claude will:** Identify the exact content to move, show you the before/after structure, and execute only after your explicit approval.

---

## EXPERIMENTS & RESULTS (Section 4)

### Task E3 — Merge MoCap/EKF content to Methodology (preview required)
**👤 HUMAN** — preview before executing

| Source | Comment 24 |
|--------|-----------|
| File   | `thesis/Sections/Experiments.tex` → `thesis/Sections/Methodology.tex` |

**Issue:** "to be merged and moved to methodology, require preview for user before executing" — more MoCap/EKF content to move. This may overlap with Task M9.

**Claude will:** Confirm scope with you before executing. Show exactly what moves.

---

### Task E12 — "add plots for this claim into annex" ✅ DONE
**🤖 ROBOT** — allocator saturation plot already in Annex

| Source | Comment 29 |
|--------|-----------|
| File   | `thesis/Sections/Experiments.tex` + `thesis/Sections/Annex.tex` |

**Resolution:** The motor allocator saturation duration claim in Experiments.tex is already supported by `\autoref{fig:allocator_saturation}` in Annex.tex (line 138-148). The plot `plots/Allocator Saturation Duration.png` exists in the repo.

---

### Task E14 — "Orient horizontally" + "get rid of white spaces" ✅ DONE
**🤖 ROBOT** — CAD drawing now landscape, correlation plot comment resolved

| Source | Comments 40, 41 |
|--------|-----------|
| File   | Annex.tex (CAD drawing), consolidated_feature_correlation.png |

**Resolution:**
- Comment 40 (CAD assembly drawing): Figure now wrapped in `\begin{landscape}` so the A1 technical drawing fills the rotated page.
- Comment 41 (correlation slopegraph whitespace): User confirmed "is fine."

------

## DISCUSSION (Section 5)

### Task D3 — "weird, Limitations 2, ask user what to do"
**👤 HUMAN** — user must clarify

| Source | Comment 38 |
|--------|-----------|
| File   | `thesis/Sections/Discussion.tex` (Limitations subsection) |

**Issue:** The Limitations subsection has something weird — maybe two limitations blocks (Limitations in Discussion + Limitations in Conclusion = duplicate)?

**User must:** Clarify what's "weird" about the limitations.

---

### Task D4 — "do question time with the user to figure out the limitations sections"
**👤 HUMAN** — collaborative session needed

| Source | Comment 35 |
|--------|-----------|
| File   | `thesis/Sections/Discussion.tex` + `thesis/Sections/Conclusion.tex` |

**Issue:** The supervisor wants the author to sit down and figure out what goes in Limitations. Currently both Discussion (Section 5.4) and Conclusion (Section 6.2) have Limitations subsections — possible overlap/duplication.

**User must:** Work through limitations content with Claude to decide what stays, what moves, what's duplicate.

---

## CONCLUSION & FUTURE WORK (Section 6)

### Task C1 — "add gimbal Elios like solution..." + wall test picture
**👤 HUMAN** — user must provide content and picture

| Source | Comment 39 |
|--------|-----------|
| File   | `thesis/Sections/Conclusion.tex` (Future Work subsection) |

**Issue:** "add gymbal Elios like solution to take odometry of Vertical Surfaces + user adds a picture from the wall test" — Future Work should mention a gimbal-mounted solution (like Elios's rotating camera) for vertical surface inspection. User has a picture from a wall test.

**User must:** Provide the wall test picture and confirm gimbal concept wording.

---

## GRAPHICS & PLOTS (cross-cutting)

### Task G1 — Architecture figure caption still overflowing
**🤖 ROBOT** (already attempted — needs further fix if still overflowing)

**Note:** Architecture figure was cropped and recompiled. Verify in latest PDF.

---

## SUMMARY TABLE

| # | Task | Actor | Status |
|---|------|-------|--------|
| M6 | Figma/Draw.io diagram | 👤 | Needs user |
| M8 | Horizontal diagram | 👤 | Needs user |
| M9 | Move MoCap to Methodology | 🤖+👤 | ✅ Done |
| E3 | Merge MoCap/EKF (preview) | 👤 | ✅ Done (merged into M9) |
| E12 | Annex plots for claim | 👤+🤖 | ✅ Done |
| E14 | Orient horizontally + crop whitespace | 👤 | ✅ Done |
| D3 | "weird Limitations 2" | 👤 | Needs user |
| D4 | Limitations session | 👤 | Needs user |
| C1 | Elios gimbal + wall picture | 👤+🤖 | Draft written, awaiting user picture |
| G1 | Architecture figure overflow | 🤖 | Verify in latest compile |

**Summary:** 14 ✅ DONE / 4 👤 pending user / 1 🤖 Claude can do
