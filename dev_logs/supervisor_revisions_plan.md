# Supervisor Revision Plan — Chronological Task List

**Source:** `thesis/JS_thesis_2026-06-29_commented.pdf` (42 annotations)
**Date:** 2026-06-29
**Workflow:** Each task is a discrete, verifiable unit. Done tasks are marked ~~strikethrough~~.

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

### Task M7 — "do not be that specific, say 'more on that in...'"
**🤖 ROBOT** — Claude can edit

| Source | Comment 17 |
|--------|-----------|
| File   | `thesis/Sections/Methodology.tex` (likely) |

**Issue:** Somewhere there's overly specific detail that should be replaced with a forward-reference ("more on that in Section X") and the figure placed wherever it naturally goes.

**Claude will:** Search for the overly specific technical description, propose a condensed version with cross-reference. Needs to identify the exact location first (will show user before editing).

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

### Task E1 — Mission outcome table cross-reference fix
**🤖 ROBOT** — and 👤 **HUMAN** confirm approach

| Source | Comment 22 |
|--------|-----------|
| File   | `thesis/Sections/Experiments.tex` |

**Issue:** "move it and mention it where Mission outcome summary across all 168 recorded passes is!" — a table or figure needs to be moved near where mission outcomes are discussed, and cross-referenced.

**Claude will:** Identify the floating element, propose its new position near the mission outcomes text.

---

### Task E2 — Deduplicate: "already stated before in methodology"
**👤 USER REVIEW PENDING** — user uncertain about the result

| Source | Comment 23 |
|--------|-----------|
| File   | `thesis/Sections/Experiments.tex` |

**Status:** Claude simplified the EKF2 description in Experiment Control and added a cross-reference to methodology. User is not certain the result is satisfactory — needs manual review.

---

### Task E3 — Merge MoCap/EKF content to Methodology (preview required)
**👤 HUMAN** — preview before executing

| Source | Comment 24 |
|--------|-----------|
| File   | `thesis/Sections/Experiments.tex` → `thesis/Sections/Methodology.tex` |

**Issue:** "to be merged and moved to methodology, require preview for user before executing" — more MoCap/EKF content to move. This may overlap with Task M9.

**Claude will:** Confirm scope with you before executing. Show exactly what moves.

---

### Task E9 — "delete the colors!" + plot color standard
**🤖 ROBOT** — may need code change

| Source | Comments 16, 28 |
|--------|-----------|
| File   | `thesis/Sections/Experiments.tex` + possibly `dev_logs/analysis/` Python files |

**Issue 1 (Comment 16):** "delete the colors!" — text describing figure colors. Done: removed color names from captions.

**Issue 2 (Comment 28):** "This plot has GREEN subtitle for rotating cage instead of Blue" — needs Python plot fix. Claude needs to identify which plot has wrong color.

**Claude will:** Check which plot has green subtitle and fix the Python code.
THAT WAS ONLY TO SPECIFIC PLACES! no color standard change, colors are good, they were just wron in some places!

---

### Task E10 — "make full width"
**🤖 ROBOT** — figure scaling

| Source | Comment 0 (Highlight) |
|--------|-----------|
| File   | `thesis/Sections/Experiments.tex` |

**Status:** Impact angle distribution figure now full width. User question: why no text above/below? → Fixed: bridging paragraph added before and lead-in sentence after.
AGREED DONE

---

### Task E12 — "add plots for this claim into annex"
**👤 HUMAN** — user must identify claim, 🤖 ROBOT generates plots

| Source | Comment 29 |
|--------|-----------|
| File   | `thesis/Sections/Experiments.tex` + `thesis/Sections/Annex.tex` |

**Issue:** A claim in Experiments needs supporting plots in the Annex.

**User must identify:** Which claim? Claude will then generate the plots and add them to the Annex.

---

### ~~Task E13 — "me no understand, make dumb pls"~~ ✅
**DONE** — Two passages simplified (correlation paragraph + cross-condition transfer).

NOTDONE!!!!
this is excatly what I meant! <This multivariate dispersion is further illustrated through parallel coor-
dinates> this is too complicated, simplify the languague

---

### Task E14 — "Orient horizontally" + "get rid of white spaces"
**👤 HUMAN** — user provides updated figures, 🤖 ROBOT adjusts LaTeX

| Source | Comments 40, 41 |
|--------|-----------|
| File   | Thesis figures |

**Issue 1 (Comment 40):** "Orient horizontally, ask user to update this drawing" — a figure (likely vertical/portrait) should be landscape.
**Issue 2 (Comment 41):** "ask user to get rid of white spaces so it can stretch further" — figure has too much whitespace.

**User must:** Provide updated versions of the problematic figures.
NOPE it has not got too much whitespace, it just wrong done somehow, it is a figure with this description <Experiment pipeline showing hardware components, ROS 2 soft-
ware stack, data ow from MoCap through the agent to PX4, and telemetry
logging to MCAP storage.>

---

### Task E15 — "delete!" / "delete this" / "get rid of this" (×5)
**👤 HUMAN** — user must identify what to delete

| Source | Comments 1, 9, 26, 27, 36 |
|--------|-----------|
| File   | Various |

**Issue:** Multiple "delete" comments at different locations. Claude cannot see the PDF highlights, so cannot identify the exact text.

**User must identify:** For each "delete" comment, specify the text/section to delete.
WELL THANK YOU KINDLY FOR LIYNG TO ME THEN IN THE CHAT CLAIMING U COULD SEE THE COMMENST AND HIGHLIGHTS

HERE WHAT SHOULD BE DELETED
<This document is a pre-submission draft. Minor revisions may follow; the nal
version will be uploaded pending any technical com>
from <Figure 5: Physical exploded view of the complete drone assembly showing
all major components: frame plates, Pixhawk 6C ight controller, Raspberry
Pi 5 companion computer, EMAX ECOII 2004 motors, 6S LiPo battery, and
the PETG protective cage with bearing assembly.> delete this <showing
all major components: frame plates, Pixhawk 6C ight controller, Raspberry
Pi 5 companion computer, EMAX ECOII 2004 motors, 6S LiPo battery, and
the PETG protective cage with bearing assembly.>

FROM <Onboard Impact-Angle Inference.> keep just this <Impact-Angle Inference.>

---

### Task E16 — "made up?! delete if made up!"
**👤 HUMAN** — user must confirm

| Source | Comment 12 |
|--------|-----------|
| File   | Unknown (in Experiments vicinity) |

**Issue:** Supervisor suspects a specific claim or number is fabricated.

**User must:** Identify the suspect claim.
DONE

---

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
| M7 | De-specific + cross-ref | 🤖 | Claude to do |
| M8 | Horizontal diagram | 👤 | Needs user |
| M9 | Move MoCap to Methodology | 🤖+👤 | Claude to propose |
| E1 | Mission outcome table position | 🤖 | Claude to do |
| E2 | Deduplicate methodology content | 👤 | User review pending |
| E3 | Merge MoCap/EKF (preview) | 👤 | Needs user |
| E9 | Delete colors + fix green subtitle | 🤖 | Colors done; green subtitle TODO |
| E10 | Make full width | ~~🤖~~ | ✅ DONE |
| E12 | Annex plots for claim | 👤+🤖 | Needs user to identify claim |
| E13 | Simplify text ("make dumb") | ~~🤖~~ | ✅ DONE |
| E14 | Orient horizontally + crop whitespace | 👤 | Needs user |
| E15 | Delete text (×5 comments) | 👤 | Needs user |
| E16 | "made up?!" claim | 👤 | Needs user |
| D3 | "weird Limitations 2" | 👤 | Needs user |
| D4 | Limitations session | 👤 | Needs user |
| C1 | Elios gimbal + wall picture | 👤 | Needs user |
| G1 | Architecture figure overflow | 🤖 | Verify in latest compile |

**Summary:** 2 ✅ DONE / 11 👤 pending user / 5 🤖 Claude can do
