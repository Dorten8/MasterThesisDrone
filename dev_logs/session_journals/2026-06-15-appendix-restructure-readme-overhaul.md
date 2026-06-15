# 2026-06-15: Appendix restructure, README overhaul, citation cleanup, critical audit

## What was done

### Appendices (new `\appendix` structure in main.tex)
- **Software Implementation Overview** (Appendix B, `AppendixSoftware.tex`): Created a full LaTeX adaptation of the README with a QR code title page (GitHub repo, `\qrcode`), system architecture description, repo tree, six-step flight loop explanation, ROS 2 component table, network reference (MAC addresses, mDNS), and the 10 critical gotchas from the dev process.
- **AI Declaration** (Appendix C, `AppendixAIDeclaration.tex`): ITU-compliant narrative covering Gemini (6 example prompts), DeepSeek/Claude Code + Antigravity (6 example prompts), Microsoft Designer, Notebook LM, and a final review statement. Prompt lists wrapped in `% TODO [USER]: Replace` comments.
- Annex (A) moved under `\appendix` (previously standalone).

### README (`README.md`)
- Added table of contents
- Added full ASCII architecture diagram showing the four-layer stack
- Added "How the Drone Flies" section with embedded thesis figures (state machine, experiment geometry, pipeline)
- Added network reference (nmap discovery, MAC addresses, SSHFS workflow from dev_notes.md)
- Added uXRCE-DDS vs MAVLink mode switching and ROS 2 topic map table
- Added git backup workflow from dev_notes.md
- Added gotchas #9 (serial port check) and #10 (IDE service worker crash)
- Added full "Rebuild from Scratch" section
- Quick-reference scripts table

### Critical fixes
- **LLM warning removed**: The `***LLM NOTE: POSTPONE...***` in Discussion §5.1→§5.2 deleted.
- **TODO cleanup**: `\todo{...}` margin notes in Methodology §3.2.1 converted to hidden `% TODO [USER]` comments.
- **YouTube citation [30] removed**: All 5 occurrences of `recursionlabsRecursionLabs6s2021` across Methodology.tex and Annex.tex replaced with Hughes & Drury *Electric Motors and Drives* (4th ed.) textbook citation, grounding the 6S efficiency claim in `P=VI → I²R` theory. Bib entry deleted.
- **SDLD → RMSLD**: Renamed throughout Experiments.tex (definition, formula `\mathrm{RMSLD}`, caption, body text). Removes the "standard deviation without mean subtraction" vulnerability.
- **Lerche citation fixed**: Changed `@article` → `@mastersthesis`, added `school = {IT University of Copenhagen}` and `year = {2025}`.
- **Flat trendline sentence**: Added to battery deceleration paragraph explaining why near-zero Huber slopes are good news (no confounding).
- **Title page**: `\title{Title}` → actual thesis title; pre-submission disclaimer added.
- **`\usepackage{qrcode}`** added.

### Critical review delivered
14 issues flagged: stick-slip uncited (🟡), 179 vs 168 flight discrepancy (🔴), column oscillation unquantified (🟡), IMU rate needs qualification (🟡), battery drain context (🟡), SSoT jargon (🟡), propeller typos in §3.1.1 (🔴), informal style in §3.1.1 (🟡), abrupt Results ending (🟡), "Kinematics" heading terminology (🟢 minor), Pi 5 justification qualitative (🟡).

## Files touched
- `README.md` (581 lines added/restructured)
- `thesis/Sections/AppendixAIDeclaration.tex` (new, 47 lines)
- `thesis/Sections/AppendixSoftware.tex` (new, 160 lines)
- `thesis/Sections/Discussion.tex` (LLM warning removed, 4 lines)
- `thesis/Sections/Experiments.tex` (SDLD→RMSLD, flat trendline, ~170 lines restructured)
- `thesis/Sections/Introduction.tex` (tone fixes, 4 lines)
- `thesis/Sections/Methodology.tex` (\todo→% TODO, 6S citation, heading fix, 8 lines)
- `thesis/Sections/Annex.tex` (citation swap, 8 lines)
- `thesis/main.tex` (qrcode package, \appendix, \title, disclaimer, 10 lines)
- `thesis/references.bib` (lerche @mastersthesis fix, 4 lines)
- `thesis/references_non_zotero.bib` (recursionlabs removed, hughes added, 19 lines)
- `thesis/main.pdf` (recompiled binary)

## Commit
`2c52f05` — comprehensive documentation and appendix overhaul
