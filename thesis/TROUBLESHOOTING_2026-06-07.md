# Thesis LaTeX Troubleshooting Note (2026-06-07)

## Current Status
- `latexmk -pdf main.tex` now completes and `main.pdf` is generated.
- Hard compile blockers were fixed without deleting thesis body text.
- Remaining issues are mostly bibliography/data completeness tasks that need your input.

## What Was Fixed Automatically
1. Package option clash and incompatible bibliography setup in `main.tex`:
   - Removed duplicate package loading behavior causing `todonotes` option clash.
   - Switched `natbib` to numeric mode for compatibility with current bibliography flow.
   - Switched bibliography style to `plainnat` because `IEEEtran.bst` is unavailable in this TeX environment.
2. Cover-page SVG dependency issue:
   - Replaced `\includesvg` use on title page with a PNG logo include to avoid `--shell-escape` requirement and missing generated `*_svg-tex.pdf` assets.
3. UTF/soul interaction crash in Abstract:
   - Updated `\hlfix` to avoid `\texthl{...}` (which broke on UTF-8 text in the abstract) while keeping todo notes intact.
4. LaTeX underscore parsing error in Experiments:
   - Escaped `flight_director.py` as `flight\_director.py`.
5. Missing bibliography database file:
   - Added placeholder `IEEEabrv.bib` so the declared bibliography files exist.

## Needs Your Attention

### A) Missing bibliography entries (61 keys)
These keys are cited in `.tex` files but do not currently exist in `references.bib`:

- `2004:ITE:1009386.1010128`
- `Ablamowicz07`
- `Abril07`
- `Andler79`
- `anisi03`
- `ArduPilotDocs`
- `AviationDrones`
- `ChinaHobbyLine2025`
- `Clarkson85`
- `Cohen07`
- `CTANacmart`
- `docker:install`
- `docker:postinstall`
- `Douglass98`
- `Editor00`
- `Editor00a`
- `EMAX2025`
- `EMAXDatasheet`
- `flyability`
- `gombok`
- `Hagerup1993`
- `Hanery`
- `Harel78`
- `Harel79`
- `Holybro2025`
- `Innov8tive`
- `JCohen96`
- `JoeScientist001`
- `Kirschmer:2010:AEI:1958016.1958018`
- `Knuth97`
- `Kosiur01`
- `Lamport:LaTeX`
- `LargeBattery2025`
- `Lee05`
- `LIGPower2025`
- `MechTex2024`
- `Meier2015`
- `MR781536`
- `MR781537`
- `MyFPVStore2025`
- `myriadxfigure`
- `Novak03`
- `Obama08`
- `OscarLiang2025`
- `OscarLiang2025BegGuide`
- `Poker06`
- `PX4Docs`
- `R`
- `RCDroneTop2024`
- `RecursionLabs2021`
- `rous08`
- `SaeediJETC10`
- `SaeediMEJ10`
- `Smith10`
- `Spector90`
- `Thornburg01`
- `TUGInstmem`
- `TytoRobotics2025`
- `UMassCitations`
- `UST2023`
- `VanGundy07`

### B) Bibliography data quality inside existing entries
Some existing entries in `references.bib` have missing fields (for example missing year and/or journal), which causes bib warnings and low-quality references in output.

### C) Thesis content placeholders to resolve manually
Some sections still include explicit placeholders like:
- `(*INCLUDE DIAGRAM of Flight director architecture logic HERE*)`
- `(*Include pictures of the settup here, perhaps one with drone in it as well*)`

These are not compile blockers, but they need your final content decisions.

## Suggested Next Pass (I can do this next)
1. Add placeholder BibTeX stubs automatically for all missing keys (so citations resolve immediately).
2. Keep TODO markers so you can replace each stub with a real source later.
3. Rebuild and verify zero undefined-citation warnings.
