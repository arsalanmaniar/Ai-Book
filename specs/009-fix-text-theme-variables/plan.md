# Implementation Plan: Fix Invisible Text via Theme Variables

**Branch**: `009-fix-text-theme-variables` | **Date**: 2025-12-25 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/[009-fix-text-theme-variables]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Fix invisible homepage text issue by properly defining missing Docusaurus Infima theme variables (--ifm-heading-color and --ifm-font-color-base) for the Physical AI & Humanoid Robotics book site. This addresses the root cause by ensuring proper color definitions for both light and dark modes, making hero text visible immediately on page load without hover interaction while maintaining accessibility and proper contrast ratios.

## Technical Context

**Language/Version**: TypeScript/JavaScript (Docusaurus 3.9.2, React 19.0.0)
**Primary Dependencies**: @docusaurus/core 3.9.2, @docusaurus/preset-classic 3.9.2, React 19.0.0
**Storage**: N/A (static site generation)
**Testing**: N/A (static site generation)
**Target Platform**: Web (static site, cross-browser compatible)
**Project Type**: Web (Docusaurus static site)
**Performance Goals**: Minimal CSS bundle size impact, no performance degradation
**Constraints**: Must maintain compatibility with Docusaurus 3.x, preserve existing animations and layout
**Scale/Scope**: Single page fix (homepage hero section) with theme-wide impact, minimal scope

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- ✅ **Documentation Standards**: Follows Docusaurus conventions for CSS customization and theme variables
- ✅ **Quality & Testing**: CSS changes will be validated through build process
- ✅ **Contribution Guidelines**: Changes made through project files as specified
- ✅ **Licensing & IP**: No IP concerns for CSS styling changes
- ✅ **Accessibility**: Implementation ensures WCAG AA contrast standards are met

## Project Structure

### Documentation (this feature)

```text
specs/009-fix-text-theme-variables/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
my-book/
├── src/
│   ├── css/
│   │   └── custom.css      # CSS file to be modified for the fix
│   └── pages/
│       └── index.tsx       # Homepage component with hero section
├── docusaurus.config.ts    # Configuration file (CSS loading)
└── src/pages/index.module.css # Module CSS for homepage styling
```

**Structure Decision**: Single web project structure using Docusaurus static site generator. The fix will involve modifying `src/css/custom.css` to properly define the missing Infima theme variables that control text visibility and contrast.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A |
