# Implementation Plan: Fix Invisible Hero Subtitle Text

**Branch**: `010-fix-hero-subtitle-colors` | **Date**: 2025-12-25 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/010-fix-hero-subtitle-colors/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan addresses the invisible hero subtitle text issue on a Docusaurus 3.x robotics-themed site by defining missing Infima theme variables (--ifm-font-color-base, --ifm-font-color-secondary, --ifm-font-color-muted) for both light and dark modes. The solution ensures immediate subtitle visibility on page load without requiring hover interaction, maintains WCAG AA accessibility standards, and preserves existing animations and layout.

## Technical Context

**Language/Version**: CSS/SCSS with Docusaurus 3.x framework
**Primary Dependencies**: Docusaurus 3.x, Infima CSS framework, Node.js ecosystem
**Storage**: N/A (CSS-only solution)
**Testing**: Manual testing via npm start and browser validation
**Target Platform**: Web application with light/dark mode support
**Project Type**: Web application (frontend CSS customization)
**Performance Goals**: Minimal CSS bundle impact, immediate text rendering
**Constraints**: Must use CSS-first approach, maintain existing animations and layout
**Scale/Scope**: Single CSS file modification (src/css/custom.css)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- CSS-first solution approach: Compliant with constraint requirements
- Docusaurus 3.x compatibility: Confirmed with existing framework
- Accessibility compliance: WCAG AA contrast standards will be maintained
- Theme variable approach: Follows Docusaurus best practices instead of CSS overrides

## Project Structure

### Documentation (this feature)

```text
specs/010-fix-hero-subtitle-colors/
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
│   └── css/
│       └── custom.css      # Primary CSS file for theme variable definitions
└── docs/                   # Docusaurus content pages
```

**Structure Decision**: Single web application with CSS-only solution. The fix is contained to the custom.css file which defines Docusaurus Infima theme variables for both light and dark modes.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
