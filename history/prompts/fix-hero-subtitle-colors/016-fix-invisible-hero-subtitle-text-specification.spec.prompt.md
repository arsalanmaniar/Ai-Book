---
id: "016"
title: "Fix Invisible Hero Subtitle Text Specification"
stage: "spec"
date: "2025-12-25"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "fix-hero-subtitle-colors"
branch: "010-fix-hero-subtitle-colors"
user: "User"
command: "/sp.specify"
labels: ["specification", "docusaurus", "css", "theme-variables", "accessibility"]
links:
  spec: "C:\\\\Users\\\\DC\\\\Desktop\\\\ai-hackathon\\\\specs\\\\010-fix-hero-subtitle-colors\\\\spec.md"
  plan: null
  ticket: null
  adr: null
  pr: null
files:
  - "C:\\\\Users\\\\DC\\\\Desktop\\\\ai-hackathon\\\\specs\\\\010-fix-hero-subtitle-colors\\\\spec.md"
  - "C:\\\\Users\\\\DC\\\\Desktop\\\\ai-hackathon\\\\specs\\\\010-fix-hero-subtitle-colors\\\\checklists\\\\requirements.md"
tests: []
---

## Prompt

Task: Fix invisible hero subtitle text by defining missing Infima font color variables

Context:
Docusaurus 3.x robotics-themed site with custom UI.
Hero title is now visible, but hero subtitle/description
is still invisible unless hovered.

Root Cause:
Missing Infima text color variables used by subtitle text:
- --ifm-font-color-base
- --ifm-font-color-secondary
- --ifm-font-color-muted

Problems to Fix:
- Hero subtitle only visible on hover
- Subtitle inherits transparent/low-contrast color
- Light and dark modes not fully defined for body text

Requirements:
- Explicitly define all Infima font color variables
- Ensure hero subtitle is visible without hover
- Apply fixes for BOTH light and dark modes
- Keep existing animations and layout unchanged
- Fix via src/css/custom.css only

Success Criteria:
- Hero subtitle visible immediately on load
- No hover-based reveal for any text
- Light/Dark mode toggle works correctly
- npm start runs without errors

Constraints:
- Docusaurus 3.x
- CSS-first solution
- No

## Response snapshot

Successfully created a comprehensive specification for fixing the invisible hero subtitle text issue by defining missing Infima font color variables. The specification includes:

1. Three prioritized user stories covering core functionality (P1: Hero section content visibility), accessibility (P2: Visual impairment support), and theme toggling (P3: Light/dark mode)

2. Nine functional requirements covering:
   - Defining required theme variables (--ifm-font-color-base, --ifm-font-color-secondary, --ifm-font-color-muted)
   - Ensuring immediate subtitle visibility without hover
   - Maintaining WCAG AA contrast standards
   - Preserving existing animations and layout
   - Enabling proper theme toggle functionality

3. Five measurable success criteria including:
   - 100% visibility of hero subtitle on page load
   - WCAG AA contrast ratios maintained
   - 0% hover-based text reveal behavior
   - Correct theme toggle functionality
   - Build process working without errors

4. Quality checklist confirming all requirements are met and no clarifications needed

The specification provides a foundation for implementing a fix that addresses the root cause of the invisible subtitle text by properly defining the missing theme variables.

## Outcome

- ✅ Impact: Created comprehensive specification for subtitle text fix approach
- 🧪 Tests: N/A - Specification phase
- 📁 Files: Created spec.md and requirements checklist
- 🔁 Next prompts: Ready for planning phase (/sp.plan)
- 🧠 Reflection: Specification addresses root cause for subtitle visibility issue

## Evaluation notes (flywheel)

- Failure modes observed: None - specification complete
- Graders run and results (PASS/FAIL): PASS - all validation items checked
- Prompt variant (if applicable): Standard specification workflow
- Next experiment (smallest change to try): Proceed to planning phase