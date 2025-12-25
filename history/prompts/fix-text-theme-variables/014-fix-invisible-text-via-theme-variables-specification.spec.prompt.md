---
id: "014"
title: "Fix Invisible Text via Theme Variables Specification"
stage: "spec"
date: "2025-12-25"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "fix-text-theme-variables"
branch: "009-fix-text-theme-variables"
user: "User"
command: "/sp.specify"
labels: ["specification", "docusaurus", "css", "theme-variables", "accessibility"]
links:
  spec: "C:\\\\Users\\\\DC\\\\Desktop\\\\ai-hackathon\\\\specs\\\\009-fix-text-theme-variables\\\\spec.md"
  plan: null
  ticket: null
  adr: null
  pr: null
files:
  - "C:\\\\Users\\\\DC\\\\Desktop\\\\ai-hackathon\\\\specs\\\\009-fix-text-theme-variables\\\\spec.md"
  - "C:\\\\Users\\\\DC\\\\Desktop\\\\ai-hackathon\\\\specs\\\\009-fix-text-theme-variables\\\\checklists\\\\requirements.md"
tests: []
---

## Prompt

Task: Fix invisible text issue caused by missing theme variables and enable proper light/dark mode

Context:
This Docusaurus 3.x site uses a custom robotics-themed UI.
Homepage hero text is invisible unless hovered.
Root cause identified: required Infima theme variables (heading and base text colors)
are missing, causing text to render transparent.

Problems to Fix:
- --ifm-heading-color is not defined
- --ifm-font-color-base is not defined
- Hero text inherits transparent color
- Hover state reveals text unintentionally
- Light/Dark mode toggle must work correctly

Requirements:
- Explicitly define --ifm-heading-color and --ifm-font-color-base
- Set correct readable colors for BOTH light and dark modes
- Ensure hero title and subtitle are visible without hover
- Preserve existing robotics animations and layout
- Enable proper light/dark mode toggle in navbar
- Fix must be applied in src/css/custom.css and config if needed

Success Criteria:
- Text visible immediately on page load
- No hover-based text revea

## Response snapshot

Successfully created a comprehensive specification for fixing the invisible text issue by addressing missing theme variables. The specification includes:

1. Three prioritized user stories covering core functionality (P1: Homepage content visibility), theme toggling (P2: Light/dark mode), and accessibility (P3: Visual impairment support)

2. Nine functional requirements covering:
   - Defining required theme variables (--ifm-heading-color, --ifm-font-color-base)
   - Ensuring immediate text visibility without hover
   - Maintaining WCAG AA contrast standards
   - Preserving existing animations and layout
   - Enabling proper theme toggle functionality

3. Five measurable success criteria including:
   - 100% visibility of hero title and subtitle on page load
   - WCAG AA contrast ratios maintained
   - 0% hover-based text reveal behavior
   - Correct theme toggle functionality

4. Quality checklist confirming all requirements are met and no clarifications needed

The specification provides a foundation for implementing a more robust fix by addressing the root cause (missing theme variables) rather than using CSS overrides.

## Outcome

- ✅ Impact: Created comprehensive specification for theme variable fix approach
- 🧪 Tests: N/A - Specification phase
- 📁 Files: Created spec.md and requirements checklist
- 🔁 Next prompts: Ready for planning phase (/sp.plan)
- 🧠 Reflection: Specification addresses root cause more effectively than CSS overrides

## Evaluation notes (flywheel)

- Failure modes observed: None - specification complete
- Graders run and results (PASS/FAIL): PASS - all validation items checked
- Prompt variant (if applicable): Standard specification workflow
- Next experiment (smallest change to try): Proceed to planning phase