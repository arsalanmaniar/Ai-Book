# Tasks: Fix Invisible Hero Subtitle Text

## Feature
Fix Invisible Hero Subtitle Text

## Implementation Strategy
Implement the theme variable fixes in phases, starting with foundational setup (understanding current theme structure) followed by the actual variable definitions (light and dark mode variables), then validation and testing. Each phase builds upon the previous one to ensure a proper fix that addresses the root cause while maintaining accessibility and proper contrast.

## Dependencies
- Docusaurus framework must be properly installed and configured
- Existing custom CSS structure should remain intact during implementation
- All CSS changes must maintain compatibility with Docusaurus theme system

## User Stories Priority
1. **P1**: Define theme variables for immediate subtitle visibility
2. **P2**: Ensure proper light/dark mode toggle functionality
3. **P3**: Maintain WCAG AA contrast standards
4. **P4**: Preserve existing animations and layout

## Parallel Execution Examples
- [P] Tasks that can be executed in parallel: Light and dark mode variable definitions
- [P] Tasks that must be sequential: Variable definition, testing and validation

---

## Phase 1: Setup (Project Initialization)

- [X] T001 Set up development environment for theme variable modifications
- [X] T002 Understand current Docusaurus theme structure and Infima variables

---

## Phase 2: Foundational (Blocking Prerequisites)

- [X] T003 [P] Analyze current CSS structure in src/css/custom.css
- [X] T004 [P] Identify missing Infima theme variables (--ifm-font-color-base, --ifm-font-color-secondary, --ifm-font-color-muted)
- [X] T005 [P] Document current color contrast ratios for accessibility validation

---

## Phase 3: [US1] Define Theme Variables for Visibility

- [ ] T006 [US1] Define --ifm-font-color-base variable for light mode in src/css/custom.css
- [ ] T007 [US1] Define --ifm-font-color-secondary variable for light mode in src/css/custom.css
- [ ] T008 [US1] Define --ifm-font-color-muted variable for light mode in src/css/custom.css
- [ ] T009 [US1] Define --ifm-font-color-base variable for dark mode in src/css/custom.css
- [ ] T010 [US1] Define --ifm-font-color-secondary variable for dark mode in src/css/custom.css
- [ ] T011 [US1] Define --ifm-font-color-muted variable for dark mode in src/css/custom.css
- [ ] T012 [US1] Test hero subtitle visibility without hover interaction

---

## Phase 4: [US2] Enable Light/Dark Mode Toggle

- [ ] T013 [US2] Verify theme toggle functionality works correctly
- [ ] T014 [US2] Test color consistency when switching between themes
- [ ] T015 [US2] Validate theme variables apply correctly in both modes
- [ ] T016 [US2] Test theme persistence across page refreshes

---

## Phase 5: [US3] Maintain Accessibility Standards

- [ ] T017 [US3] Validate WCAG AA contrast ratios in light mode
- [ ] T018 [US3] Validate WCAG AA contrast ratios in dark mode
- [ ] T019 [US3] Test with high contrast accessibility settings
- [ ] T020 [US3] Verify text readability for users with visual impairments

---

## Phase 6: [US4] Preserve Existing Features

- [ ] T021 [US4] Verify existing animations continue to work properly
- [ ] T022 [US4] Test that layout and design remain unchanged
- [ ] T023 [US4] Confirm no visual regressions in other components
- [ ] T024 [US4] Ensure robotics-themed UI elements are preserved

---

## Phase 7: Polish & Cross-Cutting Concerns

- [ ] T025 Optimize CSS for minimal bundle size impact
- [ ] T026 Build and verify the site works correctly with all changes
- [ ] T027 Test across different browsers and devices for consistency
- [ ] T028 Validate no hover-based text reveal behavior exists
- [ ] T029 Document the theme variable definitions for future maintenance