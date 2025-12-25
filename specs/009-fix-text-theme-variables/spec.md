# Feature Specification: Fix Invisible Text via Theme Variables

**Feature Branch**: `009-fix-text-theme-variables`
**Created**: 2025-12-25
**Status**: Draft
**Input**: User description: "Task: Fix invisible text issue caused by missing theme variables and enable proper light/dark mode

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
- No hover-based text revea"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - View Homepage Content (Priority: P1)

A visitor accesses the Physical AI & Humanoid Robotics textbook homepage and expects to immediately see the hero title and subtitle without needing to hover over them. The text should be clearly visible in both light and dark modes with proper contrast.

**Why this priority**: This is the core user experience - users must be able to see the main content immediately upon visiting the site, which is critical for accessibility and usability.

**Independent Test**: User can load the homepage and see the hero title and subtitle clearly visible without any hover interaction, with proper contrast in both light and dark themes.

**Acceptance Scenarios**:

1. **Given** user visits the homepage, **When** page loads, **Then** hero title and subtitle are immediately visible with readable contrast
2. **Given** user visits the homepage, **When** page loads in dark mode, **Then** hero title and subtitle are immediately visible with readable contrast

---

### User Story 2 - Toggle Light/Dark Mode (Priority: P2)

A visitor uses the theme toggle to switch between light and dark modes and expects all text elements to maintain proper visibility and contrast in both themes.

**Why this priority**: Proper theme support is essential for accessibility and user preference, ensuring the content remains readable for all users.

**Independent Test**: User can toggle between light and dark themes and see that all text elements maintain proper visibility and contrast in both modes.

**Acceptance Scenarios**:

1. **Given** user is on homepage in light mode, **When** toggles to dark mode, **Then** all text remains clearly visible with proper contrast
2. **Given** user is on homepage in dark mode, **When** toggles to light mode, **Then** all text remains clearly visible with proper contrast

---

### User Story 3 - Access Content with Visual Impairments (Priority: P3)

A user with visual impairments accesses the site and expects high contrast and clear text visibility to ensure accessibility compliance with WCAG standards.

**Why this priority**: Accessibility is critical for ensuring the educational content is available to all users regardless of their visual capabilities.

**Independent Test**: Text elements meet WCAG AA contrast standards in both light and dark modes.

**Acceptance Scenarios**:

1. **Given** user has visual impairment, **When** visiting homepage, **Then** text has sufficient contrast to meet WCAG AA standards

---

### Edge Cases

- What happens when browser does not support custom CSS variables?
- How does the system handle users with high contrast accessibility settings?
- What occurs when CSS fails to load properly?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST define --ifm-heading-color variable with appropriate color value for both themes
- **FR-002**: System MUST define --ifm-font-color-base variable with appropriate color value for both themes
- **FR-003**: System MUST ensure hero title text is visible immediately on page load without hover interaction
- **FR-004**: System MUST ensure hero subtitle text is visible immediately on page load without hover interaction
- **FR-005**: System MUST maintain readable contrast ratios meeting WCAG AA standards in both light and dark modes
- **FR-006**: System MUST preserve existing robotics animations and layout during the fix
- **FR-007**: System MUST enable proper light/dark mode toggle functionality
- **FR-008**: System MUST apply fixes through src/css/custom.css file as required
- **FR-009**: System MUST ensure no hover-based text reveal behavior exists

### Key Entities *(include if feature involves data)*

- **Theme Variables**: Docusaurus Infima CSS variables that control text color and visibility
- **Hero Text Elements**: Title and subtitle elements that need proper color assignment
- **Color Schemes**: Light and dark mode color definitions that maintain proper contrast

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of users can see hero title immediately upon page load without requiring hover interaction
- **SC-002**: 100% of users can see hero subtitle immediately upon page load without requiring hover interaction
- **SC-003**: Text elements maintain WCAG AA contrast ratios (minimum 4.5:1 for normal text, 3:1 for large text) in both light and dark modes
- **SC-004**: 0% of text elements require hover to become visible on the homepage
- **SC-005**: Theme toggle functionality works correctly and text remains visible in both modes