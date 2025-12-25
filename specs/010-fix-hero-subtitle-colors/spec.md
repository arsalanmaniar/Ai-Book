# Feature Specification: Fix Invisible Hero Subtitle Text

**Feature Branch**: `010-fix-hero-subtitle-colors`
**Created**: 2025-12-25
**Status**: Draft
**Input**: User description: "Task: Fix invisible hero subtitle text by defining missing Infima font color variables

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
- No"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - View Hero Section Content (Priority: P1)

A visitor accesses the Physical AI & Humanoid Robotics textbook homepage and expects to immediately see both the hero title and subtitle without needing to hover over them. The subtitle text should be clearly visible with proper contrast in both light and dark modes.

**Why this priority**: This is the core user experience - users must be able to see the complete hero section content immediately upon visiting the site, which is critical for accessibility and usability.

**Independent Test**: User can load the homepage and see the hero subtitle clearly visible without any hover interaction, with proper contrast in both light and dark themes.

**Acceptance Scenarios**:

1. **Given** user visits the homepage, **When** page loads, **Then** hero subtitle is immediately visible with readable contrast
2. **Given** user visits the homepage, **When** page loads in dark mode, **Then** hero subtitle is immediately visible with readable contrast

---

### User Story 2 - Navigate with Visual Impairments (Priority: P2)

A user with visual impairments accesses the site and expects high contrast and clear subtitle text visibility to ensure accessibility compliance with WCAG standards.

**Why this priority**: Accessibility is critical for ensuring the educational content is available to all users regardless of their visual capabilities.

**Independent Test**: Subtitle text elements meet WCAG AA contrast standards in both light and dark modes.

**Acceptance Scenarios**:

1. **Given** user has visual impairment, **When** visiting homepage, **Then** subtitle text has sufficient contrast to meet WCAG AA standards

---

### User Story 3 - Toggle Light/Dark Mode (Priority: P3)

A visitor uses the theme toggle to switch between light and dark modes and expects the hero subtitle to maintain proper visibility and contrast in both themes.

**Why this priority**: Proper theme support is essential for accessibility and user preference, ensuring the content remains readable for all users.

**Independent Test**: User can toggle between light and dark themes and see that the hero subtitle maintains proper visibility and contrast in both modes.

**Acceptance Scenarios**:

1. **Given** user is on homepage in light mode, **When** toggles to dark mode, **Then** subtitle text remains clearly visible with proper contrast
2. **Given** user is on homepage in dark mode, **When** toggles to light mode, **Then** subtitle text remains clearly visible with proper contrast

---

### Edge Cases

- What happens when browser does not support custom CSS variables?
- How does the system handle users with high contrast accessibility settings?
- What occurs when CSS fails to load properly?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST define --ifm-font-color-base variable with appropriate color value for both themes
- **FR-002**: System MUST define --ifm-font-color-secondary variable with appropriate color value for both themes
- **FR-003**: System MUST define --ifm-font-color-muted variable with appropriate color value for both themes
- **FR-004**: System MUST ensure hero subtitle text is visible immediately on page load without hover interaction
- **FR-005**: System MUST maintain readable contrast ratios meeting WCAG AA standards in both light and dark modes
- **FR-006**: System MUST preserve existing robotics animations and layout during the fix
- **FR-007**: System MUST apply fixes through src/css/custom.css file as required
- **FR-008**: System MUST ensure no hover-based text reveal behavior exists for subtitle text
- **FR-009**: System MUST enable proper light/dark mode toggle functionality

### Key Entities *(include if feature involves data)*

- **Theme Variables**: Docusaurus Infima CSS variables that control subtitle text color and visibility
- **Hero Subtitle Element**: Text element that needs proper color assignment
- **Color Schemes**: Light and dark mode color definitions that maintain proper contrast

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of users can see hero subtitle immediately upon page load without requiring hover interaction
- **SC-002**: Subtitle text maintains WCAG AA contrast ratios (minimum 4.5:1 for normal text, 3:1 for large text) in both light and dark modes
- **SC-003**: 0% of subtitle elements require hover to become visible on the homepage
- **SC-004**: Theme toggle functionality works correctly and subtitle remains visible in both modes
- **SC-005**: npm start runs without errors after applying the fix