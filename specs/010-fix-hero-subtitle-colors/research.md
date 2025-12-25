# Research: Fix Invisible Hero Subtitle Text

## Decision: Define Missing Infima Font Color Variables
**Rationale**: The root cause of invisible hero subtitle text is missing Docusaurus Infima theme variables that control text color and visibility. Instead of using CSS overrides, defining the proper theme variables ensures consistent behavior across the site and follows Docusaurus best practices.

**Alternatives considered**:
- CSS overrides directly targeting subtitle elements (rejected - not maintainable)
- JavaScript-based visibility fix (rejected - accessibility concerns, performance)
- Inline styles (rejected - violates Docusaurus theming principles)

## Decision: Support Both Light and Dark Modes
**Rationale**: The Docusaurus site supports theme toggling between light and dark modes. The solution must ensure proper text visibility and contrast in both themes to maintain accessibility and user experience.

**Alternatives considered**:
- Light mode only (rejected - breaks dark mode accessibility)
- Dark mode only (rejected - breaks light mode accessibility)
- Different approaches per theme (rejected - inconsistent maintenance)

## Decision: Use WCAG AA Compliant Colors
**Rationale**: To ensure accessibility for users with visual impairments, all text color definitions must meet WCAG AA contrast standards (minimum 4.5:1 for normal text, 3:1 for large text).

**Alternatives considered**:
- A11Y contrast (rejected - insufficient for users with visual impairments)
- Custom contrast ratios (rejected - doesn't meet established standards)
- Browser default colors (rejected - insufficient contrast for robotics theme)

## Decision: Preserve Existing Animations and Layout
**Rationale**: The robotics-themed site has existing animations and layout that should remain unchanged. The fix should only address the text visibility issue without affecting other visual elements.

**Alternatives considered**:
- Redesign hero section (rejected - outside scope, affects layout)
- Remove animations (rejected - reduces user experience)
- Change layout structure (rejected - affects other components)

## Decision: Implement in src/css/custom.css Only
**Rationale**: Following the requirement to fix via src/css/custom.css only, all theme variable definitions will be contained in this single file to maintain simplicity and consistency with Docusaurus conventions.

**Alternatives considered**:
- Multiple CSS files (rejected - increases complexity)
- Component-level styling (rejected - doesn't address theme system properly)
- JavaScript-based theme management (rejected - not CSS-first approach)