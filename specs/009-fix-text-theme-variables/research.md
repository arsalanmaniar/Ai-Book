# Research: Fix Invisible Text via Theme Variables

## Decision: Use Infima theme variables as the primary approach
**Rationale**: Defining the missing --ifm-heading-color and --ifm-font-color-base variables addresses the root cause rather than applying CSS overrides. This ensures consistent theming across the entire Docusaurus site.

## Decision: Define theme variables for both light and dark modes
**Rationale**: Proper theme support requires distinct color definitions for both light and dark modes to maintain accessibility and proper contrast ratios in each theme.

## Decision: Use WCAG AA compliant color values
**Rationale**: Ensuring contrast ratios of at least 4.5:1 for normal text and 3:1 for large text meets accessibility standards for users with visual impairments.

## Decision: Implement in src/css/custom.css following Docusaurus conventions
**Rationale**: This follows Docusaurus best practices for custom styling while maintaining compatibility with the theme system.

## Decision: Preserve existing styling for other elements
**Rationale**: Only the missing theme variables need to be defined; existing custom styles should remain unchanged to preserve the robotics-themed UI.