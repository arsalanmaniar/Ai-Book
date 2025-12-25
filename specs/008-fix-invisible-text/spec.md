# Feature Specification: Fix Invisible Homepage Text

## Feature Name
Fix Invisible Homepage Text

## Feature Description
Fix invisible homepage text issue on the Docusaurus 3.x site for the Physical AI & Humanoid Robotics book. The hero title and subtitle are currently invisible until mouse hover due to Docusaurus global styles overriding custom hero CSS. This requires a permanent, global fix via CSS and configuration to force all homepage hero text to be visible at all times, removing hover-only text reveal behavior and overriding Docusaurus default hero text masking.

## User Scenarios & Testing

### Primary User Scenarios
1. **Student Learner**: A student visits the homepage and expects to see the hero title and subtitle immediately visible without needing to hover over them.

2. **Educator**: An instructor accesses the site and expects all content to be immediately visible without requiring mouse interaction.

3. **Researcher**: A researcher visits the site and expects professional presentation with clear, visible content from the start.

### Acceptance Scenarios
- As a user, I can see the hero title immediately upon loading the homepage without hovering
- As a user, I can see the hero subtitle immediately upon loading the homepage without hovering
- As a user, I experience no white-on-white or transparent text anywhere on the homepage
- As a user, I see all text content with proper contrast and visibility at all times
- As a user, I can navigate the site without needing to hover over text elements to see them

### Edge Cases
- Users with accessibility needs can navigate the site with properly visible text
- Users on different devices see consistent text visibility
- Users with different browsers experience the same text visibility behavior

## Functional Requirements

### Requirement 1: Text Visibility Fix
**Description**: Force all homepage hero text to be visible at all times
**Acceptance Criteria**:
- Hero title is visible immediately upon page load without hover
- Hero subtitle is visible immediately upon page load without hover
- No text opacity effects that hide content until hover
- All text elements maintain proper contrast ratios

### Requirement 2: Remove Hover-Only Behavior
**Description**: Remove hover-only text reveal behavior
**Acceptance Criteria**:
- No CSS rules that hide text until hover interaction
- Text elements are always visible regardless of mouse position
- No transition effects that make text temporarily invisible
- All content is accessible without requiring mouse interaction

### Requirement 3: Override Docusaurus Styling
**Description**: Override Docusaurus default hero text masking
**Acceptance Criteria**:
- Custom CSS properly overrides Docusaurus global styles
- No conflicting style rules that hide text content
- Custom styles are applied with sufficient specificity
- Existing robotics animations and layout are preserved

### Requirement 4: CSS Implementation
**Description**: Apply fixes in src/css/custom.css or equivalent
**Acceptance Criteria**:
- Fixes are implemented in the proper CSS file (src/css/custom.css)
- Custom CSS rules use appropriate specificity to override defaults
- No inline styles are used for this fix
- CSS is organized and maintainable

### Requirement 5: Configuration Integration
**Description**: Ensure custom CSS is properly loaded in docusaurus.config.ts
**Acceptance Criteria**:
- Custom CSS file is properly referenced in Docusaurus configuration
- Styles are loaded consistently across all pages
- No conflicts with Docusaurus theme system
- Build process includes the custom styles

### Requirement 6: Preserve Existing Features
**Description**: Preserve existing robotics animations and layout
**Acceptance Criteria**:
- All existing animations continue to work properly
- Current layout and design remain unchanged
- No visual regressions in other components
- Robotics-themed UI elements are preserved

## Non-Functional Requirements

### Performance
- CSS changes do not impact page load performance
- No additional HTTP requests required for text visibility fix
- Minimal CSS bundle size impact

### Accessibility
- All text maintains WCAG AA contrast standards
- Text remains readable for users with visual impairments
- No impact on screen reader functionality
- Keyboard navigation unaffected

### Compatibility
- Works across modern browsers (Chrome, Firefox, Safari, Edge)
- Consistent behavior across different devices
- Compatible with Docusaurus 3.x framework

## Success Criteria

### Quantitative Measures
- 100% of users can see hero title immediately upon page load
- 100% of users can see hero subtitle immediately upon page load
- 0% of text elements require hover to become visible
- All text elements pass WCAG AA contrast standards

### Qualitative Measures
- Users perceive the content as immediately accessible and professional
- The hero section appears complete and properly visible from the start
- No visual flickering or text reveal effects occur
- The overall user experience is improved with consistent visibility

## Key Entities
- **Hero Title**: Main heading text in the hero section
- **Hero Subtitle**: Secondary text in the hero section
- **CSS Override Rules**: Custom styles that override Docusaurus defaults
- **Docusaurus Configuration**: Configuration file that loads custom CSS

## Assumptions
- The site uses Docusaurus 3.x framework
- Custom CSS can be added to src/css/custom.css
- Docusaurus configuration allows custom CSS loading
- The issue is caused by CSS opacity or text masking
- Existing animations should be preserved during the fix

## Dependencies
- Docusaurus framework compatibility
- Existing custom CSS structure
- Current hero section implementation

## Constraints
- Must maintain compatibility with Docusaurus 3.x
- Cannot modify core Docusaurus files
- Must preserve existing animations and layout
- Fixes must be applied through project files only
- No white-on-white or transparent text anywhere