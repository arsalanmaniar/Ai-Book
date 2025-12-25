# Quickstart: Fix Invisible Hero Subtitle Text

## Overview
This guide provides step-by-step instructions to fix invisible hero subtitle text by defining missing Docusaurus Infima theme variables for both light and dark modes.

## Prerequisites
- Docusaurus 3.x project set up
- Node.js and npm installed
- Access to `src/css/custom.css` file

## Implementation Steps

### Step 1: Define Light Mode Variables
Add the following CSS custom properties to the `:root` selector in `src/css/custom.css`:

```css
:root {
  /* Infima font color variables for hero subtitle visibility */
  --ifm-font-color-base: #292929; /* Dark gray for good contrast */
  --ifm-font-color-secondary: #555555; /* Secondary text color */
  --ifm-font-color-muted: #888888; /* Muted text color */
}
```

### Step 2: Define Dark Mode Variables
Add the following CSS custom properties to the `[data-theme='dark']` selector in `src/css/custom.css`:

```css
[data-theme='dark'] {
  /* Infima font color variables for hero subtitle visibility in dark mode */
  --ifm-font-color-base: #e0e0e0; /* Light gray for dark mode contrast */
  --ifm-font-color-secondary: #b0b0b0; /* Secondary text color for dark mode */
  --ifm-font-color-muted: #909090; /* Muted text color for dark mode */
}
```

### Step 3: Verify Theme Toggle Functionality
Test that the theme toggle works correctly by:
1. Starting the development server: `npm start`
2. Navigating to the homepage
3. Toggling between light and dark modes
4. Confirming the hero subtitle remains visible in both themes

### Step 4: Validate Accessibility
Ensure WCAG AA compliance by checking:
- Text contrast ratios meet minimum 4.5:1 for normal text
- Hero subtitle is clearly visible without hover interaction
- No visual regressions in other components

## Testing
After implementation, verify:
- Hero subtitle is visible immediately on page load (no hover required)
- Theme toggle works correctly
- All text elements maintain proper contrast
- npm start runs without errors
- Existing animations and layout preserved

## Troubleshooting
- If text is still invisible: Check that variables are properly defined in both light and dark modes
- If theme toggle doesn't work: Verify CSS selectors are correct and no conflicting styles exist
- If contrast is insufficient: Adjust color values to meet WCAG AA standards