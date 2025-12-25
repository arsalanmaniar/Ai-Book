# Data Model: Theme Variables for Hero Subtitle Fix

## Entity: Theme Variable
**Description**: CSS custom properties (variables) used by Docusaurus Infima framework to control text color and visibility

**Fields**:
- `name`: String - The variable name (e.g., --ifm-font-color-base)
- `lightModeValue`: String - The color value for light theme
- `darkModeValue`: String - The color value for dark theme
- `description`: String - Purpose of the variable
- `accessibilityStandard`: String - WCAG compliance requirement

**Validation Rules**:
- Must follow CSS custom property syntax (--variable-name)
- Color values must meet WCAG AA contrast ratios
- Both light and dark mode values must be defined

## Entity: Infima Color Variable Set
**Description**: Collection of Infima variables that need to be defined for proper text visibility

**Fields**:
- `fontColorBase`: Theme Variable - Primary text color
- `fontColorSecondary`: Theme Variable - Secondary text color
- `fontColorMuted`: Theme Variable - Muted/less important text color
- `headingColor`: Theme Variable - Heading text color

**Relationships**:
- Each Theme Variable belongs to an Infima Color Variable Set
- All variables in the set must have both light and dark mode definitions

## Entity: Theme Configuration
**Description**: Complete theme configuration that includes the required color variables

**Fields**:
- `lightModeVariables`: Infima Color Variable Set - Variables for light theme
- `darkModeVariables`: Infima Color Variable Set - Variables for dark theme
- `validationStatus`: String - WCAG AA compliance status
- `themeToggleSupport`: Boolean - Whether theme switching works correctly

**State Transitions**:
- `incomplete` → `complete` when all required variables are defined
- `non-compliant` → `compliant` when WCAG standards are met
- `toggle-broken` → `toggle-working` when theme switching works

## Entity: Accessibility Compliance
**Description**: WCAG AA compliance validation for text color contrast

**Fields**:
- `foregroundColor`: String - Text color value
- `backgroundColor`: String - Background color value
- `contrastRatio`: Number - Calculated contrast ratio
- `wcagAACompliant`: Boolean - Whether meets 4.5:1 minimum (3:1 for large text)
- `validationDate`: Date - When compliance was checked

**Validation Rules**:
- Normal text must have contrast ratio ≥ 4.5:1
- Large text (18pt+) must have contrast ratio ≥ 3:1
- All theme combinations must pass validation