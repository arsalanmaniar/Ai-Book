# Data Model: Fix Invisible Text via Theme Variables

## Entities

### Theme Variables
- **Name**: Theme Variables
- **Type**: CSS Custom Properties
- **Fields**:
  - name: string (variable name like --ifm-heading-color)
  - lightModeValue: string (color value for light theme)
  - darkModeValue: string (color value for dark theme)
  - description: string (purpose of the variable)
- **Validation**: Must provide sufficient contrast in both themes (WCAG AA standards)
- **Relationships**: Applied globally to affect all text elements

### Hero Text Elements
- **Name**: Hero Text Elements
- **Type**: UI Components
- **Fields**:
  - element: string (HTML element type)
  - className: string (CSS class name)
  - inheritsFrom: Theme Variables (which theme variables it uses)
- **Validation**: Must be visible without hover interaction
- **Relationships**: Consumes values from Theme Variables

### Color Schemes
- **Name**: Color Schemes
- **Type**: Theme Configuration
- **Fields**:
  - mode: string (light or dark)
  - headingColor: string (value for --ifm-heading-color)
  - textColor: string (value for --ifm-font-color-base)
  - backgroundColor: string (background for contrast)
- **Validation**: Must meet WCAG AA contrast ratios
- **Relationships**: Defines values for Theme Variables