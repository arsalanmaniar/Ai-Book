# Data Model: Fix Invisible Homepage Text

## Entities

### Hero Title
- **Name**: Hero Title
- **Type**: Text element
- **Fields**:
  - text: string (the title text content)
  - className: string (CSS class name: hero__title)
  - visibility: boolean (whether text is visible without hover)
- **Validation**: Must be visible on page load without requiring hover interaction
- **Relationships**: Part of Hero Section component

### Hero Subtitle
- **Name**: Hero Subtitle
- **Type**: Text element
- **Fields**:
  - text: string (the subtitle text content)
  - className: string (CSS class name: hero__subtitle)
  - visibility: boolean (whether text is visible without hover)
- **Validation**: Must be visible on page load without requiring hover interaction
- **Relationships**: Part of Hero Section component

### Hero Section
- **Name**: Hero Section
- **Type**: UI Component
- **Fields**:
  - title: Hero Title (contained element)
  - subtitle: Hero Subtitle (contained element)
  - cssOverrides: CSS Rules (custom styles to override defaults)
- **Validation**: All contained text elements must be visible without hover
- **Relationships**: Root component for homepage hero area

## CSS Override Rules
- **Name**: CSS Override Rules
- **Type**: Styling Rules
- **Fields**:
  - targetSelector: string (CSS selector to target)
  - properties: object (CSS properties to override)
  - specificity: number (CSS specificity level)
  - themeCompatibility: boolean (works in both light/dark modes)
- **Validation**: Must override Docusaurus default styles with higher specificity
- **Relationships**: Applied to Hero Title and Hero Subtitle elements