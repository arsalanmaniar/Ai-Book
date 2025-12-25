# Research: Fix Invisible Homepage Text

## Decision: Identified CSS properties causing invisible text
**Rationale**: The issue was caused by Docusaurus default styles using `background-clip: text` and `-webkit-text-fill-color: transparent` which make text invisible until hovered. These properties were found in both the navbar styles and potentially in hero section styles.

## Decision: CSS override approach with high specificity
**Rationale**: Using `!important` declarations to override Docusaurus default styles ensures the fix works reliably. This approach directly addresses the opacity and visibility issues without modifying core Docusaurus files, maintaining compatibility.

## Decision: Target specific CSS classes for hero text
**Rationale**: Targeting `.hero__title` and `.hero__subtitle` classes specifically ensures we only affect the homepage hero section text, avoiding unintended side effects on other parts of the site.

## Decision: Preserve existing animations and layout
**Rationale**: The fix maintains all existing animations, layout, and design elements while only addressing the visibility issue. This ensures no visual regressions occur.

## Decision: Cross-theme compatibility
**Rationale**: Implementing separate rules for light and dark modes ensures the fix works consistently across both themes, maintaining accessibility and visual consistency.