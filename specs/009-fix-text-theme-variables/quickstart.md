# Quickstart: Fix Invisible Text via Theme Variables

## Overview
This guide explains how to fix invisible hero text on the Docusaurus homepage by defining missing Infima theme variables that control text visibility.

## Prerequisites
- Node.js >= 20.0
- Docusaurus 3.x project
- Access to project files

## Steps

### 1. Identify Missing Variables
- Missing --ifm-heading-color causes heading text to be transparent
- Missing --ifm-font-color-base causes base text to be transparent
- These variables control text color in both light and dark modes

### 2. Define Theme Variables
- Edit `src/css/custom.css`
- Add theme variable definitions in :root for light mode
- Add theme variable definitions in [data-theme='dark'] for dark mode
- Use WCAG AA compliant colors for proper contrast

### 3. Verify the Fix
- Run `npm run build` to ensure no build errors
- Run `npm run start` to test the fix in development mode
- Confirm hero text is visible without hover in both themes
- Test theme toggle functionality

## Files Modified
- `src/css/custom.css` - Added theme variable definitions
- `docusaurus.config.ts` - Verified CSS loading (no changes needed)

## Expected Results
- Hero title and subtitle visible immediately on page load
- No hover required to see text
- Proper contrast in both light and dark themes
- Theme toggle works correctly
- Existing animations and layout preserved