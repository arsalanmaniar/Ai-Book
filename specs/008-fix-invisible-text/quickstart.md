# Quickstart: Fix Invisible Homepage Text

## Overview
This guide explains how to fix invisible hero text on the Docusaurus homepage where title and subtitle are only visible on hover.

## Prerequisites
- Node.js >= 20.0
- Docusaurus 3.x project
- Access to project files

## Steps

### 1. Identify the Issue
- Visit the homepage and observe that hero title and subtitle are invisible until hovered
- This is caused by Docusaurus default styles using `background-clip: text` and `-webkit-text-fill-color: transparent`

### 2. Apply CSS Fixes
- Edit `src/css/custom.css`
- Add CSS overrides for `.hero__title` and `.hero__subtitle` classes
- Use high specificity with `!important` to override Docusaurus defaults

### 3. Verify the Fix
- Run `npm run build` to ensure no build errors
- Run `npm run start` to test the fix in development mode
- Confirm hero text is visible without hover in both light and dark themes

## Files Modified
- `src/css/custom.css` - Added CSS overrides to fix visibility
- `docusaurus.config.ts` - Verified CSS loading (no changes needed)

## Expected Results
- Hero title and subtitle visible immediately on page load
- No hover required to see text
- Existing animations and layout preserved
- Works in both light and dark themes