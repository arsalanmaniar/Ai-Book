# Quickstart Guide: Docusaurus Robotics UI Redesign

## Development Setup

### Prerequisites
- Node.js (v16 or higher)
- npm or yarn package manager
- Git (optional, for version control)

### Initial Setup
1. Clone or navigate to your Docusaurus project directory
2. Install dependencies:
   ```bash
   npm install
   ```
3. Start the development server:
   ```bash
   npm start
   ```

## Key Components

### Custom Homepage
- Located at: `src/pages/index.tsx`
- Features hero section with title, subtitle, and call-to-action buttons
- Includes module cards for textbook navigation
- Implements dark futuristic theme with cyan/blue accents

### Custom Navigation
- Custom header: `src/theme/Navbar/index.tsx`
- Custom footer: `src/theme/footer/index.tsx`
- Both components feature robotics-themed styling with animated elements

### Module Cards Component
- Located at: `src/components/ModuleCards/index.tsx`
- Displays textbook modules with icons, descriptions, and navigation links
- Responsive grid layout with hover effects

## Styling

### Global Styles
- Custom CSS: `src/css/custom.css`
- Contains theme variables, typography, and global overrides
- Implements dark futuristic color scheme with improved contrast

### Component-Specific Styles
- CSS modules in respective component directories
- Scoped styles to prevent conflicts
- Responsive design for all screen sizes

## Animations

### CSS-Based Animations
- Floating effect for robotic icon
- Pulsing and rotation for accent elements
- Moving grid background in hero section
- Glowing text effects for headings

### Animation Configuration
- All animations are CSS-based for performance
- Hardware-accelerated properties for smooth performance
- Respect user's reduced motion preferences

## Testing

### Local Development
1. Run development server: `npm start`
2. Visit `http://localhost:3000` in your browser
3. Changes to components will hot-reload automatically

### Production Build
1. Build static files: `npm run build`
2. Test locally: `npm run serve`
3. Files will be generated in the `build` directory

## Deployment

### Static Hosting
The site generates static files in the `build` directory which can be deployed to:
- GitHub Pages
- Netlify
- Vercel
- Any static hosting service

### Configuration
- Update `docusaurus.config.js` with your deployment settings
- Set appropriate `baseUrl` and `url` values
- Configure any required environment variables

## Troubleshooting

### Common Issues
- If styles don't appear, clear browser cache and restart development server
- If animations are janky, check browser console for performance warnings
- If navigation doesn't work, verify all links in `docusaurus.config.js` are correct

### Accessibility
- All components meet WCAG 2.1 AA contrast requirements
- Keyboard navigation is fully supported
- Screen reader compatibility is maintained