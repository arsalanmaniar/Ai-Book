# Data Model: Docusaurus Robotics UI Redesign

## Component Data Structures

### Homepage Components

#### HeroSection
```typescript
interface HeroSection {
  title: string;           // Site title from config
  subtitle: string;        // Site tagline from config
  primaryCta: {
    text: string;          // "Start Learning" button text
    link: string;          // "/docs/intro" navigation path
  };
  secondaryCta: {
    text: string;          // "Explore VLA Systems" button text
    link: string;          // "/docs/chapter-7-vla-integration/introduction" navigation path
  };
}
```

#### ModuleCard
```typescript
interface ModuleCard {
  id: string;              // Unique identifier for the module
  title: string;           // Module title (e.g., "Module 1: Introduction to Physical AI & Robotics")
  description: string;     // Module description text
  icon: string;            // Emoji icon for the module (e.g., "🤖", "🎮", "🔬", "🧠")
  to: string;              // Navigation path to module content
  color: string;           // Accent color in hex format (e.g., "#00aaff", "#00e6e6")
}
```

#### BlogHighlight
```typescript
interface BlogHighlight {
  id: string;              // Unique identifier for the blog post
  title: string;           // Blog post title
  excerpt: string;         // Short excerpt or summary
  date: string;            // Publication date (ISO format)
  author: string;          // Author name
  tags: string[];          // Array of tags associated with the post
  link: string;            // Navigation path to full blog post
}
```

### Navigation Components

#### NavbarItem
```typescript
interface NavbarItem {
  label: string;           // Display text for the navigation item
  to: string;              // Navigation path
  target?: string;         // Optional target for external links
  position: 'left' | 'right'; // Position in the navigation bar
}
```

#### FooterSection
```typescript
interface FooterSection {
  title: string;           // Section title
  items: Array<{
    label: string;         // Link text
    to: string;            // Navigation path
    target?: string;       // Optional target for external links
  }>;
}
```

## Configuration Objects

### SiteConfiguration
```typescript
interface SiteConfiguration {
  title: string;           // Site title (e.g., "Physical AI & Robotics")
  tagline: string;         // Site tagline/description
  url: string;             // Site URL
  baseUrl: string;         // Base URL for the site
  favicon: string;         // Path to favicon
  organizationName: string; // GitHub organization/username
  projectName: string;     // GitHub project name
  themeConfig: ThemeConfiguration;
}
```

### ThemeConfiguration
```typescript
interface ThemeConfiguration {
  navbar: {
    title: string;         // Navbar title
    logo: {
      alt: string;         // Alt text for logo
      src: string;         // Path to logo image
    };
    items: NavbarItem[];   // Navigation items array
  };
  footer: {
    style: 'dark' | 'primary'; // Footer style
    links: FooterSection[]; // Footer links sections
    copyright: string;     // Copyright text
  };
  colorMode: {
    defaultMode: string;   // Default color mode ('light' or 'dark')
    disableSwitch: boolean; // Whether to disable color mode switching
    respectPrefersColorScheme: boolean; // Respect system preference
  };
}
```

## Styling Data

### ColorPalette
```typescript
interface ColorPalette {
  primary: {
    base: string;          // #00aaff - Main primary color
    dark: string;          // #0099e6 - Darker primary
    darker: string;        // #0088cc - Even darker primary
    darkest: string;       // #0077b3 - Darkest primary
    light: string;         // #1ab2ff - Lighter primary
    lighter: string;       // #33bbff - Even lighter primary
    lightest: string;      // #4dc4ff - Lightest primary
  };
  secondary: string;       // #00e6e6 - Cyan accent color
  tertiary: string;        // #00aaff - Blue accent color
  background: {
    surface: string;       // #0a0a0a - Surface background
    default: string;       // #0a0a0a - Default background
  };
  emphasis: {
    100: string;           // Lightest emphasis
    200: string;           // Light emphasis
    300: string;           // Medium-light emphasis
    400: string;           // Medium emphasis
    500: string;           // Medium-dark emphasis
    600: string;           // Dark emphasis
    700: string;           // Darker emphasis
    800: string;           // Very dark emphasis
    900: string;           // Darkest emphasis
  };
}
```

## Animation Properties

### AnimationConfig
```typescript
interface AnimationConfig {
  float: {
    duration: string;      // "6s" - Duration for floating animation
    timing: string;        // "ease-in-out" - Timing function
    iteration: string;     // "infinite" - Number of iterations
  };
  pulse: {
    duration: string;      // "3s" - Duration for pulsing animation
    timing: string;        // "ease-in-out" - Timing function
    iteration: string;     // "infinite" - Number of iterations
  };
  rotate: {
    duration: string;      // "10s" - Duration for rotation animation
    timing: string;        // "linear" - Timing function
    iteration: string;     // "infinite" - Number of iterations
  };
  gridMove: {
    duration: string;      // "20s" - Duration for grid movement
    timing: string;        // "linear" - Timing function
    iteration: string;     // "infinite" - Number of iterations
  };
  glowLine: {
    duration: string;      // "2s" - Duration for glow animation
    timing: string;        // "ease-in-out" - Timing function
    iteration: string;     // "infinite alternate" - Number of iterations
  };
}
```

## Responsive Design Breakpoints

### Breakpoints
```typescript
interface Breakpoints {
  mobile: string;          // "max-width: 768px" - Mobile devices
  tablet: string;          // "max-width: 996px" - Tablet devices
  desktop: string;         // Min-width 997px - Desktop devices
}
```