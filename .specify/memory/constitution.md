 <!--
Sync Impact Report:
Version change: 0.0.0 → 1.0.0
List of modified principles: All principles replaced.
Added sections: Project Overview, Governance Model, Contribution Guidelines, Decision Making, Release & Versioning Policy, Documentation Standards, Quality & Testing, Security & Vulnerability Disclosure, Licensing & IP, Onboarding & Mentorship, Code of Conduct, Archive & Sunset Policy, Appendices.
Removed sections: All original sections.
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Textbook Constitution

## 1. Project Overview
This project aims to create a comprehensive, open-source textbook on Physical AI and Humanoid Robotics. Its vision is to provide accessible, high-quality educational resources, fostering collaboration and knowledge sharing within the global AI and robotics community. The scope includes theoretical foundations, practical implementations, and ethical considerations.

## 2. Governance Model

### Roles
-   **Maintainers**: Oversee the project direction, merge PRs, manage releases, and resolve disputes. Assigned by existing Maintainers based on sustained significant contributions and commitment.
-   **Core Contributors**: Regularly contribute high-quality content, code, or reviews. Assigned by Maintainers.
-   **Reviewers**: Provide detailed feedback on PRs and proposals. Can be any active community member.
-   **Docs Editors**: Focus on linguistic quality, formatting, and consistency of written content. Assigned by Maintainers.

### Tenure and Removal
Roles are held as long as contributors remain active and adhere to the Code of Conduct. Inactivity or breaches of the Code of Conduct may lead to role reassignment or removal, decided by Maintainer consensus.

## 3. Contribution Guidelines

### PR Workflow
1.  Fork the repository and create a new branch.
2.  Implement changes or add content.
3.  Ensure all checks pass (lint, spellcheck, link check).
4.  Submit a Pull Request (PR) to the `main` branch.
5.  Address reviewer comments.

### Issue Templates
Contributors MUST use provided issue templates for bug reports, feature requests, and documentation improvements.

### Branch Naming
Branches MUST follow the convention: `type/descriptive-name` (e.g., `feat/new-chapter-ai-ethics`, `docs/fix-typo-ch3`).

### Commit Message Guidelines
Commit messages MUST follow Conventional Commits specification (e.g., `feat: add chapter on robot locomotion`).

### Testing Requirements
All content changes MUST pass automated quality checks (link checker, spellcheck) before merging. New code (e.g., RAG chatbot scripts) requires unit and integration tests.

### CI Checks
All PRs trigger automated CI checks for linting, spellchecking, link validation, and test suite execution.

### Required Reviewers per PR
-   **Content PRs**: At least one Docs Editor and one Core Contributor/Maintainer.
-   **Code/Infrastructure PRs**: At least two Core Contributors/Maintainers.

## 4. Decision Making

### Consensus
Decisions are primarily made through lazy consensus. If no objections are raised within 72 hours on a proposal, it is considered accepted.

### Majority Vote
For contentious issues where consensus cannot be reached, a majority vote among Maintainers will be taken. A 2/3rds majority is required for significant architectural or governance changes.

### Proposal Process
Major changes (e.g., new sections, significant refactoring, toolchain changes) MUST be proposed via an RFC (Request For Comments) or ADR (Architectural Decision Record) process. Links to templates are in the Appendices.

### Emergency Rollback Policy
In cases of critical bugs, security vulnerabilities, or severe regressions affecting the project's integrity, Maintainers have the authority to immediately roll back changes to a stable state. Post-mortem analysis and documentation are required.

## 5. Release & Versioning Policy

### Semantic Versioning
The book project follows Semantic Versioning (MAJOR.MINOR.PATCH).
-   **MAJOR**: Significant content overhaul, major architectural changes, or backward-incompatible API changes (for code components).
-   **MINOR**: New chapters, significant sections, new features (for code components).
-   **PATCH**: Typo fixes, broken link repairs, minor formatting, bug fixes (for code components).

### Release Cadence
Minor releases are planned quarterly. Patch releases are made as needed for critical fixes. Major releases are event-driven.

### Changelog Standards
All releases MUST include a detailed `CHANGELOG.md` generated from commit messages, adhering to the Keep a Changelog standard.

### Who Cuts Releases
Only Maintainers are authorized to cut official releases.

### Backport Rules
Critical bug fixes (PATCH) may be backported to the previous minor release branch if deemed necessary by Maintainers.

## 6. Documentation Standards

### Chapter Location
All book chapters reside in `docs/` within their respective section subdirectories (e.g., `docs/chapter1-introduction/`).

### Frontmatter Rules
Each Markdown file for a chapter MUST include Docusaurus-compatible YAML frontmatter with `id`, `title`, and `sidebar_position`.

### Spec-Kit-Plus Conventions
Adhere to Spec-Kit-Plus conventions for architectural documents (`spec.md`, `plan.md`, `tasks.md`, `adr/`). These reside in `specs/<feature-name>/` or `history/adr/`.

### Localization
The project supports localization, with an Urdu language toggle. New content MUST be written in English first.

### Translation Workflow
Translations are managed via a dedicated workflow, ensuring content parity and quality. Translators can submit PRs to the `i18n/<lang>/` directories.

## 7. Quality & Testing

### Minimal Tests for Book Content
-   **Link Checker**: All internal and external links MUST be valid. Automated checks are run on CI.
-   **Spellcheck**: Content MUST pass automated spellchecking.
-   **Broken Link Policy**: Broken links found in released versions are considered critical bugs and MUST be fixed immediately.

### RAG Chatbot Dataset Testing
Any datasets used for RAG chatbots or similar AI components MUST have associated validation tests for quality, relevance, and bias.

### Review Checklist
All PRs MUST adhere to the [example PR checklist](#example-pr-checklist) provided in the Appendices.

## 8. Security & Vulnerability Disclosure

### Reporting Contact
Security vulnerabilities MUST be reported privately to `security@physicalaibook.org`.

### Triage Timeline
Critical vulnerabilities will be triaged within 24 hours, high within 48 hours, medium within 72 hours.

### CVE/Patch Policy
Public disclosure, including CVE assignment (if applicable), will occur only after a patch is available and sufficient time for users to update has passed.

## 9. Licensing & IP

### Recommended License
The project uses the MIT License. All new contributions MUST be compatible with this license.

### Contributor License Expectations
Contributors implicitly grant the project a license to use their contributions under the MIT License upon submission of a PR.

### Attribution Rules for Third-Party Images/Data
All third-party images, data, or code snippets MUST be properly attributed according to their respective licenses. Public domain or similarly permissive licenses are preferred.

<h2>10. Onboarding & Mentorship</h2>
New contributors are encouraged to start with good first issues. Mentorship is available through the community Discord channel. New contributors must read the `CONTRIBUTING.md` and this Constitution.

<h2>11. Code of Conduct</h2>
This project adheres to the Contributor Covenant Code of Conduct. All participants are expected to uphold its principles. A full version is available at `CODE_OF_CONDUCT.md`.

<h2>12. Archive & Sunset Policy</h2>
The project may be archived if:
-   Maintainer activity ceases for 12 consecutive months.
-   A suitable successor organization or individual cannot be found to take over maintainership.
-   The content becomes significantly outdated with no plan for updates.

<h2>13. Appendices</h2>

### Example PR Checklist
-   [ ] Adheres to Contribution Guidelines (branch naming, commit message).
-   [ ] Passes all CI checks (lint, spellcheck, link check).
-   [ ] For content: Reviewed for clarity, accuracy, and grammar.
-   [ ] For code: Includes necessary tests and passes them.
-   [ ] Updated relevant documentation (if applicable).
-   [ ] No unresolved review comments.

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
