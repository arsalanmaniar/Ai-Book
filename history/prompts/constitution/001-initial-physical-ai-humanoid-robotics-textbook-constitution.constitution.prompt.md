---
id: 001
title: Initial Physical AI & Humanoid Robotics Textbook Constitution
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: user
command: /sp.constitution
labels: ["constitution", "initial", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md (modified)
tests:

---

## Prompt

```text
You are an expert open-source project maintainer. Create a complete `SP.CONSTITUTION.md` for a Spec-Kit Plus / Docusaurus book project called "Physical AI & Humanoid Robotics Textbook". The constitution must be in Markdown and include these sections with clear headings and actionable rules:

1. Project Overview — purpose, vision, and scope (short paragraph).
2. Governance Model — roles (Maintainers, Core Contributors, Reviewers, Docs Editors), how roles are assigned, tenure and removal rules.
3. Contribution Guidelines — PR workflow, issue templates, branch naming, commit message guidelines, testing requirements, CI checks, and required reviewers per PR.
4. Decision Making — consensus, majority vote, proposal process (use RFC/ADR links), emergency rollback policy.
5. Release & Versioning Policy — semantic versioning, release cadence, changelog standards, who cuts releases, and backport rules.
6. Documentation Standards — where chapters live in repo, frontmatter rules, spec-kit-plus conventions, localization (Urdu toggle), and translation workflow.
7. Quality & Testing — minimal tests required for book content (link checker, spellcheck, broken link policy), RAG chatbot dataset testing, and review checklist.
8. Security & Vulnerability Disclosure — reporting contact, triage timeline, and CVE/patch policy.
9. Licensing & IP — recommended license (MIT or Apache-2.0), contributor license expectations, attribution rules for third-party images/data.
10. Onboarding & Mentorship — steps for new contributors and required orientation checks.
11. Code of Conduct — short code-of-conduct summary and link to full CoC.
12. Archive & Sunset Policy — conditions for archiving the repo or handing off maintainership.
13. Appendices — templates: PR checklist, release checklist, RFC template.

Make the tone professional but concise. Add a short “example PR checklist” as a bullet list at the end. Output only the Markdown content (no extra commentary).
```

## Response snapshot

The `SP.CONSTITUTION.md` for the 'Physical AI & Humanoid Robotics Textbook' project has been successfully created and updated, fully addressing all specified sections and guidelines, including the example PR checklist.

## Outcome

- ✅ Impact: Constitution document for the project has been established, defining governance, contribution, and operational standards.
- 🧪 Tests: Not applicable for a constitution document.
- 📁 Files: One file created/modified: `.specify/memory/constitution.md`.
- 🔁 Next prompts: Monitor for user feedback or requests for further project setup.
- 🧠 Reflection: The constitution template was initially mistaken as empty, but was already populated. The task involved confirming its content against user requirements and then generating a PHR.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
