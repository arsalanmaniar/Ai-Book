---
id: 1
title: Initialize Physical AI Robotics Textbook Constitution
stage: constitution
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20251101
feature: none
branch: main
user: user
command: /sp.constitution Create a full project constitution with roles, rules, docs, versioning and PR checklist.
labels: ["constitution", "initialization", "governance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
tests:

---

## Prompt

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

## Response snapshot

The project constitution for "Physical AI & Humanoid Robotics Textbook" has been successfully created and written to `.specify/memory/constitution.md`. It includes the specified sections: Project Overview, Governance Model, Contribution Guidelines, Decision Making, Release & Versioning Policy, Documentation Standards, Quality & Testing, Security & Vulnerability Disclosure, Licensing & IP, Onboarding & Mentorship, Code of Conduct, Archive & Sunset Policy, and Appendices with an example PR checklist. The constitution is version 1.0.0, ratified and last amended on 2025-12-05. A sync impact report indicates dependent templates require updates.

## Outcome

- ✅ Impact: Constitution created and documented.
- 🧪 Tests: N/A
- 📁 Files: .specify/memory/constitution.md updated.
- 🔁 Next prompts: Review constitution, update dependent templates.
- 🧠 Reflection: Successfully created initial constitution based on detailed user prompt. Need to update dependent templates.

## Evaluation notes (flywheel)

- Failure modes observed: PHR creation script failed, fallback to agent-native tool used.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Improve PHR script robustness; implement auto-detection for 'feature' and 'branch' fields for PHR.
