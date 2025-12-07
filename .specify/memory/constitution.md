<!-- SYNC IMPACT REPORT
Version change: N/A -> 1.0.0 (initial versioning)
List of modified principles: N/A (no changes to principles, only added governance)
Added sections: Version, Ratification Date, Amendment Procedure
Removed sections: None
Templates requiring updates: ✅ updated / ⚠ pending
Follow-up TODOs: None
-->
# Project Constitution: Physical AI & Humanoid Robotics Textbook

**Version**: 1.0.0
**Ratification Date**: 2025-12-06
**Last Amended**: 2025-12-06

## Project Overview

This constitution governs the development of an AI-native, interactive textbook for teaching **Physical AI & Humanoid Robotics**. The project is part of the Hackathon I challenge and must meet all technical, pedagogical, and functional requirements as outlined in the hackathon brief.

---

## Core Principles

1.  **AI-Native Pedagogy**: Content must be designed for AI-augmented learning, with structure optimized for retrieval-augmented generation (RAG) and interactive AI tutoring.
2.  **Technical Accuracy**: All technical content related to robotics, AI, hardware, and software must be accurate, up-to-date, and verifiable.
3.  **Accessibility & Inclusivity**: Content should be accessible to learners with varying hardware/software backgrounds, with options for personalization and Urdu translation.
4.  **Open & Reproducible**: All code, configurations, and content must be open-source, reproducible, and deployable using specified tools (Docusaurus, GitHub Pages, Claude Code, Spec-Kit Plus).
5.  **Interactive Learning**: The textbook must support embedded RAG chatbots, personalized content, and multi-modal interaction (text, code, simulation, voice).

---

## Agent Skills & Context Integration

### This project utilizes Context7 to provide the agent with comprehensive, up-to-date documentation and skill acquisition. The agent is expected to learn and apply knowledge from the following Context7 resources to successfully execute this project:

1. Core Development & Documentation Framework: `https://context7.com/websites/docusaurus_io` - Learn to structure, write, and build a modern documentation website.

2. AI Agent Development: OpenAI Agents SDK (Python) : `https://context7.com/websites/openai_github_io_openai-agents-python` - Acquire skills for building stateful, agentic AI applications with 4. handoffs, guardrails, and session management.

3. Vector Database & Search: Qdrant Documentation: `http://context7.com/websites/qdrant_tech` - Understand cloud vector database setup, management, and integration for RAG systems.

4. Authentication & User Management: Better Auth Documentation: `https://context7.com/websites/better-auth` - Learn to implement secure user authentication, sessions, and background profiling.

5. Backend API Development: FastAPI Documentation: `https://context7.com/websites/fastapi_tiangolo` - Master building the backend API server to connect the frontend, database, and AI agents.

6. Serverless PostgreSQL: Neon Documentation: `https://context7.com/websites/neon` - Gain skills in using serverless Postgres for data persistence in a scalable application.

7. Robotics Simulation: NVIDIA Isaac Sim Documentation: `https://context7.com/isaac-sim/isaacsim` - Learn the fundamentals of robotics simulation and digital twin creation.

8. Robotics Middleware: ROS 2 Documentation: `https://context7.com/websites/docs_ros_org-en-humble-index.html` - Understand the core concepts of the Robot Operating System for building robotic nervous systems.

9. Project Specification & Management: Spec-Kit Plus: `https://context7.com/panaversity/spec-kit-plus` - Learn the AI-native project specification process, including constitution and technical spec phases.

### Note: The agent will use the knowledge from these resources to make informed decisions, write accurate code, and follow best practices throughout the project lifecycle.

---

## Quality Standards

### 1. Content Standards

- All technical explanations must be clear, concise, and structured for a **college-level engineering audience**.
- Code examples must be executable, documented, and aligned with ROS 2 Humble/Iron, NVIDIA Isaac Sim, and Gazebo.
- Diagrams, tables, and simulations must be referenced and explained.
- Each chapter must include:
  - Learning objectives
  - Prerequisites
  - Hands-on exercises
  - Review questions
  - Further reading
- Pedagogical approach must bridge the gap between digital AI and physical embodiment, teaching students to apply AI to control robots in simulated and real environments[citation:2].

### 2. Source & Citation Standards

- All external references (research papers, documentation, tutorials) must be cited using **APA 7th edition**.
- At least **60% of references must be from peer-reviewed or official documentation sources** (ROS, NVIDIA, IEEE, etc.).
- All claims regarding performance, hardware specs, or software capabilities must be traceable to a primary source.
- When citing industry standards, protocols, or technical documentation (e.g., IEEE, ISO, ROS specifications), follow the relevant style guide (APA, IEEE, etc.) as outlined by authoritative sources[citation:6].
- Modern, authoritative textbooks like "AI for Robotics: Toward Embodied and General Intelligence in the Physical World" (2025)[citation:8] should be referenced to frame robotics as an AI problem and cover cutting-edge techniques.

### 3. Technical Implementation Standards

- The book must be built using **Context7** for up-to-date documentation and use **Docusaurus** to build documentation website and deployed to **GitHub Pages** or **Vercel**.
- The RAG chatbot must:
  - Use **OpenAI API** with **FastAPI** backend and **Qdrant Cloud Free Tier** for vector storage (as specified in User Story 2).
  - Parse and embed all textbook content from the Docusaurus docs folder.
  - Support both general queries and user-selected text as context.
  - Be integrated as a floating chat widget in the Docusaurus frontend.
  - Strictly answer questions related only to the book's content.
- User authentication must be implemented via **Better Auth** (`https://www.better-auth.com/`), with background profiling for personalization.
- Personalization and Urdu translation features must be functional via chapter-level buttons.

### 4. Writing & Clarity Standards

- Flesch-Kincaid Grade Level: **11–13**
- Active voice preferred (>70% of sentences)
- Jargon must be defined on first use
- Each module must map to the **weekly breakdown** provided in the course outline

### 5. AI & Automation Standards

- Reusable intelligence via **Claude Code Subagents and Agent Skills** must be demonstrated and documented.
- All AI-generated content must be reviewed for accuracy and coherence.
- The RAG system must be tested for hallucination and citation accuracy.

---

## Constraints

### 1. Project Scope

- Must cover all 4 modules:
  - Robotic Nervous System (ROS 2)
  - Digital Twin (Gazebo & Unity)
  - AI-Robot Brain (NVIDIA Isaac)
  - Vision-Language-Action (VLA)
- Must include hardware requirements and lab setup guidance (Digital Twin Workstation, Physical AI Edge Kit, Robot Lab options).
- Must align with the provided **weekly breakdown** and **learning outcomes**.

### 2. Technical Stack

- **Required Tools**: Claude Code, Spec-Kit Plus (`https://github.com/panaversity/spec-kit-plus`), Docusaurus, GitHub, FastAPI, Neon, Qdrant, Better Auth.
- **Languages**: Python, Markdown, YAML, JavaScript/TypeScript (as needed).
- **Platforms**: Ubuntu 22.04 LTS (primary development environment).

### 3. Repository & Documentation

- Public GitHub repo required
- README must include:
  - Setup instructions
  - Deployment steps
  - Team/contributor info
- All code must be commented and modular.

---

## Success Criteria

### 1. Functional Requirements

- [ ] Textbook deployed and publicly accessible
- [ ] RAG chatbot fully integrated and responsive
- [ ] User signup/signin working with background profiling
- [ ] Personalization and Urdu translation buttons functional
- [ ] All chapters render correctly with interactive elements

### 2. Content Requirements

- [ ] All 4 modules covered in depth
- [ ] Code examples run without error in specified environments
- [ ] Citations are complete and correctly formatted
- [ ] Images/diagrams are clear and labeled

### 3. Hackathon Scoring

- Base functionality (100 points)
- Bonus: Reusable AI intelligence via Claude Code (up to 50 points)
- Bonus: Better Auth integration (up to 50 points)
- Bonus: Personalization feature (up to 50 points)
- Bonus: Urdu translation (up to 50 points)

### 4. Pedagogical Value

- The textbook should be usable in a real **Physical AI & Humanoid Robotics** course.
- Content should be modular, scalable, and adaptable for different learner levels.
- Interactive components should enhance comprehension and engagement.

---

## Revision & Governance

- This constitution is versioned starting at 1.0.0 as of 2025-12-06.
- Amendments require a PR review and approval if scope changes mid-project.
- Versioning follows semantic versioning:
  - MAJOR: Backward incompatible governance/principle removals or redefinitions
  - MINOR: New principle/section added or materially expanded guidance
  - PATCH: Clarifications, wording, typo fixes, non-semantic refinements
- All team members and AI agents must adhere to these standards throughout the project lifecycle.

---

## References & Key Resources

- **Spec-Kit Plus**: `https://github.com/panaversity/spec-kit-plus`
- **Context7**: `https://context7.com/`
- **Docusaurus**: `https://context7.com/websites/docusaurus_io`
- **Claude Code**: `https://www.claude.com/product/claude-code`
- **AI-Native Book Example**: `https://ai-native.panaversity.org/`
- **OpenAI Agents SDK (Python)**: `https://openai.github.io/openai-agents-python/` [citation:1][citation:7]
- **OpenAI Agents SDK (TypeScript)**: `https://openai.github.io/openai-agents-js/` [citation:9]
- **Better Auth**: `https://www.better-auth.com/`
- **Course Inspiration & Structure**: Physical AI & Humanoid Robotics Open Source Textbook (`https://danielhashmi.github.io/physical-ai-and-humanoid-robotics/`) [citation:2]
- **Modern Reference Text**: "AI for Robotics: Toward Embodied and General Intelligence in the Physical World" (Apress, 2025) [citation:8][citation:10]
- **Constitution Phase Guide**: `https://ai-native.panaversity.org/docs/SDD-RI-Fundamentals/spec-kit-plus-hands-on/constitution-phase` [citation:3]

---