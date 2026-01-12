---
id: 1
title: Create Physical AI Textbook Constitution
stage: constitution
date: 2026-01-02
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: human
command: /sp.constitution
labels: [constitution, textbook, physical-ai, robotics]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

/sp.constitution

Project Title:
AI-Native Textbook: Physical AI & Humanoid Robotics

Project Description:
This project is an AI-native, interactive technical textbook designed to teach a full university-level course on Physical AI & Humanoid Robotics. The book bridges artificial intelligence, robotics middleware, simulation, embodied cognition, and vision-language-action systems. It is written for advanced students and professionals with a computer science or engineering background and is deployed using Docusaurus with integrated AI agents for retrieval-augmented learning.

The textbook must be suitable for AI-assisted reading, querying, and tutoring via an embedded RAG chatbot powered by OpenAI Agents SDK, FastAPI, Neon Serverless Postgres, and Qdrant Cloud.

Audience:
- Senior undergraduate and graduate students
- AI engineers and robotics engineers
- Researchers transitioning from digital AI to embodied intelligence
- Startup founders building Physical AI systems

Pedagogical Goals:
- Transition learners from digital-only AI to embodied Physical AI
- Teach humanoid robotics using simulation-first, sim-to-real methodology
- Integrate LLM-based cognitive planning with robotic control systems
- Prepare students for real-world humanoid robotics development and research

Core Principles:
1. **Embodied Intelligence First**
   - Emphasize physical grounding, sensorimotor loops, and real-world constraints
   - Avoid purely abstract AI explanations without physical context

2. **Accuracy & Verifiability**
   - All factual claims must be verifiable via primary or peer-reviewed sources
   - No speculative claims without clear labeling as future research directions

3. **Academic Rigor**
   - Maintain university-level depth and precision
   - Prefer peer-reviewed papers, official documentation, and industry whitepapers

4. **Reproducibility**
   - Every technical system described must be reproducible using open tools
   - Include configuration details, architecture diagrams (described textually), and references

5. **AI-Native Design**
   - Content must be structured for machine readability and agent retrieval
   - Use clear sections, definitions, stepwise workflows, and semantic clarity
   - Avoid narrative fluff that degrades RAG performance

6. **Simulation-First Philosophy**
   - Prioritize Gazebo, Unity, and NVIDIA Isaac Sim
   - Clearly explain sim-to-real transfer challenges and mitigation strategies

Book Structure Requirements:
- Modular chapter design aligned with course modules:
  - ROS 2 (Robotic Nervous System)
  - Gazebo & Unity (Digital Twin)
  - NVIDIA Isaac (AI-Robot Brain)
  - Vision-Language-Action (Cognitive Robotics)
- Each chapter must include:
  - Conceptual theory
  - System architecture explanation
  - Practical workflows
  - Failure modes & debugging insights
  - References section

Technical Standards:
- ROS 2 terminology must follow official ROS 2 definitions
- NVIDIA Isaac components must reflect current NVIDIA documentation
- Vision, SLAM, and navigation concepts must be mathematically and algorithmically correct
- LLM integration must clearly distinguish perception, planning, and execution layers

Citation & Source Policy:
- Citation style: APA
- Minimum 15 sources total
- At least 50% must be peer-reviewed research papers
- Remaining sources may include:
  - Official ROS 2 documentation
  - NVIDIA Isaac documentation
  - OpenAI technical papers and SDK docs
- Every non-trivial claim must have a citation

Writing Quality Constraints:
- Clarity level: Flesch-Kincaid Grade 10–12
- Tone: Precise, instructional, professional
- No marketing language
- No vague claims such as "powerful", "revolutionary", or "state-of-the-art" without evidence

Ethics & Safety:
- Clearly discuss safety constraints in humanoid robotics
- Address ethical implications of autonomous physical agents
- Avoid anthropomorphic exaggeration

Plagiarism Policy:
- Zero tolerance for plagiarism
- All paraphrasing must be original and properly cited
- Content must pass automated plagiarism detection

Output Requirements:

## Response snapshot

Created the project constitution file for the AI-Native Textbook: Physical AI & Humanoid Robotics project. The constitution includes six core principles (Embodied Intelligence First, Accuracy & Verifiability, Academic Rigor, Reproducibility, AI-Native Design, and Simulation-First Philosophy), technical standards, development workflow requirements, and governance structure. The file was written to .specify/memory/constitution.md with version 1.0.0.

## Outcome

- ✅ Impact: Created initial project constitution with 6 core principles and governance structure
- 🧪 Tests: none
- 📁 Files: Created .specify/memory/constitution.md
- 🔁 Next prompts: Develop spec for textbook chapters
- 🧠 Reflection: Constitution successfully captures all specified principles and requirements

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A