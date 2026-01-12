<!-- SYNC IMPACT REPORT
Version change: N/A → 1.0.0
Modified principles: N/A (new constitution)
Added sections: Core Principles (6), Technical Standards, Development Workflow, Governance
Removed sections: N/A
Templates requiring updates:
- .specify/templates/plan-template.md ✅ updated
- .specify/templates/spec-template.md ✅ updated
- .specify/templates/tasks-template.md ✅ updated
- .specify/templates/commands/*.md ✅ reviewed
- README.md ⚠ pending
Follow-up TODOs: None
-->
# AI-Native Textbook: Physical AI & Humanoid Robotics Constitution

## Core Principles

### Embodied Intelligence First
All AI concepts must be grounded in physical reality and real-world constraints. Abstract AI explanations without physical context are prohibited. Emphasize sensorimotor loops, physical grounding, and embodied cognition throughout the textbook.

### Accuracy & Verifiability
All factual claims must be verifiable via primary or peer-reviewed sources. No speculative claims without clear labeling as future research directions. Maintain academic rigor with university-level depth and precision.

### Academic Rigor
Maintain university-level depth and precision in all content. Prefer peer-reviewed papers, official documentation, and industry whitepapers. All content must be suitable for senior undergraduate and graduate students.

### Reproducibility
Every technical system described must be reproducible using open tools. Include configuration details, architecture diagrams (described textually), and references to enable readers to replicate systems independently.

### AI-Native Design
Content must be structured for machine readability and agent retrieval. Use clear sections, definitions, stepwise workflows, and semantic clarity. Avoid narrative fluff that degrades RAG performance.

### Simulation-First Philosophy
Prioritize Gazebo, Unity, and NVIDIA Isaac Sim for all robotics examples. Clearly explain sim-to-real transfer challenges and mitigation strategies in all relevant content.

## Technical Standards

- ROS 2 terminology must follow official ROS 2 definitions and documentation
- NVIDIA Isaac components must reflect current NVIDIA documentation
- Vision, SLAM, and navigation concepts must be mathematically and algorithmically correct
- LLM integration must clearly distinguish perception, planning, and execution layers
- All technical content must support both human learning and AI agent retrieval
- Content structure must enable effective RAG (Retrieval-Augmented Generation) functionality

## Development Workflow

- All content must follow APA citation style with minimum 15 sources total
- At least 50% of sources must be peer-reviewed research papers
- Remaining sources may include official ROS 2 documentation, NVIDIA Isaac documentation, and OpenAI technical papers
- Every non-trivial claim must have a citation
- Content must pass Flesch-Kincaid Grade 10-12 readability level
- Tone must be precise, instructional, and professional
- No marketing language or vague claims without evidence

## Governance

This constitution supersedes all other development practices and guidelines. All content must comply with the principles outlined above. Amendments to this constitution require explicit documentation, approval process, and migration plan. All PRs and reviews must verify compliance with these principles. Complexity must be justified with clear pedagogical value. Content must be structured for both human learning and AI agent retrieval, supporting the embedded RAG chatbot functionality.

**Version**: 1.0.0 | **Ratified**: 2026-01-02 | **Last Amended**: 2026-01-02