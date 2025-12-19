---
id: 1
title: rag-chatbot-specification
stage: spec
date: 2025-12-13
surface: agent
model: Claude Sonnet 4.5
feature: rag-chatbot
branch: 001-rag-chatbot
user: user
command: /sp.specify
labels: [rag,chatbot,cohere,specification,backend]
links:
  spec: backend/specs/001-rag-chatbot/spec.md
  ticket: null
  
  adr: null
  pr: null
files:
 - backend/specs/001-rag-chatbot/spec.md
 - backend/specs/001-rag-chatbot/checklists/requirements.md
 - backend/history/prompts/specs/1-rag-chatbot-specification.spec.prompt.md
tests:
 -
---

## Prompt

Cohere-based RAG Chatbot for a Published Book

Target audience:
Book readers needing accurate, source-grounded answers.

Goal:
Build an embedded RAG chatbot that:
- Answers strictly from book content
- Supports "selected-text only" Q&A
- Never hallucinates outside context

Tech stack (fixed):
- LLM: Cohere (NO OpenAI)
- Backend: FastAPI
- Vector DB: Qdrant Cloud (Free Tier)
- DB: Neon Serverless Postgres
- Agents: SpecKit+ via Qwen CLI

Environment (use .env only):
- COHERE_API_KEY=<ENV>
- QDRANT_URL=<ENV>
- QDRANT_API_KEY=<ENV>
- QDRANT_CLUSTER_ID=<ENV>
- NEON_DATABASE_URL=<ENV>

Core features:
- Book-wide RAG with chunking + metadata
- Selected-text Q&A with vector search disabled
- Context-limited, source-grounded responses only

Success criteria:
- Zero answers outside provided context
- Selected-text mode never leaks global data
- Backend isolated in /backend
- Clean, reproducible architecture

Constraints:
- No OpenAI usage
- No hardcoded secrets
- No internet or cross-book search

Not building:
- Auth, payments, analytics
- Fine-tuning
- Multi-book or web RAG

## Response snapshot

Created a comprehensive specification for the Cohere-based RAG Chatbot in the backend directory. The specification includes user scenarios, functional requirements, success criteria, and a validation checklist. All backend work is properly contained within the /backend directory as required.

## Outcome

- ✅ Impact: Created complete specification for RAG chatbot feature
- 🧪 Tests:
- 📁 Files: Created spec.md and requirements checklist in backend/specs/001-rag-chatbot/
- 🔁 Next prompts: Ready for planning phase
- 🧠 Reflection: Successfully followed the requirement to keep all backend work in /backend directory

## Evaluation notes (flywheel)

- Failure modes observed:
- Graders run and results (PASS/FAIL):
- Prompt variant (if applicable):
- Next experiment (smallest change to try):