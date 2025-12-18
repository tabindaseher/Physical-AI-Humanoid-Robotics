# Implementation Plan: Cohere-based RAG Chatbot Backend

**Branch**: `001-rag-chatbot` | **Date**: 2025-12-13 | **Spec**: backend/specs/001-rag-chatbot/spec.md
**Input**: Feature specification from `backend/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a Cohere-based RAG chatbot that answers questions strictly from book content with support for selected-text Q&A mode. The system uses FastAPI backend with Qdrant Cloud for vector storage, Neon Serverless Postgres for session management, and Cohere API for embeddings and responses. The architecture ensures strict context control with no hallucination and complete isolation between book-wide and selected-text modes.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, Cohere, Qdrant Client, SQLModel, Neon Postgres
**Storage**: Qdrant Cloud (vector store), Neon Serverless Postgres (session/metadata)
**Testing**: pytest
**Target Platform**: Linux server (cloud deployment)
**Project Type**: web/backend - FastAPI service with agent orchestration
**Performance Goals**: <5 second response time for queries, handle concurrent users
**Constraints**: No OpenAI usage, responses strictly from provided context, selected-text mode must ignore global context
**Scale/Scope**: Single book RAG system with extensibility for multiple books

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Source-Grounded Responses**: All responses must be grounded in retrieved book content only; no hallucination allowed; must cite source chunk IDs internally; must fail safely when context is insufficient.

**Context Control**: Must maintain strict separation between book-wide and user-selected text contexts; selected-text Q&A must ignore global vector search and use only provided text; must explicitly block external knowledge in selected-text mode.

**LLM Independence from OpenAI**: Must use Cohere API exclusively; no OpenAI APIs or SDKs; embeddings must align with Cohere models; reranking must use Cohere-supported methods.

**Modular Architecture**: Must use FastAPI backend with clean routing; Qdrant Cloud for vector store; Neon Postgres for metadata/sessions; clean separation of frontend, backend, and data layers.

**Agent-First Design**: Must define clear agent roles (retrieval, reasoning, response); use SpecKit+ patterns for orchestration; use Qwen CLI for scaffolding; agents must fail safely when context is insufficient.

**Production-Grade Backend**: Backend code must be isolated in project structure; use FastAPI async patterns; implement clean routing structure; include error handling and logging; use production-ready deployment configuration.

## Project Structure

### Documentation (this feature)

```text
backend/specs/001-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (backend directory)

```text
backend/
├── api/                 # FastAPI routes and endpoints
│   ├── rag.py           # RAG endpoints (book-wide and selected-text Q&A)
│   └── health.py        # Health check endpoints
├── models/              # Data models and schemas
│   ├── request.py       # Request schemas
│   ├── response.py      # Response schemas
│   └── entities.py      # Core data entities
├── services/            # Business logic and service layers
│   ├── retrieval_agent.py    # Retrieval agent for content search
│   ├── context_validator.py  # Context validation agent
│   ├── answer_generator.py   # Answer generation agent
│   ├── embedding_service.py  # Embedding processing service
│   └── storage_service.py    # Storage interaction service
├── config/              # Configuration files
│   ├── settings.py      # Settings with .env loading
│   └── database.py      # Database configuration
├── utils/               # Utility functions
│   ├── chunking.py      # Content chunking utilities
│   └── validation.py    # Validation utilities
├── main.py              # FastAPI application entry point
├── requirements.txt     # Python dependencies
└── README.md            # Backend documentation
```

**Structure Decision**: Web backend structure selected with FastAPI for the service, organized in modules for API, models, services, config, and utils. Agent orchestration implemented through dedicated service files following agent-first design principles.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
