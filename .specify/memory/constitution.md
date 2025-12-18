# Integrated RAG Chatbot Constitution
<!-- Constitution for the Integrated RAG Chatbot for a Published Book (Cohere-based) -->

## Core Principles

### I. Source-Grounded Responses
<!-- The system must only respond based on retrieved content from the book -->
Responses must be strictly grounded in retrieved book content only; No hallucination of facts or external knowledge; All answers must cite source chunk IDs internally; System must fail safely when context is insufficient
<!-- Responses must be fully grounded in book content -->

### II. Context Control
<!-- Strict separation between book-wide and selected-text modes -->
Maintain strict separation between book-wide context and user-selected text context; Selected-text Q&A must ignore global vector search and use only provided text; Explicitly block external knowledge in selected-text mode
<!-- Clear distinction between different context modes -->

### III. LLM Independence from OpenAI
<!-- Use Cohere API exclusively as specified -->
Cohere API only for LLM functionality; Do NOT use OpenAI APIs or SDKs; Embeddings must align with Cohere models; Reranking must use Cohere-supported methods when available
<!-- Exclusive use of Cohere services for LLM functionality -->

### IV. Modular Architecture
<!-- Clean separation of concerns in the system design -->
Backend: FastAPI (async, clean routing); Vector Store: Qdrant Cloud (Free Tier); Metadata + sessions: Neon Serverless Postgres; Clean separation of frontend, backend, and data layers
<!-- Proper system architecture with clear component boundaries -->

### V. Agent-First Design
<!-- Implementation using SpecKit+ patterns and Qwen CLI -->
Define clear agent roles (retrieval, reasoning, response); Use SpecKit+ patterns for agent orchestration; Qwen CLI used for scaffolding, structure, and iteration; Agents must fail safely when context is insufficient
<!-- Implementation following agent-first architecture principles -->

### VI. Production-Grade Backend
<!-- Backend code must meet production standards -->
Backend code must be isolated in this project structure; FastAPI async patterns; Clean routing structure; Error handling and logging; Production-ready deployment configuration
<!-- Backend must be designed for production use -->

## Content Requirements

Each RAG pipeline must include: Chunking strategy with metadata tagging (chapter, page, section); Similarity search + reranking (if supported); Proper context window management; Source citation capabilities; Response quality validation
<!-- RAG pipeline must follow specified structural requirements -->

## Development Workflow

All backend code follows FastAPI best practices; No external knowledge in responses; Qwen CLI used for scaffolding and iteration; Agent orchestration using SpecKit+ patterns; System tested for context leakage prevention
<!-- Backend development must follow the defined workflow and quality standards -->

## Data Governance

Vector storage: Qdrant Cloud with proper metadata indexing; Database: Neon Serverless Postgres for sessions and metadata; Embeddings aligned with Cohere models; Proper data retention and privacy controls
<!-- Data handling must follow specified governance requirements -->

## Governance

Constitution supersedes all other practices; All implementations must comply with source-grounded responses, context control, LLM independence, modular architecture, agent-first design, and production-grade backend rules; System must be extensible for future chapters or books
<!-- The constitution governs all aspects of RAG chatbot development for this project -->

**Version**: 1.0.0 | **Ratified**: 2025-12-13 | **Last Amended**: 2025-12-13
<!-- Version tracking for the constitution -->