# Cohere-based RAG Chatbot for a Published Book

**Feature Branch**: `001-rag-chatbot`
**Created**: 2025-12-13
**Status**: Draft
**Input**: User description: "Cohere-based RAG Chatbot for a Published Book

Target audience:
Book readers needing accurate, source-grounded answers.

Goal:
Build an embedded RAG chatbot that:
- Answers strictly from book content
- Supports “selected-text only” Q&A
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
- Multi-book or web RAG"

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.

  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Basic Book-wide Q&A (Priority: P1)

Reader asks a question about the book content and receives an answer grounded in the book's text with proper citations.

**Why this priority**: This is the core functionality that provides the primary value of the RAG system - answering questions from book content.

**Independent Test**: Can be fully tested by providing book content, asking questions, and verifying responses are sourced from the book with proper citations.

**Acceptance Scenarios**:

1. **Given** book content is indexed in the system, **When** user asks a question about the book, **Then** system returns an answer grounded in the book content with source citations
2. **Given** user asks a question with no relevant book content, **When** system processes the query, **Then** system responds with appropriate message indicating insufficient context

---

### User Story 2 - Selected-text Q&A (Priority: P2)

Reader highlights specific text and asks questions only about that text, with the system ignoring global context.

**Why this priority**: Provides the specialized functionality for context-limited Q&A as specified in requirements.

**Independent Test**: Can be fully tested by providing user-selected text and verifying answers only come from that specific text.

**Acceptance Scenarios**:

1. **Given** user provides selected text, **When** user asks a question about that text, **Then** system returns answer based only on the provided text without accessing global knowledge

---

### User Story 3 - Context Validation (Priority: P3)

System ensures all responses are strictly grounded in provided context without hallucination.

**Why this priority**: Critical for maintaining trust and accuracy as specified in requirements.

**Independent Test**: Can be fully tested by attempting to generate responses and verifying they don't contain information outside the provided context.

**Acceptance Scenarios**:

1. **Given** user asks a question requiring external knowledge, **When** system processes the query, **Then** system responds with appropriate message indicating it can only answer from provided context

---

[Add more user stories as needed, each with an assigned priority]

### Edge Cases

- What happens when user provides very short or ambiguous selected text?
- How does system handle questions that require knowledge from multiple book sections?
- What occurs when the book content contains conflicting information?
- How does system respond when asked about content not present in the book?

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: System MUST process book content into searchable chunks with metadata (chapter, page, section)
- **FR-002**: System MUST use Cohere API exclusively for embeddings and LLM responses (no OpenAI)
- **FR-003**: Users MUST be able to ask questions about book content and receive source-grounded answers
- **FR-004**: System MUST support selected-text Q&A mode that disables global vector search
- **FR-005**: System MUST cite sources for all responses with specific book section references
- **FR-006**: System MUST validate that all responses are grounded in provided context without hallucination
- **FR-007**: System MUST store session data using Neon Serverless Postgres
- **FR-008**: System MUST use Qdrant Cloud for vector storage and similarity search
- **FR-009**: System MUST handle API keys securely through environment variables only
- **FR-010**: System MUST fail safely when insufficient context is available to answer a question

*Example of marking unclear requirements:*

- **FR-011**: System MUST handle book content up to [NEEDS CLARIFICATION: maximum book size not specified - how large are the expected books?]

### Key Entities *(include if feature involves data)*

- **QuestionSession**: Represents a user's interaction session with the chatbot, including query history and selected text context
- **BookContentChunk**: Represents a segment of book content with metadata (book ID, chapter, page, section) and vector embeddings
- **Response**: Represents an AI-generated answer with source citations and confidence indicators
- **SourceReference**: Links responses to specific book content chunks with precise location information

## Success Criteria *(mandatory)*

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

- **SC-001**: 100% of responses are grounded in provided book content without hallucination
- **SC-002**: Selected-text Q&A mode completely ignores global context and uses only provided text
- **SC-003**: Users can ask questions and receive relevant answers within 5 seconds
- **SC-004**: 95% of valid book content questions receive accurate, source-cited responses
- **SC-005**: System successfully prevents any cross-book or internet knowledge access