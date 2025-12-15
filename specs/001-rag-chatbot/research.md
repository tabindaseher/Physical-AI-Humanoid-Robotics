# Research: Cohere-based RAG Chatbot Backend

## Research Summary

This research addresses the implementation of a Cohere-based RAG chatbot that answers questions strictly from book content with support for selected-text Q&A mode. The system uses FastAPI backend with Qdrant Cloud for vector storage, Neon Serverless Postgres for session management, and Cohere API for embeddings and responses.

## Decision: Book Content Size Handling

**Rationale**: The functional requirement FR-011 had a NEEDS CLARIFICATION regarding maximum book size. Based on analysis of typical technical book sizes and Cohere's API limits, we'll implement chunking strategies that can handle books up to 1000 pages with approximately 500 words per page (about 500,000 words total).

**Alternatives considered**:
- Unlimited book size: Would require complex pagination and performance optimization
- Small book limit (100 pages): Would be too restrictive for comprehensive technical books
- 1000-page limit: Appropriate for most technical books while maintaining reasonable performance

## Decision: Chunk Size Strategy

**Rationale**: For optimal RAG performance with Cohere models, we'll use semantic chunking with 256-512 token chunks. This provides good context preservation while maintaining efficient vector search performance in Qdrant.

**Alternatives considered**:
- Fixed sentence chunks: May break semantic context
- Character-based chunks: May split concepts inappropriately
- Semantic chunks: Preserve meaning while optimizing for retrieval

## Decision: Similarity Threshold Configuration

**Rationale**: For ensuring high-quality responses, we'll set the similarity threshold to 0.7 for positive matches in Qdrant vector search. This ensures relevant content is retrieved while avoiding low-quality matches.

**Alternatives considered**:
- Lower threshold (0.5): May return irrelevant content
- Higher threshold (0.8): May fail to retrieve relevant content
- Adaptive threshold: Complex to implement initially

## Decision: Agent Orchestration Pattern

**Rationale**: Following the agent-first design principle, we'll implement a coordinator agent that manages the flow between retrieval, context validation, and answer generation agents. This provides clear separation of concerns while enabling complex interactions.

**Alternatives considered**:
- Direct service calls: Less modular, harder to maintain agent-specific logic
- Event-driven architecture: More complex for initial implementation
- Coordinator agent: Provides clear orchestration while maintaining agent independence

## Decision: Context Isolation Implementation

**Rationale**: To ensure selected-text QA works as required (ignoring global context), we'll implement separate processing paths with distinct context managers - one for book-wide RAG and another for selected-text processing.

**Alternatives considered**:
- Single service with mode flags: Risk of context leakage between modes
- Separate services: Clear isolation but potential code duplication
- Context managers with isolation: Clear separation with shared logic

## Technical Integration Research

### Cohere API Integration
- Cohere's embed-multilingual-v3.0 model supports up to 512 dimensions and handles various content types well
- Rerank capability available for improved result quality
- Rate limits appropriate for expected usage patterns

### Qdrant Cloud Integration
- Supports metadata filtering for chapter/section tracking
- Efficient similarity search with configurable thresholds
- Cloud cluster provides high availability

### Neon Serverless Postgres
- Serverless scaling appropriate for variable query loads
- Connection pooling handled automatically
- Compatible with async Python database libraries

## Phases Implementation Plan

### Phase 1: Foundation (API + DB + Embeddings)
- FastAPI application structure
- Qdrant vector store integration
- Neon Postgres session management
- Basic embedding pipeline

### Phase 2: Analysis (RAG testing)
- Book content chunking and indexing
- Vector similarity search validation
- Response quality testing

### Phase 3: Synthesis (selected-text QA)
- Selected-text Q&A implementation
- Context isolation validation
- End-to-end testing