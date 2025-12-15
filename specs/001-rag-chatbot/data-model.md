# Data Model: Cohere-based RAG Chatbot Backend

## Entity: QuestionSession

**Description**: Represents a user's interaction session with the chatbot, including query history and selected text context

**Attributes**:
- session_id: UUID (primary key)
- created_at: DateTime
- updated_at: DateTime
- selected_text: String (nullable) - User-provided text for selected-text mode
- query_history: JSON - List of previous queries and responses
- metadata: JSON - Additional session metadata

**Validation Rules**:
- session_id must be unique
- created_at must be in the past
- selected_text length must be < 10000 characters when provided

## Entity: BookContentChunk

**Description**: Represents a segment of book content with metadata (book ID, chapter, page, section) and vector embeddings

**Attributes**:
- chunk_id: UUID (primary key)
- book_id: String - Identifier for the book
- content: String - The actual text content of the chunk
- chunk_metadata: JSON - Contains chapter, page, section, etc.
- embedding_vector: JSON - Vector representation for similarity search
- hash: String - Content hash for deduplication
- created_at: DateTime

**Validation Rules**:
- content must not be empty
- chunk_metadata must contain required fields (chapter, section)
- embedding_vector must be properly formatted
- hash must be unique per book

## Entity: Response

**Description**: Represents an AI-generated answer with source citations and confidence indicators

**Attributes**:
- response_id: UUID (primary key)
- session_id: UUID (foreign key to QuestionSession)
- query: String - The original user query
- answer: String - The generated answer
- source_references: JSON - List of chunk_ids used in the response
- confidence_score: Float - Confidence level of the response
- created_at: DateTime
- mode: String - "book-wide" or "selected-text"

**Validation Rules**:
- query must not be empty
- answer must not be empty
- confidence_score must be between 0 and 1
- mode must be one of the allowed values

## Entity: SourceReference

**Description**: Links responses to specific book content chunks with precise location information (represented as part of Response entity)

**Attributes**:
- chunk_id: UUID (foreign key to BookContentChunk)
- relevance_score: Float - How relevant this chunk was to the response
- text_preview: String - Short preview of the referenced text
- location_info: JSON - Contains chapter, page, section information

**Validation Rules**:
- chunk_id must reference an existing BookContentChunk
- relevance_score must be between 0 and 1

## Entity: BookMetadata

**Description**: Contains high-level information about books in the system

**Attributes**:
- book_id: String (primary key)
- title: String
- author: String
- total_chunks: Integer
- indexed_at: DateTime
- metadata: JSON - Additional book-specific metadata

**Validation Rules**:
- book_id must be unique
- title must not be empty
- total_chunks must be non-negative

## Relationships

- QuestionSession (1) ←→ (0..n) Response: One session can have multiple responses
- Response (0..n) ←→ (0..n) BookContentChunk: Responses reference multiple content chunks
- BookMetadata (1) ←→ (0..n) BookContentChunk: One book can have multiple content chunks

## State Transitions

### QuestionSession
- NEW → ACTIVE: When first query is received
- ACTIVE → INACTIVE: After period of inactivity or explicit session end

### Response
- PENDING → COMPLETE: When response generation is finished
- COMPLETE → CACHED: When response is stored for potential reuse