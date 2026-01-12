# Data Model: AI-Native Textbook for Physical AI & Humanoid Robotics

**Created**: 2026-01-02
**Feature**: 1-ai-textbook-physical-ai

## Entity: Chapter

### Fields
- **id** (String, required): Unique identifier for the chapter
- **name** (String, required): Title of the chapter
- **slug** (String, required): URL-friendly identifier
- **module** (String, required): Associated module (ROS 2, Gazebo, etc.)
- **content** (String, required): Markdown content of the chapter
- **metadata** (Object, optional): Additional information like word count, reading time
- **references** (Array[Reference], optional): Academic sources cited in the chapter
- **learning_objectives** (Array[String], optional): What students should learn from this chapter
- **prerequisites** (Array[String], optional): Knowledge required before reading this chapter
- **created_at** (Date, required): Timestamp of creation
- **updated_at** (Date, required): Timestamp of last update

### Validation Rules
- name must be between 5-100 characters
- slug must be unique and URL-friendly (alphanumeric + hyphens)
- content must be valid Markdown
- module must be one of: "physical-ai", "ros2", "gazebo-unity", "nvidia-isaac", "vla", "conversational", "capstone"
- learning_objectives must contain at least one objective if specified

## Entity: Reference

### Fields
- **id** (String, required): Unique identifier for the reference
- **type** (String, required): Type of source (journal, conference, documentation, etc.)
- **title** (String, required): Title of the source
- **authors** (Array[String], required): Authors of the source
- **year** (Number, required): Publication year
- **url** (String, optional): URL to the source
- **doi** (String, optional): Digital Object Identifier
- **abstract** (String, optional): Brief summary of the source
- **created_at** (Date, required): Timestamp of creation

### Validation Rules
- type must be one of: "journal", "conference", "book", "documentation", "preprint", "thesis"
- title must be between 5-500 characters
- authors array must contain at least one author
- year must be between 1950 and current year + 1
- if doi is provided, url must also be provided

## Entity: UserQuery

### Fields
- **id** (String, required): Unique identifier for the query
- **query_text** (String, required): The user's question
- **timestamp** (Date, required): When the query was made
- **user_id** (String, optional): Anonymous user identifier
- **context** (String, optional): Relevant context from the textbook (chapter slug or section)
- **response** (String, required): The chatbot's response
- **confidence** (Number, optional): Confidence score of the response (0-1)
- **feedback** (Object, optional): User feedback on the response

### Validation Rules
- query_text must be between 5-1000 characters
- response must be between 10-5000 characters
- confidence must be between 0 and 1 if specified
- timestamp must be current or past time

## Entity: Embedding

### Fields
- **id** (String, required): Unique identifier for the embedding
- **content_id** (String, required): Reference to the content being embedded
- **text** (String, required): The text that was embedded
- **embedding_vector** (Array[Number], required): The vector representation of the text
- **metadata** (Object, optional): Additional metadata about the embedding (source chapter, section, etc.)
- **created_at** (Date, required): When the embedding was created

### Validation Rules
- content_id must reference an existing content item
- text must be between 50-500 characters (chunk size)
- embedding_vector must have consistent dimensions (typically 1536 for OpenAI ada-002)
- metadata must be a valid JSON object if provided

## Entity: UserSession

### Fields
- **id** (String, required): Unique identifier for the session
- **user_id** (String, required): Anonymous user identifier
- **start_time** (Date, required): When the session started
- **end_time** (Date, optional): When the session ended
- **interactions** (Array[UserQuery], optional): Queries made during this session
- **active_chapter** (String, optional): Currently viewed chapter slug

### Validation Rules
- user_id must be a valid anonymous identifier format
- end_time must be after start_time if specified
- interactions array must not exceed 100 items