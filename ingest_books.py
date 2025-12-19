import os
import re
from pathlib import Path
import logging
from typing import List, Dict, Any
from datetime import datetime
import hashlib

logger = logging.getLogger(__name__)

def extract_frontmatter(content: str) -> tuple:
    """Extract frontmatter from markdown content."""
    frontmatter_pattern = r'^---\n(.*?)\n---\n(.*)'
    match = re.match(frontmatter_pattern, content, re.DOTALL)
    
    if match:
        frontmatter_str = match.group(1)
        content_without_frontmatter = match.group(2)
        
        # Parse frontmatter (simple key-value pairs)
        frontmatter = {}
        for line in frontmatter_str.split('\n'):
            if ':' in line:
                key, value = line.split(':', 1)
                frontmatter[key.strip()] = value.strip().strip('"\'')
        
        return frontmatter, content_without_frontmatter
    else:
        return {}, content

def clean_markdown_content(content: str) -> str:
    """Remove markdown syntax and clean up content."""
    # Remove HTML-like tags
    content = re.sub(r'<[^>]+>', '', content)
    # Remove Docusaurus-specific imports
    content = re.sub(r'import.*?\n', '', content)
    # Remove image references
    content = re.sub(r'!\[.*?\]\(.*?\)', '', content)
    # Remove links with brackets
    content = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', content)
    # Remove bold/italic markers
    content = re.sub(r'[*_]{1,3}([^*_]+)[*_]{1,3}', r'\1', content)
    # Remove code blocks
    content = re.sub(r'```.*?```', '', content, flags=re.DOTALL)
    # Remove inline code
    content = re.sub(r'`(.*?)`', r'\1', content)
    
    # Normalize whitespace
    content = re.sub(r'\n\s*\n', '\n\n', content)
    content = content.strip()
    
    return content

def split_document(text: str, max_length: int = 1000) -> List[str]:
    """Split document into chunks of approximately max_length words."""
    sentences = re.split(r'[.!?]+\s+', text)
    chunks = []
    current_chunk = ""
    
    for sentence in sentences:
        sentence = sentence.strip()
        if not sentence:
            continue
            
        if len(current_chunk) + len(sentence) < max_length:
            current_chunk += " " + sentence
        else:
            if current_chunk.strip():
                chunks.append(current_chunk.strip())
            current_chunk = sentence
    
    if current_chunk.strip():
        chunks.append(current_chunk.strip())
    
    # Filter out chunks that are too short
    return [chunk for chunk in chunks if len(chunk) > 50]

def read_book_content(docs_path: str) -> List[Dict[str, Any]]:
    """Recursively read all markdown files in the docs directory."""
    documents = []
    
    docs_dir = Path(docs_path)
    
    for md_file in docs_dir.rglob("*.md"):
        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # Extract frontmatter and content
            frontmatter, raw_content = extract_frontmatter(content)
            
            # Clean the content
            cleaned_content = clean_markdown_content(raw_content)
            
            # Extract title from frontmatter if available, otherwise from filename
            title = frontmatter.get('title', md_file.stem)
            
            # Split the content into chunks
            chunks = split_document(cleaned_content)
            
            # Create document objects for each chunk
            for idx, chunk in enumerate(chunks):
                doc_id = hashlib.md5(f"{md_file.as_posix()}_{idx}".encode()).hexdigest()
                
                document = {
                    'doc_id': doc_id,
                    'title': title,
                    'source_file': md_file.as_posix(),
                    'chunk_index': idx,
                    'content': chunk,
                    'metadata': {
                        'title': title,
                        'source_file': md_file.as_posix(),
                        'chunk_index': idx,
                        'created_at': datetime.now().isoformat(),
                        'file_size': len(content),
                        'word_count': len(chunk.split()),
                        'frontmatter': frontmatter
                    }
                }
                
                documents.append(document)
        
        except Exception as e:
            logger.error(f"Error processing file {md_file}: {e}")
            continue
    
    logger.info(f"Loaded {len(documents)} document chunks from {docs_path}")
    return documents

def save_documents_to_json(documents: List[Dict[str, Any]], output_path: str):
    """Save documents to JSON file."""
    import json
    
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(documents, f, indent=2, ensure_ascii=False)
    
    logger.info(f"Saved {len(documents)} documents to {output_path}")

def load_documents_from_json(json_path: str) -> List[Dict[str, Any]]:
    """Load documents from JSON file."""
    import json
    
    with open(json_path, 'r', encoding='utf-8') as f:
        documents = json.load(f)
    
    logger.info(f"Loaded {len(documents)} documents from {json_path}")
    return documents

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    docs_path = "../docs"  # Relative to backend directory
    output_path = "data/book_documents.json"

    # Create data directory if it doesn't exist
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    # Load book content
    documents = read_book_content(docs_path)

    # Save to JSON file
    save_documents_to_json(documents, output_path)

    print(f"Processed {len(documents)} document chunks from the book.")

    # Print sample
    if documents:
        sample_doc = documents[0]
        print("\nSample document:")
        print(f"Title: {sample_doc['title']}")
        print(f"Source: {sample_doc['source_file']}")
        print(f"Content preview: {sample_doc['content'][:200]}...")