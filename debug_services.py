import sys
import os
import logging
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Set up logging to see what's happening
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

print("1. Importing services...")

from services.embedding_service import EmbeddingService
print("2. Embedding service imported")

from services.answer_generation_service import AnswerGenerationService
print("3. Answer generation service imported")

from services.rag_service import RAGService
print("4. RAG service imported")

print("5. Creating embedding service...")
try:
    embedding_service = EmbeddingService(model_name='all-MiniLM-L6-v2')
    print("   Embedding service created successfully")
except Exception as e:
    print(f"   Error creating embedding service: {e}")

print("6. Creating answer service...")
try:
    answer_service = AnswerGenerationService()
    print("   Answer service created successfully")
except Exception as e:
    print(f"   Error creating answer service: {e}")

print("7. Initializing RAG service...")
try:
    rag_service = RAGService(
        embedding_service=embedding_service,
        retrieval_service=None,  # No Qdrant available
        answer_service=answer_service,
        docs_path='../docs'
    )
    print("   RAG service initialized successfully")
except Exception as e:
    print(f"   Error initializing RAG service: {e}")
    import traceback
    traceback.print_exc()

print("8. Loading documents...")
try:
    success = rag_service.load_and_index_book_content(force_reindex=False)
    print(f'   Documents loaded: {success}')
    print(f'   Document count: {rag_service.get_document_count()}')
except Exception as e:
    print(f"   Error loading documents: {e}")
    import traceback
    traceback.print_exc()