# Physical AI & Humanoid Robotics - RAG-Powered Book Platform

[![License: MIT](https://img.shields.io/github/license/tabindaseher/Physical-AI-Humanoid-Robotics)](https://github.com/tabindaseher/Physical-AI-Humanoid-Robotics/blob/main/LICENSE)
[![Made with Python](https://img.shields.io/badge/Made%20with-Python-1f425f.svg)](https://www.python.org/)
[![Made with Node.js](https://img.shields.io/badge/Made%20with-Node.js-339933.svg)](https://nodejs.org/)
[![Docusaurus](https://img.shields.io/badge/Docusaurus-2.0-39e09b)](https://docusaurus.io/)

A comprehensive guide to Physical AI and Humanoid Robotics with an integrated RAG (Retrieval-Augmented Generation) chatbot that can answer questions about the book content in real-time.

## 🚀 Demo

<div align="center">
  <img src="https://github.com/tabindaseher/Physical-AI-Humanoid-Robotics/assets/demo-chatbot.gif" alt="RAG Chatbot Demo" width="800" />
</div>

*Interactive chatbot interface embedded in the documentation platform*

## ✨ Features

- **📚 Complete Book Content**: 8 chapters covering Physical AI & Humanoid Robotics
- **🤖 RAG-Powered Chatbot**: Real-time Q&A with context-aware responses
- **🔍 Source Attribution**: Citations showing exactly which book sections informed answers
- **⚡ Fast Semantic Search**: Vector database powered by Qdrant for intelligent retrieval
- **📱 Responsive Interface**: Works on desktop and mobile devices
- **🔧 Full API Access**: RESTful backend with comprehensive endpoints
- **🎯 Context-Aware Responses**: Answers based on entire book content, not just isolated sections

## 🛠️ Tech Stack

### Frontend
- **[Docusaurus](https://docusaurus.io/)** - Static site generator for documentation
- **[React](https://reactjs.org/)** - Component-based UI development
- **[Tailwind CSS](https://tailwindcss.com/)** - Utility-first CSS framework

### Backend
- **[FastAPI](https://fastapi.tiangolo.com/)** - Modern Python web framework
- **[Qdrant](https://qdrant.tech/)** - Vector similarity search engine
- **[Sentence Transformers](https://www.sbert.net/)** - State-of-the-art sentence embeddings
- **[PyTorch](https://pytorch.org/)** - Machine learning framework

### Infrastructure
- **[Docker](https://www.docker.com/)** - Containerization and deployment
- **[Python](https://www.python.org/)** - Backend services and ML processing
- **[Node.js](https://nodejs.org/)** - Frontend build and package management

## 📁 Project Structure

```
Physical-AI-Humanoid-Robotics/
├── docs/                       # Book content files (MDX format)
│   ├── chat.mdx               # Chatbot page
│   ├── index.md              # Homepage
│   ├── book-outline.md       # Book structure
│   ├── chapter-01/           # Chapter content
│   ├── chapter-02/           # Chapter content
│   ├── ...                   # Additional chapters
│   └── appendices/           # Reference materials
├── backend/                   # RAG backend services
│   ├── main.py              # FastAPI application entry point
│   ├── ingest_books.py      # Content ingestion script
│   ├── services/            # RAG service implementations
│   │   ├── embedding_service.py
│   │   ├── retrieval_service.py
│   │   ├── answer_generation_service.py
│   │   └── rag_service.py
│   ├── models/              # Pydantic models
│   ├── api/                 # API route definitions
│   ├── config/              # Configuration files
│   ├── data/                # Ingested document data
│   └── requirements.txt     # Python dependencies
├── src/                     # Custom React components
│   └── components/
│       └── RAGChatbot.js   # Chatbot React component
├── static/                  # Static assets
├── .env.example            # Environment variables template
├── setup.bat              # Windows setup script
├── start_app.bat          # Windows startup script
├── SETUP_RAG_CHATBOT.md   # Setup documentation
├── docusaurus.config.js   # Docusaurus configuration
├── sidebars.js           # Navigation structure
├── package.json          # Frontend dependencies
└── README.md            # This file
```

## 🚀 Quick Start

### Prerequisites
- [Python 3.8+](https://www.python.org/downloads/)
- [Node.js 18+](https://nodejs.org/)
- [Docker Desktop](https://www.docker.com/products/docker-desktop/) (for Windows/Mac)
- [Git](https://git-scm.com/)

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/tabindaseher/Physical-AI-Humanoid-Robotics.git
   cd Physical-AI-Humanoid-Robotics
   ```

2. **Install frontend dependencies**
   ```bash
   npm install
   ```

3. **Set up the backend**
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

4. **Start Qdrant vector database**
   ```bash
   docker run -d --name qdrant -p 6333:6333 qdrant/qdrant
   ```

5. **Ingest book content into the vector database**
   ```bash
   python ingest_books.py
   ```

6. **Start the backend server**
   ```bash
   uvicorn main:app --reload --host 0.0.0.0 --port 8000
   ```

7. **In a new terminal, start the frontend**
   ```bash
   cd ..  # back to project root
   npm start
   ```

### Windows Quick Setup
Use the automated setup script:
```cmd
setup.bat
```

Then start the application:
```cmd
start_app.bat
```

## 🌐 API Endpoints

### Backend API (`http://localhost:8000`)
- `GET /api/v1/health` - Health check and status
- `POST /api/v1/rag/book-wide` - Book-wide Q&A
- `POST /api/v1/rag/selected-text` - Selected text Q&A

### Example API Call
```bash
curl -X POST http://localhost:8000/api/v1/rag/book-wide \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key concepts in Physical AI?",
    "session_id": "session-123"
  }'
```

## ⚙️ Environment Variables

Create a `.env` file in the `backend` directory:

```env
# OpenAI API key (optional - fallback to rule-based generation)
OPENAI_API_KEY=your_openai_api_key_here

# Qdrant configuration (optional - defaults to localhost)
QDRANT_HOST=localhost
QDRANT_PORT=6333
QDRANT_COLLECTION_NAME=book_embeddings
```

## 🔧 Configuration

### Model Settings
- **Embedding Model**: `all-MiniLM-L6-v2` (for semantic similarity)
- **Answer Generation**: OpenAI GPT-3.5-turbo (with rule-based fallback)
- **Vector Size**: 384 dimensions
- **Similarity Threshold**: 0.3 (adjustable)

### Performance Tuning
- **Batch Size**: 32 (for embedding generation)
- **Top K**: 5 (most similar documents retrieved)
- **Response Tokens**: 500 (maximum answer length)

## 🏗️ Architecture Overview

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Frontend      │    │    Backend       │    │  Vector DB      │
│   (Docusaurus)  │◄──►│   (FastAPI)      │◄──►│  (Qdrant)       │
│                 │    │                  │    │                 │
│ • React Chatbot │    │ • RAG Services   │    │ • Book Content  │
│ • Documentation │    │ • Embedding      │    │ • Semantic      │
│ • Book Pages    │    │ • Retrieval      │    │   Search        │
└─────────────────┘    │ • Answer Gen     │    └─────────────────┘
                       │                  │
                       │ • API Endpoints  │
                       │ • Health Checks  │
                       └──────────────────┘
```

### Core Components
1. **Document Ingestion**: Parses MDX files and chunks content
2. **Embedding Service**: Converts text to semantic vectors
3. **Retrieval Service**: Finds relevant content using vector search
4. **Answer Generation**: Creates contextual responses
5. **Frontend Integration**: Seamless chatbot experience

## 🧪 Testing

### Backend Tests
```bash
cd backend
pytest
```

### API Testing
Use the health check endpoint to verify all services are running:
```bash
curl http://localhost:8000/api/v1/health
```

Expected response:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-17T15:04:02.559148",
  "dependencies": {
    "qdrant": true,
    "embedding_model": true,
    "answer_generation": true
  },
  "document_count": 295
}
```

## 🚀 Deployment

### Production Setup
1. **Backend**: Deploy with Gunicorn
2. **Frontend**: Build with `npm run build` and serve
3. **Database**: Use Qdrant Cloud or self-hosted instance

### Docker Compose (Coming Soon)
```yaml
# docker-compose.yml (planned)
version: '3.8'
services:
  qdrant:
    image: qdrant/qdrant
    ports:
      - "6333:6333"
  backend:
    build: ./backend
    ports:
      - "8000:8000"
    environment:
      - QDRANT_HOST=qdrant
  frontend:
    build: .
    ports:
      - "3000:3000"
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Add tests if applicable
5. Commit your changes (`git commit -m 'Add some amazing feature'`)
6. Push to the branch (`git push origin feature/amazing-feature`)
7. Open a Pull Request

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🐛 Issues & Support

If you encounter any issues or have questions:
- Check the [Issues](https://github.com/tabindaseher/Physical-AI-Humanoid-Robotics/issues) page
- Open a [new issue](https://github.com/tabindaseher/Physical-AI-Humanoid-Robotics/issues/new) 
- For questions, please describe your problem with detailed steps to reproduce

## 🚀 Future Improvements

- [ ] **Multi-language Support**: Support for different languages
- [ ] **Voice Integration**: Voice-to-text for hands-free interaction  
- [ ] **Advanced Analytics**: Usage metrics and performance insights
- [ ] **Content Management**: Admin panel for book content updates
- [ ] **Offline Support**: Progressive Web App capabilities
- [ ] **Mobile App**: Native mobile application
- [ ] **Collaborative Features**: Shared bookmarks and notes
- [ ] **Advanced Search**: Filter by chapter, difficulty, or topic

## 👨‍💻 Author

**Tabinda Seher**
- GitHub: [@tabindaseher](https://github.com/tabindaseher)
- LinkedIn: [Tabinda Seher](https://linkedin.com/in/tabindaseher)

## 🙏 Acknowledgments

- [Docusaurus](https://docusaurus.io/) for the excellent documentation framework
- [FastAPI](https://fastapi.tiangolo.com/) for the modern Python framework
- [Qdrant](https://qdrant.tech/) for vector search capabilities
- The open-source community for all the amazing tools and libraries

---

⭐ **Star this repo if you found it helpful!** 

For questions, suggestions, or collaboration, feel free to open an issue or contact me directly.