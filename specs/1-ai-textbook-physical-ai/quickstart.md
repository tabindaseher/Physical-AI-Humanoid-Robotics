# Quickstart Guide: AI-Native Textbook for Physical AI & Humanoid Robotics

**Created**: 2026-01-02
**Feature**: 1-ai-textbook-physical-ai

## Development Environment Setup

### Prerequisites
- Node.js (v16 or higher)
- Python 3.9+
- npm or yarn package manager
- Git
- OpenAI API account
- Neon Serverless Postgres account
- Qdrant Cloud account

### Initial Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Set up the Docusaurus environment**
   ```bash
   cd textbook
   npm install
   ```

3. **Set up the backend environment**
   ```bash
   cd backend
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```

4. **Environment variables**
   Create `.env` file in the backend directory:
   ```
   OPENAI_API_KEY=your_openai_api_key
   NEON_DATABASE_URL=your_neon_database_url
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   ```

### Running the Development Servers

1. **Frontend (Docusaurus)**
   ```bash
   cd textbook
   npm run start
   ```
   The textbook will be available at http://localhost:3000

2. **Backend (FastAPI)**
   ```bash
   cd backend
   uvicorn main:app --reload
   ```
   The API will be available at http://localhost:8000

## Content Creation Workflow

### Creating a New Chapter

1. **Add a new chapter file**
   - Navigate to `textbook/docs/chapters/[module-name]/`
   - Create a new markdown file with the pattern `chapter-<number>-<topic>.md`
   - Follow the Docusaurus documentation format with proper metadata

   Example:
   ```markdown
   ---
   id: chapter-1-physical-ai-foundations
   title: Foundations of Physical AI
   sidebar_label: Physical AI Foundations
   slug: /chapters/physical-ai-foundations
   ---

   # Foundations of Physical AI

   ## Learning Objectives
   - Understand the difference between digital and physical AI
   - Explain embodied cognition principles
   - Identify key challenges in physical AI systems

   [Chapter content in Markdown...]
   ```

2. **Add academic references**
   - Add new references to `textbook/src/data/references.json`
   - Ensure at least 50% are peer-reviewed research papers
   - Use APA citation style

3. **Update the sidebar**
   - Modify `textbook/sidebars.js` to include the new chapter in the navigation

### Generating Embeddings for RAG

1. **Process new content**
   ```bash
   cd backend
   python scripts/generate_embeddings.py
   ```

2. **Verify embeddings**
   - Check that embeddings were created in the Qdrant collection
   - Test retrieval with sample queries

### Testing the RAG Functionality

1. **Query the chatbot**
   - Use the frontend interface or make direct API calls
   - Example API call:
   ```bash
   curl -X POST http://localhost:8000/api/chat \
     -H "Content-Type: application/json" \
     -d '{"query": "What is embodied cognition?", "context": "physical-ai-foundations"}'
   ```

2. **Verify response accuracy**
   - Ensure responses are based on textbook content
   - Check that sources are properly cited
   - Validate confidence scores

## Deployment

### GitHub Pages Deployment

1. **Build the static site**
   ```bash
   cd textbook
   npm run build
   ```

2. **Deploy to GitHub Pages**
   - Configure GitHub Actions in `.github/workflows/deploy.yml`
   - Push changes to the main branch
   - The site will automatically deploy

### Backend Deployment

1. **Prepare for deployment**
   - Update environment variables for production
   - Ensure all dependencies are properly specified

2. **Deploy backend services**
   - Deploy FastAPI application to cloud provider (AWS, GCP, Azure, etc.)
   - Ensure connections to Neon Postgres and Qdrant Cloud

## Validation Checklist

### Content Validation
- [ ] All chapters meet Flesch-Kincaid Grade 10-12 readability
- [ ] Minimum 15 sources with 50%+ peer-reviewed papers
- [ ] All technical concepts align with ROS 2 and NVIDIA Isaac documentation
- [ ] Content follows AI-native design principles for RAG optimization

### System Validation
- [ ] Local Docusaurus build runs without errors
- [ ] RAG chatbot returns relevant responses
- [ ] All API endpoints function correctly
- [ ] Embeddings properly retrieve relevant content

### Academic Validation
- [ ] Content meets university-level academic rigor
- [ ] All claims are verifiable through citations
- [ ] Mathematical concepts are appropriately rigorous
- [ ] Practical examples support theoretical concepts