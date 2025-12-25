# Quickstart Guide: Physical AI & Humanoid Robotics Textbook Project

**Date**: 2025-12-06

This guide provides a quick overview of how to set up and run the Physical AI & Humanoid Robotics Textbook Project. It covers both development environment setup and high-level deployment considerations.

## 1. Prerequisites

Ensure you have the following installed:
-   Docker & Docker Compose (for local development)
-   Kubernetes cluster (e.g., Minikube, kind, or a cloud provider's managed service) and `kubectl` (for deployment)
-   Python 3.9+ (`pip` / `pipenv` / `poetry`)
-   Node.js 18+ (`npm` / `yarn`)

## 2. Local Development Setup

### 2.1. Backend (FastAPI + PostgreSQL)

1.  Navigate to the `backend/` directory.
2.  Install Python dependencies:
    ```bash
    pip install -r requirements.txt
    # or using poetry
    # poetry install
    ```
3.  Set up environment variables (e.g., `.env` file for database connection strings, secret keys). A `.env.example` will be provided.
4.  Start the PostgreSQL database and FastAPI service using Docker Compose:
    ```bash
    docker-compose up --build
    ```
    The FastAPI application will be accessible at `http://localhost:8000`.

### 2.2. Frontend (React + TypeScript)

1.  Navigate to the `frontend/` directory.
2.  Install Node.js dependencies:
    ```bash
    npm install
    # or yarn install
    ```
3.  Set up environment variables (e.g., `.env` file for backend API URL).
4.  Start the React development server:
    ```bash
    npm start
    # or yarn start
    ```
    The React application will be accessible at `http://localhost:3000`.

## 3. Running Tests

### Backend Tests (Python)
```bash
# In backend/ directory
pytest
```

### Frontend Tests (React)
```bash
# In frontend/ directory
npm test
# or yarn test
```

## 4. Deployment (Docker & Kubernetes)

1.  **Build Docker Images**:
    Navigate to respective `backend/` and `frontend/` directories and build Docker images:
    ```bash
    docker build -t physical-ai-textbook-backend:latest .
    docker build -t physical-ai-textbook-frontend:latest .
    ```
2.  **Kubernetes Configuration**:
    The `ops/kubernetes/` directory will contain Kubernetes deployment manifests (Deployments, Services, Ingress, Persistent Volumes).
    Apply these manifests to your Kubernetes cluster:
    ```bash
    kubectl apply -f ops/kubernetes/
    ```
3.  **Database Migration**:
    Database migrations will be handled as part of the backend deployment process or via a separate Kubernetes Job.

## 5. AI Chatbot Setup (RAG)

-   The RAG chatbot components will run as part of the backend services, leveraging pre-trained models and a vector database (e.g., deployed as a separate service or integrated within the FastAPI application).
-   Initial content ingestion into the vector database will be a separate setup step or automated process.

## 6. Real-time Notifications

-   Real-time notifications will be managed via WebSocket connections from the frontend to the backend, handled by FastAPI.
