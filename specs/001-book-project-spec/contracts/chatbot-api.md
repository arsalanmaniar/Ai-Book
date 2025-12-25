# API Contract: RAG Chatbot (chatbot-api.md)

**Base URL (REST)**: `/api/v1/chatbot`
**Base URL (WebSocket)**: `/ws/v1/chatbot`

## Endpoints

### `POST /query` (REST)
-   **Description**: Sends a query to the RAG chatbot for initial processing.
-   **Request Header**: `Authorization: Bearer <access_token>` (Optional, for personalized context)
-   **Request Body**:
    ```json
    {
        "question": "string",
        "chapter_context_id": "uuid" // Optional, current chapter context
    }
    ```
-   **Response (200 OK)**:
    ```json
    {
        "interaction_id": "uuid", // ID to track interaction, useful for follow-up via WebSocket
        "answer_preview": "string", // Initial quick answer or status
        "status": "string" // e.g., "processing", "completed"
    }
    ```

### `GET /ws/v1/chatbot` (WebSocket)
-   **Description**: Establishes a WebSocket connection for real-time chatbot interactions, including streaming responses and follow-up questions.
-   **Connection Parameters**:
    -   `access_token` (Query Param, Optional): For authenticated real-time interactions.
-   **Messages (Client to Server)**:
    -   **`{ "type": "query", "question": "string", "chapter_context_id": "uuid" }`**: Send a new question.
    -   **`{ "type": "follow_up", "interaction_id": "uuid", "question": "string" }`**: Send a follow-up question related to a previous interaction.

-   **Messages (Server to Client)**:
    -   **`{ "type": "partial_answer", "interaction_id": "uuid", "text": "string" }`**: Streaming partial answer.
    -   **`{ "type": "final_answer", "interaction_id": "uuid", "text": "string", "sources": ["chapter_slug1", "chapter_slug2"] }`**: Final answer with sources.
    -   **`{ "type": "error", "interaction_id": "uuid", "message": "string" }`**: Error during interaction.
    -   **`{ "type": "status", "interaction_id": "uuid", "message": "string" }`**: Informational status updates.