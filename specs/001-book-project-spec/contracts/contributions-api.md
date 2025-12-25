# API Contract: Contributions (contributions-api.md)

**Base URL**: `/api/v1/contributions`

## Endpoints

### `POST /chapters`
-   **Description**: Submits a new chapter or an update to an existing one for review.
-   **Request Header**: `Authorization: Bearer <access_token>`
-   **Request Body**:
    ```json
    {
        "title": "string",
        "slug": "string",
        "content": "markdown_string",
        "language": "string", // e.g., "en", "ur"
        "parent_chapter_id": "uuid", // Optional, if it's an update or related to an existing chapter
        "proposed_version": "string" // e.g., "1.0.0"
    }
    ```
-   **Response (202 Accepted)**:
    ```json
    {
        "submission_id": "uuid",
        "status": "pending_review",
        "message": "Chapter submission received for review."
    }
    ```
-   **Error (400 Bad Request)**: Invalid content, missing required fields.
-   **Error (401 Unauthorized)**: Invalid or missing token.

### `GET /submissions/{submission_id}`
-   **Description**: Retrieves the status of a chapter contribution submission.
-   **Request Header**: `Authorization: Bearer <access_token>`
-   **Path Parameters**:
    -   `submission_id` (UUID): The ID of the chapter submission.
-   **Response (200 OK)**:
    ```json
    {
        "submission_id": "uuid",
        "status": "string", // e.g., "pending_review", "in_review", "approved", "rejected"
        "reviewer_comments": "string", // Optional, if reviewed
        "submitted_at": "timestamp"
    }
    ```