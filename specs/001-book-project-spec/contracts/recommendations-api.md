# API Contract: Recommendations (recommendations-api.md)

**Base URL**: `/api/v1/recommendations`

## Endpoints

### `GET /`
-   **Description**: Retrieves personalized chapter recommendations for the authenticated user.
-   **Request Header**: `Authorization: Bearer <access_token>`
-   **Query Parameters**:
    -   `limit` (Optional, Integer): Maximum number of recommendations to return. Default: 5.
-   **Response (200 OK)**:
    ```json
    [
        {
            "chapter_id": "uuid",
            "title": "string",
            "slug": "string",
            "score": "float"
        }
    ]
    ```
-   **Error (401 Unauthorized)**: Invalid or missing token.