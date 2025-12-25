# API Contract: Content & Navigation (content-api.md)

**Base URL**: `/api/v1/content`

## Endpoints

### `GET /chapters`
-   **Description**: Retrieves a list of all available chapters, possibly filtered by language.
-   **Query Parameters**:
    -   `lang` (Optional, String): Filter by language (e.g., "en", "ur"). Default to user's preferred language or "en".
-   **Response (200 OK)**:
    ```json
    [
        {
            "id": "uuid",
            "title": "string",
            "slug": "string",
            "order": "integer",
            "language": "string",
            "version": "string"
        }
    ]
    ```

### `GET /chapters/{chapter_slug}`
-   **Description**: Retrieves the full content of a specific chapter by its slug.
-   **Path Parameters**:
    -   `chapter_slug` (String): The URL-friendly slug of the chapter.
-   **Query Parameters**:
    -   `lang` (Optional, String): Request specific language version. Default to user's preferred language or "en".
-   **Response (200 OK)**:
    ```json
    {
        "id": "uuid",
        "title": "string",
        "slug": "string",
        "content": "markdown_string",
        "order": "integer",
        "language": "string",
        "version": "string",
        "published_at": "timestamp"
    }
    ```
-   **Error (404 Not Found)**: Chapter or specific language version not found.

### `GET /search`
-   **Description**: Searches across all textbook content.
-   **Query Parameters**:
    -   `query` (String, Required): Search query string.
    -   `lang` (Optional, String): Language to search within. Default to user's preferred language or "en".
-   **Response (200 OK)**:
    ```json
    [
        {
            "chapter_id": "uuid",
            "chapter_title": "string",
            "snippet": "string" // Relevant text snippet
        }
    ]
    ```