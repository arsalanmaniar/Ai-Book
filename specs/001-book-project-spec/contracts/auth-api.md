# API Contract: Authentication (auth-api.md)

**Base URL**: `/api/v1/auth`

## Endpoints

### `POST /register`
-   **Description**: Registers a new user.
-   **Request Body**:
    ```json
    {
        "username": "string",
        "email": "user@example.com",
        "password": "string" // Min 8 chars, strong requirements
    }
    ```
-   **Response (201 Created)**:
    ```json
    {
        "id": "uuid",
        "username": "string",
        "email": "user@example.com"
    }
    ```
-   **Error (400 Bad Request)**: Invalid input (e.g., email already exists, weak password).

### `POST /login`
-   **Description**: Authenticates a user and issues an access token.
-   **Request Body**:
    ```json
    {
        "email": "user@example.com",
        "password": "string"
    }
    ```
-   **Response (200 OK)**:
    ```json
    {
        "access_token": "jwt_token",
        "token_type": "bearer"
    }
    ```
-   **Error (401 Unauthorized)**: Invalid credentials.

### `POST /logout`
-   **Description**: Invalidates the current user session/token.
-   **Request Header**: `Authorization: Bearer <access_token>`
-   **Response (204 No Content)**.

### `GET /me`
-   **Description**: Retrieves the profile of the authenticated user.
-   **Request Header**: `Authorization: Bearer <access_token>`
-   **Response (200 OK)**:
    ```json
    {
        "id": "uuid",
        "username": "string",
        "email": "user@example.com",
        "preferred_language": "string",
        "reading_history": ["chapter_id1", "chapter_id2"]
    }
    ```
-   **Error (401 Unauthorized)**: Invalid or missing token.

### `PUT /me/preferences`
-   **Description**: Updates the authenticated user's preferences.
-   **Request Header**: `Authorization: Bearer <access_token>`
-   **Request Body**:
    ```json
    {
        "preferred_language": "string", // e.g., "en", "ur"
        "theme": "string" // e.g., "dark", "light"
    }
    ```
-   **Response (200 OK)**: Updated user profile (same as `GET /me`).