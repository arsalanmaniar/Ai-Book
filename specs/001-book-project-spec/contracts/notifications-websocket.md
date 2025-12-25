# WebSocket Contract: Real-time Notifications (notifications-websocket.md)

**Base URL (WebSocket)**: `/ws/v1/notifications`

## Endpoints

### `GET /ws/v1/notifications` (WebSocket)
-   **Description**: Establishes a WebSocket connection for real-time notifications to the authenticated user.
-   **Connection Parameters**:
    -   `access_token` (Query Param, Required): For authenticated notifications.

-   **Messages (Server to Client)**:
    -   **`{ "type": "new_content", "chapter_id": "uuid", "title": "string", "message": "New chapter published!" }`**: Notification for new content.
    -   **`{ "type": "content_update", "chapter_id": "uuid", "title": "string", "message": "Chapter updated." }`**: Notification for content updates.
    -   **`{ "type": "recommendation_update", "message": "New personalized recommendations available!" }`**: Notification for new recommendations.
    -   **`{ "type": "contribution_status", "submission_id": "uuid", "status": "string", "message": "Your contribution status updated to {status}." }`**: Notification for contribution status changes.
    -   **`{ "type": "general_message", "message": "string" }`**: General broadcast message from administrators.
    -   **`{ "type": "error", "message": "string" }`**: Error notification related to the WebSocket connection or service.
