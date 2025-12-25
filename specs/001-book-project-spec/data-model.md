# Data Model: Physical AI & Humanoid Robotics Textbook Project

**Date**: 2025-12-06

This data model outlines the core entities and their relationships for the textbook project, leveraging PostgreSQL as the primary data store.

## Entities

### User
Represents a user of the textbook platform.
-   `id` (UUID/Integer): Unique identifier for the user.
-   `username` (String): User's chosen username (unique).
-   `email` (String): User's email address (unique, for authentication).
-   `password_hash` (String): Hashed password for authentication.
-   `preferences` (JSON/Text): Stores user preferences (e.g., theme, notification settings).
-   `reading_history` (Array of Chapter IDs): List of chapters read by the user (for chapter-level tracking).
-   `preferred_language` (String): User's chosen display language (e.g., "en", "ur").
-   `created_at` (Timestamp): Timestamp of user creation.
-   `updated_at` (Timestamp): Timestamp of last user update.

### Chapter
Represents a chapter or section of the textbook content.
-   `id` (UUID/Integer): Unique identifier for the chapter.
-   `title` (String): Title of the chapter.
-   `slug` (String): URL-friendly slug for the chapter (unique).
-   `content` (Text): The full Markdown content of the chapter.
-   `order` (Integer): The display order of the chapter within the textbook.
-   `language` (String): The language of this specific chapter version (e.g., "en", "ur").
-   `version` (String): Semantic version of the chapter content (e.g., "1.0.0").
-   `published_at` (Timestamp): Date when the chapter was published.
-   `created_at` (Timestamp): Timestamp of initial chapter creation.
-   `updated_at` (Timestamp): Timestamp of last chapter update.

### Recommendation
Represents a personalized recommendation for a user.
-   `id` (UUID/Integer): Unique identifier for the recommendation.
-   `user_id` (UUID/Integer): Foreign key linking to the User.
-   `chapter_id` (UUID/Integer): Foreign key linking to the recommended Chapter.
-   `score` (Float): Relevance score for the recommendation.
-   `recommended_at` (Timestamp): When the recommendation was generated.
-   `clicked_at` (Timestamp, Nullable): When the user clicked on the recommendation.

### Question/Answer (RAG Chatbot Interaction)
Represents an interaction with the RAG chatbot.
-   `id` (UUID/Integer): Unique identifier for the interaction.
-   `user_id` (UUID/Integer, Nullable): Foreign key linking to the User (if logged in).
-   `question_text` (Text): The user's query to the chatbot.
-   `answer_text` (Text): The chatbot's generated response.
-   `source_citations` (JSON/Array of Strings): List of cited sections/chapters from the textbook.
-   `asked_at` (Timestamp): When the question was asked.

### Translation (Implicit/Managed through Chapter Entity)
Translations are managed by having multiple `Chapter` entries with the same logical content `id` (or a `parent_chapter_id`) but different `language` codes. This simplifies content management and retrieval.
