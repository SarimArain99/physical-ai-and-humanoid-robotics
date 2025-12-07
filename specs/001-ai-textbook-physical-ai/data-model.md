# Data Model: AI-Native Textbook on Physical AI & Humanoid Robotics

## Overview

This document defines the data models for the AI-Native Textbook application based on the entities identified in the feature specification. The models are designed to support all functional requirements while maintaining data integrity and performance, with special attention to the security and encryption requirements from the clarifications.

## Core Entities

### 1. User

**Description**: Represents a registered user of the textbook system

**Fields**:
- `id` (UUID): Unique identifier for the user
- `email` (String, unique, required): User's email address for authentication (encrypted at rest)
- `name` (String, required): User's full name (encrypted at rest)
- `created_at` (DateTime, required): Timestamp of account creation
- `updated_at` (DateTime, required): Timestamp of last update
- `is_active` (Boolean, default: true): Account status flag
- `encrypted_profile_data` (JSONB): Encrypted profile information including technical background and hardware access

**Relationships**:
- One-to-many with UserSession (one user has many sessions)
- One-to-many with ChatQuery (one user has many chat queries, optional)

**Validation Rules**:
- Email must be a valid email format
- Email must be unique across all users
- Name must be 2-100 characters
- Cannot have multiple active sessions simultaneously (implementation detail)

### 2. UserProfile

**Description**: Contains detailed profile information for personalization (stored as encrypted data in User.encrypted_profile_data)

**Fields** (as part of encrypted_profile_data):
- `programming_experience` (Enum: beginner, intermediate, advanced): User's programming skill level
- `robotics_background` (Enum: none, basic, intermediate, expert): User's robotics experience
- `hardware_access` (JSON object): Information about user's hardware access
  - `gpu_capability` (Enum: none, basic, mid, high): GPU access level
  - `robot_access` (Enum: none, simulation_only, physical_robot): Access to physical robots
  - `development_environment` (Enum: windows, macos, linux): Primary development platform
- `preferred_language` (Enum: en, ur, default: en): Default content language preference
- `learning_goals` (Text): User's specific learning objectives

**Relationships**:
- One-to-one with User (profile belongs to one user, stored encrypted)

**Validation Rules**:
- Programming experience must be one of the defined enum values
- Hardware access JSON must have valid structure
- Preferred language must be one of supported languages

### 3. TextbookChapter

**Description**: Represents a chapter in the Physical AI & Humanoid Robotics textbook

**Fields**:
- `id` (UUID): Unique identifier for the chapter
- `title` (String, required): Chapter title
- `slug` (String, unique, required): URL-friendly identifier
- `module` (Enum: ros2, digital_twin, ai_robot_brain, vla): Which module the chapter belongs to
- `week_number` (Integer, required): Week number in the 13-week course
- `content` (Text, required): Markdown content of the chapter
- `learning_objectives` (JSON array of strings): List of learning objectives
- `prerequisites` (JSON array of strings): Prerequisites for this chapter
- `exercises` (JSON array of objects): Hands-on exercises in this chapter
- `review_questions` (JSON array of objects): Review questions for this chapter
- `further_reading` (JSON array of objects): Additional reading resources
- `created_at` (DateTime, required): Timestamp of chapter creation
- `updated_at` (DateTime, required): Timestamp of last update

**Relationships**:
- One-to-many with TranslationCache (chapter can have multiple translations)
- One-to-many with PersonalizationRule (chapter can have multiple personalization rules)

**Validation Rules**:
- Title must be 5-200 characters
- Slug must be URL-friendly and unique
- Week number must be between 1-13
- Content must be valid Markdown
- Module must be one of the defined enum values

### 4. UserSession

**Description**: Tracks authenticated user sessions (tokens encrypted)

**Fields**:
- `id` (UUID): Unique identifier for the session
- `user_id` (UUID, foreign key to User, required): Reference to the user
- `encrypted_session_token` (String, unique, required): Encrypted session authentication token
- `expires_at` (DateTime, required): Expiration timestamp
- `created_at` (DateTime, required): Timestamp of session creation
- `last_accessed_at` (DateTime, required): Timestamp of last activity

**Relationships**:
- Many-to-one with User (session belongs to one user)

**Validation Rules**:
- User ID must reference an existing user
- Session token must be cryptographically secure and encrypted at rest
- Expires_at must be in the future
- Cannot have multiple active sessions per user (implementation detail)

### 5. ChatQuery

**Description**: Represents a query made to the RAG chatbot system

**Fields**:
- `id` (UUID): Unique identifier for the query
- `user_id` (UUID, foreign key to User, nullable): Reference to user (null for anonymous queries)
- `encrypted_query_text` (Text, required): The user's encrypted question
- `context_type` (Enum: full_textbook, selected_text, default: full_textbook): Type of context used
- `encrypted_selected_text` (Text, nullable): Encrypted text that was selected by the user (if applicable)
- `encrypted_response_text` (Text, required): The encrypted chatbot's response
- `confidence_score` (Float between 0-1): Confidence in the response
- `source_documents` (JSON array of objects): Documents referenced in the response
- `created_at` (DateTime, required): Timestamp of query creation

**Relationships**:
- Many-to-one with User (query belongs to one user, nullable)

**Validation Rules**:
- Query text must be 5-2000 characters (when decrypted)
- Context type must be one of the defined enum values
- Confidence score must be between 0 and 1
- Response must be generated based on source documents

### 6. TranslationCache

**Description**: Caches translated content to improve performance (encrypted content)

**Fields**:
- `id` (UUID): Unique identifier for the cache entry
- `chapter_id` (UUID, foreign key to TextbookChapter, required): Reference to the chapter
- `target_language` (String, required): Target language code (e.g., 'ur' for Urdu)
- `source_content_hash` (String, required): Hash of original content for cache invalidation
- `encrypted_translated_content` (Text, required): The encrypted translated content
- `created_at` (DateTime, required): Timestamp of cache creation
- `updated_at` (DateTime, required): Timestamp of last update

**Relationships**:
- Many-to-one with TextbookChapter (cache entry belongs to one chapter)

**Validation Rules**:
- Chapter ID must reference an existing chapter
- Target language must be a valid language code
- Source content hash must match the current chapter content for valid cache
- Cache entries should expire after 24 hours or when source content changes

### 7. PersonalizationRule

**Description**: Defines how content should be personalized based on user profile

**Fields**:
- `id` (UUID): Unique identifier for the rule
- `chapter_id` (UUID, foreign key to TextbookChapter, required): Reference to the chapter
- `condition` (JSON object): Conditions that trigger this rule (e.g., "user.gpu_capability == 'none'")
- `action` (JSON object): Action to take when condition is met (e.g., "replace_section", "add_alternative")
- `encrypted_content_variant` (Text): Encrypted alternative content to show when rule is triggered
- `priority` (Integer, default: 0): Priority of the rule (higher numbers take precedence)
- `created_at` (DateTime, required): Timestamp of rule creation
- `updated_at` (DateTime, required): Timestamp of last update

**Relationships**:
- Many-to-one with TextbookChapter (rule belongs to one chapter)

**Validation Rules**:
- Chapter ID must reference an existing chapter
- Condition must be a valid JSON object with proper structure
- Priority must be between 0-100
- Content variant must be valid Markdown if replacing content

## Relationships Summary

```
User (1) → (N) UserSession
User (1) → (N) ChatQuery (optional)
TextbookChapter (1) → (N) TranslationCache
TextbookChapter (1) → (N) PersonalizationRule
ChatQuery (N) → (1) User (optional)
```

## Indexing Strategy

### Required Indexes:
1. **User.email**: Unique index for authentication
2. **TextbookChapter.slug**: Unique index for URL routing
3. **TextbookChapter.module**: Index for module-based filtering
4. **TextbookChapter.week_number**: Index for course progression
5. **UserSession.encrypted_session_token**: Unique index for session validation (partial decryption for validation)
6. **UserSession.user_id**: Index for user session management
7. **TranslationCache.chapter_id**: Index for chapter translation lookups
8. **TranslationCache.target_language**: Index for language-based queries
9. **PersonalizationRule.chapter_id**: Index for chapter personalization rules

## State Transitions

### UserSession States:
- **Active**: Session token is valid and not expired
- **Expired**: Session token has passed expiration time
- **Invalidated**: Session was explicitly invalidated (user logout)

### Transition Rules:
- Active → Expired: When `expires_at` is reached
- Active → Invalidated: When user logs out or session is terminated
- Expired/Invalidated → Active: When user logs in again (new session created)

## Data Integrity Constraints

1. **Referential Integrity**: All foreign keys must reference existing records
2. **Unique Constraints**: Email and session token must be unique
3. **Check Constraints**:
   - Confidence scores between 0-1
   - Week numbers between 1-13
   - Priorities between 0-100
4. **Not Null Constraints**: Required fields must have values
5. **Length Constraints**: Text fields have maximum lengths as specified
6. **Encryption Requirements**: All sensitive user data must be encrypted at rest (FR-014 compliance)

## Security & Encryption Considerations

Based on clarification requirement FR-014, the following fields must be encrypted at rest:
- User email (if containing PII)
- User name (if containing PII)
- UserProfile data (entire encrypted_profile_data field)
- UserSession tokens (encrypted_session_token)
- ChatQuery content (query_text, selected_text, response_text)
- TranslationCache content (encrypted_translated_content)
- PersonalizationRule content (encrypted_content_variant)

Encryption approach: Use AES-256 for data at rest with proper key management. Field-level encryption allows for selective decryption when needed for processing while maintaining security.