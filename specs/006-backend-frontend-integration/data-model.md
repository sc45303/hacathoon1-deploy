# Data Model: Backend-Frontend Integration for RAG Chatbot

## Core Entities

### UserQuery
- **queryId**: string - Unique identifier for the query
- **text**: string - The natural language question submitted by the user
- **timestamp**: datetime - When the query was submitted
- **sessionId**: string? - Optional identifier for tracking conversation context
- **userId**: string? - Optional identifier for the user making the query

### ChatMessage
- **id**: string - Unique identifier for the message
- **content**: string - The text content of the message
- **sender**: enum(user, agent) - Indicates whether the message is from the user or agent
- **timestamp**: datetime - When the message was created
- **sources**: list[RetrievedChunk]? - Optional list of source chunks used in the agent response

### APIResponse
- **requestId**: string - Unique identifier for the API request
- **status**: enum(success, error) - Status of the API response
- **data**: object? - Response data if successful
- **error**: object? - Error details if unsuccessful
- **timestamp**: datetime - When the response was generated

### RetrievedChunk (from existing agent system)
- **chunkId**: string - Unique identifier for the retrieved text chunk
- **content**: string - The text content of the retrieved chunk
- **url**: string - The source URL of the content in the book
- **position**: integer - The position of the chunk in the original document
- **relevanceScore**: float - Similarity score from the retrieval process
- **sourceMetadata**: dict - Additional metadata about the source document

### ChatSession
- **sessionId**: string - Unique identifier for the chat session
- **createdAt**: datetime - When the session was created
- **lastInteraction**: datetime - When the last interaction occurred
- **status**: enum(active, inactive, archived) - Current status of the session
- **messages**: list[ChatMessage] - List of messages in the conversation

### LoadingState
- **isLoading**: boolean - Whether the system is processing a request
- **progress**: float - Progress indicator (0-100) if available
- **message**: string? - Optional message to display to the user during loading

### ErrorState
- **hasError**: boolean - Whether an error occurred
- **errorMessage**: string? - Description of the error
- **errorType**: enum(network, validation, server, timeout) - Type of error that occurred
- **retryAvailable**: boolean - Whether the action can be retried

## Relationships

- A `UserQuery` generates one or more `ChatMessage` objects (user message + agent response)
- Multiple `ChatMessage` objects belong to one `ChatSession`
- An `APIResponse` contains either a `ChatMessage` or an `ErrorState`
- An agent response `ChatMessage` contains `RetrievedChunk` references
- `LoadingState` and `ErrorState` are UI states that affect how `ChatMessage` is displayed

## Validation Rules

### From Functional Requirements

- **FR-001**: System MUST provide an API endpoint for querying the backend Agent from the frontend
  - API requests must follow the defined schema with proper authentication if required

- **FR-002**: System MUST implement frontend code to call backend API asynchronously
  - Request objects must include all required fields for backend processing

- **FR-003**: System MUST display answers in a chatbot UI embedded within book pages
  - ChatMessage objects must contain properly formatted content for display

- **FR-004**: System MUST show loading indicators during request processing
  - LoadingState must be properly set during API calls

- **FR-005**: System MUST implement error handling and display error messages appropriately
  - ErrorState must be properly set when errors occur and contain user-friendly messages

- **FR-006**: System MUST ensure responses are limited to book content and retrieval context
  - Agent responses must reference RetrievedChunk objects from the book content

- **FR-010**: System MUST validate user inputs to prevent injection attacks
  - UserQuery.text must be sanitized to prevent XSS and injection attacks

### Success Criteria Compliance

- **SC-002**: Answers display correctly in the book's embedded chat interface
  - ChatMessage content must render properly in the frontend component