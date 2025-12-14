# RAG Chatbot Integration Guide

This document explains how to integrate the RAG (Retrieval Augmented Generation) chatbot into your Docusaurus pages.

## Overview

The chatbot allows users to ask questions about the book content and receive contextual answers. The component communicates with the backend RAG agent service to retrieve relevant information from the book.

## Prerequisites

- Docusaurus v2.x or later
- Backend RAG agent service running and accessible
- (Optional) Custom backend URL if not hosted on the same origin

## Installation

No installation is required if you're working with the existing repository. The chatbot component is already built into the project.

## Basic Usage

### In MDX Files

You can add the chatbot to any MDX file by importing and using the component:

```md
import Chatbot from '@site/src/components/Chatbot/Chatbot';

<Chatbot />
```

### With Custom Properties

You can customize the chatbot behavior with properties:

```md
import Chatbot from '@site/src/components/Chatbot/Chatbot';

<Chatbot
  title="Book Assistant"
  placeholderText="Ask a question about this book..."
  backendUrl="https://your-backend-domain.com"
  sessionId="unique-session-id"
/>
```

## Properties

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `title` | string | `"Book Assistant"` | Title displayed in the chat header |
| `placeholderText` | string | `"Ask a question about the book..."` | Placeholder text in the input area |
| `backendUrl` | string | `""` | Backend URL (uses the same origin if empty) |
| `sessionId` | string | auto-generated | Session identifier for conversation continuity |
| `initialMessages` | ChatMessage[] | `[]` | Initial messages to display in the chat |

## Advanced Usage

### Programmatic Control

You can programmatically control the chatbot by creating a ref:

```jsx
import React, { useRef, useEffect } from 'react';
import Chatbot from '@site/src/components/Chatbot/Chatbot';

const MyPage = () => {
  const chatbotRef = useRef(null);
  
  // Example: Auto-focus the chatbot when the page loads
  useEffect(() => {
    // Any programmatic control would happen here
  }, []);
  
  return (
    <div>
      <h1>My Page</h1>
      <Chatbot ref={chatbotRef} />
    </div>
  );
};
```

### Styling

The chatbot comes with default styles. If you need to override them, you can add custom CSS targeting the classes:

- `.chatbot-container` - Main container
- `.chat-window` - Messages area
- `.input-area` - Input field area
- `.user-message` - User's message bubbles
- `.agent-message` - Agent's response bubbles

## Error Handling

The chatbot handles errors gracefully:

- Network errors show a user-friendly message with a "Retry" button
- Server errors show descriptive messages
- Validation errors are caught and displayed appropriately

## Backend Integration

The chatbot communicates with the backend using these endpoints:

- `POST /ask` - Submit questions to the RAG agent
- `GET /health` - Check backend service health
- `POST /agent/run` - Run custom agent instructions

The backend should support CORS requests from your frontend domain.

## Best Practices

1. Place the chatbot where users are likely to have questions
2. Use a consistent placement across pages
3. Consider using a persistent chatbot for longer documentation
4. Provide clear instructions on how to use the chatbot
5. Monitor backend API usage and set up appropriate rate limiting

## Troubleshooting

### Chatbot Not Appearing

- Check that the import path is correct
- Verify that you're using Docusaurus with MDX support
- Look for JavaScript errors in the browser console

### Connection Errors

- Verify the backend URL is accessible
- Check browser console for CORS errors
- Confirm the backend RAG service is running

### Slow Responses

- Verify network connectivity
- Check backend performance
- Consider adding loading indicators for longer queries

## Support

For support, contact the development team or submit an issue to the repository.