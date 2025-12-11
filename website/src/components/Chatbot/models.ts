// Frontend data models for the RAG chatbot integration
// Based on the entities defined in the data model specification

export interface UserQuery {
  queryId: string;
  text: string;
  timestamp: Date;
  sessionId?: string;
  userId?: string;
}

export interface RetrievedChunk {
  chunkId: string;
  content: string;
  url: string;
  position: number;
  relevanceScore: number;
  sourceMetadata: Record<string, any>;
}

export interface ChatMessage {
  id: string;
  content: string;
  sender: 'user' | 'agent';
  timestamp: Date;
  sources?: RetrievedChunk[];
}

export interface APIResponse {
  requestId: string;
  status: 'success' | 'error';
  data?: any;
  error?: any;
  timestamp: Date;
}

export interface ChatSession {
  sessionId: string;
  createdAt: Date;
  lastInteraction: Date;
  status: 'active' | 'inactive' | 'archived';
  messages: ChatMessage[];
}

export interface LoadingState {
  isLoading: boolean;
  progress?: number; // 0-100 if available
  message?: string; // Optional message to display
}

export interface ErrorState {
  hasError: boolean;
  errorMessage?: string;
  errorType?: 'network' | 'validation' | 'server' | 'timeout';
  retryAvailable: boolean;
}