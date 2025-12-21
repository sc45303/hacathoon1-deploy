// API service module for communicating with the backend RAG agent
import { UserQuery, ChatMessage, RetrievedChunk } from './models';

export interface AskRequest {
  query: string;
  session_id?: string;
  user_id?: string;
}

export interface AskResponse {
  response_id: string;
  answer: string;
  sources: RetrievedChunk[];
  query_id: string;
  timestamp: string;
  confidence_score?: number;
}

export interface ErrorResponse {
  error: string;
  error_code: string;
}

export class ChatbotAPIService {
  private baseUrl: string;

  constructor(baseUrl: string = '') {
    // If no base URL is provided, use the current origin (for relative API calls)
    this.baseUrl = baseUrl || window.location.origin;
  }

  /**
   * Send a query to the backend RAG agent
   */
  async askQuestion(request: AskRequest): Promise<AskResponse> {
    const response = await fetch(`${this.baseUrl}/agent/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        question: request.query,
        selected_text: null,  // For now, not using selected text
        mode: null  // Will default to book RAG mode
      }),
    });

    if (!response.ok) {
      const errorResponse: ErrorResponse = await response.json();
      throw new Error(`API Error: ${errorResponse.error}`);
    }

    const data: AskResponse = await response.json();

    // Map the response to the expected format
    return {
      response_id: 'response-' + Date.now(),  // Backend doesn't return response_id, so generate one
      answer: data.answer,
      sources: data.sources || [],
      query_id: 'query-' + Date.now(),  // Backend doesn't return query_id, so generate one
      timestamp: new Date().toISOString(),
      confidence_score: 0.8  // Backend doesn't return confidence, so use a default
    };
  }

  /**
   * Check the health status of the backend service
   */
  async checkHealth(): Promise<any> {
    const response = await fetch(`${this.baseUrl}/health`, {
      method: 'GET',
    });

    if (!response.ok) {
      throw new Error('Health check failed');
    }

    const data = await response.json();
    return data;
  }

  /**
   * Run a custom agent with instructions
   */
  async runAgent(instructions: string, input_data?: Record<string, any>, session_id?: string): Promise<any> {
    const response = await fetch(`${this.baseUrl}/agent/run`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        instructions,
        input_data,
        session_id
      }),
    });

    if (!response.ok) {
      throw new Error('Agent run failed');
    }

    const data = await response.json();
    return data;
  }
}

// Default instance of the API service
const apiService = new ChatbotAPIService();
export default apiService;