import React, { useState, useEffect, useRef } from 'react';
import { ChatMessage, LoadingState, ErrorState } from './models';
import { createErrorState, formatErrorMessage, handleAsyncOperation } from './errorHandler';
import apiService from './api';
import ChatWindow from './ChatWindow';
import InputArea from './InputArea';
import './styles.css';

interface ChatbotProps {
  initialMessages?: ChatMessage[];
  backendUrl?: string;
  placeholderText?: string;
  title?: string;
  sessionId?: string;
}

const Chatbot: React.FC<ChatbotProps> = ({
  initialMessages = [],
  backendUrl = '',
  placeholderText = 'Ask a question about the book...',
  title = 'Book Assistant',
  sessionId
}) => {
  const [messages, setMessages] = useState<ChatMessage[]>(initialMessages);
  const [inputValue, setInputValue] = useState('');
  const [loadingState, setLoadingState] = useState<LoadingState>({ isLoading: false });
  const [errorState, setErrorState] = useState<ErrorState>(createErrorState(false));
  const [currentSessionId, setCurrentSessionId] = useState<string>(sessionId || '');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Update API service with new backend URL if provided
  useEffect(() => {
    if (backendUrl) {
      // We could reconfigure the apiService here if needed
    }
  }, [backendUrl]);

  // Scroll to bottom when new messages arrive
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    if (!inputValue.trim()) return;

    // Add user message to the chat
    const userMessage: ChatMessage = {
      id: `msg-${Date.now()}`,
      content: inputValue,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setLoadingState({ isLoading: true, message: 'Processing your question...' });
    setErrorState(createErrorState(false));

    // Use the API service to send the question to the backend
    const result = await handleAsyncOperation(async () => {
      return await apiService.askQuestion({
        query: inputValue,
        session_id: currentSessionId,
      });
    }, setErrorState);

    setLoadingState({ isLoading: false });

    if (result.success && result.data) {
      const response = result.data;

      // Update session ID if new one was returned
      if (response.query_id && !currentSessionId) {
        setCurrentSessionId(response.query_id);
      }

      // Create agent response message
      const agentMessage: ChatMessage = {
        id: `msg-${Date.now()}-${Math.random()}`,
        content: response.answer,
        sender: 'agent',
        timestamp: new Date(response.timestamp),
        sources: response.sources
      };

      setMessages(prev => [...prev, agentMessage]);
    }
  };

  const handleRetry = () => {
    setErrorState(createErrorState(false));
  };

  return (
    <div className="chatbot-container">
      <div className="chatbot-header">
        <h3>{title}</h3>
      </div>
      <ChatWindow 
        messages={messages} 
        loadingState={loadingState} 
        errorState={errorState} 
        formatErrorMessage={formatErrorMessage}
        onRetry={handleRetry}
      />
      <InputArea 
        inputValue={inputValue}
        setInputValue={setInputValue}
        handleSubmit={handleSubmit}
        loadingState={loadingState}
        placeholderText={placeholderText}
      />
      <div ref={messagesEndRef} />
    </div>
  );
};

export default Chatbot;