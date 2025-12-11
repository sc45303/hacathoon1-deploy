import React from 'react';
import { ChatMessage, LoadingState, ErrorState } from './models';
import Message from './Message';

interface ChatWindowProps {
  messages: ChatMessage[];
  loadingState: LoadingState;
  errorState: ErrorState;
  formatErrorMessage: (errorState: ErrorState) => string;
  onRetry: () => void;
}

const ChatWindow: React.FC<ChatWindowProps> = ({
  messages,
  loadingState,
  errorState,
  formatErrorMessage,
  onRetry
}) => {
  return (
    <div className="chat-window">
      <div className="messages-container">
        {messages.map((message) => (
          <Message key={message.id} message={message} />
        ))}
        
        {/* Show loading indicator when needed */}
        {loadingState.isLoading && (
          <div className="message agent-message">
            <div className="message-content">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
              {loadingState.message && (
                <p className="loading-message">{loadingState.message}</p>
              )}
            </div>
          </div>
        )}
        
        {/* Show error message if needed */}
        {errorState.hasError && (
          <div className="message error-message">
            <div className="message-content">
              <p>{formatErrorMessage(errorState)}</p>
              <button onClick={onRetry} className="retry-button">Try Again</button>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default ChatWindow;