import React from 'react';
import { ChatMessage } from './models';

interface MessageProps {
  message: ChatMessage;
}

const Message: React.FC<MessageProps> = ({ message }) => {
  const isUser = message.sender === 'user';
  
  return (
    <div className={`message ${isUser ? 'user-message' : 'agent-message'}`}>
      <div className="message-content">
        <p>{message.content}</p>
        
        {/* Display sources if this is an agent message with sources */}
        {message.sender === 'agent' && message.sources && message.sources.length > 0 && (
          <div className="sources-section">
            <h4>Sources:</h4>
            <ul>
              {message.sources.map((source) => (
                <li key={source.chunkId}>
                  <a href={source.url} target="_blank" rel="noopener noreferrer">
                    {source.content.substring(0, 100)}{source.content.length > 100 ? '...' : ''}
                  </a>
                </li>
              ))}
            </ul>
          </div>
        )}
      </div>
      <div className="message-meta">
        <small>{message.timestamp.toLocaleString()}</small>
      </div>
    </div>
  );
};

export default Message;