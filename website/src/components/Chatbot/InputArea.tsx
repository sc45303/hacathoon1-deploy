import React from 'react';
import { LoadingState } from './models';

interface InputAreaProps {
  inputValue: string;
  setInputValue: (value: string) => void;
  handleSubmit: (e: React.FormEvent) => void;
  loadingState: LoadingState;
  placeholderText: string;
}

const InputArea: React.FC<InputAreaProps> = ({
  inputValue,
  setInputValue,
  handleSubmit,
  loadingState,
  placeholderText
}) => {
  return (
    <div className="input-area">
      <form onSubmit={handleSubmit} className="input-form">
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder={placeholderText}
          disabled={loadingState.isLoading}
          className="input-field"
        />
        <button 
          type="submit" 
          disabled={loadingState.isLoading || !inputValue.trim()}
          className="submit-button"
        >
          {loadingState.isLoading ? 'Sending...' : 'Send'}
        </button>
      </form>
    </div>
  );
};

export default InputArea;