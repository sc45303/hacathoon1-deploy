// Error handling utilities for the frontend-backend communication
import { ErrorState } from './models';

/**
 * Creates an error state object based on an error type and message
 */
export function createErrorState(hasError: boolean, message?: string, errorType?: 'network' | 'validation' | 'server' | 'timeout'): ErrorState {
  return {
    hasError,
    errorMessage: message,
    errorType,
    retryAvailable: hasError && errorType !== 'validation' // validation errors typically can't be retried
  };
}

/**
 * Handles an error from an API call and returns the appropriate ErrorState
 */
export function handleError(error: any): ErrorState {
  // Determine the type of error based on properties
  if (error instanceof TypeError && error.message.includes('fetch')) {
    // Network error (e.g., server offline, DNS failure)
    return createErrorState(true, 'Network error: Unable to connect to the server', 'network');
  } else if (error.message.includes('API Error') || error.status >= 400) {
    // Server error (e.g., bad request, server error)
    const status = error.status ? `(${error.status})` : '';
    return createErrorState(true, `Server error: ${error.message} ${status}`, 'server');
  } else if (error.name === 'ValidationError') {
    // Validation error
    return createErrorState(true, `Validation error: ${error.message}`, 'validation');
  } else {
    // Generic error
    return createErrorState(true, `An unexpected error occurred: ${error.message || error}`, 'server');
  }
}

/**
 * Formats error messages for user display
 */
export function formatErrorMessage(errorState: ErrorState): string {
  if (!errorState.hasError || !errorState.errorMessage) {
    return '';
  }

  switch (errorState.errorType) {
    case 'network':
      return 'Unable to connect to the server. Please check your internet connection and try again.';
    case 'server':
      return `Server error: ${errorState.errorMessage}`;
    case 'validation':
      return `Invalid input: ${errorState.errorMessage}`;
    case 'timeout':
      return 'Request timed out. Please try again.';
    default:
      return `An error occurred: ${errorState.errorMessage}`;
  }
}

/**
 * Async function wrapper that handles errors and returns a standardized result
 */
export async function handleAsyncOperation<T>(
  operation: () => Promise<T>,
  onError?: (error: ErrorState) => void
): Promise<{ success: boolean; data?: T; error?: ErrorState }> {
  try {
    const data = await operation();
    return { success: true, data };
  } catch (error: any) {
    const errorState = handleError(error);
    if (onError) onError(errorState);
    return { success: false, error: errorState };
  }
}