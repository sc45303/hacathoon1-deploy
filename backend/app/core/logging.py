import logging
import sys
from app.core.config import settings


def setup_logging():
    """Configure the application logging."""
    # Configure root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(getattr(logging, settings.log_level.upper()))
    
    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(getattr(logging, settings.log_level.upper()))
    
    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    console_handler.setFormatter(formatter)
    
    # Add handler to root logger
    root_logger.addHandler(console_handler)
    
    # Prevent duplicate logs if logging is configured elsewhere
    root_logger.propagate = False
    
    # Log that logging is configured
    logging.info(f"Logging configured with level: {settings.log_level}")


def get_logger(name: str) -> logging.Logger:
    """Get a logger with the specified name."""
    return logging.getLogger(name)


# Initialize logging when module is imported
setup_logging()