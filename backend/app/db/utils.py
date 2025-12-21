"""
Utility functions for the database layer of the RAG chatbot backend.

This module provides utilities for validating database connections
and other common database-related operations.
"""
import asyncio
from typing import Dict, Any, List
import logging

from .neon import NeonDB
from .qdrant import QdrantDB

logger = logging.getLogger(__name__)


async def validate_connections() -> bool:
    """
    Validates that both Neon and Qdrant connections are properly configured.
    
    This function checks that both database connections can be established
    without executing operations that consume resources or exceed limits.
    
    Returns:
        Boolean indicating if both connections are valid
    """
    logger.info("Starting database connection validation...")
    
    neon_db = NeonDB()
    qdrant_db = QdrantDB()
    
    try:
        # Validate Neon connection
        neon_valid = await neon_db.validate_connection()
        logger.info(f"Neon connection validation: {'PASSED' if neon_valid else 'FAILED'}")
        
        # Validate Qdrant connection
        qdrant_valid = await qdrant_db.validate_connection()
        logger.info(f"Qdrant connection validation: {'PASSED' if qdrant_valid else 'FAILED'}")
        
        # Both connections must be valid
        all_valid = neon_valid and qdrant_valid
        
        if all_valid:
            logger.info("All database connections validated successfully")
        else:
            logger.error("One or more database connection validations failed")
        
        return all_valid
    
    except Exception as e:
        logger.error(f"Error during database connection validation: {e}")
        return False
    
    finally:
        # Ensure connections are closed
        await neon_db.close_connections()
        await qdrant_db.close_connections()


async def get_health_status() -> Dict[str, Any]:
    """
    Gets the health status of all database connections.
    
    Returns:
        Dictionary with health status for both Neon and Qdrant databases
    """
    logger.info("Fetching database health status...")
    
    neon_db = NeonDB()
    qdrant_db = QdrantDB()
    
    try:
        # Get health status for Neon
        neon_health = await neon_db.health_check()
        
        # Get health status for Qdrant
        qdrant_health = await qdrant_db.health_check()
        
        # Combine health statuses
        overall_status = "healthy" if (
            neon_health.get("status") == "healthy" and 
            qdrant_health.get("status") == "healthy"
        ) else "unhealthy"
        
        health_report = {
            "overall_status": overall_status,
            "timestamp": neon_health.get("timestamp"),
            "databases": {
                "neon": neon_health,
                "qdrant": qdrant_health
            }
        }
        
        logger.info(f"Health check completed with overall status: {overall_status}")
        return health_report
    
    except Exception as e:
        logger.error(f"Error during health check: {e}")
        
        return {
            "overall_status": "error",
            "timestamp": None,
            "databases": {
                "neon": {"status": "error", "error": str(e)},
                "qdrant": {"status": "error", "error": str(e)}
            }
        }
    
    finally:
        # Ensure connections are closed
        await neon_db.close_connections()
        await qdrant_db.close_connections()


async def validate_environment_vars(required_vars: List[str] = None) -> Dict[str, bool]:
    """
    Validates that required environment variables are set.
    
    Args:
        required_vars: List of required environment variable names.
                      If None, defaults to the ones needed for the database layer.
                      
    Returns:
        Dictionary mapping variable names to whether they're set
    """
    if required_vars is None:
        required_vars = [
            "NEON_DATABASE_URL",
            "QDRANT_URL", 
            "QDRANT_API_KEY",
            "QDRANT_COLLECTION_NAME"
        ]
    
    validation_results = {}
    
    for var in required_vars:
        is_set = True
        try:
            import os
            if not os.getenv(var):
                is_set = False
        except Exception:
            is_set = False
        
        validation_results[var] = is_set
        logger.info(f"Environment variable {var}: {'SET' if is_set else 'NOT SET'}")
    
    all_set = all(validation_results.values())
    logger.info(f"Environment validation: {'PASSED' if all_set else 'FAILED'}")
    
    return validation_results


async def test_connection_without_querying(neon_db: NeonDB, qdrant_db: QdrantDB) -> bool:
    """
    Tests that the database objects can be created without actually executing database operations.
    
    This function verifies that the database connectors can be initialized and that
    connection pools/clients can be established without performing any operations
    that would consume resources or count against usage limits.
    
    Args:
        neon_db: NeonDB instance
        qdrant_db: QdrantDB instance
        
    Returns:
        Boolean indicating if connection setup is successful
    """
    try:
        # Test that we can get connection objects without executing queries
        await neon_db._get_pool()
        await qdrant_db._get_client()
        
        logger.info("Database connection setup validated without executing queries")
        return True
    except Exception as e:
        logger.error(f"Failed to set up database connections: {e}")
        return False