#!/usr/bin/env python3
"""
Test script to verify that database modules can be imported without errors.
"""
import sys
import os

# Add the backend directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def test_imports():
    """Test that all database modules can be imported without errors."""
    print("Testing database module imports...")
    
    try:
        print("  - Importing NeonDB...")
        from app.db.neon import NeonDB
        print("  [OK] NeonDB imported successfully")

        print("  - Importing QdrantDB...")
        from app.db.qdrant import QdrantDB
        print("  [OK] QdrantDB imported successfully")

        print("  - Importing utils...")
        from app.db.utils import validate_connections, get_health_status
        print("  [OK] utils imported successfully")

        print("  - Importing config...")
        from app.core.config import settings
        print("  [OK] config imported successfully")

        print("\n[OK] All database modules imported successfully!")
        return True

    except ImportError as e:
        print(f"  [ERROR] Import error: {e}")
        return False
    except Exception as e:
        print(f"  [ERROR] Unexpected error: {e}")
        return False

def test_instantiation():
    """Test that database objects can be instantiated."""
    print("\nTesting database object instantiation...")

    try:
        # Import again to ensure they're available in this function
        from app.db.neon import NeonDB
        from app.db.qdrant import QdrantDB

        print("  - Creating NeonDB instance...")
        neon_db = NeonDB()
        print("  [OK] NeonDB instance created successfully")

        print("  - Creating QdrantDB instance...")
        qdrant_db = QdrantDB()
        print("  [OK] QdrantDB instance created successfully")

        print("\n[OK] All database objects instantiated successfully!")
        return True

    except Exception as e:
        print(f"  [ERROR] Instantiation error: {e}")
        return False

if __name__ == "__main__":
    print("Running database module tests...\n")

    imports_ok = test_imports()
    instantiation_ok = test_instantiation()

    if imports_ok and instantiation_ok:
        print("\n[OK] All tests passed! Database modules are ready.")
        sys.exit(0)
    else:
        print("\n[ERROR] Some tests failed!")
        sys.exit(1)