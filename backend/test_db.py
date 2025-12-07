#!/usr/bin/env python3
"""
Test script to verify database connection and table creation
"""
from src.database import engine, Base
from sqlalchemy import text
from src.models.user import User, UserProfile  # noqa: F401

def test_db():
    print("Testing database connection and table creation...")

    try:
        # Test basic connection
        with engine.connect() as conn:
            result = conn.execute(text("SELECT 1"))
            print("✓ Basic connection works:", result.fetchone())

        print("Creating tables using Base.metadata.create_all()...")
        # Create tables using the Base metadata directly
        Base.metadata.create_all(bind=engine)
        print("✓ Tables created successfully using Base.metadata.create_all()")

        # Check all schemas first
        with engine.connect() as conn:
            result = conn.execute(text("""
                SELECT schema_name
                FROM information_schema.schemata
                ORDER BY schema_name;
            """))
            schemas = [row[0] for row in result.fetchall()]
            print(f"✓ Available schemas: {schemas}")

        # Check if users table exists in any schema
        with engine.connect() as conn:
            result = conn.execute(text("""
                SELECT table_schema, table_name
                FROM information_schema.tables
                WHERE table_name = 'users';
            """))
            tables = result.fetchall()
            if tables:
                print(f"✓ Found 'users' table in: {tables}")
            else:
                print("✗ 'users' table not found in any schema")

            # Check all tables
            result = conn.execute(text("""
                SELECT table_schema, table_name
                FROM information_schema.tables
                WHERE table_schema NOT IN ('information_schema', 'pg_catalog')
                ORDER BY table_schema, table_name;
            """))
            all_tables = result.fetchall()
            print(f"✓ All user tables: {all_tables}")

        # Try to create the table manually using raw SQL as a test
        print("\nTrying to create users table manually...")
        with engine.connect() as conn:
            trans = conn.begin()  # Begin transaction
            try:
                # Check if table exists first
                result = conn.execute(text("""
                    SELECT EXISTS (
                        SELECT FROM information_schema.tables
                        WHERE table_name = 'users'
                    );
                """))
                table_exists = result.fetchone()[0]

                if not table_exists:
                    print("Creating users table manually...")
                    conn.execute(text("""
                        CREATE TABLE users (
                            id SERIAL PRIMARY KEY,
                            email VARCHAR(255) UNIQUE NOT NULL,
                            name VARCHAR(255) NOT NULL,
                            password_hash VARCHAR(255) NOT NULL,
                            provider VARCHAR(50) DEFAULT 'local',
                            provider_id VARCHAR(255),
                            email_verified BOOLEAN DEFAULT FALSE,
                            is_active BOOLEAN DEFAULT TRUE,
                            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                            updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                        );
                    """))
                    conn.execute(text("""
                        CREATE TABLE user_profiles (
                            id SERIAL PRIMARY KEY,
                            user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,
                            technical_background TEXT,
                            hardware_access TEXT,
                            learning_goals TEXT,
                            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                            updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                        );
                    """))
                    trans.commit()  # Commit the transaction
                    print("✓ Manual table creation successful")
                else:
                    print("✓ Users table already exists")

                # Check again
                result = conn.execute(text("""
                    SELECT table_schema, table_name
                    FROM information_schema.tables
                    WHERE table_name = 'users';
                """))
                tables = result.fetchall()
                if tables:
                    print(f"✓ Found 'users' table after manual creation: {tables}")
                else:
                    print("✗ 'users' table still not found after manual creation")

            except Exception as e:
                trans.rollback()
                print(f"✗ Manual table creation failed: {e}")

    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_db()