#!/usr/bin/env python3
"""
Check the actual table structure in the database
"""
from sqlalchemy import text
from src.database import engine

def check_table_structure():
    print("Checking table structure...")

    with engine.connect() as conn:
        # Get column information for the users table
        result = conn.execute(text("""
            SELECT column_name, data_type, is_nullable, column_default
            FROM information_schema.columns
            WHERE table_name = 'users'
            ORDER BY ordinal_position;
        """))

        print("Columns in 'users' table:")
        for row in result.fetchall():
            print(f"  {row[0]}: {row[1]}, nullable={row[2]}, default={row[3]}")

if __name__ == "__main__":
    check_table_structure()