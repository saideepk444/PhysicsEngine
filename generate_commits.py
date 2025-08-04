#!/usr/bin/env python3
"""
Script to generate commits with backdated timestamps to fill out GitHub contribution graph.
This creates meaningful-looking commits that won't break your project.
"""

import os
import subprocess
import random
from datetime import datetime, timedelta
import json

def run_command(command, cwd=None):
    """Run a shell command and return the result."""
    try:
        result = subprocess.run(command, shell=True, capture_output=True, text=True, cwd=cwd)
        return result.returncode == 0, result.stdout, result.stderr
    except Exception as e:
        return False, "", str(e)

def get_random_commit_message():
    """Generate a random but realistic commit message."""
    messages = [
        "Update documentation",
        "Fix minor bugs",
        "Improve code formatting",
        "Add comments",
        "Refactor code structure",
        "Update README",
        "Fix typo in comments",
        "Optimize performance",
        "Clean up unused imports",
        "Update build configuration",
        "Add error handling",
        "Improve variable naming",
        "Update dependencies",
        "Fix compilation warnings",
        "Add unit tests",
        "Update license headers",
        "Improve code readability",
        "Fix memory leaks",
        "Add logging statements",
        "Update CMake configuration"
    ]
    return random.choice(messages)

def create_random_file_change():
    """Create a small random change to a file."""
    files_to_modify = [
        "README.md",
        "CMakeLists.txt",
        "src/main.cpp",
        "include/RigidBody.hpp"
    ]
    
    # Only modify files that exist
    existing_files = [f for f in files_to_modify if os.path.exists(f)]
    if not existing_files:
        return False
    
    file_to_modify = random.choice(existing_files)
    
    if file_to_modify == "README.md":
        # Add a small comment or update
        with open(file_to_modify, 'r') as f:
            content = f.read()
        
        # Add a small comment at the end
        comment = f"\n<!-- Updated on {datetime.now().strftime('%Y-%m-%d')} -->\n"
        with open(file_to_modify, 'a') as f:
            f.write(comment)
    
    elif file_to_modify == "CMakeLists.txt":
        # Add a small comment
        with open(file_to_modify, 'r') as f:
            content = f.read()
        
        # Add a comment at the end
        comment = f"\n# Updated on {datetime.now().strftime('%Y-%m-%d')}\n"
        with open(file_to_modify, 'a') as f:
            f.write(comment)
    
    elif file_to_modify.endswith(('.cpp', '.hpp')):
        # Add a small comment to C++ files
        with open(file_to_modify, 'r') as f:
            content = f.read()
        
        # Add a comment at the end
        comment = f"\n// Updated on {datetime.now().strftime('%Y-%m-%d')}\n"
        with open(file_to_modify, 'a') as f:
            f.write(comment)
    
    return True

def make_commit_with_date(date_str, commit_message):
    """Make a commit with a specific date."""
    # Set the environment variables for the commit date
    env = os.environ.copy()
    env['GIT_AUTHOR_DATE'] = date_str
    env['GIT_COMMITTER_DATE'] = date_str
    
    # Add all changes
    success, stdout, stderr = run_command("git add .")
    if not success:
        print(f"Error adding files: {stderr}")
        return False
    
    # Make the commit
    success, stdout, stderr = run_command(f'git commit -m "{commit_message}"', env=env)
    if not success:
        print(f"Error making commit: {stderr}")
        return False
    
    return True

def generate_commit_dates():
    """Generate dates for the past 12 months with varying frequency."""
    end_date = datetime.now()
    start_date = end_date - timedelta(days=365)
    
    dates = []
    current_date = start_date
    
    while current_date <= end_date:
        # Higher probability of commits on weekdays
        if current_date.weekday() < 5:  # Monday to Friday
            if random.random() < 0.3:  # 30% chance
                dates.append(current_date)
        else:  # Weekend
            if random.random() < 0.1:  # 10% chance
                dates.append(current_date)
        
        current_date += timedelta(days=1)
    
    return dates

def main():
    """Main function to generate commits."""
    print("🚀 Starting commit generation for GitHub contribution graph...")
    
    # Check if we're in a git repository
    success, stdout, stderr = run_command("git status")
    if not success:
        print("❌ Error: Not in a git repository or git not available")
        return
    
    # Generate dates for the past 12 months
    commit_dates = generate_commit_dates()
    print(f"📅 Generated {len(commit_dates)} commit dates")
    
    # Make commits for each date
    successful_commits = 0
    for i, date in enumerate(commit_dates):
        # Create a small change
        if create_random_file_change():
            # Format date for git
            date_str = date.strftime('%Y-%m-%d %H:%M:%S')
            commit_message = get_random_commit_message()
            
            # Make the commit
            if make_commit_with_date(date_str, commit_message):
                successful_commits += 1
                if successful_commits % 10 == 0:
                    print(f"✅ Made {successful_commits} commits...")
            else:
                print(f"❌ Failed to make commit for {date_str}")
    
    print(f"🎉 Successfully created {successful_commits} commits!")
    print("💡 To push to GitHub, run: git push origin main")
    print("⚠️  Note: This will create a lot of commits. Consider using --force-with-lease when pushing.")

if __name__ == "__main__":
    main() 