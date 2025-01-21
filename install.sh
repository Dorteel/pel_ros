#!/bin/bash

# Define variables for directories and repositories
WORKSPACE_DIR=~/ros_workspace
SRC_DIR=$WORKSPACE_DIR/src
REPOS=(
    "https://github.com/example_repo1.git"
    "https://github.com/example_repo2.git"
    # Add more repositories as needed
)

# Helper function to print messages
print_message() {
    echo -e "\n========== $1 ==========\n"
}

# Step 1: Create workspace and src directory
print_message "Creating ROS workspace and src directory"
if [ ! -d "$WORKSPACE_DIR" ]; then
    mkdir -p "$SRC_DIR"
    print_message "Workspace created at $WORKSPACE_DIR"
else
    print_message "Workspace already exists at $WORKSPACE_DIR"
fi

# Step 2: Clone GitHub repositories into src directory
print_message "Cloning GitHub repositories"
cd "$SRC_DIR" || exit
for REPO in "${REPOS[@]}"; do
    REPO_NAME=$(basename "$REPO" .git)
    if [ ! -d "$REPO_NAME" ]; then
        git clone "$REPO"
        print_message "Cloned $REPO_NAME"
    else
        print_message "$REPO_NAME already exists, skipping clone"
    fi
done

# Step 3: Install Python requirements from each repo
print_message "Installing Python requirements"
for REPO in "${REPOS[@]}"; do
    REPO_NAME=$(basename "$REPO" .git)
    REPO_DIR="$SRC_DIR/$REPO_NAME"
    if [ -f "$REPO_DIR/requirements.txt" ]; then
        print_message "Installing requirements for $REPO_NAME"
        pip install -r "$REPO_DIR/requirements.txt"
    else
        print_message "No requirements.txt found for $REPO_NAME"
    fi
done

# Step 4: Build the workspace using catkin_make
print_message "Running catkin_make"
cd "$WORKSPACE_DIR" || exit
catkin_make

if [ $? -eq 0 ]; then
    print_message "catkin_make completed successfully"
else
    print_message "catkin_make failed, check for errors"
    exit 1
fi

# Step 5: Source the setup.bash file
print_message "Sourcing setup.bash"
source "$WORKSPACE_DIR/devel/setup.bash"

print_message "Installation completed successfully!"
