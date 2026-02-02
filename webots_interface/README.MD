# Adding a new world
Move the .wbt files to the /worlds folder together with any additional resources that you might have.
Then the file needs to be added to the setup.py script, together with any additional resources.

Then build the package again:
```
colcon build
source install/setup.bash
```

# Adding a new robot