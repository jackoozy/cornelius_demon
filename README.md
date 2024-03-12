# cornelius_demon

### To install all dependancies:
~~~Ruby
sudo apt update
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

cd ~/cornelius_ws # or whatever your workspace is named
rosdep install --from-paths src --ignore-src -r -y
~~~


## To run line detection test script:

~~~Ruby
rosrun selfie_drawing_robot test_line_detection

# pick a number to select a face
# a window should pop up. Once you close this window,
# another window will pop up showing the edges.
~~~
