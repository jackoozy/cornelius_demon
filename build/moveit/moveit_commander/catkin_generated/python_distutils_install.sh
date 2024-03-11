#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/jackoozy/cornelius_demon_ws/src/moveit/moveit_commander"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/jackoozy/cornelius_demon_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jackoozy/cornelius_demon_ws/install/lib/python3/dist-packages:/home/jackoozy/cornelius_demon_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jackoozy/cornelius_demon_ws/build" \
    "/usr/bin/python3" \
    "/home/jackoozy/cornelius_demon_ws/src/moveit/moveit_commander/setup.py" \
    egg_info --egg-base /home/jackoozy/cornelius_demon_ws/build/moveit/moveit_commander \
    build --build-base "/home/jackoozy/cornelius_demon_ws/build/moveit/moveit_commander" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/jackoozy/cornelius_demon_ws/install" --install-scripts="/home/jackoozy/cornelius_demon_ws/install/bin"
