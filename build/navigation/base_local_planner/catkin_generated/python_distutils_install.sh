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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/robond/ROS_Udacity/src/navigation/base_local_planner"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/robond/ROS_Udacity/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/robond/ROS_Udacity/install/lib/python2.7/dist-packages:/home/robond/ROS_Udacity/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/robond/ROS_Udacity/build" \
    "/usr/bin/python" \
    "/home/robond/ROS_Udacity/src/navigation/base_local_planner/setup.py" \
    build --build-base "/home/robond/ROS_Udacity/build/navigation/base_local_planner" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/robond/ROS_Udacity/install" --install-scripts="/home/robond/ROS_Udacity/install/bin"
