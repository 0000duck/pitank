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

echo_and_run cd "/home/sergio/Downloads/3pi_gaming/src/qt_ros/qt_create"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/sergio/Downloads/3pi_gaming/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/sergio/Downloads/3pi_gaming/install/lib/python2.7/dist-packages:/home/sergio/Downloads/3pi_gaming/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/sergio/Downloads/3pi_gaming/build" \
    "/usr/bin/python" \
    "/home/sergio/Downloads/3pi_gaming/src/qt_ros/qt_create/setup.py" \
    build --build-base "/home/sergio/Downloads/3pi_gaming/build/qt_ros/qt_create" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/sergio/Downloads/3pi_gaming/install" --install-scripts="/home/sergio/Downloads/3pi_gaming/install/bin"
