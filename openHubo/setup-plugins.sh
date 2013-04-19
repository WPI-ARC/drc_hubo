#!/bin/bash

function verify-dep()
{
    local RESULT=`dpkg-query -l $1 | grep "ii"`
    local INSTALL_MISSING_DEP=''
    if [[ ${#RESULT} == 0 ]]
    then
        read -p "Install missing dependent package/library $1 [Y/n]?" INSTALL_MISSING_DEP
        if [[ $INSTALL_MISSING_DEP == 'n' || $INSTALL_MISSING_DEP == 'N' ]]
        then
            echo "WARNING: skipping dependent package/library $1..."
        else
            sudo apt-get install $1
        fi
    else
        echo "package/library $1 is installed"
    fi
}

#Bener Suay says: I had to replace the following line because this directory is not the git dir.
#BASE_DIR=`git rev-parse --show-toplevel`
BASE_DIR=`pwd`

echo ""
echo "Building OpenMR Servo Controller Plugin..."
echo ""

cd $BASE_DIR/openmr/
[ -d build ] || mkdir build
cd build
cmake ../src/
make
make install

echo ""
echo "Building OpenGRASP ForceSensor plugin..."
echo ""

cd $BASE_DIR/forceSensor
[ -d build ] || mkdir build
cd build
cmake ../
make
make install

echo ""
echo "Building CoMPS plugins..."
echo ""

#CoMPS prereqs and build steps

for dep in libqhull-dev libqhull5 libnewmat10-dev libnewmat10ldbl libboost-regex-dev
do
    verify-dep $dep
done

for pkg in generalik cbirrt2 manipulation2
do
    cd $BASE_DIR/comps-plugins/$pkg
    [ -d build ] || mkdir build
    cd build
    cmake ../
    make
    make install
done

cd $BASE_DIR

ENV_SOURCED_SEARCH=`grep $BASE_DIR/env.sh ~/.bashrc`

if [[ ${#ENV_SOURCED_SEARCH} == 0 ]]
then
    echo "source $BASE_DIR/env.sh" >> ~/.bashrc
fi
 
#Strip out base folder definition and update with current
sed "s,\( OPENHUBO_DIR=\),\1$BASE_DIR," .env.template > env.sh
echo ""
echo "OpenHubo base folder is $BASE_DIR"
source env.sh

PLUGINS_LIST=`openrave.py --listplugins`

for f in `ls plugins/*.so`
do
    PLUGIN_CHECK=`echo $PLUGINS_LIST | grep $f`
    if [[ ${#PLUGIN_CHECK} == 0 ]]
    then
        echo "$f NOT found by OpenRAVE, please check that the plugins dir is properly sourced."
    else
        echo "$f found by OpenRAVE"
    fi
done
