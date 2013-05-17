#!/bin/bash
# written by Felix Kolbe

if [ ${1:-not} = "help" ]; then
  	echo "Usage:"
  	echo "converts xacro file and creates graph gv/pdf file"
  	echo "xacro_script.sh [<file>] [open]"
  	echo "<file>    set input instead of default file"
  	echo "open      open pdf after successfull creation"
  	exit
fi


name=scitos_haw_schunk  # default

# use parameter if given and not 'open'
if [ $# -ge 1 -a ${1:-not} != "open" ]; then
    name=${1:-$name}
fi

xacrofile=xacro/${name}.xacro

# check file readability
if [ ! -r $xacrofile ]; then
    echo "file $xacrofile for $name not found, aborting"
    exit
fi

# backup old pdf
if [ -f ${name}.pdf ]; then
	mv ${name}.pdf ${name}.old.pdf
fi

echo generating urdf from xacro..
rosrun xacro xacro.py $xacrofile > ${name}.urdf

echo checking urdf..
rosrun urdf_parser check_urdf ${name}.urdf

echo generating pdf from urdf..
rosrun urdf_parser urdf_to_graphiz ${name}.urdf

# open pdf if requested and possible
#echo ${#-1}
if [ ${1:-not} = "open" -o ${2:-not} = "open" ]; then
    if [  -f ${name}.pdf ]; then
	    echo showing pdf.
    	evince $name.pdf &
    else
    	echo no pdf to show!
    fi
fi

