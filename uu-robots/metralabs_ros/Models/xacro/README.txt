README

2011/Juni Felix Kolbe

Ordnung der Dateien, beginnend mit Kopfdatei, welche etwa auch an xacro-Skript übergeben wird. Etwas tricky ist bzgl. der xacro_script.sh im Oberordner, dass xacro-Datei und robot gleich heißen müssen.


Plattform:

    scitos_haw_only     => robot name = scitos_haw_only
        >scitos_haw_properties
    
    
Plattform mit Arm: 

    scitos_haw_schunk   => robot name = scitos_haw_schunk
        >scitos_haw_only
            >scitos_haw_properties
        >scitos_haw_schunk_links
        >scitos_haw_schunk_joints


xacro: 
rosrun xacro xacro.py model.xacro > model.urdf

http://www.ros.org/wiki/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File
