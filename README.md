# test_for_bioptimgui

This repo is a sandbox for the code generation for acrobatics (somersaults, twists) in bioptim_gui.
  
  https://github.com/Erdnaxela3/bioptim_gui/commits/acrobatics_ui

ocp.py and trampo_quaternions.py are the base work to begin with.

gen.py will contain the script for a general template for any kind of acrobatics.

variables to take into account are (for now):
- chosen biorbd model
- number of somersaults
- number of half twists in each somersault
- phases time
- final time margin
- number of shooting points
- jump position (straight, tuck, pike)
