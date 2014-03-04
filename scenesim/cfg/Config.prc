## Custom Panda3d configuration variables.

enforce-attrib-lock 0
# Prevents Bullet memory leaks 
# (see: http://www.panda3d.org/forums/viewtopic.php?t=15645)
garbage-collect-states 0

# scenesim-specific variables
viewer-use-shaders #t
viewer-resize-window #t
viewer-set-cam-lens #t

bullet-filter-algorithm	groups-mask