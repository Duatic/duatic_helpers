# A highlight plugin that the HighlightedRobotModel display can load.

# Name to use in `highlight.enabled`: <package>/<Class>
string name

# Description from the plugin description file
string description

# Priority of the highlight (the parameter `highlight.<name>.priority`, higher is applied later). 0 if not loaded.
int32 priority

# Position in the order the loaded highlights are applied in, from 0. A higher rank is applied later and so overrules
# the lower ones on a link. Equal priorities are ranked by position in `highlight.enabled` (later is higher).
# -1 if the highlight is not loaded.
int32 rank
