# LSS NCAP Tool

Extract the complete ZIP to a writable folder. On Windows, double-click
`LSS_NCAP_Tool.exe`. On Ubuntu, open `LSS_NCAP_Tool` from your file manager
(enable executable launch if your file manager asks). Keep the `_internal`
folder next to the executable.

Use **Export CSV** or **Export SWA** in the application to choose a destination.
The save dialog opens in Documents when available, otherwise your home folder.

If startup fails, read `desktop.log` in `%LOCALAPPDATA%\LSS_NCAP_Tool\logs`
on Windows or `${XDG_STATE_HOME:-~/.local/state}/LSS_NCAP_Tool/logs` on Linux.
If that location is not writable, the tool uses the system temporary directory.
On Linux, a graphical desktop with Tk system libraries is required. No Python
installation is needed.
