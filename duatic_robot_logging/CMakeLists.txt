cmake_minimum_required(VERSION 3.22)
project(duatic_robot_logging NONE)

# Relative to the /usr prefix, where systemd reads vendor configuration.
install(FILES systemd/50-duatic.conf DESTINATION lib/systemd/journald.conf.d)
install(FILES tmpfiles/duatic-logging.conf DESTINATION lib/tmpfiles.d)
