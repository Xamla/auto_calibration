package = "auto_calibration"
version = "scm-1"

source = {
   url = "",
}

description = {
   summary = "Automatic calculation of camera and hand-eye calibration",
   detailed = [[
   ]],
   homepage = "https://github.com/Xamla/auto_calibration",
   license = "GPLv2"
}

dependencies = {
   "torch >= 7.0"
}

build = {
   type = "command",
   build_command = [[
cmake -E make_directory build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$(PREFIX)" && $(MAKE)
]],
   install_command = "cd build && $(MAKE) install"
}
