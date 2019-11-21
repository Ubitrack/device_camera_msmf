from conans import ConanFile, CMake, tools


class UbitrackCoreConan(ConanFile):
    name = "ubitrack_device_camera_msmf"
    version = "1.3.0"

    description = "Ubitrack Device Camera MSMF"
    url = "https://github.com/Ubitrack/device_camera_msmf.git"
    license = "GPL"

    short_paths = True
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake"

    options = { 
        "workspaceBuild" : [True, False],
    }

    default_options = {
        "ubitrack_core:shared" : True,
        "ubitrack_vision:shared" : True,
        "ubitrack_dataflow:shared" : True,
        "workspaceBuild" : False,
        }

    # all sources are deployed with the package
    exports_sources = "cmake/*", "doc/*", "src/*", "CMakeLists.txt"

    def requirements(self):
        userChannel = "ubitrack/stable"
        if self.options.workspaceBuild:
            userChannel = "local/dev"

        self.requires("ubitrack_core/%s@%s" % (self.version, userChannel))
        self.requires("ubitrack_dataflow/%s@%s" % (self.version, userChannel))
        self.requires("ubitrack_vision/%s@%s" % (self.version, userChannel))

    # def imports(self):
    #     self.copy(pattern="*.dll", dst="bin", src="bin") # From bin to bin
       
    def build(self):
        cmake = CMake(self)
        cmake.definitions['WITH_OPENCL'] = self.options['ubitrack_vision'].with_opencl
        cmake.configure()
        cmake.build()
        cmake.install()

    def package(self):
        pass

    def package_info(self):
        pass
