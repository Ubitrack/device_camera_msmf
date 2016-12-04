Microsoft MediaFoundation Capture Device
==========
This is the device_camera_msmf Ubitrack capture driver.

Description
----------
The device_camera_msmf contains MS Windows specific mediafoundation framegrabber.

Usage
-----
In order to use it, you have to clone the buildenvironment, change to the ubitrack directory and add the device_camera_msmf by executing:

    git clone https://github.com/Ubitrack/device_camera_msmf.git modules/device_camera_msmf


Dependencies
----------

Since this component is MS Windows specific, the directshow can only be built on MS Windows (7-10).


License Note:
Driver code was copied from OpenCV 3.1 cap_msmf.*