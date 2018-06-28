Making A Custom Plugin
======================

Custom plugins should be ROS packages in the same workspace as the simulator package. The plugin's package.xml file should specify at least the following in order 
to resolve build order.

.. code-block:: xml

	<depend>sdsmt_simulator</depend>
	<depend>sdsmt_simulator_box2d</depend>

Once Ament is aware of package dependencies, the CMakeLists file must be set up to find the required libraries and files and build the plugin correctly.

First, there are a number of definitions and values that need to be set in order to compile and link the Qt-Specific portions of the plugin

.. code-block:: cmake

	find_package(Qt5 REQUIRED COMPONENTS
  	Core Gui
	)

	set(CMAKE_INCLUDE_CURRENT_DIR ON)
	set(CMAKE_AUTOMOC ON)

	add_definitions(-DQT_PLUGIN)
	add_definitions(-DQT_SHARED)

	include_directories( ${CMAKE_BINARY_DIR} )


In order to resolve dependencies for Box2D and the header files from the simulator, find\_package needs to be called for the associated packages. Then \lstinline|include_directories| needs to be called so the Qt MOC can resolve headers

.. code-block:: cmake

	find_package(sdsmt_simulator REQUIRED)
	find_package(sdsmt_simulator_box2d REQUIRED)
    
	ament_export_dependencies(
    		sdsmt_simulator
    		sdsmt_simulator_box2d
	)

	include_directories(${sdsmt_simulator_api_INCLUDE_DIRS})
	include_directories(${sdsmt_simulator_box2d_INCLUDE_DIRS})

Next, any headers in the plugin containing the Q_OBJECT or other Q_* macros need to be preprocessed by the MOC

.. code-block:: cmake

	set(plugin_moc_hdrs a.h b.h ... z.h)
	qt5_wrap_cpp(MOC_SRCS ${plugin_moc_hdrs})

Finally, the plugin needs to be built as a shared library and linked against Qt Core libraries and the Box2D library.

.. code-block:: cmake

	add_compile_options(-fPIC)

	add_library([plugin name] SHARED ${CPP_SRCS} ${MOC_SRCS})
	qt5_use_modules([plugin name] Core Gui)

	ament_target_dependencies([plugin name]
	"sdsmt_simulator_box2d"
	"sdsmt_simulator_api"
	)

The last detail is that the plugin must be deployed in the directory above the simulator executable. This can be achieved by installing the plugin to 
the lib directory of the workspace install

.. code-block:: cmake

	install(
		TARGETS [plugin name]
		DESTINATION lib
	)

A Note on ROS Communications
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is believed by the project team that, because the rclcpp::spin() method may be called from a thread other than the main one, all ROS message callbacks may be run 
from a non-main thread. Care should be taken when defining callbacks in components to prevent race conditions which would result from this design. This issue was 
resolved in the Touch Sensor Ring plugin with the use of a Qt Signal and Slot. When a Qt Signal triggers a Slot of an object which resides in a different thread, 
the slot is queued for the second thread to receive naturally during its event loop, preventing any race conditions. The Touch Sensor Ring component has an internal 
signal and slot specifically for this purpose; the signal is emitted by the ROS callback function, and that copies the data from the callback into the main thread 
where it can be processed safely.

