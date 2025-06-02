if (CONFIG_APP_KALMAN_FILTER)
	zephyr_include_directories(kalman_filter)
	zephyr_include_directories(helper_files)
	zephyr_include_directories(data)
	
	zephyr_library_sources(kalman_filter/kalman_filter.c)
	zephyr_library_sources(helper_files/helper_files.c)
	zephyr_library_sources(data/matrix_data.c)
endif()