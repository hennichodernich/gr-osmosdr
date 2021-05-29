IF(NOT GNURADIO_IIO_FOUND)
  pkg_check_modules (GNURADIO_IIO_PKG gnuradio-iio)
  find_path(GNURADIO_IIO_INCLUDE_DIRS NAMES iio/api.h
    PATHS
    ${GNURADIO_IIO_PKG_INCLUDE_DIRS}
    /usr/local/include
    /usr/include
  )

  find_library(GNURADIO_IIO_LIBRARIES NAMES gnuradio-iio
    PATHS
    ${GNURADIO_IIO_PKG_LIBRARY_DIRS}
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
  )

if(GNURADIO_IIO_INCLUDE_DIRS AND GNURADIO_IIO_LIBRARIES)
  set(GNURADIO_IIO_FOUND TRUE CACHE INTERNAL "gnuradio-iio found")
  message(STATUS "Found gnuradio-iio: ${GNURADIO_IIO_INCLUDE_DIRS}, ${GNURADIO_IIO_LIBRARIES}")
else(GNURADIO_IIO_INCLUDE_DIRS AND GNURADIO_IIO_LIBRARIES)
  set(GNURADIO_IIO_FOUND FALSE CACHE INTERNAL "gnuradio-iio found")
  message(STATUS "gnuradio-iio not found.")
endif(GNURADIO_IIO_INCLUDE_DIRS AND GNURADIO_IIO_LIBRARIES)

mark_as_advanced(GNURADIO_IIO_LIBRARIES GNURADIO_IIO_INCLUDE_DIRS)

endif(NOT GNURADIO_IIO_FOUND)
