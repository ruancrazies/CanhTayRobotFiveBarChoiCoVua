// DECOY: stands in for a header named protocol.h that lives on the SDK include path of the real ESP32 build
// (PlatformIO appends the project's -I flags AFTER the framework's include directories, so such a file can
// win a bare #include "protocol.h"). If the compiler reads this file, a project header reached the wrong
// protocol.h and the build on the real core would break. Used only by run_host_tests.sh and
// tests/test_firmware_headers.py; never part of a PlatformIO build.
#error "wrong protocol.h: an SDK-style header shadowed the project's firmware/include/protocol.h; include the project header by a path relative to the including file"
