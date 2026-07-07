# Standalone verification tests

These compile with plain g++ (no ROS2 needed) and verify the two pieces of
logic most worth checking independently: the UM982 NMEA parser and the mapping
geometry (Mercator projection + deskew transform invariants).

```bash
# NMEA parser
g++ -std=c++17 -I ../ws/src/um982_driver/include \
    test_nmea_parser_standalone.cpp \
    ../ws/src/um982_driver/src/nmea_parser.cpp -o /tmp/nmea_test && /tmp/nmea_test

# Geometry (needs Eigen headers on the include path)
g++ -std=c++17 -I /usr/include/eigen3 test_geometry_standalone.cpp -o /tmp/geom_test && /tmp/geom_test
```

The canonical gtest suite lives in `ws/src/um982_driver/test/` and runs under
`colcon test`.
