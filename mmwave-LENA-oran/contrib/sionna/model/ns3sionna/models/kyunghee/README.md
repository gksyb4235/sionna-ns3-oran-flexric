# Kyunghee Sionna RT scene

This directory contains the self-contained Kyunghee campus scene used by
`kyunghee_server.py`:

- `Kyunghee.xml`
- `meshes/` with the 92 PLY files referenced by the XML

The assets were copied from the local Sionna tutorial scene previously used by
the project. They are regular files rather than symbolic links, so the ns-3
and Sionna RT integration does not depend on another checkout at runtime.
