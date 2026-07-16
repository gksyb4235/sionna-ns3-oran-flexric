try:
    import sionna.rt
    from importlib.metadata import version

    print("Package sionna-rt is installed")
    print(version("sionna-rt"))
except ImportError:
    print("Package sionna-rt is not installed")

try:
    import drjit as dr
    import mitsuba as mi

    print("Package mitsuba is installed")
    print(mi.__version__)
    print("Mitsuba variant:", mi.variant())
    print("CUDA backend:", dr.has_backend(dr.JitBackend.CUDA))
    print("LLVM backend:", dr.has_backend(dr.JitBackend.LLVM))
except ImportError:
    print("Package mitsuba or drjit is not installed")

try:
    import numpy as np
    print("Package numpy is installed")
    print(np.__version__)
except ImportError:
    print("Package numpy is not installed")

try:
    import zmq
    print("Package zmq is installed")
except ImportError:
    print("Package zmq is not installed")
