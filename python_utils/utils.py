import numpy as np
from matplotlib.axes import Axes


def axis_equal_3d(ax: Axes) -> None:
    extents = np.array([getattr(ax, f"get_{dim}lim")() for dim in "xyz"])
    sz = extents[:, 1] - extents[:, 0]
    centers = np.mean(extents, axis=1)
    maxsize = max(abs(sz))
    r = maxsize / 2
    for ctr, dim in zip(centers, "xyz"):
        getattr(ax, f"set_{dim}lim")(ctr - r, ctr + r)
