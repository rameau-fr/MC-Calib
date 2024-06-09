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


# pylint: disable=unused-variable
def prepare_figure(ax: Axes) -> None:
    ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    axis_equal_3d(ax)
    ax.set_xlabel("x (cm)", fontsize=20)
    ax.set_ylabel("y (cm)", fontsize=20)
    ax.set_zlabel("z (cm)", fontsize=20)
