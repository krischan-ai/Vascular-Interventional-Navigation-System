from types import SimpleNamespace

import numpy as np

from cathsim.dm.observables import _decode_segmentation_buffer


def test_segmentation_decoder_treats_white_as_background():
    scene = SimpleNamespace(
        ngeom=1,
        geoms=[SimpleNamespace(segid=0, objid=7, objtype=5)],
    )
    rgb_buffer = np.array(
        [
            [[0, 0, 0], [255, 255, 255]],
            [[1, 0, 0], [255, 255, 255]],
        ],
        dtype=np.uint8,
    )

    decoded = _decode_segmentation_buffer(rgb_buffer, scene)

    assert decoded.shape == (2, 2, 2)
    assert tuple(decoded[0, 0]) == (7, 5)
    assert tuple(decoded[0, 1]) == (-1, -1)
    assert tuple(decoded[1, 0]) == (-1, -1)
