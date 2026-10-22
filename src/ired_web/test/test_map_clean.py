import numpy as np

from ired_web.map_clean import FREE, OCC, UNKNOWN, clean

RES = 0.05
META = {'occupied_thresh': 0.65, 'free_thresh': 0.25, 'negate': 0}


def field_map():
    """7 x 7 m map, field frame (origin -1, -1): 5 x 5 m walled field, a ray outside,
    a speck and a hole inside."""
    n = 140
    img = np.full((n, n), UNKNOWN, np.uint8)

    def rc(x, y):                      # field metres -> row, col
        return n - 1 - int((y + 1) / RES), int((x + 1) / RES)
    for x in np.arange(0.0, 5.0, RES / 2):
        for y in np.arange(0.0, 5.0, RES / 2):
            img[rc(x, y)] = FREE
    for t in np.arange(-0.05, 5.05, RES / 2):
        for w in (-0.05, 5.0):
            img[rc(t, w)] = OCC
            img[rc(w, t)] = OCC
    for x in np.arange(5.1, 5.9, RES / 2):            # laser ray outside the field
        img[rc(x, 2.5)] = FREE
    img[rc(2.5, 2.5)] = OCC                          # speck in the middle
    img[rc(1.5, 3.5)] = UNKNOWN                      # hole
    img[rc(1.55, 3.5)] = UNKNOWN
    return img, rc


def test_clean_removes_outside_specks_and_holes_keeps_walls():
    img, rc = field_map()
    out, rep = clean(img, RES, (-1.0, -1.0, 0.0), META, hint=(0.0, 0.0, 0.0))
    assert rep['field'] == 'walls fitted'
    assert out[rc(5.5, 2.5)] == UNKNOWN              # ray gone
    assert out[rc(2.5, 2.5)] == FREE                 # speck gone
    assert out[rc(1.5, 3.5)] == FREE                 # hole filled
    walls = img == OCC
    walls[rc(2.5, 2.5)] = False
    assert (out[walls] == OCC).all()                 # every wall cell kept
    assert rep['speck_cells_removed'] == 1 and rep['hole_cells_filled'] == len({rc(1.5, 3.5), rc(1.55, 3.5)})


def test_original_array_untouched():
    img, _ = field_map()
    before = img.copy()
    clean(img, RES, (-1.0, -1.0, 0.0), META, hint=(0.0, 0.0, 0.0))
    assert (img == before).all()
