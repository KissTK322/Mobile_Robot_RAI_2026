"""Geometry and save/preview contract for fine map adjustment."""
import io
import math
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from ired_web.mapping import MapStore
from ired_web import field_align


@pytest.fixture
def store(tmp_path):
    img = np.full((160, 160), 254, np.uint8)
    # An asymmetric landmark at (3.5, 3.5), sampled at 5 cm.
    img[50:70, 110:130] = 0
    field_align.save_map(img, .05, (-1., -1., 0.), str(tmp_path), 'source', {})
    return MapStore(str(tmp_path), None, str(tmp_path))


@pytest.mark.parametrize('angle', [-1., -.5, -.2, .2, .5, 1.])
def test_landmark_rotates_about_field_centre(angle):
    res = .005
    img = np.full((1000, 1000), 254, np.uint8)
    img[290:310, 690:710] = 0  # centre (3.5, 3.5)
    out, origin = MapStore._nudge_image(img, res, (0., 0., 0.), .02, -.01, angle)
    actual = field_align.cell_centres(out == 0, res, origin).mean(0)
    a = math.radians(angle)
    expected = [2.5 + math.cos(a) - math.sin(a) + .02,
                2.5 + math.sin(a) + math.cos(a) - .01]
    np.testing.assert_allclose(actual, expected, atol=res)


def test_combined_save_is_one_copy_and_matches_preview(store):
    before = {p.name: p.read_bytes() for p in Path(store.src_dir).glob('*') if p.is_file()}
    preview = store.preview_png('source', dx=.02, dy=-.05, dyaw_deg=-.5)
    result = store.nudge('source', .02, -.05, -.5)
    assert result['name'] == 'source-adj'
    assert store.preview_png('source-adj') == preview
    for name, content in before.items():
        assert (Path(store.src_dir) / name).read_bytes() == content
    names = {p.name for p in Path(store.src_dir).glob('source-adj.*')}
    assert names == {'source-adj.yaml', 'source-adj.pgm'}
    assert store.nudge('source', 0, 0, .2)['name'] == 'source-adj-2'


def test_shift_only_preserves_pixels(store):
    store.nudge('source', .02, -.01)
    original, res, origin, _ = field_align.load_map(store.src_dir + '/source.yaml')
    saved, _, shifted, _ = field_align.load_map(store.src_dir + '/source-adj.yaml')
    np.testing.assert_array_equal(original, saved)
    assert shifted == (-.98, -1.01, 0.)


@pytest.mark.parametrize('values', [(float('nan'), 0, 0), (0, float('inf'), 0), (0, 0, float('nan')), (.51, 0, 0), (0, 0, 10.1)])
def test_rejects_invalid_adjustment_without_writing(store, values):
    with pytest.raises(ValueError):
        store.nudge('source', *values)
    assert len(store.list()) == 1


def test_start_reference_is_preview_only(store):
    preview = np.array(Image.open(io.BytesIO(store.preview_png('source'))))
    assert ((preview == [225, 95, 0]).all(2)).sum() > 100
    store.nudge('source', 0, 0, -.5)
    img, _, _, _ = field_align.load_map(store.src_dir + '/source-adj.yaml')
    assert set(np.unique(img)) <= {0, 205, 254}


def test_existing_yaml_yaw_is_respected():
    img = np.full((100, 100), 254, np.uint8)
    img[19:21, 69:71] = 0
    origin = (1., 2., .1)
    out, new_origin = MapStore._nudge_image(img, .01, origin, 0., 0., -.5)
    actual = field_align.cell_centres(out == 0, .01, new_origin).mean(0)
    # Original pixel local point (.7, .8), first YAML yaw then fine rotation.
    a, b = .1, math.radians(-.5)
    x, y = 1 + .7*math.cos(a)-.8*math.sin(a), 2 + .7*math.sin(a)+.8*math.cos(a)
    expected = [2.5+(x-2.5)*math.cos(b)-(y-2.5)*math.sin(b),
                2.5+(x-2.5)*math.sin(b)+(y-2.5)*math.cos(b)]
    np.testing.assert_allclose(actual, expected, atol=.015)


def test_real_map_origin_preview_matches_saved_copy(store):
    path = Path(store.src_dir) / 'source.yaml'
    import yaml
    meta = yaml.safe_load(path.read_text())
    meta['origin'] = [-.9925, -.7724, 0.]
    path.write_text(yaml.safe_dump(meta))
    before = store.preview_png('source', dx=.02, dy=-.01, dyaw_deg=-.5)
    store.nudge('source', .02, -.01, -.5)
    assert store.preview_png('source-adj') == before
