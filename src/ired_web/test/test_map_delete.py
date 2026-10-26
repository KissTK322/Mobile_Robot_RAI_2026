import os

import pytest

from ired_web.mapping import MapStore


def make(tmp_path):
    src, share, state = tmp_path / 'src', tmp_path / 'share', tmp_path / 'state'
    for d in (src, share, state):
        d.mkdir()
    for n in ('Field', 'Field-clean', 'Field-clean-aligned', 'Other'):
        (src / f'{n}.yaml').write_text(f'image: {n}.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
        (src / f'{n}.pgm').write_bytes(b'P5 1 1 255\n\x00')
        os.symlink(src / f'{n}.yaml', share / f'{n}.yaml')
        os.symlink(src / f'{n}.pgm', share / f'{n}.pgm')
    (src / 'Field-clean.zones.json').write_text('{}')
    m = MapStore(str(src), str(share), str(state))
    m.select('Other')
    return m, src, share


def test_delete_copy_removes_its_files_and_links_only(tmp_path):
    m, src, share = make(tmp_path)
    r = m.delete('Field-clean')
    assert set(r['removed']) == {'Field-clean.yaml', 'Field-clean.pgm', 'Field-clean.zones.json',
                                 'share/Field-clean.yaml', 'share/Field-clean.pgm'}
    assert not (src / 'Field-clean.yaml').exists() and not os.path.lexists(share / 'Field-clean.pgm')
    assert (src / 'Field.pgm').exists() and (src / 'Field-clean-aligned.pgm').exists()


def test_active_map_cannot_be_deleted(tmp_path):
    m, src, _ = make(tmp_path)
    with pytest.raises(ValueError, match='Navigation uses'):
        m.delete('Other', original_ok=True)
    assert (src / 'Other.yaml').exists()


def test_original_needs_extra_confirmation(tmp_path):
    m, src, _ = make(tmp_path)
    with pytest.raises(ValueError, match='original'):
        m.delete('Field')
    assert (src / 'Field.yaml').exists()
    m.delete('Field', original_ok=True)
    assert not (src / 'Field.yaml').exists()


def test_image_of_another_map_is_never_removed(tmp_path):
    m, src, _ = make(tmp_path)
    (src / 'Field-clean-aligned.yaml').write_text('image: Field.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    m.delete('Field-clean-aligned')
    assert (src / 'Field.pgm').exists()


def test_unknown_or_bad_names(tmp_path):
    m, _, _ = make(tmp_path)
    for bad in ('Nope', '../Field', ''):
        with pytest.raises(ValueError):
            m.delete(bad, original_ok=True)


def test_is_original():
    assert MapStore.is_original('Titan-1') and MapStore.is_original('Smart-map')
    assert not MapStore.is_original('Smart-map-clean-2-aligned') and not MapStore.is_original('X-adj-3')
