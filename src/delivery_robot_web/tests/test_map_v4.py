import json
from io import BytesIO
from pathlib import Path
from types import SimpleNamespace as NS

from PIL import Image

from delivery_robot_web.maps import (
    build_display_images,
    live_map_clean_image,
    live_map_image,
)


def make_yaml_map(root: Path, image: Image.Image, *, negate=0, mode='trinary'):
    root.mkdir(parents=True, exist_ok=True)
    image.save(root / 'map.pgm')
    (root / 'map.yaml').write_text(
        'image: map.pgm\n'
        'resolution: 0.05\n'
        'origin: [-1.0, -2.0, 0.0]\n'
        f'negate: {negate}\n'
        'free_thresh: 0.25\n'
        'occupied_thresh: 0.65\n'
        f'mode: {mode}\n'
    )


def test_map_v4_technical_interprets_yaml_like_occupancy(tmp_path):
    image = Image.new('L', (3, 1))
    image.putdata([0, 128, 255])
    make_yaml_map(tmp_path, image)
    build_display_images(tmp_path)
    technical = Image.open(tmp_path / 'map_technical.png').convert('L')
    # black=occupied, mid-threshold=unknown, white=free
    assert list(technical.tobytes()) == [0, 205, 254]


def test_map_v4_honors_negate(tmp_path):
    image = Image.new('L', (2, 1))
    image.putdata([0, 255])
    make_yaml_map(tmp_path, image, negate=1)
    build_display_images(tmp_path)
    technical = Image.open(tmp_path / 'map_technical.png').convert('L')
    assert list(technical.tobytes()) == [254, 0]


def test_map_v4_clean_removes_only_visual_single_speck(tmp_path):
    image = Image.new('L', (5, 5), 255)
    image.putpixel((2, 2), 0)
    make_yaml_map(tmp_path, image)
    original = (tmp_path / 'map.pgm').read_bytes()
    build_display_images(tmp_path)
    technical = Image.open(tmp_path / 'map_technical.png').convert('L')
    clean = Image.open(tmp_path / 'map_clean.png').convert('L')
    assert technical.getpixel((2, 2)) == 0
    assert clean.getpixel((2, 2)) == 254
    assert (tmp_path / 'map.pgm').read_bytes() == original
    meta = json.loads((tmp_path / 'display_meta.json').read_text())
    assert meta['changed_cells'] == 1
    assert meta['navigation_map_modified'] is False


def test_map_v4_clean_preserves_thin_wall(tmp_path):
    image = Image.new('L', (7, 5), 255)
    for x in range(1, 6):
        image.putpixel((x, 2), 0)
    make_yaml_map(tmp_path, image)
    build_display_images(tmp_path)
    clean = Image.open(tmp_path / 'map_clean.png').convert('L')
    # Interior cells of a one-pixel wall have occupied neighbours and remain.
    assert [clean.getpixel((x, 2)) for x in range(2, 5)] == [0, 0, 0]


def test_live_clean_view_never_mutates_ros_grid():
    data = [0] * 25
    data[12] = 100
    msg = NS(info=NS(width=5, height=5), data=list(data))
    original = list(msg.data)
    technical = Image.open(BytesIO(live_map_image(msg))).convert('L')
    clean = Image.open(BytesIO(live_map_clean_image(msg))).convert('L')
    assert 0 in technical.tobytes()
    assert 0 not in clean.tobytes()
    assert msg.data == original



def test_map_v4_scale_interpolation_matches_nav2_gradient_examples(tmp_path):
    image = Image.new('L', (4, 1))
    image.putdata([90, 147, 204, 205])
    tmp_path.mkdir(parents=True, exist_ok=True)
    image.save(tmp_path / 'map.pgm')
    (tmp_path / 'map.yaml').write_text(
        'image: map.pgm\nresolution: 0.05\norigin: [0,0,0]\nnegate: 0\n'
        'free_thresh: 0.196\noccupied_thresh: 0.65\nmode: scale\n'
    )
    build_display_images(tmp_path)
    technical = Image.open(tmp_path / 'map_technical.png').convert('L')
    # Occupancy values 99, 50, 1, 0 rendered with the same grayscale rule as /map.
    assert list(technical.tobytes()) == [3, 127, 251, 254]

def test_map_v4_ui_exposes_clean_and_technical_modes():
    root = Path(__file__).resolve().parents[1] / 'delivery_robot_web' / 'static'
    html = (root / 'index.html').read_text()
    js = (root / 'app.js').read_text()
    for token in ('id="liveMapMode"', 'id="localizationMapMode"', 'id="showRobotLoc"', 'id="mapRenderSource"'):
        assert token in html
    for token in ('/api/map/live-clean.png', '/api/map/ros', '/view/technical.png', '/view/clean.png'):
        assert token in js


def test_ros_map_observation_can_be_cleared_without_touching_slam_cache():
    import threading
    from delivery_robot_web.ros_adapter import RosAdapter
    r = RosAdapter.__new__(RosAdapter)
    r.lock = threading.RLock()
    r.map_capture_enabled = False
    r.map_msg = 'slam-cache'
    r.map_rev = 7
    r.map_at = 1.0
    r.ros_map_msg = 'old-static-map'
    r.ros_map_at = 2.0
    r.ros_map_rev = 3
    r.clear_ros_map_observation()
    rev, msg, age = r.current_ros_map()
    assert rev == 4 and msg is None and age is None
    assert r.map_msg == 'slam-cache' and r.map_rev == 7
