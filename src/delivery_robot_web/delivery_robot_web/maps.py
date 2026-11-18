"""Map import/rendering. Map identity is immutable; waypoints reference a version."""
from __future__ import annotations
import hashlib
import io
import json
import math
import shutil
import time
import uuid
from array import array
from pathlib import Path
import yaml
from PIL import Image
from .core import Store, UserError, validate_name

MAX_CELLS = 16_000_000


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def load_yaml_map(path: Path, allowed_root: Path):
    path = path.resolve()
    if not path.is_relative_to(allowed_root.resolve()) or path.suffix.lower() not in ('.yaml', '.yml'):
        raise UserError('Bản đồ nằm ngoài thư mục cho phép.')
    if path.stat().st_size > 64 * 1024:
        raise UserError('YAML bản đồ quá lớn.')
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or not all(k in data for k in ('image', 'resolution', 'origin')):
        raise UserError('Đây không phải YAML bản đồ; cần image, resolution, origin.')
    try:
        res = float(data['resolution'])
        origin = [float(v) for v in data['origin']]
        free, occupied = float(data.get('free_thresh', 0.25)), float(data.get('occupied_thresh', 0.65))
        negate = int(data.get('negate', 0))
    except (TypeError, ValueError):
        raise UserError('Metadata bản đồ không hợp lệ.')
    if res <= 0 or not math.isfinite(res) or len(origin) != 3 or not all(map(math.isfinite, origin)):
        raise UserError('Độ phân giải hoặc origin không hợp lệ.')
    if not 0 <= free < occupied <= 1 or negate not in (0, 1):
        raise UserError('Ngưỡng occupancy không hợp lệ.')
    if data.get('mode', 'trinary') not in ('trinary', 'scale'):
        raise UserError('Map V4 hỗ trợ map trinary/scale; chưa hỗ trợ mode raw.')
    image = (path.parent / str(data['image'])).resolve()
    if not image.is_relative_to(allowed_root.resolve()) or image.suffix.lower() not in ('.pgm', '.png', '.bmp'):
        raise UserError('Ảnh map nằm ngoài thư mục hoặc không phải PGM/PNG/BMP.')
    with Image.open(image) as im:
        if im.width * im.height > MAX_CELLS:
            raise UserError('Bản đồ vượt giới hạn 16 triệu ô của V1.')
        width, height = im.size
        im.verify()
    meta = {'resolution': res, 'origin': origin, 'width': width, 'height': height,
            'frame_id': 'map', 'free_thresh': free, 'occupied_thresh': occupied,
            'negate': negate, 'mode': data.get('mode', 'trinary')}
    return data, image, meta


def import_candidates(root: Path):
    root = root.expanduser().resolve()
    if not root.exists():
        return []
    result = []
    for path in sorted(root.rglob('*.yaml'))[:500]:
        try:
            _, _, meta = load_yaml_map(path, root)
            result.append({'key': hashlib.sha256(str(path).encode()).hexdigest(),
                           'name': path.stem, 'path': str(path), 'meta': meta})
        except (OSError, UserError, ValueError, yaml.YAMLError):
            continue
    return result



def _map_occupancy_values(data: dict, image_path: Path):
    """Interpret a saved map image with the same YAML thresholds used by map_server.

    Returned values follow nav_msgs/OccupancyGrid semantics: -1 unknown, 0 free,
    1..99 scaled probability (scale mode), 100 occupied. Image order remains
    top-left-first because saved map images are already written for display.
    """
    free = float(data.get('free_thresh', 0.25))
    occupied = float(data.get('occupied_thresh', 0.65))
    negate = int(data.get('negate', 0))
    mode = data.get('mode', 'trinary')
    with Image.open(image_path) as source:
        gray = source.convert('L')
        width, height = gray.size
        raw = gray.tobytes()
    values = array('b')
    span = occupied - free
    for shade in raw:
        probability = (shade / 255.0) if negate else (1.0 - shade / 255.0)
        if probability > occupied:
            value = 100
        elif probability < free:
            value = 0
        elif mode == 'scale':
            value = max(0, min(100, int(round(100.0 * (probability - free) / span))))
        else:
            value = -1
        values.append(value)
    return width, height, values


def _occupancy_png(width: int, height: int, values):
    pixels = bytes(205 if value < 0 else max(0, min(254, round(254 * (1 - value / 100))))
                   for value in values)
    image = Image.frombytes('L', (width, height), pixels)
    out = io.BytesIO()
    image.save(out, format='PNG', optimize=True)
    return out.getvalue()


def _clean_display_values(width: int, height: int, values):
    """Very conservative visual-only speckle cleanup.

    This never changes map.yaml or the image used by AMCL/Nav2. It only removes
    single-pixel salt/pepper artifacts when the 8-neighbourhood strongly agrees.
    Thin walls with neighbouring occupied pixels are intentionally preserved.
    """
    if width * height > 2_000_000 or width < 3 or height < 3:
        return array('b', values), 0
    src = array('b', values)
    dst = array('b', values)
    changed = 0
    for row in range(1, height - 1):
        base = row * width
        for col in range(1, width - 1):
            idx = base + col
            value = src[idx]
            neighbors = (
                src[idx-width-1], src[idx-width], src[idx-width+1],
                src[idx-1], src[idx+1],
                src[idx+width-1], src[idx+width], src[idx+width+1],
            )
            occupied_n = sum(v >= 90 for v in neighbors)
            free_n = sum(v == 0 for v in neighbors)
            unknown_n = sum(v < 0 for v in neighbors)
            new_value = value
            # Remove only a truly isolated strong occupied speck surrounded by
            # confirmed free space. Do not erase near unknown boundaries.
            if value >= 90 and occupied_n <= 1 and free_n >= 6 and unknown_n == 0:
                new_value = 0
            # Fill a one-cell white pinhole inside an otherwise solid obstacle.
            elif value == 0 and occupied_n >= 7:
                new_value = 100
            # Resolve a one-cell unknown pinhole only when its neighbourhood is
            # overwhelmingly known and homogeneous.
            elif value < 0 and free_n >= 7:
                new_value = 0
            elif value < 0 and occupied_n >= 7:
                new_value = 100
            if new_value != value:
                dst[idx] = new_value
                changed += 1
    return dst, changed


def build_display_images(directory: Path):
    """Build immutable display derivatives without modifying the navigation map."""
    yaml_path = directory / 'map.yaml'
    data, image_path, meta = load_yaml_map(yaml_path, directory)
    width, height, values = _map_occupancy_values(data, image_path)
    technical = _occupancy_png(width, height, values)
    clean_values, changed = _clean_display_values(width, height, values)
    clean = _occupancy_png(width, height, clean_values)
    (directory / 'map_technical.png').write_bytes(technical)
    (directory / 'map_clean.png').write_bytes(clean)
    # Backward-compatible preview defaults to the simplified operator view.
    (directory / 'preview.png').write_bytes(clean)
    (directory / 'display_meta.json').write_text(json.dumps({
        'version': 4,
        'technical_source': 'map.yaml interpreted with occupancy thresholds',
        'clean_visual_only': True,
        'changed_cells': changed,
        'total_cells': width * height,
        'navigation_map_modified': False,
    }, ensure_ascii=False, indent=2))
    return meta


def ensure_display_images(store: Store, ident: str):
    record = store.map(ident)
    directory = store.root / 'maps' / ident
    required = (directory / 'map_technical.png', directory / 'map_clean.png', directory / 'display_meta.json')
    if not all(path.is_file() for path in required):
        build_display_images(directory)
    try:
        display_meta = json.loads((directory / 'display_meta.json').read_text())
    except Exception:
        display_meta = {'version': 4, 'clean_visual_only': True, 'navigation_map_modified': False}
    return record, directory, display_meta

def register_map(store: Store, directory: Path, name: str, source: str, ident=None):
    ident = ident or uuid.uuid4().hex
    yaml_path = directory / 'map.yaml'
    data, img, meta = load_yaml_map(yaml_path, directory)
    digest = hashlib.sha256(json.dumps(meta, sort_keys=True).encode() + img.read_bytes()).hexdigest()
    build_display_images(directory)
    store.add_map(ident, name, meta, digest, source)
    return store.map(ident)


def import_existing(store: Store, key: str, name: str):
    name = validate_name(name)
    root = Path(store.config['import_root']).expanduser().resolve()
    candidate = next((v for v in import_candidates(root) if v['key'] == key), None)
    if not candidate:
        raise UserError('Không tìm thấy map trong thư mục đã cấu hình.')
    src = Path(candidate['path'])
    data, image, _ = load_yaml_map(src, root)
    ident = uuid.uuid4().hex
    target = store.root / 'maps' / ident
    target.mkdir(mode=0o700)
    try:
        image_name = 'map' + image.suffix.lower()
        shutil.copyfile(image, target / image_name)
        data['image'] = image_name
        (target / 'map.yaml').write_text(yaml.safe_dump(data, sort_keys=False))
        result = register_map(store, target, name, str(src), ident)
        store.audit('map_import', ident)
        return result
    except Exception:
        shutil.rmtree(target, ignore_errors=True)
        raise


def map_pixel(meta, x: float, y: float):
    ox, oy, angle = meta['origin']
    dx, dy = x - ox, y - oy
    gx = (math.cos(angle)*dx + math.sin(angle)*dy) / meta['resolution']
    gy = (-math.sin(angle)*dx + math.cos(angle)*dy) / meta['resolution']
    return int(math.floor(gx)), meta['height'] - 1 - int(math.floor(gy))


def validate_point_cell(store: Store, ident, x, y):
    record = store.map(ident)
    col, row = map_pixel(record['meta'], x, y)
    m = record['meta']
    if not 0 <= col < m['width'] or not 0 <= row < m['height']:
        raise UserError('Điểm nằm ngoài ảnh bản đồ.')
    path = store.root / 'maps' / ident / 'map.yaml'
    _, image, _ = load_yaml_map(path, path.parent)
    with Image.open(image) as im:
        shade = im.convert('L').getpixel((col, row)) / 255.0
    occupancy = shade if m['negate'] else 1.0 - shade
    # Conservative admission: canonical gray unknown (205) is never a goal,
    # even when a legacy YAML free threshold would otherwise classify it free.
    if occupancy >= min(m['free_thresh'], 0.10):
        raise UserError('Ô chọn chưa được xác nhận là vùng trống. Chọn ô trống khác.')
    # A free cell is not a footprint/path feasibility certificate.


def live_map_image(msg):
    width, height = int(msg.info.width), int(msg.info.height)
    if width < 1 or height < 1 or width * height > MAX_CELLS or len(msg.data) != width*height:
        raise UserError('Kích thước dữ liệu map chưa hợp lệ.')
    pixels = bytes(205 if v < 0 else max(0, min(254, round(254 * (1 - v / 100)))) for v in msg.data)
    im = Image.frombytes('L', (width, height), pixels)
    im = im.transpose(Image.Transpose.FLIP_TOP_BOTTOM)
    output = io.BytesIO()
    im.save(output, format='PNG')
    return output.getvalue()


def live_map_clean_image(msg):
    """Visual-only cleaned rendering of a ROS OccupancyGrid.

    The OccupancyGrid message itself is never modified; SLAM/AMCL/Nav2 continue
    to use the original ROS data.
    """
    width, height = int(msg.info.width), int(msg.info.height)
    if width < 1 or height < 1 or width * height > MAX_CELLS or len(msg.data) != width * height:
        raise UserError('Kích thước dữ liệu map chưa hợp lệ.')
    values, _changed = _clean_display_values(width, height, array('b', msg.data))
    pixels = bytes(205 if v < 0 else max(0, min(254, round(254 * (1 - v / 100)))) for v in values)
    im = Image.frombytes('L', (width, height), pixels)
    im = im.transpose(Image.Transpose.FLIP_TOP_BOTTOM)
    output = io.BytesIO()
    im.save(output, format='PNG', optimize=True)
    return output.getvalue()


def live_meta(msg):
    return {'width': int(msg.info.width), 'height': int(msg.info.height),
            'resolution': float(msg.info.resolution),
            'origin': [msg.info.origin.position.x, msg.info.origin.position.y,
                       yaw_of(msg.info.origin.orientation)],
            'frame_id': msg.header.frame_id}
