"""Default real-world wiring for the CLI and the ROS node: catalog, macros, places, nav_sim map."""
import json
import os

import yaml
from ament_index_python.packages import get_package_share_directory as share
from bt_common.catalog import discover_catalog, format_listing
from bt_common.macros import expand, load_macros, load_whitelist, spec_schema
from bt_slm.fallback import build_xml
from bt_validator.validate import validate_xml
from nav_sim.occupancy import OccupancyMap

from bt_rehearsal.sim import KinematicNavSim

FRAMEWORK_VERSION = '0.1.0'


def make_env(map_yaml='', places_yaml='', scene=None):
    fb = share('framework_bringup')
    cat, macros = discover_catalog(), load_macros()
    whitelist = load_whitelist()
    places = (yaml.safe_load(open(places_yaml or os.path.join(fb, 'param', 'places.yaml'))) or {}).get('places', {})
    sim = KinematicNavSim(OccupancyMap.from_yaml(map_yaml or os.path.join(fb, 'maps', 'test_room.yaml')),
                          {k: (v['x'], v['y']) for k, v in places.items()}, scene_summary=scene)
    ids = set(cat.all())
    return {
        'schema': spec_schema(list(cat.all().values()), macros),
        'hint': 'Valid places: ' + ', '.join(sorted(places)) + '.',
        'knowledge': open(os.path.join(fb, 'param', 'llm_knowledge.md')).read(),
        'catalog': cat, 'sim': sim, 'places': places,
        'listing': format_listing(cat.all().values(), with_desc=True) + '\nPlaces: ' + ', '.join(sorted(places)),
        'build': lambda spec: build_xml(json.dumps(expand(json.loads(spec), macros, whitelist)), ids),
        'validate': lambda xml: validate_xml(xml, cat),
    }
