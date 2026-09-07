"""Fail CI if a required regression suite silently disappears or is all skipped."""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

root = Path(sys.argv[1])
for package, pattern, minimum in (
        ('anello_ros_driver', '**/test_anello_decoding.gtest.xml', 60),
        ('anello_ros_driver', '**/test_ros_regressions.xunit.xml', 20),
        ('ntrip_client', '**/pytest.xml', 55)):
    files = list((root / package).glob(pattern))
    if not files:
        raise SystemExit('Missing test results: ' + package)
    executed = sum(1 for path in files for case in ET.parse(path).iter('testcase')
                   if case.find('skipped') is None)
    if executed < minimum:
        raise SystemExit('{}: only {} tests executed'.format(package, executed))
    print('{}: {} tests executed'.format(package, executed))
