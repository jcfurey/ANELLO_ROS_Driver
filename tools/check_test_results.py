"""Reject missing regression suites and missing, failed, or skipped driver lint checks."""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

root = Path(sys.argv[1])
for package, pattern, minimum in (
        ('anello_ros_driver', '**/test_anello_decoding.gtest.xml', 92),
        ('anello_ros_driver', '**/test_ros_regressions.xunit.xml', 49),
        ('ntrip_client', '**/pytest.xml', 70)):
    files = list((root / package).glob(pattern))
    if not files:
        raise SystemExit('Missing test results: ' + package)
    executed = sum(1 for path in files for case in ET.parse(path).iter('testcase')
                   if case.find('skipped') is None)
    if executed < minimum:
        raise SystemExit('{}: only {} tests executed'.format(package, executed))
    print('{}: {} tests executed'.format(package, executed))

for name in ('copyright', 'cpplint', 'uncrustify', 'cppcheck', 'flake8',
             'pep257', 'lint_cmake', 'xmllint'):
    files = list((root / 'anello_ros_driver').glob('**/' + name + '.xunit.xml'))
    if not files:
        raise SystemExit('Missing driver lint results: ' + name)
    cases = [case for path in files for case in ET.parse(path).iter('testcase')]
    if not cases or any(case.find('skipped') is not None or case.find('failure') is not None
                        or case.find('error') is not None for case in cases):
        raise SystemExit('Driver lint failed, skipped checks, or ran no checks: ' + name)
    print('{}: {} lint checks passed'.format(name, len(cases)))
