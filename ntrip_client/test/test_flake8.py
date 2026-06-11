"""Run flake8 over the package with the config in setup.cfg."""

import os

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    config = os.path.join(os.path.dirname(__file__), '..', 'setup.cfg')
    rc, errors = main_with_errors(argv=['--config', config])
    assert rc == 0, \
        'Found {} code style errors / warnings:\n'.format(len(errors)) + \
        '\n'.join(errors)
