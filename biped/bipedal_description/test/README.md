# Quality & Compliance Unit Tests (`bipedal_description/test`)

Contains unit tests for verifying Python code style compliance, docstring standards, and copyright headers across the package.

## Tests Included
- **`bipedal.py`**: Unit test module for basic functionality checks.
- **`test_copyright.py`**: Verifies `ament_copyright` license header compliance.
- **`test_flake8.py`**: Runs `flake8` static code analysis for PEP 8 styling.
- **`test_pep257.py`**: Runs `pep257` docstring convention validation.

## Running Tests
Run pytest / colcon test across the package:
```bash
colcon test --select-packages-select Bipedal_description
```
