Distributions are hosted on PyPI. Upload is completed using twine:
    `pip install twine`

Install python wheel to generate a wheel for easy installations.
    `pip install wheel`

To generate a distribution:
    1) Update the version number in setup.py
    2) Run `python setup.py sdist`
    3) Run `python setup.py bdist_wheel --universal`
    4) Run `twine upload dist/rslabel-{VERSION}.tar.gz
        - Username is PalouseRobosub. Password can be acquired from leadership.
    5) Run `twine upload dist/rslabel-{VERSION}-py2.py3-none-any.whl
        - Username is PalouseRobosub. Password can be acquired from leadership.
