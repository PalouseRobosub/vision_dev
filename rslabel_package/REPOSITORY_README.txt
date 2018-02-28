To generate a distribution:
1) Modify the setup.py version to a new version.
2) Generate the distribution with
    setup.py sdist
3) Copy the dist/rslabel-$VERSION.tar.gz to the robosub server.
    rysnc -P dist/rslabel-$VERSION robosub.eecs.wsu.edu:/var/www/python_repo/rslabel/
4) Modify the user and group of the distribution on the server to www-data
    sudo chown www-data:www-data /var/www/python_repo/rslabel/rslabel-$VERSION.tar.gz
