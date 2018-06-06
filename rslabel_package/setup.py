#!/usr/bin/python

from setuptools import setup

setup(
    # Application name:
    name="rslabel",

    # Version number:
    version="0.9.1",

    # Application author details:
    author="Ryan Summers",
    author_email="summers.ryan.m@gmail.com",

    # Packages
    packages=["rslabel"],

    # Details
    url="http://robosub.eecs.wsu.edu/wiki/cs/vision/image_tagging/start#data-management",

    #
    # license="LICENSE.txt",
    description="Used for Palouse Robosub image labeling management.",

    long_description=open("README.txt").read(),

    scripts=['bin/rslabel'],

    # Dependent packages (distributions)
    install_requires=[
        "pysftp",
        "progressbar2",
    ],
)
