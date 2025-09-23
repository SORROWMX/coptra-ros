#!/usr/bin/env python3

from distutils.core import setup
import sys

print("Installing coptra package for backwards compatibility...")

setup(name='coptra',
      version='1.0',
      description='Coptra transitional package for backwards compatibility',
      author='Coptra',
      packages=['coptra'],
     )

print("✅ coptra package installed successfully!")
