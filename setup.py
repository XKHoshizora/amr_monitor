from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=find_packages('src'),
    package_dir={'': 'src'},
    package_data={
        'amr_monitor': [
            'config/*.yaml',
            'config/topics/*.yaml',
            'resources/ui/*.ui',
            'resources/ui/monitors/*.ui',
            'resources/themes/*.qss',
            'resources/icons/*.png',
        ]
    },
    install_requires=[
        'numpy>=1.20.0',
        'pandas>=1.3.0',
        'PySide6>=6.2.0',
        'pyqtgraph>=0.12.0',
        'pyyaml>=5.4.0',
        'scipy>=1.7.0',
        'psutil>=5.8.0',
        'h5py>=3.3.0',
        'rospkg>=1.3.0',
        'catkin_pkg>=0.4.24',
        'tf>=0.2.2',
        'rospy>=1.15.0',
    ],
    tests_require=[
        'pytest>=6.2.0',
        'pytest-qt>=4.0.0',
        'pytest-cov>=2.12.0',
        'mock>=4.0.0',
    ],
    test_suite='tests',
    python_requires='>=3.8',
)

setup(**d,
      entry_points={
          'console_scripts': [
              'amr_monitor = amr_monitor.scripts.amr_monitor_node:main'
          ]
      },
      classifiers=[
          'Development Status :: 4 - Beta',
          'Intended Audience :: Science/Research',
          'License :: OSI Approved :: MIT License',
          'Operating System :: POSIX :: Linux',
          'Programming Language :: Python :: 3.8',
          'Programming Language :: Python :: 3.9',
          'Topic :: Scientific/Engineering :: Robotics',
      ]
)
