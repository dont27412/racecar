import os
import stat
from glob import glob

from setuptools import setup
from setuptools.command.install import install

package_name = 'turtlebot3_wp_following'
executable = 'waypoint_follower'
# All console entry points shipped by this package.  Each one has a matching
# wrapper in scripts/ that is installed into lib/<pkg>/ below so that
# ros2 run / ros2 launch can find it in the libexec directory.
executables = ['waypoint_follower', 'path_cacher', 'path_tracker']


class PostInstallFixup(install):
    """Ensure the lib/<pkg>/ wrappers exist and stay executable.

    ``ament_python`` packages normally create the libexec directory
    ``install/<pkg>/lib/<pkg>`` that ``ros2 run`` / ``ros2 launch`` need.
    Depending on the colcon/setuptools version this may not happen, so this
    hook both guarantees the scripts are executable and, if the libexec
    directory is missing, creates the wrappers.  Mirrors the working
    ``turtlebot3_ekf_nav`` package.
    """

    def _wrapper(self, name):
        return (
            '#!/usr/bin/env python3\n'
            '\n'
            f'from {package_name}.{name} import main\n'
            '\n'
            "if __name__ == '__main__':\n"
            '    main()\n'
        )

    def run(self):
        super().run()

        # 1) Make sure the bin/ scripts keep their executable bit.
        for name in executables:
            for base in (self.install_scripts or '', self.install_base or ''):
                if base:
                    bin_path = os.path.join(base, name)
                    if os.path.isfile(bin_path):
                        st = os.stat(bin_path)
                        os.chmod(bin_path,
                                 st.st_mode | stat.S_IXUSR | stat.S_IXGRP
                                 | stat.S_IXOTH)

        # 2) Guarantee the libexec wrappers exist for every executable.
        prefix = None
        if self.install_scripts:
            prefix = os.path.dirname(self.install_scripts)
        elif self.install_base:
            prefix = self.install_base
        elif self.install_lib:
            prefix = self.install_lib
            for _ in range(3):
                prefix = os.path.dirname(prefix)

        if prefix:
            libexec_dir = os.path.join(prefix, 'lib', package_name)
            os.makedirs(libexec_dir, exist_ok=True)
            for name in executables:
                libexec_path = os.path.join(libexec_dir, name)
            with open(libexec_path, 'w') as handle:
                    handle.write(self._wrapper(name))
            os.chmod(libexec_path, 0o755)


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install the executable scripts into lib/<package_name>/ so that
        # ros2 run / ros2 launch can find them (libexec directory).
        (os.path.join('lib', package_name),
            [os.path.join('scripts', name) for name in executables]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mlh090300',
    maintainer_email='mlh090300@todo.todo',
    description=(
        'Waypoint following for TurtleBot3 via Nav2 NavigateToPose '
        '(draws the planned path on the RViz map).'),
    license='Apache-2.0',
    tests_require=['pytest'],
    cmdclass={'install': PostInstallFixup},
    entry_points={
        'console_scripts': [
            'waypoint_follower = '
            'turtlebot3_wp_following.waypoint_follower:main',
            'path_cacher = '
            'turtlebot3_wp_following.path_cacher:main',
            'path_tracker = '
            'turtlebot3_wp_following.path_tracker:main',
        ],
    },
)


