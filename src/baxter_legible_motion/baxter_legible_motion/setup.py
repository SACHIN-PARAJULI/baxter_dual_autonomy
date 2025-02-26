from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['pick_and_place_baxter_moveit'],
    scripts=['src/gen_traj.py', 'src/plot_traj.py'],
    package_dir={'': 'src'}
)

setup(**d)
