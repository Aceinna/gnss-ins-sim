import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="gnss-ins-sim",
    version="2.1",
    author="Aceinna",
    author_email="xgdong@aceinna.com",
    description="GNSS-INS-SIM is an GNSS/INS simulation project, "
        "which generates reference trajectories, IMU sensor output, "
        "GPS output, odometer output and magnetometer output.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Aceinna/gnss-ins-sim",
    packages=setuptools.find_packages(),
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        "Programming Language :: Python :: 2.7",
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)
