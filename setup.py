from setuptools import find_packages, setup

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    name="rluni",  # Replace with your package name
    version="0.1.0",
    author="team2411 UBC EngPhys",
    author_email="rlunicycle2411@gmail.com",
    description="A package for controlling a unicycle robot.",
    url="https://github.com/Team-2411-RL-Unicycle/rl-unicycle",  # Replace with your repo URL
    package_dir={"": "src"},  # Source directory
    packages=find_packages(where="src"),
    include_package_data=True,  # Include non-Python files as specified in MANIFEST.in
    install_requires=[
        "msgpack",
        "moteus",
        "paho-mqtt==1.6.1",
        "imufusion",
        "smbus2",
        "simple_pid",
        "onnxruntime",
        "PYyaml",
        "transforms3d",
        # Add other dependencies as needed
    ],
    python_requires=">=3.8",
)
