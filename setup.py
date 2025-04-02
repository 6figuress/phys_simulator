from setuptools import find_packages, setup

setup(
    name="simulator",
    version="0.1",
    packages=find_packages(),
    install_requires=[
        "numpy>=2.2.4",
        "pybullet>=3.2.7",
    ],
    python_requires=">=3.11,<3.12",
    author="6 Figures",
    description="A package to simulate the ur3e robot arm physics for security usages",
    package_data={
        "simulator": ["urdfs/**"],
    },
)
