from setuptools import setup, find_packages

setup(
    name="eskf-lie",
    version="0.1.0",
    description="Extended Kalman Filter with Lie Groups",
    author="Your Name",
    license="MIT",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "numpy",
        "scipy",
    ],
    extras_require={
        "dev": ["pytest", "pytest-cov"],
    },
)
