from setuptools import setup, find_packages

setup(
    name="nclt-slam",
    version="0.1.0",
    description="LiDAR SLAM with learned place recognition on NCLT dataset",
    packages=find_packages(),
    python_requires=">=3.10",
    install_requires=[
        "numpy>=1.24.0",
        "torch>=2.0.0",
        "open3d>=0.17.0",
        "pandas>=2.0.0",
        "pyyaml>=6.0",
        "tqdm>=4.65.0",
        "scipy>=1.10.0",
        "scikit-learn>=1.2.0",
        "matplotlib>=3.7.0",
    ],
    extras_require={
        "dev": ["pytest>=7.3.0", "black>=23.3.0", "mypy>=1.3.0"],
        "notebooks": ["jupyter>=1.0.0", "ipywidgets>=8.0.0", "plotly>=5.14.0"],
    },
)
