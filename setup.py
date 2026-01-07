from setuptools import setup, find_packages

setup(
    name="fanuc-rmi",
    version="0.1.0",
    author="",
    author_email="",
    description="FANUC机器人RMI API客户端库",
    long_description=open("README.md", encoding="utf-8").read(),
    long_description_content_type="text/markdown",
    url="",
    packages=["fanuc_rmi"],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
    install_requires=[],
)