import setuptools
import dorna2
with open("README.md", "r") as fh:
    readme = fh.read()

version = dorna2.__version__

setuptools.setup(
    name="dorna2",
    author="Dorna Robotics",
    version=version,
    author_email="info@dorna.ai",
    description="Python API for Dorna 2",
    long_description=readme,
    long_description_content_type='text/markdown',
    url="https://dorna.ai/",
    project_urls={
        'gitHub': 'https://github.com/dorna-robotics/dorna2-python',
    },
    package_data={
        'dorna2': ['cfg/*'],
    },    
    packages=setuptools.find_packages(),
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python :: 3.8',
        "Operating System :: OS Independent",
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
    install_requires=[],
    license="MIT",
    include_package_data=True,
    zip_safe = False,
)