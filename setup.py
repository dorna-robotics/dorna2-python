import setuptools

with open("README.md", "r") as fh:
    readme = fh.read()

setuptools.setup(
    name="dorna2",
    version= "1.10",
    author="Dorna Robotics",
    author_email="info@dorna.ai",
    description="Python API for Dorna 2",
    long_description=readme,
    long_description_content_type='text/markdown',
    url="https://dorna.ai/",
    project_urls={
        'gitHub': 'https://github.com/dorna-robotics/dorna2-python',
    },    
    packages=setuptools.find_packages(),
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python :: 3.7',
        "Operating System :: OS Independent",
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
    install_requires=[
        "setuptools",
    ],
    license="MIT",
    include_package_data=True,
    zip_safe = False,
)