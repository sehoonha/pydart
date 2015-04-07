PyDart

![dlogo](https://github.com/dartsim/dart/raw/master/doxygen/DART%20logo.png =150x)
![plog](https://www.python.org/static/community_logos/python-logo.png =150x)
======
PyDART is an open source python binding of [DART](https://github.com/dartsim/dart)(4.3.4), an open source physics simulator.
All APIs are designed to provide a concise and powerful control on [DART](https://github.com/dartsim/dart) physics worlds.
Further, a user can write simulations with a numerous python scientific libraries, 
such as [NumPy](http://www.numpy.org/)(linear algebra),
[SciPy](http://www.scipy.org/)(optimization), 
[scikit-learn] (http://scikit-learn.org/stable/) (machine learning),
[PyBrain](http://pybrain.org/)(machine learning),
and so on.

## Requirement
- [DART](https://github.com/dartsim/dart)(4.3.4)
 + https://github.com/dartsim/dart/wiki/Installation
- [SWIG](http://www.swig.org/) (Simplified Wrapper and Interface Generator)
- [NumPy](http://www.numpy.org/) & [SciPy](http://www.scipy.org/)
- [PyOpenGL](http://pyopengl.sourceforge.net/) 
- [PyQt4](http://www.riverbankcomputing.com/software/pyqt/download)(Optional)
```
sudo aptitude install swig
sudo aptitude install python-numpy python-scipy 
sudo apt-get install python-pip
sudo pip install PyOpenGL PyOpenGL_accelerate
```

## Building instruction
- Checkout the project
```
git clone https://github.com/sehoonha/pydart.git
cd pydart
```
- Setup the python package for development
```
sudo python setup.py develop
```
- Compile the API
```
mkdir build
cd build
cmake ..
make
make install
```
- Run the first application
```
python apps/helloPyDART/main.py
```
