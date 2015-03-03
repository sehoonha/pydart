pydart
======
Python interface for dart
Built on DART 4.3.4

## Requirement
- DART 4.3.4
 + https://github.com/dartsim/dart
- SWIG (Simplified Wrapper and Interface Generator)
 + http://www.swig.org/
- NumPy & SciPy
 + http://www.scipy.org/install.html
 + sudo pip install scipy numpy (not verified)
- PyOpenGL 
 + http://pyopengl.sourceforge.net/
 + sudo pip install PyOpenGL PyOpenGL_accelerate
- PyQt4 (Optional for falling)
 + http://www.riverbankcomputing.co.uk/software/pyqt/download
- Plot.ly (Optional for falling)
 + https://plot.ly/python/getting-started/
 + sudo pip install plotly
- CMA (Optional for falling)
 + sudo pip install cma

## Building instruction
``` cmake
mkdir build
cd build
cmake ..
make
```

## Usage
In build directory, 
```
python ../app/hello_pydart/main.py
python ../app/falling/main.py
```
