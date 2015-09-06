## PyDart

<table>
<tr>
<td>  
  <img src="https://github.com/dartsim/dart/raw/master/doxygen/DART%20logo.png" width="150" height="50" />
</td>
<td>
  <img src="https://www.python.org/static/community_logos/python-logo.png" width="150" height="50" />
</td>
</tr>
</table>

- News! (06/SEP/2015)
 + PyDART is updated for DART 5.0. 

======
PyDART is an open source python binding of [DART](https://github.com/dartsim/dart)(5.0), an open source physics simulator.
All APIs are designed to provide a concise and powerful control on [DART](https://github.com/dartsim/dart) physics worlds.
Further, a user can write simulations with a numerous python scientific libraries, 
such as [NumPy](http://www.numpy.org/)(linear algebra),
[SciPy](http://www.scipy.org/)(optimization), 
[scikit-learn] (http://scikit-learn.org/stable/) (machine learning),
[PyBrain](http://pybrain.org/)(machine learning),
and so on.

## Requirements
- [DART](https://github.com/dartsim/dart)(5.0)
 + https://github.com/dartsim/dart/wiki/Installation
- [SWIG](http://www.swig.org/) (Simplified Wrapper and Interface Generator)
- [NumPy](http://www.numpy.org/) & [SciPy](http://www.scipy.org/)
- [PyOpenGL](http://pyopengl.sourceforge.net/) 
- [PyQt4](http://www.riverbankcomputing.com/software/pyqt/download)(Optional)
```
sudo apt-get install swig python-pip libatlas-base-dev gfortran 
sudo pip install numpy scipy PyOpenGL PyOpenGL_accelerate
```

## Installation
- Checkout the project
```
git clone https://github.com/sehoonha/pydart.git
cd pydart
```
- Compile the API
```
mkdir build
cd build
cmake ..
make
make install
```
- Setup the python package for development
```
cd ..
sudo python setup.py develop
```
- Run the first application
```
python apps/helloPyDART/main.py
```

## Screenshots
|![](https://github.com/sehoonha/pydart/blob/master/data/images/frame_bipedjump.png)|![](https://github.com/sehoonha/pydart/blob/master/data/images/frame_bipedstand.png)|
|---|---|
|![](https://github.com/sehoonha/pydart/blob/master/data/images/frame_rigidchain.png)|![](https://github.com/sehoonha/pydart/blob/master/data/images/frame_softbodies.png)|

## Code snippets
- Modifying a skeleton pose using dof names (q is still a numerical vector)
```python
q = skel.q
q["j_pelvis_rot_y"] = -0.2
q["j_thigh_left_z", "j_shin_left", "j_heel_left_1"] = 0.15, -0.4, 0.25
q["j_thigh_right_z", "j_shin_right", "j_heel_right_1"] = 0.15, -0.4, 0.25
q["j_abdomen_2"] = 0.0
skel.set_positions(q)
```
- Add damping forces to the rigid chain
```python
class DampingController:
    """ Add damping force to the skeleton """
    def __init__(self, skel):
        self.skel = skel

    def compute(self):
        damping = -0.01 * self.skel.qdot
        for i in range(1, self.skel.ndofs, 3):
            damping[i] *= 0.1
        return damping
```