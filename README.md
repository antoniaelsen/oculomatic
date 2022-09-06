# Oculomatic

Forked from [Oculomatic-Comedi](https://github.com/oculomatic/oculomatic-comedi)

Tracks pupil location and centroid size.
Updated to support a variety of cameras with OpenCV.
Outputs are x and y raw position.

## Usage

### Installation

Requires

- [cmake](https://cmake.org/)
- [conan](https://conan.io/) - C++ package manager
- (Optional)
  - FlyCapture 2 SDK
  - Comedi SDK

Run

```
make install-dependencies

make

make run
```

**FlyCapture**

To enable FlyCapture cameras, first install the SDK -- see `doc/installation.md`.
Once FlyCapture is installed, enable FlyCapture in the Oculomatic source with

```
TODO
```
