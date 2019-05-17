pbrt, Version 3
===============

[![Build Status](https://travis-ci.org/mmp/pbrt-v3.svg?branch=master)](https://travis-ci.org/mmp/pbrt-v3)
[![Build status](https://ci.appveyor.com/api/projects/status/mlm9g91ejxlcn67s/branch/master?svg=true)](https://ci.appveyor.com/project/mmp/pbrt-v3/branch/master)

This repository holds the source code to the version of pbrt that is
described in the third edition of *Physically Based Rendering: From
Theory to Implementation*, by [Matt Pharr](http://pharr.org/matt), [Wenzel
Jakob](http://www.mitsuba-renderer.org/~wenzel/), and Greg Humphreys.  As
before, the code is available under the BSD license.

The [pbrt website](http://pbrt.org) has general information about both the
*Physically Based Rendering* book as well as many other resources for pbrt.
As of October 2018, the full [text of the book](http://www.pbr-book.org) is
now available online, for free.

Global Illumination Shadow Layers
---------------------------------

The code specific to the Global Illumination Shadow Layers paper implementation
is available under the GPL v3 license. A test scene `shadow.pbrt` is provided as
a starter.

### Objects ###

Objects are identified using a proxy material of type `identifier` to avoid
storing additional data in the geometry. It accepts two parameters:

* `string identifier` is the identifier of the object.
* `string material` is the type of the underlying material.

Any other parameter is forwarded to the underlying material.

### Shadow layers rendering ###

The `ShadowIntegrator` is simply called `shadow` in scene files. It accepts
the same parameters as `path` along with:

* `integer maxskips` sets the maximum number of intersection skips that can be
performed when recovering lost radiance across surfaces. Default: `maxdepth`.
* `string casters` is a list of strings containing the material identifiers of
the objects that will cast (emit) shadows.
* `string catchers` is a list of strings containing the material identifiers of
the objects that will catch (receive) shadows.
* `string noselfshadow` is a list of strings containing the material identifiers
of the objects that should not display self-shadowing on their surface.
* `bool singlefile` indicates if a multi-layered EXR file should be exported
instead of separate images. Default is `true`.
* `bool splitlights` indicates if shadow layers should be
separated per light in the scene. Note that for area lights, each triangle of
the mesh is considered a separate light. Default is `false`.
* `bool splitdirect` indicates if shadow layers should be separated into direct
and indirect components. Default is `false`.
* `float skipprob` sets the probability of skipping a surface during
propagation. Default is `0.5`.

### Performance tests ###

We implemented variants of the `PathIntegrator` and `ShadowIntegrator` that
measure and display statistics about the render. They can be invoked as
`pathstats` and `shadowstats` and take additional parameters.

* `string mode` should be `normal`, `time` or `variance` to use either a fixed
sample count, time, or variance. Default is `normal`.

#### Fixed time ####

* `integer batchsize` sets the number of samples evaluated at once. A bigger
value increases cache benefits but reduces the precision of time measurement.
Default is `32`.
* `integer seconds` sets the maximum allowed render time in seconds. However,
the last batch of samples is allowed to finish past the timer. Default is `60`.

The render log will show the batch size and number of batches rendered.

#### Fixed variance ####

* `integer maxsamples` sets the maximum number of samples that can be used to
render the image. Default is 4 times the `pixelsamples` value of the sampler.
* `float variance` sets the maximum variance per pixel. The variance is
estimated after having evaluated `pixelsamples` samples. If a pixel cannot
converge under this threshold, it uses the maximum number of samples allowed.
Default is `0.1`.

The render log will show the percentage of unconverged pixels, the mean number
of samples used, and the mean variance.

Example scenes
--------------

Over 8GB of example scenes are available for download. (Many are new and
weren't available with previous versions of pbrt.)  See the [pbrt-v3 scenes
page](http://pbrt.org/scenes-v3.html) on the pbrt website for information
about how to download them.

After downloading them, see the `README.md.html` file in the scene
distribution for more information about the scenes and preview images.

Additional resources
--------------------

* There is a [pbrt Google
  Groups](https://groups.google.com/forum/#!forum/pbrt) mailing list that can
  be a helpful resource.
* Please see the [User's Guide](http://pbrt.org/users-guide.html) for more
  information about how to check out and build the system as well as various
  additional information about working with pbrt.
* Should you find a bug in pbrt, please report it in the [bug
  tracker](https://github.com/mmp/pbrt-v3/issues).
* Please report any errors you find in the *Physically Based Rendering*
  book to authors@pbrt.org.

Note: we tend to let bug reports and book errata emails pile up for a few
months for processing them in batches. Don't think we don't appreciate
them. :-)

Building pbrt
-------------

To check out pbrt together with all dependencies, be sure to use the
`--recursive` flag when cloning the repository, i.e.
```bash
$ git clone --recursive https://github.com/mmp/pbrt-v3/
```
If you accidentally already cloned pbrt without this flag (or to update an
pbrt source tree after a new submodule has been added, run the following
command to also fetch the dependencies:
```bash
$ git submodule update --init --recursive
```

pbrt uses [cmake](http://www.cmake.org/) for its build system.  On Linux
and OS X, cmake is available via most package management systems.  To get
cmake for Windows, or to build it from source, see the [cmake downloads
page](http://www.cmake.org/download/).  Once you have cmake, the next step
depends on your operating system.

### Makefile builds (Linux, other Unixes, and Mac) ###

Create a new directory for the build, change to that directory, and run
`cmake [path to pbrt-v3]`. A Makefile will be created in the current
directory.  Next, run `make` to build pbrt, the obj2pbrt and imgtool
utilities, and an executable that runs pbrt's unit tests.  Depending on the
number of cores in your system, you will probably want to supply make with
the `-j` parameter to specify the number of compilation jobs to run in
parallel (e.g. `make -j8`).

By default, the makefiles that are created that will compile an optimized
release build of pbrt. These builds give the highest performance when
rendering, but many runtime checks are disabled in these builds and
optimized builds are generally difficult to trace in a debugger.

To build a debug version of pbrt, set the `CMAKE_BUILD_TYPE` flag to
`Debug` when you run cmake to create build files to make a debug build.  To
do so, provide cmake with the argument `-DCMAKE_BUILD_TYPE=Debug` and build
pbrt using the resulting makefiles. (You may want to keep two build
directories, one for release builds and one for debug builds, so that you
don't need to switch back and forth.)

Debug versions of the system run much more slowly than release
builds. Therefore, in order to avoid surprisingly slow renders when
debugging support isn't desired, debug versions of pbrt print a banner
message indicating that they were built for debugging at startup time.

### Xcode ###

To make an Xcode project on OS X, run `cmake -G Xcode [path to pbrt-v3]`.
A `PBRT-V3.xcodeproj` project file that can be opened in Xcode.  Note that
the default build settings have an optimization level of "None"; you'll
almost certainly want to choose "Faster" or "Fastest".

### MSVC on Windows ###

On Windows, first point the cmake GUI at the directory with pbrt's source
code.  Create a separate directory to hold the result of the build
(potentially just a directory named "build" inside the pbrt-v3 directory)
and set that for "Where to build the binaries" in the GUI.

Next, click "Configure".  Note that you will want to choose the "Win64"
generator for your MSVC installation unless you have a clear reason to need
a 32-bit build of pbrt.  Once cmake has finished the configuration step,
click "Generate"; when that's done, there will be a "PBRT-V3.sln" file in
the build directory you specified. Open that up in MSVC and you're ready to
go.

### Build Configurations ###

There are two configuration settings that must be set when configuring the
build. The first controls whether pbrt uses 32-bit or 64-bit values for
floating-point computation, and the second controls whether tristimulus RGB
values or sampled spectral values are used for rendering.  (Both of these
aren't amenable to being chosen at runtime, but must be determined at
compile time for efficiency).  The cmake configuration variables
`PBRT_FLOAT_AS_DOUBLE` and `PBRT_SAMPLED_SPECTRUM` configure them,
respectively.

If you're using a GUI version of cmake, those settings should be available
in the list of configuration variables; set them as desired before choosing
'Generate'.

With command-line cmake, their values can be specified when you cmake via
`-DPBRT_FLOAT_AS_DOUBLE=1`, for example.
