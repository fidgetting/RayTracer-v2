Simple Ray Tracer
=================

Tracer
------

A simply ray tracer that reads a subset of obj.

To Run:
```bash
./Tracer input.obj output.png
```

This program will use the extension of the output to determine
the type of the image to save. This uses the gtk gui library for
saving the image so any extension that works with that library will
work here. The teapot file in the models directory works the best
for this binary.


ObjRender
---------

This is in progress. The goal is to create a simply obj file viewer
that allows the user to rotate the camera around and render the
object using the ray tracer.

To Run:
```base
./ObjRender 
```
