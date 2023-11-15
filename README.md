# lwmodelingtoolset
Lightwave modeling toolset is made of three plugins: a lattice tool, an edge slider and the sketch drag tool.


Lattice Tool

How to use:
Activate the tool and define the lattice subdivisions. Once you’ll move one of the lattice handles, the subdivisions will freeze and you won’t be able to change them anymore.
Interpolations: bezier is for smoother results, while linear for higher control on local areas of the mesh.
Use “Handle Radius” and “Show Wires” to edit the lattice display settings.
The algorithm uses multi-threaded evaluation to make deformations faster.

Works with Symmetry.
Works with polygon and point selection.
Use Shift and Ctrl to add and toggle lattice handles selection.

Known limitations:
Does not work with symmetry.

Edge Slider

How to use:
Select the edges you wish to slide and activate the tool. Left click on a viewport and drag to edge-slide. Right click on viewport and drag to edge-loop-slide.
Works with symmetry.

Known limitations:
Our edge slider is topology and loop based. For this reason, when selecting multiple non-contiguous edges the sliding direction may not be as expected.

Sketch Drag

How to use:
When nothing is selected the tool will work with silhouette edges, otherwise it will work on the selected ones. POINTS is the edges points initially moved with the stroke (i.e. when “iterations allowed” is zero).
Spread mode will spread POINTS evenly along the stroke.
Closest mode will match POINTS with each closest point on the stroke.
“Propagate” is the strength applied to the points connected to POINTS.
“Iterations allowed” is the maximum number of visited edges connected to POINTS.
Works with symmetry.

Known limitations:
The algorithm searches the closest edge and then follow the stroke direction, for this reason it may not always modify all the points for the selected edges.
