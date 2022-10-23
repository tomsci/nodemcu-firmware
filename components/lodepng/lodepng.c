// Workaround for LodePNG insisting on naming its C-compatible source file
// lodepng.cpp which our toolchain tries to compile as C++, which we really
// don't want. Doing this is easier than persuading cmake to compile it as C.
#include "lodepng/lodepng.cpp"
