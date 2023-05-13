# LodePNG PNG decoder module
| Since  | Origin / Contributor  | Maintainer  | Source  |
| :----- | :-------------------- | :---------- | :------ |
| 2022-10-23 | https://github.com/lvandeve/lodepng integrated by [Tom Sutcliffe](https://github.com/tomsci) | [Tom Sutcliffe](https://github.com/tomsci) | [lodepng.c](../../components/modules/lodepng.c)|

This module wraps the LodePNG PNG library https://github.com/lvandeve/lodepng. Currently, only decode is supported.

The memory requirements for decoding images are relatively high; this module does not support incremental decode and as such you will (temporarily) need several times as much free RAM as the size of the output buffer.

## lodepng.decode_file()
Decodes a PNG file into a memory buffer.

#### Syntax
`lodepng.decode_file(path, format)`

#### Parameters
- `path` Path to the file.
- `format` The desired format of the resulting buffer. Note that it does not matter what format the PNG file is, if it is different LodePNG will convert it to the requested format. Must be one of:
    - `lodepng.RGB` (24-bit RGB)
    - `lodepng.RGBA` (32-bit RGBA)
    - `lodepng.RGB_565)` (16-bit RGB in 5-6-5 format)
    - `lodepng.PALETTE` (8-bit, see note below)

Note, the `PALETTE` format should only be used if the PNG file is known in advance to be using a palette, _and_ the exact palette being used is also known in advance. In this mode the returned data will be 8 bits per pixel containing the index of the pixel colour in the palette. The main reason for using this format is that it uses less memory during the decode than any of the other formats.

#### Returns
`buf, width, height` if the PNG was successfully decoded, `nil, err` otherwise. `buf` is a string of the raw pixel data in the requested format. The length of `buf` will be `width * height * 2` for `RBG_565` format, `width * height * 3` for `RGB`, `width * height * 4` for `RGBA`, and `width * height` for `PALETTE`. If an error occurs, `err` will be a string describing the error.

#### Example
```lua
buf, width, height = lodepng.decode_file("myimage.png", lodepng.RGB_565)
if buf then
    print(string.format("PNG is %dx%d pixels", width, height))
else
    local err = width -- ie the second result
    print(string.format("Failed to decode png, error=%s", err))
end
```

## lodepng.decode()
Decodes a PNG file from an in-memory buffer.

#### Syntax
`lodepng.decode(inbuf, format)`

#### Parameters
- `inbuf` A string containing the entire PNG file contents.
- `format` The desired format of the resulting buffer. Note that it does not matter what format the PNG file is, if it is different LodePNG will convert it to the requested format. Must be one of:
    - `lodepng.RGB` (24-bit RGB)
    - `lodepng.RGBA` (32-bit RGBA)
    - `lodepng.RGB_565)` (16-bit RGB in 5-6-5 format)

#### Returns
`buf, width, height` if the PNG was successfully decoded, `nil, err` otherwise. `buf` is a string of the raw pixel data in the requested format. The length of `buf` will be `width * height * 2` for `RBG_565` format, `width * height * 3` for `RGB`, or `width * height * 4` for `RGBA`. If an error occurs, `err` will be a string describing the error.

#### Example
```lua
buf, width, height = lodepng.decode(my_png_buf, lodepng.RGB)
if buf then
    print(string.format("PNG is %dx%d pixels", width, height))
else
    local err = width -- ie the second result
    print(string.format("Failed to decode png, error=%s", err))
end
```
