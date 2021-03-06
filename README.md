                  +-------------------+
                  | color_blob_drawer |
                  +-------------------+

[![Build Status](https://travis-ci.org/arnaud-ramey/color_blob_drawer.svg)](https://travis-ci.org/arnaud-ramey/color_blob_drawer)

A color tracking application, that enables you to draw arbitrary shapes on
the screen by moving a colorful object. In fact, the program tracks a moving
blob on a video. The blob is defined by its range of color. For instance, if
you use an orange pen, the blob is the cap of the pen, and the range of
colors is a range of orange (in fact the conditions on the range of colors
are made of a minimum and a maximum allowed values for the 3 Hue, Saturation
and Value components of the colors, that is 6 thresholds).

![First capture](doc/capture.png)

Here is how you localize the blob on a frame.
1) We first define the search area. If the blob was not found on the
  previous picture, the search area is the whole current frame, otherwise it
  is just the neighborhood of the location of the previous blob.
2) Then, you scan every pixel of the search area in the current frame and
  see if its color belongs to the allowed range of colors.
3) You will then determine the biggest connected component in the search
  area. We name this component B.
4) if the blob was not found on the previous frame, then B is the new range.
  If it was found on the previous frame, we also require B is to be of a
  similar size (say the same +/- 10% ).
5) By connecting the centers of the blobs on the set of pictures, you can
  illustrate the movement of the blob, and then "draw".

License :                  see the LICENSE file.
Authors :                  see the AUTHORS file.
How to build the program:  see the INSTALL file.

________________________________________________________________________________

How to use the program
________________________________________________________________________________
Ensure yout webcam is plugged, then launch the program in a terminal.
It will create two windows: the filter window and the drawing window.

1) Setting up the filter window
  Select the color of the object using the sliders in the filter window.
  The color filter is made of three components:
  * a Hue component
  * a Saturation component
  * a Value component
  For more information about what these components mean, check Wikipedia:
  http://en.wikipedia.org/wiki/HSL_and_HSV
  The white pixels correspond to the pixels that pass the current HSV filter.
  A black pixel corresponds
  to a pixel which did not pass the filter, while a white pixel corresponds to
  a filter that was in the boundaries. The blue rectangle corresponds to the
  search area in the picture. The red circle in the bottom right corner means
  the blob was found in the picture, The red cross across the frame means it
  wasn't. The red frame appears when the blob was found in the frame, but you
  are close from the frame borders.
  You should adjust the sliders until only the object to track is marked in white.

2) Using the drawing window
  Once the filter is set up, you can use the drawing window.
  Move your object around to draw in the drawing window.
  Hide it for a few seconds to clear the drawing.
