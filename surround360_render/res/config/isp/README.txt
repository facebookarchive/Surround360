Field description for configuring the soft ISP

BlackLevel
   3 channel vector value [r, g, b]

     offset in range 0.0, 1.0 where 1.0 == 2^bits_per_channel - This
     varies by camera & sensor vendor. For the usb3 grasshoppers it is
     12.0/255.0 equally for all 3 channels.


Vignette rolloff
   3 channel vector value [r, g, b]

       This is the center to edge fall off in the range 0.0, 1.0 where
       1.0 is 100% fall off. Usually this value is around 10% to 20%.
       It is not unusual to to have each of the channels roll off at
       different rates to handle chromatic vignetting effects.


WhiteBalanceGain
   3 channel vector value [r, g, b]

       Nominally the red and blue channels are usually around 2x gain
       to the green channels. Strictly speaking this should be
       computed using a global white balance algrotihm.


stuckPixelThreshold
   Single value

       The absolute difference of the center pixel from the mean of
       it's neighbors such that it is considered a candidate stuck
       pixel. A value of 0.05 is around 13 pixel counts out of 255.

stuckPixelVarianceThreshold
   Single value

       A measure of local smoothness. Such that if the local region is
       smooth and the center pixel is "candidate stuck" pixel then the
       current pixel is replaced with the local mean.  Both the mean
       and variance calculation excludes the current pixel.

denoise
   Single value

      Noise threshold value.  Zero disables noise coring. Bigger value
      remove more noise but also more signal potentially.

denoiseRadius
    Single value

      Radius for noise reduction 3 or 4 is reasonable.  1 will be fast but will leave low frequency noise behind.
 
ccm
   3x3 matrix  [[m00, m01, m01], ..., [m20, m21, m22]]

       An identity leaves the color unchanged. This matrix shold
       calculated from a color calibration session. It usually best if
       the rows sume unity to conserve energy.

sharpenning
   3 channel vector value [r, g, b]

       1.0 means no sharpenning. Below 1.o means blurring and values
       greater than 1.0 will sharpen the image.  Execessive
       sharpenning will cause "ringing."

saturation
    Single value

      1.0 means no saturation gain.  0.0 means no color at all
      (e.g. B & W) and values greater than one increase saturation.

contrast
     Single value

      1.0 means no contrast gain.  Values greater than one increase
      contrast.

gamma
   3 channel vector value [r, g, b]

      Raises the pixel value to the power of the gamma value. 0.454545
      is the  classic 2.2 gamma  used to to  move from linear  sRGB to
      gamm corrected sRGB.



bayerPattern
    Single value string


      The string is expected to contain a combination of "R" "G" and
      "B" in the payer pattern order left to right, top to bottom. So
      a Bayer pattern of:

         R G
         G B

      Would be "RGGB"
