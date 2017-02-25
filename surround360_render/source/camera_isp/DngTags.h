#ifndef DNGTAGS_H
#define DNGTAGS_H

struct TiffIfdEntry {
  uint16_t uTag;
  uint16_t uType;
  uint32_t uCount;
  uint32_t uOffset;
};

// Type values
const uint16_t kTiffTypeBYTE = 1;
const uint16_t kTiffTypeASCII = 2;
const uint16_t kTiffTypeSHORT = 3;
const uint16_t kTiffTypeLONG = 4;
const uint16_t kTiffTypeRATIONAL = 5;
const uint16_t kTiffTypeSBYTE = 6;
const uint16_t kTiffTypeUNDEFINED = 7;
const uint16_t kTiffTypeSSHORT = 8;
const uint16_t kTiffTypeSLONG = 9;
const uint16_t kTiffTypeSRATIONAL = 10;
const uint16_t kTiffTypeFLOAT = 11;
const uint16_t kTiffTypeDOUBLE = 12;

// Tag values
const uint16_t kTiffTagNewSubFileType = 254;
const uint16_t kTiffTagImageWidth = 256;
const uint16_t kTiffTagImageLength = 257;
const uint16_t kTiffTagBitsPerSample = 258;
const uint16_t kTiffTagCompression = 259;
const uint16_t kTiffTagPhotometricInterpretation = 262;
const uint16_t kTiffTagMake = 271;
const uint16_t kTiffTagModel = 272;
const uint16_t kTiffTagStripOffsets = 273;
const uint16_t kTiffTagOrientation = 274;
const uint16_t kTiffTagSamplesPerPixel = 277;
const uint16_t kTiffTagRowsPerStrip = 278;
const uint16_t kTiffTagStripByteCounts = 279;
const uint16_t kTiffTagPlanarConfiguration = 284;
const uint16_t kTiffTagResolutionUnit = 296;
const uint16_t kTiffTagSoftware = 305;
const uint16_t kTiffTagDateTime = 306;

const uint16_t kTiffEpTagCFARepeatPatternDim = 33421;
const uint16_t kTiffEpTagCFAPattern = 33422;

const uint16_t kDngTagDNGVersion = 50706;
const uint16_t kDngTagDNGBackwardVersion = 50707;
const uint16_t kDngTagUniqueCameraModel = 50708;
const uint16_t kDngTagLocalizedCameraModel = 50709;
const uint16_t kDngTagCFAPlaneColor = 50710;
const uint16_t kDngTagCFALayout = 50711;
const uint16_t kDngTagBlackLevelRepeatDim = 50713;
const uint16_t kDngTagBlackLevel = 50714;
const uint16_t kDngTagBlackLevelDeltaH = 50715;
const uint16_t kDngTagBlackLevelDeltaV = 50716;
const uint16_t kDngTagWhiteLevel = 50717;
const uint16_t kDngTagDefaultScale = 50718;
const uint16_t kDngTagDefaultCropOrigin = 50719;
const uint16_t kDngTagDefaultCropSize = 50720;
const uint16_t kDngTagColorMatrix1 = 50721;
const uint16_t kDngTagColorMatrix2 = 50722;
const uint16_t kDngTagForwardMatrix1 = 50964;
const uint16_t kDngTagForwardMatrix2 = 50965;
const uint16_t kDngTagCameraCalibration1 = 50723;
const uint16_t kDngTagCameraCalibration2 = 50724;
const uint16_t kDngTagReductionMatrix1 = 50725;
const uint16_t kDngTagReductionMatrix2 = 50726;
const uint16_t kDngTagAnalogBalance = 50727;
const uint16_t kDngTagAsShotNeutral = 50728;
const uint16_t kDngTagAsShotWhiteXY = 50729;
const uint16_t kDngTagBaselineExposure = 50730;
const uint16_t kDngTagBaselineNoise = 50731;
const uint16_t kDngTagBaselineSharpness = 50732;
const uint16_t kDngTagBayerGreenSplit = 50733;
const uint16_t kDngTagLinearResponseLimit = 50734;
const uint16_t kDngTagLensInfo = 50736;
const uint16_t kDngTagChromaBlurRadius = 50737;
const uint16_t kDngTagAntiAliasStrength = 50738;
const uint16_t kDngTagShadowScale = 50739;
const uint16_t kDngTagDNGPrivateData = 50740;
const uint16_t kDngTagMakerNoteSafety = 50741;
const uint16_t kDngTagCalibrationIlluminant1 = 50778;
const uint16_t kDngTagCalibrationIlluminant2 = 50779;
const uint16_t kDngTagBestQualityScale = 50780;
const uint16_t kDngTagActiveArea = 50829;
const uint16_t kDngTagMaskedAreas = 50830;
const uint16_t kDngTagNoiseProfile = 51041;

#endif
