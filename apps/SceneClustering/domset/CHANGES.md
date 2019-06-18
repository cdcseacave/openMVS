# CHANGES
- Updated NanoFLANN
- Trimmed some files
- Wording: computeViewSimilaity -> computeViewSimilarity
- Fix a couple of warnings
- Added this (bugfix)[https://github.com/srivathsanmurali/domsetLibrary/pull/6/commits/f3ad7bf010957405bb3c0542f55a6b44c28f04b4]
- Added some changes introduced into OpenMVG but
    - kept the nanoflann interface
    (in any case the current FLANN usage in openMVG seems to give wrong results) 
    - kept the DOMSET_USE_OPENMP FLAG 
- Changed CMAKE to play nicely with OpenMVS

TODO: Maybe ADD double later...