#!/usr/bin/env python3
"""Regenerate the HEIC test fixtures in apps/Tests/data/images/heif/.

Produces three files from apps/Tests/data/images/00000.jpg:

  sample.heic       - normal orientation (EXIF Orientation=1), no container
                      rotation. Baseline decode-parity / EXIF-bridge fixture.
  sample_rot6.heic  - same pixel content, but stored rotated BOTH ways: a
                      genuine ISOBMFF container-level 'irot' property AND an
                      EXIF Orientation=6 tag. Locks the "don't rotate twice"
                      guard (HEIF applies irot at decode time; honoring the
                      EXIF tag on top of that would rotate the pixels again).
  sample_alpha.heic - same pixel content as sample.heic (normal orientation,
                      no container rotation), but encoded with a genuine
                      *opaque* alpha channel, exactly like real iPhone HEIC
                      photos. Locks the "advertising a 32-bit format routes
                      gray loads through the alpha-copy path instead of a
                      luminance of RGB" regression (see CImageHEIF::ReadHeader
                      and CImage::FilterFormat's PF_A8/PF_GRAY8 case): a
                      regressed reader would decode this file's gray path to
                      a flat, near-zero-variance buffer (constant 255) instead
                      of real photographic content.

Required external tools (offline, dev-machine only):
  - Python 3 with Pillow and pillow-heif ("pip install --user pillow-heif").
    pillow-heif bundles libheif with its HEVC (x265) ENCODER built in.
  - exiftool is NOT required to run this script; it is only useful
    afterwards as an independent read-only check of the tags this script
    embeds (exiftool's own HEIC *write* path was found to corrupt the
    Orientation tag on this machine -- see "Why hand-rolled EXIF" below --
    so this script never asks exiftool to modify a .heic file).

Why x265/pillow-heif is fine here but never in the build
---------------------------------------------------------
x265 is GPL-2.0. OpenMVS's vcpkg.json pins libheif with
`"default-features": false` specifically to keep the GPL x265 *encoder* out
of the shipped binary (OpenMVS only ever needs to *read* HEIC/HEIF files;
the LGPL libde265 decoder is enough for that). pillow-heif is used here only
as an OFFLINE, DEVELOPER-MACHINE fixture-generation tool to produce these three
small binary files once; it must never be added to vcpkg.json, any
CMakeLists.txt, or any other build configuration.

Why the EXIF blob is hand-rolled instead of built with PIL.Image.Exif
----------------------------------------------------------------------
Two independent reasons pushed this script away from the "obvious" approach
of using exiftool -tagsFromFile / PIL's Exif class:

  1. PIL.Image.Exif().tobytes() (Pillow 12.2, this machine) raises
     "TypeError: bad operand type for abs(): 'tuple'" for any 3-rational GPS
     field (GPSLatitude/GPSLongitude), i.e. it cannot serialize a GPS IFD.
  2. exiftool 13.41's HEIC *write* path is unreliable for the Orientation
     tag specifically: `exiftool -Orientation=1 sample.heic` was observed to
     write raw IFD0 tag 274 = 3 ("Rotate 180") instead of 1 -- verified by
     manually parsing the raw TIFF bytes, not just trusting exiftool's own
     read-back. exiftool's *read* path is fine and was used throughout to
     independently verify these fixtures.

So this script builds the "Exif\\0\\0"-prefixed TIFF blob for each fixture by
hand (see build_exif_blob()) and hands it to pillow-heif's encoder as the
`exif=` kwarg. This sidesteps both bugs entirely and gives byte-exact control
over every tag.

How the container 'irot' box gets written (fixture 2)
---------------------------------------------------------
pillow-heif derives the encoder's "image_orientation" parameter from the
EXIF Orientation tag found in the `exif=` bytes passed to .save() (see
pillow_heif/misc.py: _get_orientation_for_encoder()). Handing it a blob with
Orientation=6 at encode time makes libheif write a genuine 'irot' item
property (verified by manually walking the ISOBMFF box tree: an 'irot' box
appears in ipco with payload angle_field=3, i.e. 270 deg CCW = 90 deg CW,
referenced as an *essential* property from ipma for the primary item) --
this is a real container-level rotation, not a simulated/fallback one. The
original (unrotated) EXIF Orientation=6 tag is preserved as-is in the stored
Exif metadata item (pillow-heif does not reset it), so the fixture ends up
with both signals set, which is exactly what the "don't rotate twice" test
needs to exercise.
"""
from __future__ import annotations

import struct
from pathlib import Path

import pillow_heif
from PIL import Image

SCRIPT_DIR = Path(__file__).resolve().parent
SRC_IMAGE = SCRIPT_DIR.parent / "00000.jpg"

TARGET_WIDTH = 256          # downscaled fixture width; height derived to preserve aspect ratio
HEIC_QUALITY = 80           # x265 quality; ~32-33 KB output, comfortably under the 50 KB budget

# Grafted verbatim from 00000.jpg's own EXIF (ExifIFD FocalLengthIn35mmFormat = 39 mm).
FOCAL_LENGTH_35MM = 39

# 00000-00003.jpg carry NO GPS EXIF tags at all (verified with exiftool), so a literal
# "graft GPS from the source JPG" is impossible. These coordinates are therefore
# synthetic, not sourced from the JPG -- picked only to be well-formed, plausible
# WGS84 values so TinyEXIF's GeoLocation.hasLatLon() and focal-ladder tests have
# real data to exercise.
GPS_LATITUDE = 45.523000    # degrees, +N
GPS_LONGITUDE = 8.556000    # degrees, +E
GPS_ALTITUDE_M = 250.0      # metres above sea level


def _rational(num: int, den: int) -> bytes:
    return struct.pack(">II", num, den)


def _deg_min_sec(value: float) -> bytes:
    """Pack an absolute decimal-degree value as 3 EXIF RATIONALs (deg, min, sec)."""
    d = int(value)
    m_full = (value - d) * 60
    m = int(m_full)
    s_hundredths = round((m_full - m) * 60 * 100)
    return _rational(d, 1) + _rational(m, 1) + _rational(s_hundredths, 100)


def build_exif_blob(orientation: int, focal_length_35mm: int,
                     gps_lat: float, gps_lon: float, gps_alt_m: float) -> bytes:
    """Hand-build a raw "Exif\\0\\0"-prefixed TIFF blob (big-endian, matching the
    source JPGs' byte order) with three IFDs:
      IFD0:    Orientation(274), ExifIFD pointer(34665), GPSInfo pointer(34853)
      ExifIFD: FocalLengthIn35mmFilm(41989)
      GPS IFD: LatRef/Lat/LonRef/Lon/AltRef/Alt (tags 1..6)

    Returned bytes satisfy TinyEXIF::parseFromEXIFSegment's "starts with
    Exif\\0\\0" contract directly.
    """
    ifd0_offset = 8
    ifd0_entry_count = 3
    ifd0_size = 2 + ifd0_entry_count * 12 + 4
    exif_ifd_offset = ifd0_offset + ifd0_size

    exif_ifd_entry_count = 1
    exif_ifd_size = 2 + exif_ifd_entry_count * 12 + 4
    gps_ifd_offset = exif_ifd_offset + exif_ifd_size

    gps_ifd_entry_count = 6
    gps_ifd_size = 2 + gps_ifd_entry_count * 12 + 4
    overflow_offset = gps_ifd_offset + gps_ifd_size

    lat_offset = overflow_offset
    lon_offset = lat_offset + 24  # 3 rationals * 8 bytes
    alt_offset = lon_offset + 24  # 3 rationals * 8 bytes

    def entry(tag: int, typ: int, count: int, value_bytes: bytes) -> bytes:
        assert len(value_bytes) == 4
        return struct.pack(">HHI", tag, typ, count) + value_bytes

    header = b"MM\x00\x2a" + struct.pack(">I", ifd0_offset)

    ifd0 = struct.pack(">H", ifd0_entry_count)
    ifd0 += entry(274, 3, 1, struct.pack(">HH", orientation, 0))    # Orientation (SHORT)
    ifd0 += entry(34665, 4, 1, struct.pack(">I", exif_ifd_offset))  # ExifIFD pointer (LONG)
    ifd0 += entry(34853, 4, 1, struct.pack(">I", gps_ifd_offset))   # GPSInfo pointer (LONG)
    ifd0 += struct.pack(">I", 0)  # no next IFD
    assert len(ifd0) == ifd0_size

    exif_ifd = struct.pack(">H", exif_ifd_entry_count)
    exif_ifd += entry(41989, 3, 1, struct.pack(">HH", focal_length_35mm, 0))  # FocalLengthIn35mmFilm
    exif_ifd += struct.pack(">I", 0)
    assert len(exif_ifd) == exif_ifd_size

    gps_ifd = struct.pack(">H", gps_ifd_entry_count)
    gps_ifd += entry(1, 2, 2, b"N\x00\x00\x00" if gps_lat >= 0 else b"S\x00\x00\x00")
    gps_ifd += entry(2, 5, 3, struct.pack(">I", lat_offset))
    gps_ifd += entry(3, 2, 2, b"E\x00\x00\x00" if gps_lon >= 0 else b"W\x00\x00\x00")
    gps_ifd += entry(4, 5, 3, struct.pack(">I", lon_offset))
    gps_ifd += entry(5, 1, 1, bytes([0, 0, 0, 0]))  # AltitudeRef: 0 = above sea level
    gps_ifd += entry(6, 5, 1, struct.pack(">I", alt_offset))
    gps_ifd += struct.pack(">I", 0)
    assert len(gps_ifd) == gps_ifd_size

    overflow = (_deg_min_sec(abs(gps_lat)) + _deg_min_sec(abs(gps_lon))
                + _rational(round(gps_alt_m * 10), 10))

    return b"Exif\x00\x00" + header + ifd0 + exif_ifd + gps_ifd + overflow


def openmvs_gray_stddev(rgb: "Image.Image") -> float:
    """Replicate libs/Common/Maths.h's RGB24TO8(r,g,b) macro (integer weights
    30/59/11 over 100, i.e. the same fixed-point luma OpenMVS's FilterFormat
    uses for 24-bit-to-gray conversion) and return the population stddev of
    the result. Used only to print a number directly comparable to the
    ImageIOHEIFTest C++ assertion on sample_alpha.heic's gray-loaded stddev --
    not bit-exact (Python here reads the already-HEVC-decoded RGB, same as the
    C++ reader would), but the same formula.
    """
    import numpy as np

    arr = np.asarray(rgb.convert("RGB"), dtype=np.uint32)
    r, g, b = arr[..., 0], arr[..., 1], arr[..., 2]
    gray = (r * 30 + g * 59 + b * 11) // 100
    return float(gray.astype(np.float64).std())


def main() -> None:
    if not SRC_IMAGE.exists():
        raise SystemExit(f"source image not found: {SRC_IMAGE}")

    src = Image.open(SRC_IMAGE).convert("RGB")
    w = TARGET_WIDTH
    h = round(src.size[1] * w / src.size[0])
    resized = src.resize((w, h), Image.Resampling.LANCZOS)
    print(f"source {SRC_IMAGE.name}: {src.size} -> resized {resized.size}")

    outdir = SCRIPT_DIR

    # Fixture 1: normal orientation, no rotation anywhere.
    exif_normal = build_exif_blob(1, FOCAL_LENGTH_35MM, GPS_LATITUDE, GPS_LONGITUDE, GPS_ALTITUDE_M)
    out1 = outdir / "sample.heic"
    pillow_heif.from_pillow(resized).save(out1, quality=HEIC_QUALITY, exif=exif_normal)
    print(f"wrote {out1.name}: {out1.stat().st_size} bytes")

    # Fixture 2: same pixels, rotation expressed both as container irot (derived
    # automatically by pillow-heif/libheif from the Orientation tag below) and
    # as an EXIF Orientation=6 tag that survives unmodified in the stored blob.
    exif_rot6 = build_exif_blob(6, FOCAL_LENGTH_35MM, GPS_LATITUDE, GPS_LONGITUDE, GPS_ALTITUDE_M)
    out2 = outdir / "sample_rot6.heic"
    pillow_heif.from_pillow(resized).save(out2, quality=HEIC_QUALITY, exif=exif_rot6)
    print(f"wrote {out2.name}: {out2.stat().st_size} bytes")

    # Fixture 3: same pixels and orientation as sample.heic (Orientation=1, no
    # container rotation), but with a real, fully-OPAQUE alpha channel added --
    # exactly what real iPhone HEIC photos carry. pillow-heif reads the
    # "alpha-ness" off the source PIL image's mode: handing it "RGBA" (as
    # opposed to "RGB" for fixtures 1/2) makes libheif encode a genuine
    # auxiliary alpha image item (urn:mpeg:hevc:2015:auxid:1, linked to the
    # primary item via an 'auxl' entry in 'iref') -- not a simulated/four-th
    # channel hack. See "Independently verify" below for how this was proven
    # by walking the raw ISOBMFF box tree, not just trusted from pillow-heif's
    # own has_alpha self-report.
    rgba = resized.convert("RGBA")
    alpha_plane = rgba.split()[3]
    assert alpha_plane.getextrema() == (255, 255), "expected a fully-opaque alpha plane"
    exif_alpha = build_exif_blob(1, FOCAL_LENGTH_35MM, GPS_LATITUDE, GPS_LONGITUDE, GPS_ALTITUDE_M)
    out3 = outdir / "sample_alpha.heic"
    pillow_heif.from_pillow(rgba).save(out3, quality=HEIC_QUALITY, exif=exif_alpha)
    print(f"wrote {out3.name}: {out3.stat().st_size} bytes")

    # Sanity round-trip: reopen all three and report decoded dims + corner
    # pixels (fixture 1) / alpha presence (fixture 3), so a regenerate always
    # prints the same self-check the original fixtures were validated with.
    for path in (out1, out2, out3):
        heif_file = pillow_heif.open_heif(path)
        heif_image = heif_file[0]
        decoded = heif_image.to_pillow()
        print(f"  decoded {path.name}: dims={decoded.size} mode={decoded.mode} has_alpha={heif_image.has_alpha}")
        if path is out1:
            ww, hh = decoded.size
            corners = {
                "TL": decoded.getpixel((0, 0)),
                "TR": decoded.getpixel((ww - 1, 0)),
                "BL": decoded.getpixel((0, hh - 1)),
                "BR": decoded.getpixel((ww - 1, hh - 1)),
            }
            print(f"    corner RGB values: {corners}")
        if path is out3:
            assert heif_image.has_alpha, "sample_alpha.heic must decode with has_alpha=True"
            assert decoded.mode == "RGBA", "sample_alpha.heic must decode as RGBA"
            decoded_alpha_extrema = decoded.split()[3].getextrema()
            print(f"    alpha plane extrema: {decoded_alpha_extrema} (expect fully opaque (255, 255))")
            assert decoded_alpha_extrema == (255, 255), "decoded alpha must stay fully opaque"
            # The number that matters for the C++ regression test: standard deviation of the
            # OpenMVS 24-bit-to-gray conversion applied to the *decoded RGB* (alpha dropped).
            # Under the bug being locked, the reader would advertise a 32-bit format and this
            # same gray path would instead copy the (constant, 255) alpha byte, i.e. stddev ~= 0.
            stddev = openmvs_gray_stddev(decoded)
            print(f"    OpenMVS-formula gray stddev (alpha correctly dropped): {stddev:.2f} (constant-image bug would give ~0)")

    print("Done. Independently verify with exiftool (read-only), e.g.:")
    print(f'  exiftool -a -G1 -s -Orientation -GPSLatitude -GPSLongitude -FocalLengthIn35mmFormat "{out1}"')
    print(f'  exiftool -a -G1 -s -Orientation -GPSLatitude -GPSLongitude -FocalLengthIn35mmFormat "{out2}"')
    print(f'  exiftool -a -G1 -s -Orientation -GPSLatitude -GPSLongitude -FocalLengthIn35mmFormat "{out3}"')


if __name__ == "__main__":
    main()
