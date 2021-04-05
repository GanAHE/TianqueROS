#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/video/rgbcolors.h>
#include <nuttx/nx/nxcursor.h>

#if CONFIG_NXWIDGETS_BPP == 8
#  define FGCOLOR1             RGB8_WHITE
#  define FGCOLOR2             RGB8_BLACK
#  define FGCOLOR3             RGB8_GRAY
#elif CONFIG_NXWIDGETS_BPP == 16
#  define FGCOLOR1             RGB16_WHITE
#  define FGCOLOR2             RGB16_BLACK
#  define FGCOLOR3             RGB16_GRAY
#elif CONFIG_NXWIDGETS_BPP == 24 || CONFIG_NXWIDGETS_BPP == 32
#  define FGCOLOR1             RGB24_WHITE
#  define FGCOLOR2             RGB24_BLACK
#  define FGCOLOR3             RGB24_GRAY
#else
#  error "Pixel depth not supported (CONFIG_NXWIDGETS_BPP)"
#endif

static const uint8_t g_waitImage[] =
{
  0xaa, 0xaa, 0xaa, 0x80,    /* Row 0 */
  0x09, 0x55, 0x58, 0x00,    /* Row 1 */
  0x09, 0x55, 0x58, 0x00,    /* Row 2 */
  0x09, 0x55, 0x58, 0x00,    /* Row 3 */
  0x09, 0x55, 0x58, 0x00,    /* Row 4 */
  0x02, 0x55, 0x60, 0x00,    /* Row 5 */
  0x00, 0x95, 0x80, 0x00,    /* Row 6 */
  0x00, 0x26, 0x00, 0x00,    /* Row 7 */
  0x00, 0x26, 0x00, 0x00,    /* Row 8 */
  0x00, 0x95, 0x80, 0x00,    /* Row 9 */
  0x02, 0x55, 0x60, 0x00,    /* Row 10 */
  0x09, 0x55, 0x58, 0x00,    /* Row 11 */
  0x09, 0x55, 0x58, 0x00,    /* Row 12 */
  0x09, 0x55, 0x58, 0x00,    /* Row 13 */
  0x09, 0x55, 0x58, 0x00,    /* Row 14 */
  0xaa, 0xaa, 0xaa, 0x80,    /* Row 15 */
};

const struct nx_cursorimage_s g_waitCursor =
{
  .size =
  {
    .w = 13,
    .h = 16
  },
  .color1 =
  {
    FGCOLOR1
  },
  .color2 =
  {
    FGCOLOR1
  },
  .color3 =
  {
    FGCOLOR3
  },
  .image  = g_waitImage
};

