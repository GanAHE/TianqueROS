/****************************************************************************
 * apps/gpsutils/minmea/minmea.c
 *
 * Copyright © 2014 Kosma Moczek <kosma@cloudyourcar.com>
 *
 * Released under the NuttX BSD license with permission from the author:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <wchar.h>
#include <stdarg.h>

#include "gpsutils/minmea.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define boolstr(s) ((s) ? "true" : "false")

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int hex2int(char c)
{
  if (c >= '0' && c <= '9')
    {
      return c - '0';
    }

  if (c >= 'A' && c <= 'F')
    {
      return c - 'A' + 10;
    }

  if (c >= 'a' && c <= 'f')
    {
      return c - 'a' + 10;
    }

  return -1;
}

static inline bool minmea_isfield(char c)
{
  return isprint((unsigned char) c) && c != ',' && c != '*';
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint8_t minmea_checksum(FAR const char *sentence)
{
  uint8_t checksum = 0x00;

  /* Support senteces with or without the starting dollar sign. */

  if (*sentence == '$')
    {
      sentence++;
    }

  /* The optional checksum is an XOR of all bytes between "$" and "*". */

  while (*sentence && *sentence != '*')
    {
      checksum ^= *sentence++;
    }

  return checksum;
}

bool minmea_check(FAR const char *sentence, bool strict)
{
  uint8_t checksum = 0x00;

  /* Sequence length is limited. */

  if (strlen(sentence) > MINMEA_MAX_LENGTH + 3)
    {
      return false;
    }

  /* A valid sentence starts with "$". */

  if (*sentence++ != '$')
    {
      return false;
    }

  /* The optional checksum is an XOR of all bytes between "$" and "*". */

  while (*sentence && *sentence != '*' &&
         isprint((unsigned char) *sentence))
    {
      checksum ^= *sentence++;
    }

  /* If checksum is present... */

  if (*sentence == '*')
    {
      int upper;
      int lower;
      int expected;

      /* Extract checksum. */

      sentence++;
      upper = hex2int(*sentence++);

      if (upper == -1)
        {
          return false;
        }

      lower = hex2int(*sentence++);
      if (lower == -1)
        {
          return false;
        }

      expected = upper << 4 | lower;

      /* Check for checksum mismatch. */

      if (checksum != expected)
        {
          return false;
        }
    }
  else if (strict)
    {
      /* Discard non-checksummed frames in strict mode. */

      return false;
    }

  /* The only stuff allowed at this point is a newline. */

  if (*sentence && strcmp(sentence, "\n") &&
      strcmp(sentence, "\r\n"))
    {
      return false;
    }

  return true;
}

bool minmea_scan(FAR const char *sentence, FAR const char *format, ...)
{
  bool result = false;
  bool optional = false;
  va_list ap;
  va_start(ap, format);

  const char *field = sentence;
#define next_field() \
  do \
    { \
      /* Progress to the next field. */ \
      while (minmea_isfield(*sentence)) \
        { \
          sentence++; \
        } \
      /* Make sure there is a field there. */ \
      if (*sentence == ',') \
        { \
          sentence++; \
          field = sentence; \
        } \
      else \
        { \
          field = NULL; \
        } \
    } \
  while (0)

  while (*format)
    {
      char type = *format++;

      if (type == ';')
        {
          /* All further fields are optional. */

          optional = true;
          continue;
        }

      if (!field && !optional)
        {
          /* Field requested but we ran out if input. Bail out. */

          goto parse_error;
        }

      switch (type)
        {
          /* Single character field (char). */

          case 'c':
            {
              char value = '\0';

              if (field && minmea_isfield(*field))
                {
                  value = *field;
                }

              *va_arg(ap, char *) = value;
            }
            break;

          /* Single character direction field (int). */

          case 'd':
            {
              int value = 0;

              if (field && minmea_isfield(*field))
                {
                  switch (*field)
                    {
                      case 'N':
                      case 'E':
                        value = 1;
                        break;

                      case 'S':
                      case 'W':
                        value = -1;
                        break;

                      default:
                        goto parse_error;
                    }
                }

              *va_arg(ap, int *) = value;
            }
            break;

          /* Fractional value with scale (struct minmea_float). */

          case 'f':
            {
              int sign = 0;
              int_least32_t value = -1;
              int_least32_t scale = 0;

              if (field)
                {
                  while (minmea_isfield(*field))
                    {
                      if (*field == '+' && !sign && value == -1)
                        {
                          sign = 1;
                        }
                      else if (*field == '-' && !sign && value == -1)
                        {
                          sign = -1;
                        }
                      else if (isdigit((unsigned char) *field))
                        {
                          int digit = *field - '0';

                          if (value == -1)
                            {
                              value = 0;
                            }

                          if (value > (INT_LEAST32_MAX-digit) / 10)
                            {
                              /* we ran out of bits, what do we do? */

                              if (scale)
                                {
                                  /* truncate extra precision */

                                  break;
                                }
                              else
                                {
                                  /* integer overflow. bail out. */

                                  goto parse_error;
                                }
                            }

                          value = (10 * value) + digit;
                          if (scale)
                            {
                              scale *= 10;
                            }
                        }
                      else if (*field == '.' && scale == 0)
                        {
                          scale = 1;
                        }
                      else if (*field == ' ')
                        {
                          /* Allow spaces at the start of the field. Not NMEA
                           * conformant, but some modules do this.
                           */

                          if (sign != 0 || value != -1 || scale != 0)
                            {
                              goto parse_error;
                            }
                        }
                      else
                        {
                          goto parse_error;
                        }

                      field++;
                    }
                }

              if ((sign || scale) && value == -1)
                {
                  goto parse_error;
                }

              if (value == -1)
                {
                  /* No digits were scanned. */

                  value = 0;
                  scale = 0;
                }
              else if (scale == 0)
                {
                  /* No decimal point. */

                  scale = 1;
                }

              if (sign)
                {
                  value *= sign;
                }

              *va_arg(ap, struct minmea_float *) =
                  (struct minmea_float) {value, scale};
            }
            break;

          /* Integer value, default 0 (int). */

          case 'i':
            {
              int value = 0;

              if (field)
                {
                  FAR char *endptr;

                  value = strtol(field, &endptr, 10);
                  if (minmea_isfield(*endptr))
                    {
                      goto parse_error;
                    }
                }

              *va_arg(ap, int *) = value;
            }
            break;

          /* String value (char *). */

          case 's':
            {
              char *buf = va_arg(ap, char *);

              if (field)
                {
                  while (minmea_isfield(*field))
                    {
                      *buf++ = *field++;
                    }
                }

              *buf = '\0';
            }
            break;

          /* NMEA talker+sentence identifier (char *). */

          case 't':
            {
              char *buf;
              int f;

              /* This field is always mandatory. */

              if (!field)
                {
                  goto parse_error;
                }

              if (field[0] != '$')
                {
                  goto parse_error;
                }

              for (f = 0; f < 5; f++)
                {
                  if (!minmea_isfield(field[1+f]))
                    {
                      goto parse_error;
                    }
                }

              buf = va_arg(ap, char *);
              memcpy(buf, field+1, 5);
              buf[5] = '\0';
            }
            break;

          /* Date (int, int, int), -1 if empty. */

          case 'D':
            {
              struct minmea_date *date = va_arg(ap, struct minmea_date *);
              int d = -1;
              int m = -1;
              int y = -1;
              int f;

              if (field && minmea_isfield(*field))
                {
                  /* Always six digits. */

                  for (f = 0; f < 6; f++)
                    {
                      if (!isdigit((unsigned char) field[f]))
                        {
                          goto parse_error;
                        }
                    }

                  d = strtol((char[]) {field[0], field[1], '\0'}, NULL, 10);
                  m = strtol((char[]) {field[2], field[3], '\0'}, NULL, 10);
                  y = strtol((char[]) {field[4], field[5], '\0'}, NULL, 10);
                }

              date->day   = d;
              date->month = m;
              date->year   = y;
            }
            break;

          /* Time (int, int, int, int), -1 if empty. */

          case 'T':
            {
              struct minmea_time *time_ = va_arg(ap, struct minmea_time *);
              int h = -1;
              int i = -1;
              int s = -1;
              int u = -1;
              int f;

              if (field && minmea_isfield(*field))
                {
                  /* Minimum required: integer time. */

                  for (f = 0; f < 6; f++)
                    {
                      if (!isdigit((unsigned char) field[f]))
                        {
                          goto parse_error;
                        }
                    }

                  h = strtol((char[]) {field[0], field[1], '\0'}, NULL, 10);
                  i = strtol((char[]) {field[2], field[3], '\0'}, NULL, 10);
                  s = strtol((char[]) {field[4], field[5], '\0'}, NULL, 10);
                  field += 6;

                  /* Extra: fractional time. Saved as microseconds. */

                  if (*field++ == '.')
                    {
                      int value = 0;
                      int scale = 1000000;

                      while (isdigit((unsigned char) *field) && scale > 1)
                        {
                          value = (value * 10) + (*field++ - '0');
                          scale /= 10;
                        }

                      u = value * scale;
                    }
                  else
                    {
                      u = 0;
                    }
                }

              time_->hours        = h;
              time_->minutes      = i;
              time_->seconds      = s;
              time_->microseconds = u;
            }
            break;

          /* Ignore the field. */

          case '_':
            {
            }
            break;

          default:
            {
              goto parse_error;
            }
            break;
        }

      next_field();
    }

  result = true;

parse_error:
  va_end(ap);
  return result;
}

bool minmea_talker_id(char talker[3], FAR const char *sentence)
{
  char type[6];

  if (!minmea_scan(sentence, "t", type))
    {
      return false;
    }

  talker[0] = type[0];
  talker[1] = type[1];
  talker[2] = '\0';

  return true;
}

enum minmea_sentence_id minmea_sentence_id(FAR const char *sentence,
                                           bool strict)
{
  if (!minmea_check(sentence, strict))
    return MINMEA_INVALID;

  char type[6];
  if (!minmea_scan(sentence, "t", type))
    {
      return MINMEA_INVALID;
    }

  if (!strcmp(type+2, "RMC"))
    {
      return MINMEA_SENTENCE_RMC;
    }

  if (!strcmp(type+2, "GGA"))
    {
      return MINMEA_SENTENCE_GGA;
    }

  if (!strcmp(type+2, "GSA"))
    {
      return MINMEA_SENTENCE_GSA;
    }

  if (!strcmp(type+2, "GLL"))
    {
      return MINMEA_SENTENCE_GLL;
    }

  if (!strcmp(type+2, "GST"))
    {
      return MINMEA_SENTENCE_GST;
    }

  if (!strcmp(type+2, "GSV"))
    {
      return MINMEA_SENTENCE_GSV;
    }

  return MINMEA_UNKNOWN;
}

bool minmea_parse_rmc(FAR struct minmea_sentence_rmc *frame,
                      FAR const char *sentence)
{
  /* $GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62 */

  char type[6];
  char validity;
  int latitude_direction;
  int longitude_direction;
  int variation_direction;

  if (!minmea_scan(sentence, "tTcfdfdffDfd",
                   type,
                   &frame->time,
                   &validity,
                   &frame->latitude, &latitude_direction,
                   &frame->longitude, &longitude_direction,
                   &frame->speed,
                   &frame->course,
                   &frame->date,
                   &frame->variation, &variation_direction))
    {
      return false;
    }

  if (strcmp(type+2, "RMC"))
    {
      return false;
    }

  frame->valid = (validity == 'A');
  frame->latitude.value *= latitude_direction;
  frame->longitude.value *= longitude_direction;
  frame->variation.value *= variation_direction;

  return true;
}

bool minmea_parse_gga(FAR struct minmea_sentence_gga *frame,
                      FAR const char *sentence)
{
  /* $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47 */

  char type[6];
  int latitude_direction;
  int longitude_direction;

  if (!minmea_scan(sentence, "tTfdfdiiffcfci_",
                   type,
                   &frame->time,
                   &frame->latitude, &latitude_direction,
                   &frame->longitude, &longitude_direction,
                   &frame->fix_quality,
                   &frame->satellites_tracked,
                   &frame->hdop,
                   &frame->altitude, &frame->altitude_units,
                   &frame->height, &frame->height_units,
                   &frame->dgps_age))
    {
      return false;
    }

  if (strcmp(type+2, "GGA"))
    {
      return false;
    }

  frame->latitude.value *= latitude_direction;
  frame->longitude.value *= longitude_direction;

  return true;
}

bool minmea_parse_gsa(FAR struct minmea_sentence_gsa *frame,
                      FAR const char *sentence)
{
  /* $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39 */

  char type[6];

  if (!minmea_scan(sentence, "tciiiiiiiiiiiiifff",
                   type,
                   &frame->mode,
                   &frame->fix_type,
                   &frame->sats[0],
                   &frame->sats[1],
                   &frame->sats[2],
                   &frame->sats[3],
                   &frame->sats[4],
                   &frame->sats[5],
                   &frame->sats[6],
                   &frame->sats[7],
                   &frame->sats[8],
                   &frame->sats[9],
                   &frame->sats[10],
                   &frame->sats[11],
                   &frame->pdop,
                   &frame->hdop,
                   &frame->vdop))
    {
      return false;
    }

  if (strcmp(type+2, "GSA"))
    {
      return false;
    }

  return true;
}

bool minmea_parse_gll(FAR struct minmea_sentence_gll *frame,
                      FAR const char *sentence)
{
  /* $GPGLL,3723.2475,N,12158.3416,W,161229.487,A,A*41$; */

  char type[6];
  int latitude_direction;
  int longitude_direction;

  if (!minmea_scan(sentence, "tfdfdTc;c",
                   type,
                   &frame->latitude, &latitude_direction,
                   &frame->longitude, &longitude_direction,
                   &frame->time,
                   &frame->status,
                   &frame->mode))
    {
      return false;
    }

  if (strcmp(type+2, "GLL"))
    {
      return false;
    }

  frame->latitude.value *= latitude_direction;
  frame->longitude.value *= longitude_direction;

  return true;
}

bool minmea_parse_gst(FAR struct minmea_sentence_gst *frame,
                      FAR const char *sentence)
{
  /* $GPGST,024603.00,3.2,6.6,4.7,47.3,5.8,5.6,22.0*58 */

  char type[6];

  if (!minmea_scan(sentence, "tTfffffff",
                   type,
                   &frame->time,
                   &frame->rms_deviation,
                   &frame->semi_major_deviation,
                   &frame->semi_minor_deviation,
                   &frame->semi_major_orientation,
                   &frame->latitude_error_deviation,
                   &frame->longitude_error_deviation,
                   &frame->altitude_error_deviation))
    {
      return false;
    }

  if (strcmp(type+2, "GST"))
    {
      return false;
    }

  return true;
}

bool minmea_parse_gsv(FAR struct minmea_sentence_gsv *frame,
                      FAR const char *sentence)
{
  /* $GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74
   * $GPGSV,3,3,11,22,42,067,42,24,14,311,43,27,05,244,00,,,,*4D
   * $GPGSV,4,2,11,08,51,203,30,09,45,215,28*75
   * $GPGSV,4,4,13,39,31,170,27*40
   * $GPGSV,4,4,13*7B */

  char type[6];

  if (!minmea_scan(sentence, "tiii;iiiiiiiiiiiiiiii",
                   type,
                   &frame->total_msgs,
                   &frame->msg_nr,
                   &frame->total_sats,
                   &frame->sats[0].nr,
                   &frame->sats[0].elevation,
                   &frame->sats[0].azimuth,
                   &frame->sats[0].snr,
                   &frame->sats[1].nr,
                   &frame->sats[1].elevation,
                   &frame->sats[1].azimuth,
                   &frame->sats[1].snr,
                   &frame->sats[2].nr,
                   &frame->sats[2].elevation,
                   &frame->sats[2].azimuth,
                   &frame->sats[2].snr,
                   &frame->sats[3].nr,
                   &frame->sats[3].elevation,
                   &frame->sats[3].azimuth,
                   &frame->sats[3].snr))
    {
      return false;
    }

  if (strcmp(type+2, "GSV"))
    {
      return false;
    }

  return true;
}

int minmea_gettime(FAR struct timespec *ts,
                   FAR const struct minmea_date *date,
                   FAR const struct minmea_time *time_)
{
  struct tm tm;

  if (date->year == -1 || time_->hours == -1)
    {
      return -1;
    }

  memset(&tm, 0, sizeof(tm));
  tm.tm_year = 2000 + date->year - 1900;
  tm.tm_mon  = date->month - 1;
  tm.tm_mday = date->day;
  tm.tm_hour = time_->hours;
  tm.tm_min  = time_->minutes;
  tm.tm_sec  = time_->seconds;

  /* See README.md if your system lacks timegm(). */

  time_t timestamp = mktime(&tm);
  if (timestamp != -1)
    {
      ts->tv_sec = timestamp;
      ts->tv_nsec = time_->microseconds * 1000;
      return 0;
    }
  else
    {
      return -1;
    }
}
