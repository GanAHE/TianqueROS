/****************************************************************************
 * examples/udpblaster/udpblaster_text.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include "config.h"
#include "udpblaster.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

const char g_udpblaster_text[] =
  "ONE day Henny-penny was picking up corn in the cornyard when--whack!--"
  "something hit her upon the head. 'Goodness gracious me!' said Henny-"
  "penny; 'the sky's a-going to fall; I must go and tell the king.'\n\n"

  "So she went along and she went along and she went along till she met "
  "Cocky-locky. 'Where are you going, Hennypenny?' says Cocky-locky. 'Oh! "
  "I'm going to tell the king the sky's a-falling,' says Henny-penny. 'May "
  "I come with you?' says Cocky-locky. 'Certainly,' says Henny-penny. So "
  "Hennypenny and Cocky-locky went to tell the king the sky was falling.\n\n"

  "They went along, and they went along, and they went along, till they "
  "met Ducky-daddles. 'Where are you going to, Hennypenny and Cocky-locky?' "
  "says Ducky-daddles. 'Oh! we're going to tell the king the sky's a-falling,' "
  "said Henny-penny and Cocky-locky. 'May I come with you?' said Ducky-daddles. "
  "'Certainly,' said Henny-penny and Cocky-locky. So Hennypenny, Cocky-locky, "
  "and Ducky-daddles went to tell the king the sky was a-falling.\n\n"

  "So they went along and they went along, and they went along, till they met "
  "Goosey-poosey. 'Where are you going to, Henny-penny, Cocky-locky, and "
  "Ducky-daddles?' said Gooseypoosey. 'Oh! we're going to tell the king the "
  "sky's a-falling,' said Henny-penny and Cocky-locky and Ducky-daddles. 'May "
  "I come with you?' said Goosey-poosey. 'Certainly,' said Hennypenny, "
  "Cocky-locky, and Ducky-daddles. So Henny-penny, Cocky-locky, Ducky-daddles, "
  "and Goosey-poosey went to tell the king the sky was a-falling.\n\n"

  "So they went along, and they went along, and they went along, till they met "
  "Turkey-lurkey. 'Where are you going, Henny-penny, Cocky-locky, Ducky-daddles, "
  "and Gooseypoosey?' says Turkey-turkey. 'Oh! we're going to tell the king the "
  "sky's a-falling,' said Henny-penny, Cocky-locky, Duckydaddies, and Goosey-poosey. "
  "'May I come with you, Hennypenny, Cocky-locky, Ducky-daddles, and Goosey-poosey?' "
  "said Turkey-lurkey. 'Oh, certainly, Turkey-turkey,' said Henny-penny, Cocky-locky, "
  "Ducky-daddles, and Gooseypoosey. So Henny-penny, Cocky-locky, Ducky-daddles, "
  "Goosey-poosey, and Turkey-lurkey all went to tell the king the sky was a-"
  "falling.\n\n"

  "So they went along, and they went along, and they went along, till they met "
  "Foxy-woxy, and Foxy-woxy said to Hennypenny, Cocky-locky, Ducky-daddles, "
  "Goosey-poosey, and Turkey-lurkey: 'Where are you going, Henny-penny, "
  "Cockylocky, Ducky-daddles, Goosey-poosey, and Turkey-lurkey?' And Henny-penny, "
  "Cocky-locky, Ducky-daddles, Goosey poosey, and Turkey-lurkey said to "
  "Foxy-woxy: 'We' re going to tell the king the sky's a-falling.' 'Oh! but "
  "this is not the way to the king, Henny-penny, Cocky-locky, Ducky-daddles, "
  "Goosey-poosey, and Turkey-lurkey,' says Foxy-woxy; 'I know the proper way; "
  "shall I show it you?' 'Oh, certainly, Foxywoxy,' said Henny-penny, Cocky-locky, "
  "Ducky-daddles, Goosey-poosey, and Turkey-lurkey. So Henny-penny, Cockylocky, "
  "Ducky-daddles, Goosey-poosey, Turkey-lurkey, and Foxy-woxy all went to tell "
  "the king the sky was a-falling. So they went along, and they went along, and "
  "they went along, till they came to a narrow and dark hole. Now this was the "
  "door of Foxy-woxy's cave. But Foxy-woxy said to Henny-penny, Cocky-locky, "
  "Ducky-daddles, Goosey-poosey, and Turkeyturkey: 'This is the short way to the "
  "king's palace: you'll soon get there if you follow me. I will go first and you "
  "come after, Henny-penny, Cocky-locky, Ducky-daddles, Goosey-poosey, and "
  "Turkey-turkey.' 'Why, of course, certainly, without doubt, why not?' said "
  "Henny-penny, Cocky-locky, Ducky-daddles, Goosey-poosey, and Turkey-lurkey.\n\n"

  "So Foxy-woxy went into his cave, and he didn't go very far, but turned round "
  "to wait for Henny-penny, Cocky-locky, Ducky-daddles, Goosey-poosey, and "
  "Turkey-lurkey. So at last at first Turkey-lurkey went through the dark hole "
  "into the cave. He hadn't got far when 'Hrumph', Foxy-woxy snapped off Turkey-"
  "lurkey's head and threw his body over his left shoulder. Then Goosey-poosey "
  "went in, and 'Hrumph', off went her head and Goosey-poosey was thrown beside "
  "Turkey-lurkey. Then Ducky-daddles waddled down, and 'Hrumph', snapped "
  "Foxy-woxy, and Ducky-daddles's head was off and Duckydaddies was thrown "
  "alongside Turkey-turkey and Gooseypoosey. Then Cocky-locky strutted down "
  "into the cave, and he hadn't gone far when 'Snap, Hrumph!' went Foxy-woxy, "
  "and Cocky-locky was thrown alongside of Turkey-lurkey, Gooseypoosey, and "
  "Ducky-daddles.\n\n"

  "But Foxy-woxy had made two bites at Cocky-locky, and when the first snap "
  "only hurt Cocky-locky, but didn't kill him, he called out to Henny-penny. "
  "But she turned tail and off she ran home, so she never told the king the "
  "sky was a-falling.\n\n";

const int g_udpblaster_strlen = sizeof(g_udpblaster_text) - 1;
