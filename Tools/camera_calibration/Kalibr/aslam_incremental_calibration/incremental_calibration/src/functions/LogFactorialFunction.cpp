/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "aslam/calibration/functions/LogFactorialFunction.h"

#include <cmath>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    LogFactorialFunction::LogFactorialFunction() {
    }

    LogFactorialFunction::~LogFactorialFunction() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    double LogFactorialFunction::getValue(const VariableType& argument) const {
      if (argument) {
        double value = 0.0;
        for (size_t x = 1; x < argument; ++x)
          value += log(x + 1);
        return value;
      }
      else
        return 0.0;
    }

  }
}
