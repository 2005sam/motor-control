// Copyright (C) 2025 b2(shengpengxiang1@outlook.com)
// This This program is free software:
// you can redistribute it and/or modify it under the terms of the
// GNU General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.

// this file is a slide mode control compute file
#include "slide_mode_compute.h"
#include "math.h"
float k_t = 0.03f;
float b = 0.01f;
float k = 100;
float d = 0.1f;
float SlideModeCompute(float sp, float cur)
{
  float error = sp - cur;
  float u_eq = b / k_t * error;
  float u_sw = k * error / (fabs(error) + d);
  return u_eq + u_sw;
}